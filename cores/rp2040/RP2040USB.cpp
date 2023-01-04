/*
    Shared USB for the Raspberry Pi Pico RP2040
    Allows for multiple endpoints to share the USB controller

    Copyright (c) 2021 Earle F. Philhower, III <earlephilhower@yahoo.com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#if !defined(USE_TINYUSB) && !defined(NO_USB)

// clang-format off
#include <Arduino.h>
#include "CoreMutex.h"
// clang-format on
#include "RP2040USB.h"
#include "class/audio/audio.h"
#include "class/hid/hid_device.h"
#include "class/midi/midi.h"
#include "hardware/irq.h"
#include "pico/mutex.h"
#include "pico/time.h"
#include "pico/unique_id.h"
#include "tusb.h"

// Big, global USB mutex, shared with all USB devices to make sure we don't
// have multiple cores updating the TUSB state in parallel
mutex_t __usb_mutex;

// USB processing will be a periodic timer task
#define USB_TASK_INTERVAL 1000
static int __usb_task_irq;

// USB VID/PID (note that PID can change depending on the add'l interfaces)
#define USBD_VID (0x2E8A) // Raspberry Pi

#ifdef SERIALUSB_PID
#define USBD_PID (SERIALUSB_PID)
#else
#define USBD_PID (0x000a) // Raspberry Pi Pico SDK CDC
#endif

static __USBDeviceAttributes __usb_device_attrs_default = {
    USBD_VID,       // vid
    USBD_PID,       // pid
    "Raspberry Pi", // manufacturer name
    "PicoArduino",  // product name
    "",             // serial number, assigned on demand if blank
};

static __USBDeviceAttributes &__usb_device_attrs = __usb_device_attrs_default;

void __USBSetDeviceAttributes(__USBDeviceAttributes &attrs) {
    __usb_device_attrs = attrs;
}

#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

#define USBD_ITF_CDC (0) // needs 2 interfaces
#define USBD_ITF_MAX (2)

#define USBD_CDC_EP_CMD (0x81)
#define USBD_CDC_EP_OUT (0x02)
#define USBD_CDC_EP_IN (0x82)
#define USBD_CDC_CMD_MAX_SIZE (8)
#define USBD_CDC_IN_OUT_MAX_SIZE (64)

#define USBD_STR_0 (0x00)
#define USBD_STR_MANUF (0x01)
#define USBD_STR_PRODUCT (0x02)
#define USBD_STR_SERIAL (0x03)
#define USBD_STR_CDC (0x04)

#define EPNUM_HID 0x83

#define USBD_MSC_EPOUT 0x03
#define USBD_MSC_EPIN 0x84
#define USBD_MSC_EPSIZE 64

#define EPNUM_HID2_EPOUT 0x05
#define EPNUM_HID2_EPIN 0x85

const uint8_t *tud_descriptor_device_cb(void) {
    uint16_t vendorId = __usb_device_attrs.vendorId;
    uint16_t productId = __usb_device_attrs.productId;

    if (productId == USBD_PID) {
        // Need a multi-endpoint config which will require changing the PID to help Windows not barf
        if (__USBInstallKeyboard) {
            productId |= 0x8000;
        }
        if (__USBInstallMouse) {
            productId |= 0x4000;
        }
        if (__USBInstallJoystick) {
            productId |= 0x0100;
        }
        if (__USBInstallMassStorage) {
            productId ^= 0x2000;
        }
        if (__USBInstallSecondHID_RawHID) {
            productId ^= 0x1000;
        }
    }

    static tusb_desc_device_t usbd_desc_device = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = TUSB_CLASS_MISC,
        .bDeviceSubClass = MISC_SUBCLASS_COMMON,
        .bDeviceProtocol = MISC_PROTOCOL_IAD,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor = vendorId,
        .idProduct = productId,
        .bcdDevice = 0x0100,
        .iManufacturer = USBD_STR_MANUF,
        .iProduct = USBD_STR_PRODUCT,
        .iSerialNumber = USBD_STR_SERIAL,
        .bNumConfigurations = 1
    };
    return (const uint8_t *) &usbd_desc_device;
}

int __USBGetKeyboardReportID() { return 1; }

int __USBGetMouseReportID() { return 1 + (__USBInstallKeyboard ? 1 : 0); }

int __USBGetJoystickReportID() {
    return 1 + (__USBInstallKeyboard ? 1 : 0) + (__USBInstallMouse ? 1 : 0);
}

int __USBGetConsumerControlReportID() {
    return 1 + (__USBInstallKeyboard ? 1 : 0) + (__USBInstallMouse ? 1 : 0) + (__USBInstallJoystick ? 1 : 0);
}

int __USBGetHIDInstanceIndexForSharedHID() {
    bool hasSerial = __USBInstallSerial;
    return hasSerial ? 0 : -1;
}

int __USBGetHIDInstanceIndexForRawHID() {
    bool hasHID = __USBInstallKeyboard || __USBInstallMouse || __USBInstallJoystick;
    bool hasHID2 = __USBInstallSecondHID_RawHID;
    if (hasHID && hasHID2) {
        return 1;
    } else if (hasHID2) {
        return 0;
    } else {
        return -1;
    }
}

static int __hid_report_len = 0;
static uint8_t *__hid_report = nullptr;

static int __hid2_report_len = 0;
static uint8_t *__hid2_report = nullptr;

static uint8_t *GetDescHIDReport(int *len) {
    if (len) {
        *len = __hid_report_len;
    }
    return __hid_report;
}

static uint8_t *GetDescHID2Report(int *len) {
    if (len) {
        *len = __hid2_report_len;
    }
    return __hid2_report;
}

enum {
    Report_Type_Keyboard = 0,
    Report_Type_Mouse,
    Report_Type_Joystick,
    Report_Type_ConsumerControl,
    Report_Type_Count
};
void __SetupDescHIDReport() {
    static int report_types[Report_Type_Count];
    int pos = 0;
    {
        if (__USBInstallKeyboard) {
            report_types[pos++] = Report_Type_Keyboard;
        }
        if (__USBInstallMouse) {
            report_types[pos++] = Report_Type_Mouse;
        }
        if (__USBInstallJoystick) {
            report_types[pos++] = Report_Type_Joystick;
        }
        if (__USBInstallConsumerControl) {
            report_types[pos++] = Report_Type_ConsumerControl;
        }
    }
    int count = pos;

    if (count == 0) {
        __hid_report = nullptr;
        __hid_report_len = 0;
        return;
    }

    int size = 0;
    {
        // allocate memory for the HID report descriptors. We don't use them, but need the size here.
        uint8_t desc_hid_report_keyboard[] = { TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(1)) };
        uint8_t desc_hid_report_mouse[] = { TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(1)) };
        uint8_t desc_hid_report_joystick[] = { TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(1)) };
        uint8_t desc_hid_report_consumer_control[] = { TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(1)) };

        for (int i = 0; i < count; i++) {
            int report_type = report_types[i];
            if (report_type == Report_Type_Keyboard) {
                size += sizeof(desc_hid_report_keyboard);
            } else if (report_type == Report_Type_Mouse) {
                size += sizeof(desc_hid_report_mouse);
            } else if (report_type == Report_Type_Joystick) {
                size += sizeof(desc_hid_report_joystick);
            } else if (report_type == Report_Type_ConsumerControl) {
                size += sizeof(desc_hid_report_consumer_control);
            }
        }
    };

    // allocate the "real" HID report descriptor
    __hid_report = (uint8_t *) malloc(size);
    if (__hid_report) {
        __hid_report_len = size;
        int offset = 0;
        for (int i = 0; i < count; i++) {
            int report_type = report_types[i];
            uint8_t report_id = i + 1;
            if (report_type == Report_Type_Keyboard) {
                uint8_t desc[] = { TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(report_id)) };
                memcpy(__hid_report + offset, desc, sizeof(desc));
                offset += sizeof(desc);
            } else if (report_type == Report_Type_Mouse) {
                uint8_t desc[] = { TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(report_id)) };
                memcpy(__hid_report + offset, desc, sizeof(desc));
                offset += sizeof(desc);
            } else if (report_type == Report_Type_Joystick) {
                uint8_t desc[] = { TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(report_id)) };
                memcpy(__hid_report + offset, desc, sizeof(desc));
                offset += sizeof(desc);
            } else if (Report_Type_ConsumerControl) {
                uint8_t desc[] = { TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(report_id)) };
                memcpy(__hid_report + offset, desc, sizeof(desc));
                offset += sizeof(desc);
            }
        }
    }
}

void __SetupDescHID2Report() {
    if (__USBInstallSecondHID_RawHID) {
        uint8_t desc_hid_report_genericHID[] = { TUD_HID_REPORT_DESC_GENERIC_INOUT(64) };
        int size = sizeof(desc_hid_report_genericHID);
        __hid2_report = (uint8_t *) malloc(size);
        memcpy(__hid2_report, desc_hid_report_genericHID, size);
        __hid2_report_len = size;
    } else {
        __hid2_report = nullptr;
        __hid2_report_len = 0;
    }
}

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    if (instance == 0) {
        return GetDescHIDReport(nullptr);
    } else {
        return GetDescHID2Report(nullptr);
    }
}

static uint8_t *usbd_desc_cfg = nullptr;
const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return usbd_desc_cfg;
}

void __SetupUSBDescriptor() {
    if (!usbd_desc_cfg) {
        bool hasSerial = __USBInstallSerial;
        bool hasHID = __USBInstallKeyboard || __USBInstallMouse || __USBInstallJoystick;
        bool hasMSD = __USBInstallMassStorage;
        bool hasHID2 = __USBInstallSecondHID_RawHID;

        uint8_t itf_cdc = -1;
        uint8_t itf_hid = -1;
        uint8_t itf_msd = -1;
        uint8_t itf_hid2 = -1;

        uint8_t itf_pos = 0;
        if (hasSerial) {
            itf_cdc = itf_pos;
            itf_pos += 2;
        }
        if (hasHID) {
            itf_hid = itf_pos;
            itf_pos++;
        }
        if (hasMSD) {
            itf_msd = itf_pos;
            itf_pos++;
        }
        if (hasHID2) {
            itf_hid2 = itf_pos;
            itf_pos++;
        }
        uint8_t interface_count = itf_pos;

        uint8_t cdc_desc[TUD_CDC_DESC_LEN] = {
            // Interface number, string index, protocol, report descriptor len, EP In & Out address, size & polling interval
            TUD_CDC_DESCRIPTOR(itf_cdc, USBD_STR_CDC, USBD_CDC_EP_CMD, USBD_CDC_CMD_MAX_SIZE, USBD_CDC_EP_OUT, USBD_CDC_EP_IN, USBD_CDC_IN_OUT_MAX_SIZE)
        };

        int hid_report_len;
        GetDescHIDReport(&hid_report_len);
        uint8_t hid_desc[TUD_HID_DESC_LEN] = {
            // Interface number, string index, protocol, report descriptor len, EP In & Out address, size & polling interval
            TUD_HID_DESCRIPTOR(itf_hid, 0, HID_ITF_PROTOCOL_NONE, hid_report_len, EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, 10)
        };

        uint8_t msd_desc[TUD_MSC_DESC_LEN] = { TUD_MSC_DESCRIPTOR(itf_msd, 0, USBD_MSC_EPOUT, USBD_MSC_EPIN, USBD_MSC_EPSIZE) };

        int hid2_report_len;
        GetDescHID2Report(&hid2_report_len);
        uint8_t hid2_desc[TUD_HID_INOUT_DESC_LEN] = {
            // Interface number, string index, protocol, report descriptor len, EP In & Out address, size & polling interval
            TUD_HID_INOUT_DESCRIPTOR(itf_hid2, 0, HID_ITF_PROTOCOL_NONE, hid2_report_len, EPNUM_HID2_EPOUT, EPNUM_HID2_EPIN, CFG_TUD_HID_EP_BUFSIZE, 10)
        };

        int usbd_desc_len = TUD_CONFIG_DESC_LEN + (hasSerial ? sizeof(cdc_desc) : 0) +
                            (hasHID ? sizeof(hid_desc) : 0) + (hasMSD ? sizeof(msd_desc) : 0) +
                            (hasHID2 ? sizeof(hid2_desc) : 0);

        uint8_t tud_cfg_desc[TUD_CONFIG_DESC_LEN] = {
            // Config number, interface count, string index, total length, attribute, power in mA
            TUD_CONFIG_DESCRIPTOR(1, interface_count, USBD_STR_0, usbd_desc_len, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, USBD_MAX_POWER_MA)
        };

        // Combine to one descriptor
        usbd_desc_cfg = (uint8_t *) malloc(usbd_desc_len);
        if (usbd_desc_cfg) {
            bzero(usbd_desc_cfg, usbd_desc_len);
            uint8_t *ptr = usbd_desc_cfg;
            memcpy(ptr, tud_cfg_desc, sizeof(tud_cfg_desc));
            ptr += sizeof(tud_cfg_desc);
            if (hasSerial) {
                memcpy(ptr, cdc_desc, sizeof(cdc_desc));
                ptr += sizeof(cdc_desc);
            }
            if (hasHID) {
                memcpy(ptr, hid_desc, sizeof(hid_desc));
                ptr += sizeof(hid_desc);
            }
            if (hasMSD) {
                memcpy(ptr, msd_desc, sizeof(msd_desc));
                ptr += sizeof(msd_desc);
            }
            if (hasHID2) {
                memcpy(ptr, hid2_desc, sizeof(hid2_desc));
                ptr += sizeof(hid2_desc);
            }
        }
    }
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
#define DESC_STR_MAX (20)
    static uint16_t desc_str[DESC_STR_MAX];

    static char idString[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];

    snprintf(idString, sizeof(idString), __usb_device_attrs.serialNumberText);

    static const char *const usbd_desc_str[] = {
        [USBD_STR_0] = "",
        [USBD_STR_MANUF] = __usb_device_attrs.manufacturerName,
        [USBD_STR_PRODUCT] = __usb_device_attrs.productName,
        [USBD_STR_SERIAL] = idString,
        [USBD_STR_CDC] = "Board CDC",
    };

    if (!idString[0]) {
        pico_get_unique_board_id_string(idString, sizeof(idString));
    }

    uint8_t len;
    if (index == 0) {
        desc_str[1] = 0x0409; // supported language is English
        len = 1;
    } else {
        if (index >= sizeof(usbd_desc_str) / sizeof(usbd_desc_str[0])) {
            return nullptr;
        }
        const char *str = usbd_desc_str[index];
        for (len = 0; len < DESC_STR_MAX - 1 && str[len]; ++len) {
            desc_str[1 + len] = str[len];
        }
    }

    // first byte is length (including header), second byte is string type
    desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * len + 2);

    return desc_str;
}

static void usb_irq() {
    // if the mutex is already owned, then we are in user code
    // in this file which will do a tud_task itself, so we'll just do nothing
    // until the next tick; we won't starve
    if (mutex_try_enter(&__usb_mutex, nullptr)) {
        tud_task();
        mutex_exit(&__usb_mutex);
    }
}

static int64_t timer_task(__unused alarm_id_t id, __unused void *user_data) {
    irq_set_pending(__usb_task_irq);
    return USB_TASK_INTERVAL;
}

void __USBStart() __attribute__((weak));

void __USBStart() {
    if (tusb_inited()) {
        // Already called
        return;
    }

    __SetupDescHIDReport();
    __SetupDescHID2Report();
    __SetupUSBDescriptor();

    mutex_init(&__usb_mutex);

    tusb_init();

    __usb_task_irq = user_irq_claim_unused(true);
    irq_set_exclusive_handler(__usb_task_irq, usb_irq);
    irq_set_enabled(__usb_task_irq, true);

    add_alarm_in_us(USB_TASK_INTERVAL, timer_task, nullptr, true);
}

static __USBHIDSetReportCallbackFn __hid_set_report_callback_functions[2];
static int __hid_set_report_callback_functions_count = 0;

void __USBSubscribeHIDSetReportCallback(__USBHIDSetReportCallbackFn fn) {
    if (__hid_set_report_callback_functions_count < 2) {
        __hid_set_report_callback_functions[__hid_set_report_callback_functions_count++] = fn;
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
extern "C" uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    // TODO not implemented
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
extern "C" void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    for (int i = 0; i < 2; i++) {
        __USBHIDSetReportCallbackFn fn = __hid_set_report_callback_functions[i];
        if (fn) {
            fn(instance, report_id, report_type, buffer, bufsize);
        }
    }
}

extern "C" int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) __attribute__((weak));
extern "C" int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    (void) lun;
    (void) lba;
    (void) offset;
    (void) buffer;
    (void) bufsize;
    return -1;
}

extern "C" bool tud_msc_test_unit_ready_cb(uint8_t lun) __attribute__((weak));
extern "C" bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void) lun;
    return false;
}

extern "C" int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) __attribute__((weak));
extern "C" int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    (void) lun;
    (void) lba;
    (void) offset;
    (void) buffer;
    (void) bufsize;
    return -1;
}

extern "C" int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize) __attribute__((weak));
extern "C" int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize) {
    (void) lun;
    (void) scsi_cmd;
    (void) buffer;
    (void) bufsize;
    return 0;
}

extern "C" void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) __attribute__((weak));
extern "C" void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    (void) lun;
    *block_count = 0;
    *block_size = 0;
}

extern "C" void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) __attribute__((weak));
extern "C" void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
    (void) lun;
    vendor_id[0] = 0;
    product_id[0] = 0;
    product_rev[0] = 0;
}

#endif
