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

#include "pico/mutex.h"

typedef struct {
    uint16_t vendorId;
    uint16_t productId;
    const char *manufacturerName;
    const char *productName;
    const char *serialNumberText;
} __USBDeviceAttributes;

void __USBSetDeviceAttributes(__USBDeviceAttributes &attrs);

// Weak function definitions for each type of endpoint
extern void __USBInstallSerial() __attribute__((weak));
extern void __USBInstallKeyboard() __attribute__((weak));
extern void __USBInstallJoystick() __attribute__((weak));
extern void __USBInstallMouse() __attribute__((weak));
extern void __USBInstallConsumerControl() __attribute__((weak));
extern void __USBInstallMassStorage() __attribute__((weak));

extern void __USBInstallSecondHID_RawHID() __attribute__((weak));

// Big, global USB mutex, shared with all USB devices to make sure we don't
// have multiple cores updating the TUSB state in parallel
extern mutex_t __usb_mutex;

// HID report ID inquiry (report ID will vary depending on the number/type of
// other HID)
int __USBGetKeyboardReportID();
int __USBGetMouseReportID();
int __USBGetJoystickReportID();
int __USBGetConsumerControlReportID();

int __USBGetHIDInstanceIndexForSharedHID();
int __USBGetHIDInstanceIndexForRawHID();

typedef void (*__USBHIDSetReportCallbackFn)(uint8_t instance, uint8_t report_id,
                                            uint8_t report_type,
                                            uint8_t const *buffer,
                                            uint16_t bufsize);

void __USBSubscribeHIDSetReportCallback(__USBHIDSetReportCallbackFn fn);

// Called by main() to init the USB HW/SW.
void __USBStart();
