#pragma once
#include <cstdint>
#include "tusb.h"

// Latched at boot/re-enumeration: changing RAM config must not change live USB routing.
extern bool usb_xbox_mode;
void xbox_usb_task(const uint8_t *ds, uint16_t len);
void xbox_usb_stop();
void xbox_usb_flush();
void xbox_usb_device_id(uint8_t id[8]);
bool xbox_vendor_control(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request);
