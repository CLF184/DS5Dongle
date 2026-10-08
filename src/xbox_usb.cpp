#include "tusb.h"
#include "device/usbd_pvt.h"
#include "xbox_usb.h"
#include "xbox_protocol.h"
#include "usb.h"
#include "bt.h"
#include "config.h"
#include "utils.h"
#include "pico/rand.h"
#include "pico/time.h"
#include <cstring>

bool usb_xbox_mode = false;
namespace {
constexpr uint8_t ep_in = 0x81, ep_out = 0x01;
bool opened = false, paused = false;
SetStateData pending_state{};
bool output_pending = false;
alignas(4) uint8_t in_buffer[64], out_buffer[64];
uint32_t now_ms() { return to_ms_since_boot(get_absolute_time()); }
bool send(void *, const uint8_t *p, size_t len) {
    if (!opened || !tud_mounted() || tud_suspended() || usb_reconfiguring) return false;
    if (!usbd_edpt_claim(0, ep_in)) return false;
    std::memcpy(in_buffer, p, len);
    if (usbd_edpt_xfer(0, ep_in, in_buffer, len, false)) return true;
    usbd_edpt_release(0, ep_in);
    return false;
}
void apply(void *, const xbox::Motors &motors) {
    if (!bt_is_connected()) return;
    SetStateData state{};
    state.EnableRumbleEmulation = 1;
    state.UseRumbleNotHaptics = 1;
    // Legacy emulation deliberately uses half strength, as in Sony/SDL's fallback path.
    state.RumbleEmulationLeft = (uint16_t(motors[2]) * 127 + 50) / 100;
    state.RumbleEmulationRight = (uint16_t(motors[3]) * 127 + 50) / 100;
    state.AllowLeftTriggerFFB = state.AllowRightTriggerFFB = 1;
    xbox::trigger_effect(motors[0], state.LeftTriggerFFB);
    xbox::trigger_effect(motors[1], state.RightTriggerFFB);
    if (get_config().trigger_reduce) {
        state.AllowMotorPowerLevel = 1;
        state.TriggerMotorPowerReduction = get_config().trigger_reduce;
    }
    pending_state = state;
    output_pending = true;
    xbox_usb_flush();
}
xbox::Protocol protocol(send, apply, nullptr);
void reset(uint8_t) {
    if (opened) protocol.stop_motors();
    opened = paused = false;
}
void init() { opened = paused = false; }
bool deinit() { reset(0); return true; }
uint16_t open(uint8_t rhport, tusb_desc_interface_t const *itf, uint16_t max_len) {
    constexpr uint16_t length = sizeof(tusb_desc_interface_t) + 2 * sizeof(tusb_desc_endpoint_t);
    if (!usb_xbox_mode || usb_keyboard_only || max_len < length || itf->bInterfaceNumber != 0
        || itf->bInterfaceClass != 0xff || itf->bInterfaceSubClass != 0x47
        || itf->bInterfaceProtocol != 0xd0 || itf->bNumEndpoints != 2) return 0;
    auto *ep = reinterpret_cast<tusb_desc_endpoint_t const *>(reinterpret_cast<uint8_t const *>(itf) + sizeof(*itf));
    for (unsigned i = 0; i < 2; ++i) {
        if (ep[i].bLength != sizeof(*ep) || ep[i].bDescriptorType != TUSB_DESC_ENDPOINT
            || ep[i].bmAttributes.xfer != TUSB_XFER_INTERRUPT || ep[i].wMaxPacketSize != 64
            || ep[i].bEndpointAddress != (i ? ep_out : ep_in)) return 0;
        if (!usbd_edpt_open(rhport, &ep[i])) return 0;
    }
    opened = true; paused = false;
    uint8_t id[8]; xbox_usb_device_id(id);
    protocol.reset(id, now_ms());
    if (!usbd_edpt_xfer(rhport, ep_out, out_buffer, sizeof(out_buffer), false)) {
        opened = false; return 0;
    }
    return length;
}
bool control(uint8_t, uint8_t, tusb_control_request_t const *) { return false; }
bool xfer(uint8_t rhport, uint8_t ep, xfer_result_t result, uint32_t count) {
    if (ep == ep_out) {
        if (result == XFER_RESULT_SUCCESS && count <= sizeof(out_buffer))
            protocol.receive(out_buffer, count, now_ms());
        return usbd_edpt_xfer(rhport, ep_out, out_buffer, sizeof(out_buffer), false);
    }
    return ep == ep_in;
}
const usbd_class_driver_t driver = {
    .name = "Xbox GIP", .init = init, .deinit = deinit, .reset = reset, .open = open,
    .control_xfer_cb = control, .xfer_cb = xfer, .xfer_isr = nullptr, .sof = nullptr,
};
}
void xbox_usb_device_id(uint8_t id[8]) {
    // Keep one ID for this boot, including mode changes, USB reset and reconnect.
    // Pico 2 W's pico_rand uses the RP2350 hardware TRNG.
    static const uint32_t boot_random = get_rand_32();
    xbox::make_device_id(boot_random, id);
}
extern "C" usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *count) {
    *count = 1;
    return &driver;
}
void xbox_usb_stop() { if (opened) protocol.stop_motors(); }
void xbox_usb_flush() {
    if (!output_pending) return;
    if (!bt_is_connected()) { output_pending = false; return; }
    extern uint8_t reportSeqCounter;
    uint8_t report[78]{};
    report[0] = 0x31; report[1] = reportSeqCounter << 4; report[2] = 0x10;
    std::memcpy(report + 3, &pending_state, sizeof(pending_state));
    // Keep the latest motor state for retry if OLED/lightbar traffic fills the BT queue.
    // In particular, never discard a stop command because of queue backpressure.
    if (bt_try_write(report, sizeof(report))) {
        reportSeqCounter = (reportSeqCounter + 1) & 0x0f;
        output_pending = false;
    }
}
void xbox_usb_task(const uint8_t *ds, uint16_t len) {
    if (!usb_xbox_mode || !opened) return;
    if (!tud_mounted() || tud_suspended() || usb_reconfiguring || !bt_is_connected()) {
        if (!paused) { protocol.stop_motors(); paused = true; }
        return;
    }
    paused = false;
    protocol.tick(now_ms(), ds, len);
}
bool xbox_vendor_control(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    if (!usb_xbox_mode || usb_keyboard_only || request->bmRequestType != 0xc0
        || request->bRequest != 0x90 || request->wIndex != 4 || request->wValue != 0) return false;
    if (stage != CONTROL_STAGE_SETUP) return true;
    static const uint8_t compatible_id[40] = {
        40,0,0,0, 0,1, 4,0, 1,0,0,0,0,0,0,0,
        0,1, 'X','G','I','P','1','0',0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,
    };
    return tud_control_xfer(rhport, request, const_cast<uint8_t *>(compatible_id), sizeof(compatible_id));
}
