//
// Created by awalol on 2026/3/4.
//

#include <cstdio>
#include "bsp/board_api.h"
#include "bt.h"
#include "button_functions.h"
#include "utils.h"
#include "resample.h"
#include "audio.h"
#include "btstack_util.h"
#if ENABLE_DEBUG
#include "debug.h"
#endif
#include "wake.h"
#include "usb.h"
#ifdef ENABLE_WAKE_HID
#include "ps_shortcut.h"
#endif
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "pico/platform.h"
#if ENABLE_SERIAL
#include "pico/stdio_usb.h"
#endif
#include "config.h"
#include "cmd.h"
#include "dse.h"
#include "status_gpio.h"
#if ENABLE_BATT_LED
#include "battery_led.h"
#endif
#include "oled.h"

// Pico SDK speciifically for waiting on conditions
#include "pico/critical_section.h"

uint8_t reportSeqCounter = 0;
uint8_t packetCounter = 0;
bool spk_active = false;

// Mic-debug instrumentation: count every 0x31 BT input report regardless
// of mic-tag bit, accumulate OR-mask of every byte-2 value seen (tells us
// which bits ever fire) and remember the last byte-2 value. Also track
// observed frame-length range. Surfaced on the OLED Diagnostics screen.
volatile uint32_t g_bt_31_packets = 0;
volatile uint32_t g_bt_other_packets = 0;
volatile uint8_t  g_last_other_id = 0;
volatile uint8_t  g_other_id_or = 0;
volatile uint8_t  g_last_31_b2 = 0;
volatile uint8_t  g_31_b2_or = 0;
volatile uint16_t g_31_len_min = 0xFFFF;
volatile uint16_t g_31_len_max = 0;
volatile uint8_t  g_last_other_prefix[8] = {0};
volatile uint8_t  g_last_any_prefix[16] = {0};
volatile uint16_t g_longest_len = 0;
volatile uint8_t  g_longest_frame[80] = {0};
uint32_t bt_31_packet_count() { return g_bt_31_packets; }
uint8_t  bt_31_last_byte2()  { return g_last_31_b2; }
uint8_t  bt_31_b2_or_mask()  { return g_31_b2_or; }
uint16_t bt_31_len_min()     { return g_31_len_min == 0xFFFF ? 0 : g_31_len_min; }
uint16_t bt_31_len_max()     { return g_31_len_max; }

// Trigger-flow diagnostics. Counts host → dongle → BT path for adaptive
// trigger effects. Lets us tell which link in the chain breaks when games
// like Death Stranding 2 don't produce trigger tension via the dongle:
//   out02_total     - every 0x02 HID OUT report received from host
//   out02_trig_allow - of those, how many set AllowRight/LeftTriggerFFB
//                     (valid_flag0 bits 2 & 3) — i.e. the host actually
//                     told us "apply trigger FFB"
//   out02_to_bt     - 0x02 reports that we forwarded to the controller as
//                     a BT 0x31 sub-0x10 packet (gated off when speaker is
//                     active; audio.cpp's 0x36 path carries state then)
// Surfaced on the OLED Diagnostics screen.
volatile uint32_t g_host_out02_total = 0;
volatile uint32_t g_host_out02_trig_allow = 0;
volatile uint32_t g_host_out02_to_bt = 0;
uint32_t host_out02_total()       { return g_host_out02_total; }
uint32_t host_out02_trig_allow()  { return g_host_out02_trig_allow; }
uint32_t host_out02_to_bt()       { return g_host_out02_to_bt; }

uint8_t interrupt_in_data[63] = {
    0x7f, 0x7d, 0x7f, 0x7e, 0x00, 0x00, 0xa7,
    0x08, 0x00, 0x00, 0x00, 0x52, 0x43, 0x30, 0x41,
    0x01, 0x00, 0x0e, 0x00, 0xef, 0xff, 0x03, 0x03,
    0x7b, 0x1b, 0x18, 0xf0, 0xcc, 0x9c, 0x60, 0x00,
    0xfc, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xa7, 0xad, 0x60, 0x00, 0x29, 0x18, 0x00,
    0x53, 0x9f, 0x28, 0x35, 0xa5, 0xa8, 0x0c, 0x8b
};

critical_section_t report_cs;
volatile bool report_dirty = false;

void __not_in_flash_func(interrupt_loop)() {
    // OLED Edition: hold PS + Mute for 2 seconds to soft-reboot the dongle.
    // Works whether or not the OLED add-on is present. PS+Mute is uncommon
    // during gameplay and the long hold avoids accidental triggers.
    {
        static uint32_t combo_first_us = 0;
        constexpr uint8_t kComboBits = 0x05;       // byte 9: PS (bit 0) + Mute (bit 2)
        constexpr uint32_t kComboHoldUs = 2000000; // 2 seconds
        const bool held = (interrupt_in_data[9] & kComboBits) == kComboBits;
        if (held) {
            const uint32_t now = time_us_32();
            if (combo_first_us == 0) combo_first_us = now;
            else if ((now - combo_first_us) > kComboHoldUs) watchdog_reboot(0, 0, 0);
        } else {
            combo_first_us = 0;
        }
    }

    if (usb_keyboard_only || usb_reconfiguring || !tud_hid_ready()) return;

    // TODO: Refactor for better code reuse
    if (get_config().polling_rate_mode != 2) {
        if (!tud_hid_report(0x01, interrupt_in_data, 63)) {
            printf("[USBHID] tud_hid_report error\n");
        }
        return;
    }

    bool should_send = false;
    // Local buffer to hold the report data while we prepare it to send. 
    uint8_t safe_report[63];


    critical_section_enter_blocking(&report_cs);
    if (report_dirty) {
        memcpy(safe_report, interrupt_in_data, 63);
        report_dirty = false;
        should_send = true;
    }
    critical_section_exit(&report_cs);

    // Only send to TinyUSB if we actually grabbed fresh data
    if (should_send) {
        if (!tud_hid_report(0x01, safe_report, 63)) {
            printf("[USBHID] tud_hid_report error\n");

            // If the report failed to queue, restore the dirty flag 
            // so we try again on the next loop iteration.
            critical_section_enter_blocking(&report_cs);
            report_dirty = true;
            critical_section_exit(&report_cs);
        }
    }
}

void __not_in_flash_func(on_bt_data)(CHANNEL_TYPE channel, uint8_t *data, uint16_t len) {
    // printf("[Main] BT data callback: channel=%u len=%u\n", channel, len);
    // Track ALL INTERRUPT input reports, not just 0x31. The mic stream
    // may live on a different report ID — confirmed 2026-05-19 that data[2]
    // bit 0 (and bit 1) is NOT a mic flag, just the report-type indicator;
    // every "mic-tagged" frame turned out to be standard input.
    if (channel == INTERRUPT && len > 1) {
        if (data[1] == 0x31) g_bt_31_packets++;
        else {
            g_bt_other_packets++;
            g_last_other_id = data[1];
            g_other_id_or = (uint8_t)(g_other_id_or | data[1]);
            for (uint16_t i = 0; i < 8 && i < len; i++) {
                g_last_other_prefix[i] = data[i];
            }
        }
        if (len > 2) {
            g_last_31_b2 = data[2];
            g_31_b2_or = (uint8_t)(g_31_b2_or | data[2]);
        }
        if (len < g_31_len_min) g_31_len_min = len;
        if (len > g_31_len_max) g_31_len_max = len;
        for (uint16_t i = 0; i < 16 && i < len; i++) {
            g_last_any_prefix[i] = data[i];
        }

        // Capture the entire content of the longest 0x31 frame we've
        // seen. Long frames almost certainly carry the mic audio appended
        // after the standard 63-byte input report — this lets us look
        // at the trailing bytes directly via 0xFD diagnostic.
        if (data[1] == 0x31 && len > g_longest_len) {
            g_longest_len = len;
            for (uint16_t i = 0; i < 80 && i < len; i++) {
                g_longest_frame[i] = data[i];
            }
        }
    }

    if (channel == INTERRUPT && len > 2 && data[1] == 0x31) {
        // Mic audio: controller signals mic payload via bit1 of data[2];
        // the opus-encoded mic frame starts at data+4.
        if ((data[2] >> 1) & 1) {
            if (len >= 4) {
                mic_add_queue(data + 4, len - 4);
            }
            return;
        }
        if ((data[56] & 1) != (interrupt_in_data[53] & 1)) {
            set_headset(data[56] & 1);
        }
        if (((data[56] >> 2) & 1) != ((interrupt_in_data[53] >> 2) & 1)) {
            const SetStateData state{
                .AllowMuteLight = 1,
                .MuteLightMode = ((data[56] >> 2) & 1) ? MuteLight::On : MuteLight::Off,
            };
            update_state(state);
        }
        /*if (((data[12] >> 2) & 1) != ((interrupt_in_data[9] >> 2) & 1)) {
            // 如果开启了扬声器静音，这时候再按下麦克风静音，会导致扬声器静音接触。实测有线连接 DS5 也会有这个 bug
            // 有 bug，会导致游戏设置与固件设置冲突。但是实测有线连接在游戏外也不支持开关静音，先不做了。
            const SetStateData state{
                .AllowAudioMute = 1,
                .MicMute = !((interrupt_in_data[56] >> 2) & 1),
            };
            update_state(state);
        }*/

        // Wake-on-PS must observe every BT input report regardless of polling
        // mode: the wake feature has its own state to maintain (button-byte
        // diff for edge detection) and short-circuiting it on non-2 polling
        // modes silently breaks wake while the host is suspended.
        wake_on_bt_input(data + 3, len - 3);
        #ifdef ENABLE_WAKE_HID
        if (!usb_keyboard_only && !usb_reconfiguring) {
            ps_shortcut_tick(data + 3, len - 3);
        }
        #endif

        if (get_config().polling_rate_mode != 2) {
            memcpy(interrupt_in_data, data + 3, 63);
#if ENABLE_BATT_LED
            battery_led_note_report();
#endif
            return;
        }

        // We add the critical section here to avoid any race conditions when writing to the interrupt_in_data buffer,
        // which is shared between the main loop and this callback.
        // The critical section ensures that only one thread can access the buffer at a time,
        // preventing data corruption and ensuring thread safety.
        // We also set the report_dirty flag to true to indicate that new data is available
        //  and needs to be sent in the next interrupt report.
        critical_section_enter_blocking(&report_cs);
        memcpy(interrupt_in_data, data + 3, 63);
        report_dirty = true;
        critical_section_exit(&report_cs);
#if ENABLE_BATT_LED
        battery_led_note_report();
#endif
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
#ifdef ENABLE_WAKE_HID
    if (itf == usb_keyboard_instance()) {
        if (reqlen >= 8) {
            memset(buffer, 0, 8);
            return 8;
        }
        return 0;
    }
#endif
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    // --- DualSense feature reports that Linux's hid_playstation reads at probe ---
    // Without valid answers the kernel never creates a gamepad device, so games
    // outside Steam Input (Heroic/Proton/native) see no controller. The host asks
    // only for the report DATA (reqlen = report_size - 1); usbhid prepends the
    // report-id byte itself. The two CRC'd reports validate crc32 over
    // [0xA3 feature-seed, report_id, data...] in the last 4 bytes.
    // hid_playstation (kernel) AND the game's native DualSense detection both read
    // 0x09 (pairing), 0x20 (firmware) and 0x05 (calibration). The KERNEL only checks
    // size + crc, but the GAME validates the actual CONTENT — so synthesized zeros
    // pass the kernel yet get rejected by the game (a ~156x GET retry storm, and no
    // native adaptive triggers). Serve the REAL controller data, which init_feature()
    // caches from the controller over BT (get_feature_data returns it incl. the
    // report-id at [0]). Fall back to a crc-valid synthetic answer ONLY when the
    // controller isn't linked yet (USB-enumeration probe before the BT link), so the
    // kernel still binds at that moment.
    if (report_id == 0x09 || report_id == 0x20 || report_id == 0x05) {
        if (reqlen == 0) return 0;
        std::vector<uint8_t> real = get_feature_data(report_id, reqlen);
        // 合并后 feature_data 采用上游约定：**不含报告 ID**（bt.cpp 存 packet+2）。
        // 主机侧 usbhid/WebHID 会自己把 report id 补在返回缓冲区第 0 字节，所以
        // 这里原样整段拷贝即可——旧 fork 约定（向量第 0 字节是 ID）才需要 +1。
        if (!real.empty()) {                       // real cached response present
            uint16_t n = (uint16_t) real.size();
            if (n > reqlen) n = reqlen;
            memcpy(buffer, real.data(), n);
            return n;
        }
        memset(buffer, 0, reqlen);
        if (report_id == 0x09) {                   // not linked yet: MAC-only stub
            if (reqlen >= 6) bt_get_addr(buffer);
            return reqlen;
        }
        if (reqlen < 5) return 0;                  // 0x20 / 0x05 stub: zeros + valid crc32
        uint8_t tmp[2 + 64];
        tmp[0] = 0xA3; tmp[1] = report_id;
        memcpy(tmp + 2, buffer, reqlen - 4);
        uint32_t crc = crc32_seeded(tmp, (size_t)(2 + (reqlen - 4)), 0);
        buffer[reqlen - 4] = (uint8_t)(crc);
        buffer[reqlen - 3] = (uint8_t)(crc >> 8);
        buffer[reqlen - 2] = (uint8_t)(crc >> 16);
        buffer[reqlen - 1] = (uint8_t)(crc >> 24);
        return reqlen;
    }

    if (is_pico_cmd(report_id)) {
        return pico_cmd_get(report_id, buffer, reqlen);
    }

    // DSE profiles: while the unlock + prefetch is still in progress, return 0
    // (NAK) for profile reads so the PS app retries rather than caching an
    // empty snapshot. Still kick off the background BT fetch.
    if (dse_is_profile_report(report_id) && !dse_profiles_ready()) {
        get_feature_data(report_id, reqlen);
        return 0;
    }

    std::vector<uint8_t> feature_data = get_feature_data(report_id, reqlen);
    if (!feature_data.empty()) {
        memcpy(buffer, feature_data.data(), feature_data.size());
    }

    return feature_data.empty() ? 0 : feature_data.size();
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void) rhport;
    uint8_t const itf = tu_u16_low(p_request->wIndex); // wInterface
    uint8_t const alt = tu_u16_low(p_request->wValue); // bAlternateSetting

    if (itf == 1) {
        printf("[AUDIO] Set interface Speaker to alternate setting %d\n", alt);
        spk_active = alt;
    }
    if (itf == 2) { // ITF_NUM_AUDIO_STREAMING_IN (microphone)
        printf("[AUDIO] Set interface Microphone to alternate setting %d\n", alt);
        set_mic_active(alt);
    }

    return true;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
#ifdef ENABLE_WAKE_HID
    if (itf == usb_keyboard_instance()) {
        // Drop keyboard SET_REPORT (host LED state).
        return;
    }
#endif
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;

    if (is_pico_cmd(report_id)) {
#if ENABLE_VERBOSE
        printf("[HID] Receive 0xf6 setting config, funcid:0x%02X\n", buffer[0]);
#endif
        pico_cmd_set(report_id, buffer, bufsize);
        return;
    }

    // INTERRUPT OUT
    if (report_id == 0) {
        switch (buffer[0]) {
            case 0x02: {
                g_host_out02_total++;
                // valid_flag0 lives at buffer[1] (right after the 0x02 report id).
                // Bits 2 & 3 are AllowRight/LeftTriggerFFB.
                if (bufsize > 1 && (buffer[1] & 0x0C)) {
                    g_host_out02_trig_allow++;
                }
                // 灯条 HOST 模式要在 OLED 上显示"主机当前颜色"：把这份 0x02 的 RGB 记到
                // oled.cpp 的镜像里（旧 state_mgr 的职责；上游没有灯条 UI，故只此一处）。
                if (bufsize > 1) oled_note_host_state(buffer + 1, (uint16_t)(bufsize - 1));
                // 反向移植：音频帧不再携带 state[]（上游 0x39 双包只装 haptics+speaker），
                // 因此这里必须像上游一样"无条件直发" 0x31——否则播放音频期间主机的
                // 马达/扳机/LED 变化会被丢掉（旧 fork 靠音频帧捎带，故可跳过直发）。
                uint8_t outputData[78]{};
                outputData[0] = 0x31;
                outputData[1] = reportSeqCounter << 4;
                reportSeqCounter = (reportSeqCounter + 1) & 0x0F;
                outputData[2] = 0x10;
                SetStateData state{};
                memcpy(&state,buffer + 1,sizeof(SetStateData));

                const auto &config = get_config();
                if (config.trigger_reduce > 0) {
                    state.AllowMotorPowerLevel = 1;
                    state.TriggerMotorPowerReduction = config.trigger_reduce;
                }
                if (config.speaker_gain > 0) {
                    state.AllowAudioControl2 = 1;
                    state.SpeakerCompPreGain = config.speaker_gain;
                }
                if (config.mic_select != 0) {
                    state.AllowAudioControl = 1;
                    state.MicSelect = config.mic_select;
                    state.NoiseCancelEnable = 1;
                }
                if (config.lock_volume) {
                    state.AllowHeadphoneVolume = 0;
                    state.AllowMicVolume = 0;
                    state.AllowSpeakerVolume = 0;
                    state.AllowAudioMute = 0;
                    state.AllowMuteLight = 0;
                }

                // 固件接管灯条时（OLED 动画/充电脉冲）不让主机的 RGB 打断动画：
                // 把这份转发包里的 LED 三字节清零（等价于旧 state_mgr 的 AllowLedColor 抑制）。
                if (oled_lightbar_override()) {
                    // 让手柄**彻底忽略**主机这份包里的灯条部分：不仅清 RGB，还要清掉所有
                    // LED 相关的"允许位"——否则主机只要换一个开关就能把灯干掉：
                    //   AllowLedColor=1 + RGB=0        → 被当成"设成黑色"
                    //   ResetLights=1                  → 把灯从固件控制里"释放"（灭）
                    //   AllowLightBrightnessChange=1   → 用亮度把灯压暗/压灭
                    //   AllowColorLightFadeAnimation=1 → 用淡出动画把灯淡到黑
                    // 叠加起来就是"琥珀 ↔ 黑"闪烁。旧 state_mgr 只清第一位，靠音频帧每
                    // 10ms 重刷固件颜色掩盖；我们的直发播放音频时只有 10Hz，盖不住，故清全。
                    state.AllowLedColor = 0;
                    state.ResetLights = 0;
                    state.AllowLightBrightnessChange = 0;
                    state.AllowColorLightFadeAnimation = 0;
                    state.LedRed = 0;
                    state.LedGreen = 0;
                    state.LedBlue = 0;
                }

                memcpy(outputData + 3, &state, sizeof(SetStateData));
                bt_write(outputData, sizeof(outputData));
#if ENABLE_VERBOSE
                printf_hexdump(outputData,sizeof(outputData));
#endif
                g_host_out02_to_bt++;
                break;
            }
        }
    }
    if (report_id == 0x80 ||
        // DSE: Write Profile Block
        report_id == 0x60 ||
        report_id == 0x62 ||
        report_id == 0x61) {
        // fork 有意保留（上游 8eb8612 注释掉了它）：0x80 = 网页工具的"测试命令"，
        // 必须原样转发给手柄；0x81 读侧由 bt.cpp 的 get_feature_data 当场查询并
        // 原样返回（72d4d21 的透传查询机制，两半缺一不可）。
        // 0x60-0x62 是 Edge 的档位写入，对普通 DS5 不会出现。
        set_feature_data(report_id, const_cast<uint8_t *>(buffer), bufsize);
    }
}

int main() {
#if SYS_CLOCK_KHZ != 150000
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
#endif

    board_init();
    config_load();
#if !ENABLE_SERIAL
    usb_keyboard_only = get_config().enable_wake;
#endif
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
#if !ENABLE_SERIAL
    if (!usb_keyboard_only) {
        sleep_ms(150);
        tud_disconnect();
    }
#endif
    board_init_after_tusb();
#if ENABLE_SERIAL
    stdio_usb_init();
    while (!stdio_usb_connected()) {
        tud_task();
    }
    sleep_ms(150);
#endif

    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

#ifdef CYW43_WL_GPIO_SMPS_PIN
    cyw43_arch_gpio_put(CYW43_WL_GPIO_SMPS_PIN, true);
#endif

#if ENABLE_BATT_LED
    battery_led_init();
#endif

#if !ENABLE_SERIAL
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // 当崩溃重启以后，闪三下灯
        for (int i = 0; i < 6; i++) {
            if (i % 2 == 0) {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            } else {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            }
            sleep_ms(500);
        }
    } else {
        printf("Clean boot\n");
    }
#endif

    // Initialize the critical section for the report buffer
    critical_section_init(&report_cs);
    wake_init();

    gpio_on_disconnect();

    bt_init();
    bt_register_data_callback(on_bt_data);

    audio_init();
    oled_init();

#if !ENABLE_SERIAL
    watchdog_enable(1000, true);
#endif

    while (1) {
#if !ENABLE_SERIAL
        watchdog_update();
#endif
        cyw43_arch_poll();
        bt_connection_watchdog_tick();
        tud_task();
        wake_task();
        audio_loop();
#if ENABLE_DEBUG
        debug_log_core1_stack_usage();
#endif
        interrupt_loop();
        oled_loop();
#if ENABLE_BATT_LED
        battery_led_tick();
#endif
        button_check();
        bt_inquiring_led();
        dse_task();
    }
}
