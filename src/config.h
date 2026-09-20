//
// Created by awalol on 2026/5/4.
//

#ifndef DS5_BRIDGE_CONFIG_H
#define DS5_BRIDGE_CONFIG_H

#include <cstdint>

struct __attribute__((packed)) Config_body {
    uint8_t config_version; // Config Version
    float haptics_gain; // [1.0,2.0]
    uint8_t speaker_volume; // [0,127] // unused
    uint8_t headset_volume; // [0,127] // max 0x7f // unused
    uint8_t speaker_gain; // [0,7] (0: auto)
    uint8_t inactive_time; // [0,60] min (0: disable)
    uint8_t disable_pico_led; // bool
    uint8_t polling_rate_mode; // 0: 250Hz, 1: 500Hz, 2: real-time
    uint8_t audio_buffer_length; // [16,127]
    uint8_t controller_mode; // 0: DS5, 1: DSE, 2: Auto
    uint8_t enable_usb_sn; // 0: disable,1: enable
    uint8_t ps_shortcut_enabled; // 0: disabled, 1: enabled (Xbox Game Bar via HID keyboard)
    uint8_t mic_select; // 0: auto, 1: builtin, 2: headphone, 3: disable
    uint8_t speaker_select; // 0: auto, 1: builtin, 2: headphone, 3: disable
    uint8_t enable_wake; // bool: 0 disabled (default), 1 wake host on PS press (USB remote wakeup)
    uint8_t trigger_reduce; // [0,10] (0: auto)
    uint8_t lock_volume; // bool
    uint8_t status_gpio_pin; // board-usable GPIO, 0xff: disabled
    uint8_t status_gpio_mode; // 0: high while connected, 1: button pulse on connect
    // ---- 以下为 OLED Edition 特有字段（上游没有；一律追加在末尾，别插到上面去）----
    uint8_t current_slot;    // [0..3] 当前多槽位配对槽（Phase G）
    // Audio Auto Haptics：从扬声器音频派生的触觉（DSP 借自 loteran/DS5Dongle 5d6bc2f）
    uint8_t auto_haptics_enable;  // 0=Off, 1=Fallback (default), 2=Mix, 3=Replace
    uint8_t auto_haptics_gain;    // [0,200] percent, default 100
    uint8_t auto_haptics_lowpass; // 0=80Hz, 1=160Hz (default), 2=250Hz, 3=400Hz
    // Lightbar：灯条模式/收藏色（编号与 oled.cpp 的 kNumLbModes/kLbModeHost 同步）
    uint8_t lightbar_mode;
    uint8_t lb_fav_r[4];
    uint8_t lb_fav_g[4];
    uint8_t lb_fav_b[4];
    // OLED 空闲熄屏阶梯（分钟，0 = 该级禁用）
    uint8_t screen_dim_timeout;
    uint8_t screen_off_timeout;
    // OLED 亮度档（kBrightLevels 下标）
    uint8_t screen_brightness;
    // 0 = 手柄输入不再唤醒 OLED（只有 OLED 自己的按键会）
    uint8_t controller_wakes_display;
};

struct __attribute__((packed)) Config {
    uint32_t magic;
    uint32_t crc32; // Config_body crc32, only calc and verify when save
    uint16_t size;  // Config_body size
    Config_body body;
};

void config_default();
void config_load();
bool config_save();
Config_body& get_config();
void set_config(const uint8_t *new_config, const uint16_t len);
void config_valid();
void set_config(const Config_body &new_config);
extern bool is_dse;

#endif //DS5_BRIDGE_CONFIG_H
