// Audio Auto Haptics：从扬声器音频派生触觉的 DSP（借自 loteran/DS5Dongle 5d6bc2f）。
// 让"不发触觉数据"的游戏（如 Linux+Steam 的 Ghost of Tsushima）也有震动。
// 模式（config.auto_haptics_enable）：1=Fallback（默认，仅在原生静默时接管）、2=Mix、3=Replace。
//
// .h = 全部逻辑（sample/begin/end 一律 inline：sample() 是逐样点热路径，跨 TU 调用
// 会拖慢 core1 音频循环；begin()/end() 每 USB 帧一次，内联零成本）
// .cpp = 非内联部分：模块单实例、跨核发布存储、外部访问器 audio_ah_out_peak()
// audio.cpp 的用法：
//   g_auto_haptics.begin();
//   ... 逐样点: g_auto_haptics.sample(spk_l, spk_r, h_l, h_r);
//   g_auto_haptics.end(native_max);
#pragma once
#include <cstdint>

#include "config.h"

// 跨核发布：core1 写在 end() 路径，core0（OLED/网页诊断）读 —— 所以 volatile。
// 实体定义在 auto_haptics.cpp。
extern volatile uint16_t g_ah_out_peak;

struct AutoHaptics {
    // 逐样点：低通 + 包络 + 饱和，按模式并入 h_l/h_r；mode==0 直接返回。
    inline void sample(float spk_l, float spk_r, float &h_l, float &h_r) {
        if (mode == 0) return;
        lp_l += lp_a * (spk_l - lp_l);
        lp_r += lp_a * (spk_r - lp_r);
        const float abs_l = lp_l < 0.0f ? -lp_l : lp_l;
        const float abs_r = lp_r < 0.0f ? -lp_r : lp_r;
        env_l = (abs_l > env_l) ? env_l + ENV_ATK * (abs_l - env_l)
                                : env_l + ENV_REL * (abs_l - env_l);
        env_r = (abs_r > env_r) ? env_r + ENV_ATK * (abs_r - env_r)
                                : env_r + ENV_REL * (abs_r - env_r);
        float al = lp_l * (1.0f + 3.0f * env_l) * gain;
        float ar = lp_r * (1.0f + 3.0f * env_r) * gain;
        al = al / (1.0f + (al < 0.0f ? -al : al));
        ar = ar / (1.0f + (ar < 0.0f ? -ar : ar));

        // 只在波形真的写进输出的场合累计峰值（Replace/Mix 恒计；Fallback 仅原生静默时）
        if (mode == 3) {              // Replace
            h_l = al; h_r = ar;
            track(al, ar);
        } else if (mode == 2) {       // Mix
            float m_l = h_l + al, m_r = h_r + ar;
            h_l = m_l / (1.0f + (m_l < 0.0f ? -m_l : m_l));
            h_r = m_r / (1.0f + (m_r < 0.0f ? -m_r : m_r));
            track(al, ar);
        } else if (mode == 1 && fallback_active) {  // Fallback（默认）
            h_l = al; h_r = ar;
            track(al, ar);
        }
    }

    // 每 USB 帧：读 config、算增益/低通、判定 Fallback（沿用上一帧的静默计数）。
    inline void begin() {
        const Config_body &cfg = get_config();
        mode = cfg.auto_haptics_enable;
        static const float LP_COEFF[4] = { 0.01039f, 0.02074f, 0.03095f, 0.05123f };
        lp_a = LP_COEFF[cfg.auto_haptics_lowpass & 3];
        gain = (mode > 0) ? (cfg.auto_haptics_gain / 100.0f) * cfg.haptics_gain : 0.0f;
        fallback_active = (mode == 1) && (native_silent_count >= NATIVE_SILENT_TIMEOUT);
        peak = 0.0f; // per-frame reset（原来是循环里的局部变量）
    }

    // 每帧收尾：发布输出峰值 + 按本帧原生触觉峰值更新静默计数。
    inline void end(float native_max) {
        g_ah_out_peak = (uint16_t)(peak * 127.0f);
        if (native_max > NATIVE_THRESHOLD) {
            native_silent_count = 0;
        } else if (native_silent_count < NATIVE_SILENT_TIMEOUT * 2) {
            native_silent_count++;
        }
    }

  private:
    inline void track(float a, float b) {
        const float aa = a < 0.0f ? -a : a;
        const float bb = b < 0.0f ? -b : b;
        const float a_m = aa > bb ? aa : bb;
        if (a_m > peak) peak = a_m;
    }

    static constexpr float ENV_ATK = 0.40f;
    static constexpr float ENV_REL = 0.025f;
    static constexpr int   NATIVE_SILENT_TIMEOUT = 100;
    static constexpr uint16_t NATIVE_THRESHOLD   = 256;

    uint8_t mode = 0;
    float   gain = 0.0f;
    float   lp_a = 0.0f;
    bool    fallback_active = false;
    float   lp_l = 0.0f, lp_r = 0.0f;
    float   env_l = 0.0f, env_r = 0.0f;
    float   peak = 0.0f;
    int     native_silent_count = NATIVE_SILENT_TIMEOUT * 2;
};

// 模块单实例（实体定义在 auto_haptics.cpp）。
extern AutoHaptics g_auto_haptics;
