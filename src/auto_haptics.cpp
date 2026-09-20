// Auto Haptics 的非内联部分（函数本体见 auto_haptics.h）。
// 这里放：模块单实例、跨核发布存储、外部访问器 audio_ah_out_peak()。
// DSP 借自 loteran/DS5Dongle 5d6bc2f。
#include "auto_haptics.h"

#include <cstdint>

AutoHaptics g_auto_haptics;

// Auto-haptics output peak (0-127 = int8 amplitude of the derived waveform).
// Counted only where the auto branch actually writes h_l/h_r (Fallback mode
// stays 0 while native haptics are active), so the OLED "AH" row reflects
// real auto-haptics usage rather than mere audio energy.
volatile uint16_t g_ah_out_peak = 0;

// 外部访问器（声明在 audio.h）：调用方 oled.cpp/cmd.cpp 只 include audio.h，
// 不做 inline 才不用要求每位调用者都能看到定义。
uint16_t audio_ah_out_peak() { return g_ah_out_peak; }
