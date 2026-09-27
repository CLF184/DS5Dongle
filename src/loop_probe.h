#ifndef DS5_BRIDGE_LOOP_PROBE_H
#define DS5_BRIDGE_LOOP_PROBE_H

#include <cstdint>
#include "hardware/timer.h"   // timer_time_us_32()：static inline，直接读 TIMER 寄存器

// 主循环抖动探针（Diag 屏的 "LoopN" 行）。
//
// 只记一件事：本核主循环相邻两轮之间的**最大**间隔（µs）。凡是会阻塞转发的
// 东西都会体现在这个数上 —— OLED 刷屏、config_save 的 flash 擦写、
// frequency_count_khz 的忙等……两个核各记各的：core0 = USB/BT/OLED 转发主循环，
// core1 = 音频自旋环。
//
// 性能（这是硬要求）：
//   * 每轮成本 ≈ 8 条指令：一次 TIMER 寄存器读 + 一次减法 + 一次比较。用
//     timer_time_us_32() 而不是 time_us_32()，前者是 static inline 的寄存器读
//     （内联进调用者），后者在 SDK 里是 flash 函数 —— core1 的循环是刻意
//     RAM-resident 的，不能因为探针又引入 flash 取指。
//   * 每轮不写任何共享变量：共享槽位只在 1 s 窗口结束时写一次。
//   * 模块内零锁、零临界区：每个核只写自己的槽位，槽位是 32 位对齐的
//     volatile uint32 —— Cortex-M33 上对齐 32 位读写是单拷贝原子的，另一核
//     直接读即可；RP2350 双核共享 SRAM 无 D-cache，volatile 足够（不需要屏障）。
//
// 用法：每个核各持有一个 LoopProbe 实例（只有本核碰它），在循环体最顶部调用
// loop_probe_tick(probe, core_index)。任何核都能读 g_loop_max_us[core]。
//
// ENABLE_LOOP_PROBE=0 时 tick 变成空函数，调用点被编译器整段删掉。

#ifndef ENABLE_LOOP_PROBE
#define ENABLE_LOOP_PROBE 1
#endif

constexpr uint32_t kLoopProbeWindowUs = 1000000u;   // 1 s 一窗（与 Diag 的速率行同节奏）

struct LoopProbe {
    uint32_t last_us;       // 上一轮的时间戳（0 = 还没开始）
    uint32_t win_start_us;  // 当前窗口起点
    uint32_t win_max_us;    // 当前窗口内的最大间隔
    uint32_t win_iters;     // 当前窗口内的轮数（算频率用）
};

extern volatile uint32_t g_loop_max_us[2];  // [0]=core0 [1]=core1：最近一个完整窗口的最大间隔
extern volatile uint32_t g_loop_hz[2];      // 同窗口的循环频率（轮/秒）

#if ENABLE_LOOP_PROBE

inline void loop_probe_tick(LoopProbe &p, int core) {
    const uint32_t now = timer_time_us_32(timer_hw);
    if (p.last_us == 0) {           // 第一轮：只对表，别把"开机至今"算成一次抖动
        p.last_us = now;
        p.win_start_us = now;
        return;
    }
    const uint32_t dt = now - p.last_us;   // uint32 减法，回绕安全
    p.last_us = now;
    if (dt > p.win_max_us) p.win_max_us = dt;
    p.win_iters++;

    const uint32_t elapsed = now - p.win_start_us;
    if (elapsed >= kLoopProbeWindowUs) {
        g_loop_max_us[core] = p.win_max_us;
        g_loop_hz[core] = (uint32_t)((uint64_t)p.win_iters * 1000000u / elapsed);
        p.win_max_us = 0;
        p.win_iters = 0;
        p.win_start_us = now;
    }
}

#else

inline void loop_probe_tick(LoopProbe &, int) {}

#endif // ENABLE_LOOP_PROBE

#endif // DS5_BRIDGE_LOOP_PROBE_H
