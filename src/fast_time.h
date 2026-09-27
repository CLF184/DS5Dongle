#ifndef DS5_BRIDGE_FAST_TIME_H
#define DS5_BRIDGE_FAST_TIME_H

#include <cstdint>
#include "hardware/timer.h"   // timer_time_us_32()：static inline，直接读 TIMER 寄存器

// 主循环每秒要跑几万轮，里面那些"到点了吗"的判断不该用 time_us_64() /
// to_ms_since_boot()：
//   * time_us_64() 是 flash 函数调用 + 64 位读；
//   * to_ms_since_boot() 还要再做一次 ÷1000 —— 64 位除法。开机 71.6 分钟后
//     time_us_64() 的高 32 位不再为 0，这条路会退化成 __aeabi_uldivmod 库调用
//     （反汇编实测：button_check / dse_task 里就是这样）。
// 这些开销合计每轮 1~3 µs，按 23k 轮/秒算就是几个百分点的 CPU，全花在"问现在几点"。
//
// 这里给两个内联读数：
//   fast_now_us()  = 一次 TIMER 寄存器读（~5 周期）
//   fast_now_ms()  = 同上 ÷1000，32 位常量除法被编译成一次 umull + 移位
//
// 语义与 time_us_32() / to_ms_since_boot() 一致（都是开机以来的计数），所以原有的
// "now - last >= 阈值"判断照旧写即可 —— 32 位无符号差值天然回绕安全。
//
// 两个注意：
//   1. **同一模块内要统一**：时间戳必须全部来自同一套时钟，别一半 time_us_64()、
//      一半 fast_now_ms()（两者的回绕周期不同，跨过回绕点就会算错）。
//   2. 只在同一核内用。要跨核比较的时间戳用 64 位 time_us_64()。
//   3. fast_now_ms() 的回绕周期是 71.6 分钟（因为底层是 32 位 µs），不是 49.7 天 ——
//      这里的窗口都只有几百毫秒到几秒，远小于回绕周期，安全。

inline uint32_t fast_now_us() { return timer_time_us_32(timer_hw); }
inline uint32_t fast_now_ms() { return fast_now_us() / 1000u; }

#endif // DS5_BRIDGE_FAST_TIME_H
