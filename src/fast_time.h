#ifndef DS5_BRIDGE_FAST_TIME_H
#define DS5_BRIDGE_FAST_TIME_H

#include <cstdint>
#include "hardware/timer.h"   // timer_time_us_32()：static inline，直接读 TIMER 寄存器

// 主循环的短时定时器直接保存 32 位微秒时间戳，再以无符号微秒差
// 比较阈值：uint32_t(now_us - started_us) >= timeout_us。
// timer_time_us_32() 是内联寄存器读，避免热路径中的 64 位读数和除法。
//
// 微秒计数约每 71.6 分钟按模 2^32 回绕。上述差值在实际经过的时间
// 小于一个完整回绕周期时有效；长时计时使用 SDK 的 time_us_64()。
//
// 不要先把读数除以 1000 再相减：除后的计数并非按模 2^32 回绕，
// 毫秒无符号减法不能抵消底层的微秒回绕。需要开机以来的毫秒数时，
// 使用 to_ms_since_boot(get_absolute_time())，从 SDK 的 64 位计数转换。
// 同一组差值的两个时间戳必须使用相同单位；跨核共享还需正确同步。

inline uint32_t fast_now_us() { return timer_time_us_32(timer_hw); }

#endif // DS5_BRIDGE_FAST_TIME_H
