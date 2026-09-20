#ifndef DS5_BRIDGE_OLED_H
#define DS5_BRIDGE_OLED_H

#include <cstdint>

void oled_init();
void oled_loop();

// main.cpp 的 0x02 处理调用：记录主机最近一次 0x02 里的 LED 颜色（灯条 HOST 模式显示用）。
void oled_note_host_state(const uint8_t *state, uint16_t len);
// true = 固件正在接管灯条（OLED 动画/充电脉冲），0x02 转发时应清掉主机 RGB。
bool oled_lightbar_override();

#endif // DS5_BRIDGE_OLED_H
