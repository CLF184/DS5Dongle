// Persistent 4-slot multi-controller pairing storage. Stores the 4 bonded
// DS5 bd_addrs in a custom flash sector (BTstack's NVM keeps the link_keys).
//
// Multi-slot pairing modeled on zurce/DS5Dongle-OLED. Credit to zurce.

#ifndef DS5_BRIDGE_SLOTS_H
#define DS5_BRIDGE_SLOTS_H

#include <cstdint>

constexpr int kNumSlots = 4;

void slots_load();
bool slot_occupied(int slot);
void slot_get_addr(int slot, uint8_t out[6]);
int  slot_owner_of(const uint8_t addr[6]);
void slot_assign(int slot, const uint8_t addr[6]);
void slot_forget(int slot);
void slots_wipe_all();
bool slots_any_empty();

// bt.cpp 的状态机钩子：可发现性跟随槽位占用（实现见 slots.cpp 末尾）
void slots_update_discoverable();

// bt.cpp 的槽位钩子（实现见 slots.cpp 末尾）
bool slots_filter_inquiry(const uint8_t addr[6]);
void slots_assign_current(const uint8_t addr[6]);

#endif // DS5_BRIDGE_SLOTS_H
