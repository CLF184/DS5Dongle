// Flash-backed slot table for 4-slot persistent BT pairing.
// Modeled on zurce/DS5Dongle-OLED (bt.cpp:29-115). Credit to zurce.

#include "slots.h"

#include <cstring>
#include <cstdio>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"
#include "pico/btstack_flash_bank.h"

#include "bt.h"            // bt_is_connected / bt_disconnect（槽位 API 从 bt.cpp 搬来）
#include "config.h"        // 当前槽游标随 config 持久化
#include "gap.h"           // discoverable / inquiry / link-key 操作
#include "btstack_util.h"  // bd_addr_* 辅助

constexpr uint32_t SLOTS_MAGIC = 0x44533502u;  // "DS5\x02"
// 扇区布局：SDK 的 BTstack TLV（= 经典蓝牙 link key 库）占用
// PICO_FLASH_BANK_STORAGE_OFFSET 起的 2 个扇区，双 bank 交替写
// （bank0 = SIZE-12K、bank1 = SIZE-8K）。自建存储必须从 bank 往下堆：
// config 在 bank 下方第一个扇区，slots 再往下让一个。
// 旧值曾是 PICO_FLASH_SIZE_BYTES - 2*FLASH_SECTOR_SIZE（正好 = bank1）：写槽位会
// 擦掉手柄的 link key 库（需重新配对），TLV 轮换也会反过来擦掉槽位。
// 改址后旧数据自然失效（magic 校验不过 → 槽位恢复默认，手柄配对不受影响）。
constexpr uint32_t SLOTS_FLASH_OFFSET = PICO_FLASH_BANK_STORAGE_OFFSET - 2u * FLASH_SECTOR_SIZE;

struct __attribute__((packed)) SlotsData {
    uint32_t magic;
    uint8_t  addrs[kNumSlots][6];
    uint8_t  occupied[kNumSlots];
};

static_assert(sizeof(SlotsData) <= FLASH_PAGE_SIZE);
static_assert(SLOTS_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0);
// 编译期防呆：slots 必须整个落在 BTstack bank 下方（bank 从 STORAGE_OFFSET 起）。
static_assert(SLOTS_FLASH_OFFSET + FLASH_SECTOR_SIZE <= PICO_FLASH_BANK_STORAGE_OFFSET);

static SlotsData g_slots{};

// 当前槽游标（原先在 bt.cpp：装载后从 config 恢复；bt.cpp 经 bt_get_slot() 读取）
static int g_current_slot = 0;

static const SlotsData *flash_slots() {
    return reinterpret_cast<const SlotsData *>(XIP_BASE + SLOTS_FLASH_OFFSET);
}

// Runs with core1 parked (flash_safe_execute) and core0 interrupts disabled, so
// neither core touches XIP flash while the sector is erased/programmed. Without
// the core1 park this races the audio core and corrupts audio (buzzing). Same
// pattern as config.cpp:config_save_flash_op.
static void slots_save_flash_op(void *param) {
    const auto page = static_cast<const uint8_t *>(param);
    const uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(SLOTS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SLOTS_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(interrupts);
}

static bool save_slots_to_flash() {
    alignas(4) uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xff, sizeof(page));
    memcpy(page, &g_slots, sizeof(g_slots));

    const int rc = flash_safe_execute(slots_save_flash_op, page, 1000);
    if (rc != PICO_OK) {
        printf("[Slots] save flash_safe_execute failed: %d\n", rc);
        return false;
    }

    SlotsData verify{};
    memcpy(&verify, flash_slots(), sizeof(verify));
    if (memcmp(&verify, &g_slots, sizeof(g_slots)) == 0) {
        printf("[Slots] flash write verified\n");
        return true;
    }
    printf("[Slots] flash write VERIFY FAILED\n");
    return false;
}

void slots_load() {
    memcpy(&g_slots, flash_slots(), sizeof(g_slots));
    if (g_slots.magic != SLOTS_MAGIC) {
        printf("[Slots] flash sector empty/invalid, initializing\n");
        memset(&g_slots, 0, sizeof(g_slots));
        g_slots.magic = SLOTS_MAGIC;
        save_slots_to_flash();
    }
    for (int i = 0; i < kNumSlots; i++) {
        if (g_slots.occupied[i]) {
            printf("[Slots] %d: %02X:%02X:%02X:%02X:%02X:%02X\n", i,
                   g_slots.addrs[i][0], g_slots.addrs[i][1], g_slots.addrs[i][2],
                   g_slots.addrs[i][3], g_slots.addrs[i][4], g_slots.addrs[i][5]);
        } else {
            printf("[Slots] %d: (empty)\n", i);
        }
    }

    // 当前槽游标（原先在 bt.cpp 的 bt_init：装载后从 config 恢复）
    g_current_slot = get_config().current_slot;
    if (g_current_slot < 0 || g_current_slot >= kNumSlots) g_current_slot = 0;
    printf("[BT] Boot slot = %d\n", g_current_slot);
}

bool slot_occupied(int slot) {
    if (slot < 0 || slot >= kNumSlots) return false;
    return g_slots.occupied[slot] != 0;
}

void slot_get_addr(int slot, uint8_t out[6]) {
    if (slot < 0 || slot >= kNumSlots) {
        memset(out, 0, 6);
        return;
    }
    memcpy(out, g_slots.addrs[slot], 6);
}

int slot_owner_of(const uint8_t addr[6]) {
    for (int i = 0; i < kNumSlots; i++) {
        if (g_slots.occupied[i] && memcmp(g_slots.addrs[i], addr, 6) == 0) return i;
    }
    return -1;
}

void slot_assign(int slot, const uint8_t addr[6]) {
    if (slot < 0 || slot >= kNumSlots) return;
    memcpy(g_slots.addrs[slot], addr, 6);
    g_slots.occupied[slot] = 1;
    save_slots_to_flash();
}

void slot_forget(int slot) {
    if (slot < 0 || slot >= kNumSlots) return;
    memset(g_slots.addrs[slot], 0, 6);
    g_slots.occupied[slot] = 0;
    save_slots_to_flash();
}

void slots_wipe_all() {
    for (int i = 0; i < kNumSlots; i++) {
        memset(g_slots.addrs[i], 0, 6);
        g_slots.occupied[i] = 0;
    }
    save_slots_to_flash();
}

bool slots_any_empty() {
    for (int i = 0; i < kNumSlots; i++) {
        if (!g_slots.occupied[i]) return true;
    }
    return false;
}

// ---- BT 侧槽位 API（从 bt.cpp 搬入）------------------------------------------
// 可发现性策略：只要还有空槽就保持可发现（覆盖首次配对与部分清空的状态）；
// 4 槽全满后关掉，免得路过的手机乱配。bt.cpp 的状态机不再直接持有槽位游标 ——
// 只经 bt_get_slot() 读数、由本文件改值；其余调用方见 bt.h 的同名声明。
void slots_update_discoverable() {
    if (slots_any_empty()) {
        gap_discoverable_control(1);
    } else {
        gap_discoverable_control(0);
    }
}

int bt_get_slot() { return g_current_slot; }

void bt_set_slot(int slot) {
    if (slot < 0 || slot >= kNumSlots) return;
    if (slot == g_current_slot) return;
    g_current_slot = slot;

    Config_body cfg = get_config();
    cfg.current_slot = (uint8_t)slot;
    set_config(cfg);
    config_save();

    if (bt_is_connected()) {
        // DISCONNECTION_COMPLETE will restart inquiry under the new filter.
        bt_disconnect();
    } else {
        gap_inquiry_stop();
        gap_inquiry_start(30);
    }
    slots_update_discoverable();
}

bool bt_slot_occupied(int slot) { return slot_occupied(slot); }
void bt_slot_get_addr(int slot, uint8_t out[6]) { slot_get_addr(slot, out); }

void bt_forget_slot(int slot) {
    if (slot < 0 || slot >= kNumSlots) return;
    if (slot_occupied(slot)) {
        uint8_t addr[6];
        slot_get_addr(slot, addr);
        gap_drop_link_key_for_bd_addr(addr);
    }
    slot_forget(slot);
    slots_update_discoverable();
    if (slot == g_current_slot && bt_is_connected()) {
        bt_disconnect();
    }
}

void bt_wipe_all_slots() {
    btstack_link_key_iterator_t it;
    if (gap_link_key_iterator_init(&it)) {
        bd_addr_t snapshot[16];
        int n = 0;
        bd_addr_t addr;
        link_key_t key;
        link_key_type_t type;
        while (n < 16 && gap_link_key_iterator_get_next(&it, addr, key, &type)) {
            bd_addr_copy(snapshot[n++], addr);
        }
        gap_link_key_iterator_done(&it);
        for (int i = 0; i < n; i++) {
            gap_drop_link_key_for_bd_addr(snapshot[i]);
        }
    }
    slots_wipe_all();
    slots_update_discoverable();
    if (bt_is_connected()) {
        bt_disconnect();
    }
}

// bt.cpp 的槽位钩子（从那里搬入，bt.cpp 只留调用）。
// 寻呼结果过滤：别的槽的地址跳过；本槽已占用时只接受其精确 bd_addr；空槽接受新地址。
bool slots_filter_inquiry(const uint8_t addr[6]) {
    const int owner = slot_owner_of(addr);
    if (owner >= 0 && owner != g_current_slot) {
        printf("[HCI] Gamepad %s belongs to slot %d, skip (cur=%d)\n",
               bd_addr_to_str(addr), owner, g_current_slot);
        return false;
    }
    if (slot_occupied(g_current_slot)) {
        uint8_t want[6];
        slot_get_addr(g_current_slot, want);
        if (memcmp(want, addr, 6) != 0) {
            printf("[HCI] Slot %d wants different addr, skip %s\n",
                   g_current_slot, bd_addr_to_str(addr));
            return false;
        }
    }
    return true;
}

// 首次配对：当前槽为空时把新地址记入（HID Control 通道打开时由 bt.cpp 调用）。
void slots_assign_current(const uint8_t addr[6]) {
    if (slot_occupied(g_current_slot)) return;
    slot_assign(g_current_slot, addr);
    printf("[Slots] Assigned %s to slot %d\n", bd_addr_to_str(addr), g_current_slot);
    slots_update_discoverable();
}
