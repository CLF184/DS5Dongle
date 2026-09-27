#include "oled.h"
#include "oled_font.h"
#include "bt.h"
#include "slots.h"
#include "audio.h"
#include "config.h"
#include "utils.h"   // SetStateData（记录主机 0x02 的 LED 颜色）
#include <cstddef>  // offsetof
#include "status_gpio.h" // 上游"状态 GPIO"特性（合法引脚判定 + STATUS_GPIO_DISABLED）
#include "usb.h"         // usb_reconnect（USB 序列号开关保存后重新枚举）
#include "loop_probe.h"  // Diag 屏的 Loop0/Loop1 抖动行
#include "fast_time.h"   // 每轮的门控判断用内联 µs 读数（见该文件注释）

#include <cstdio>
#include <cstring>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "cmd.h"
#include "pico/time.h"

extern uint8_t interrupt_in_data[63]; // defined in main.cpp

// Mic diagnostic counters (defined in main.cpp).
extern uint32_t bt_31_packet_count();
extern uint32_t host_out02_total();
extern uint32_t host_out02_trig_allow();
extern uint32_t host_out02_to_bt();
extern uint8_t  bt_31_last_byte2();
extern uint8_t  bt_31_b2_or_mask();
extern uint16_t bt_31_len_min();
extern uint16_t bt_31_len_max();
extern bool     spk_active; // main.cpp: true while host USB speaker stream is open

// true while an OLED lightbar mode or the charging pulse owns the LED.
// main.cpp 的 0x02 转发路径会读它，把主机那份包里的 RGB 清零，防止动画被打断
// （旧 state_mgr 的 AllowLedColor 抑制；state_mgr 已按上游做法移除）。
bool g_lightbar_override = false;
bool oled_lightbar_override() { return g_lightbar_override; }

// 主机最近一次 0x02 里的 RGB —— 供灯条 HOST 模式在 OLED 上显示（见 main.cpp）。
static uint8_t g_host_led[3] = {0xff, 0xd7, 0x00}; // 初值 = fork 的默认 LED 色
void oled_note_host_state(const uint8_t *st, uint16_t len) {
    if (len < offsetof(SetStateData, LedBlue) + 1) return;
    g_host_led[0] = st[offsetof(SetStateData, LedRed)];
    g_host_led[1] = st[offsetof(SetStateData, LedGreen)];
    g_host_led[2] = st[offsetof(SetStateData, LedBlue)];
}

namespace {

constexpr uint kPinDC = 8;
constexpr uint kPinCS = 9;
constexpr uint kPinCLK = 10;
constexpr uint kPinMOSI = 11;
constexpr uint kPinRST = 12;
constexpr uint kPinKey0 = 15;
constexpr uint kPinKey1 = 17;

constexpr int kW = 128;
constexpr int kH = 64;
constexpr int kRowBytes = kW / 8;
constexpr int kFbBytes = kRowBytes * kH;

uint8_t fb[kFbBytes];

uint32_t last_render_us = 0;
constexpr uint32_t kFrameUs = 100000;
// 开机 splash 的保持窗口（boot_splash() 设置）。splash 只画一次、到点前不重绘，
// 所以主循环（tud_task/BT）立刻就能跑起来，而不是睡 1.5 秒。
// splash_done 让每轮只剩一次 bool 判断 —— 只有开机那 1.5 秒才需要看时间。
uint32_t splash_deadline_us = 0;
bool     splash_done = true;
bool key0_prev = true;
bool key1_prev = true;
uint32_t key0_t_us = 0;
uint32_t key1_t_us = 0;
constexpr uint32_t kDebounceUs = 20000;

// Single-press latch — armed on rising edge, fired on release. KEY0 was
// previously a double-click reboot trigger; that gesture moved to the
// KEY0+KEY1 chord below because rapid forward-navigation kept tripping it.
bool key0_armed = false;

// KEY1 long-press detection (for brightness cycling)
uint32_t key1_press_us = 0;
bool key1_was_pressed = false;
constexpr uint32_t kLongPressUs = 1500000;

// KEY0 + KEY1 simultaneous hold → watchdog_reboot. 1 s hold is long enough
// to filter accidental two-button taps but short enough to feel responsive.
uint32_t chord_held_since_us = 0;
constexpr uint32_t kChordHoldUs = 1000000;

// Brightness levels (SH1107 contrast register 0x81). User cycles via KEY1 long-press.
constexpr uint8_t kBrightLevels[] = {0xFF, 0x7F, 0x3F, 0x10};
constexpr int kNumBrightLevels = sizeof(kBrightLevels) / sizeof(kBrightLevels[0]);
int bright_idx = 0;
uint8_t current_contrast = 0xFF;

// Auto-dim / auto-off after idle. Tracks last button/input activity.
// Tier 1: Active → full brightness (bright_idx).
// Tier 2: idle > dim threshold → contrast drops to kDimContrast (deep dim).
// Tier 3: idle > off threshold → SH1107 panel turned fully off (cmd 0xAE)
//         to prevent OLED burn-in on long unattended sits.
// The two thresholds are user-configurable (Config_body.screen_dim_timeout /
// screen_off_timeout, minutes; 0 = tier disabled) — issue #5. last_activity_us
// is 64-bit µs so the full 0..250 min range is representable without the ~71 min
// wrap of time_us_32().
// kDimContrast tuned by eye: 0x10 looked like only ~10% reduction on this
// panel (contrast-vs-brightness is heavily non-linear near the bottom of
// the register range). 0x02 is visibly dim while still legible up close.
uint64_t last_activity_us = 0;
uint32_t last_input_hash = 0;
constexpr uint8_t kDimContrast = 0x01;
enum OledPowerState { OLED_ACTIVE, OLED_DIM, OLED_OFF };
OledPowerState oled_power_state = OLED_ACTIVE;
bool prev_bt_connected = false;

// Screen ordering — single source of truth. Reorder by editing this block;
// oled_loop's switch and handle_buttons' KEY1 contextual checks use these
// names, so the indices can move without touching that code.
constexpr int kScreenStatus    = 0;
constexpr int kScreenSlots     = 1;
constexpr int kScreenLightbar  = 2;
constexpr int kScreenTriggers  = 3;
constexpr int kScreenGyro      = 4;
constexpr int kScreenTouchpad  = 5;
constexpr int kScreenDiag      = 6;
constexpr int kScreenCpu       = 7;
constexpr int kScreenRssi      = 8;
constexpr int kScreenVU        = 9;
constexpr int kScreenSettings  = 10;
constexpr int kNumScreens      = 11;
int current_screen = 0;

// Lightbar mode cycle: 0=LIVE, 1-4=FAV0-3, 5=BREATHING, 6=RAINBOW, 7=FADE,
// 8=HOST (passthrough — let the host/game own the LED). HOST is the default so
// the dongle doesn't hijack game player-indicator LEDs out of the box. Keep
// this numbering in sync with Config_body::lightbar_mode (src/config.h).
constexpr int kLbModeHost = 8;
constexpr int kNumLbModes = 9;

// Settings screen state
// ---- 设置项描述表（单一事实来源）--------------------------------------------
// 一行 = 一个菜单项。菜单显示、左右调值、顺序编号、长按动作全部由这张表推出，
// 不再有"常量编号 + adjust switch + format switch"三处手工同步的老结构。
// 加一项 = 加一行；宏控项（如 Wake/PS Short）直接 #ifdef 包住那一行，宏开/关时
// 编号自动衔接，不用再手工挪 Reset / Wipe 的位置。
enum SetKind : uint8_t {
    SK_BOOL,   // 翻转开关：左右都 ^=1，显示 names[v]（默认 off/on）
    SK_NUM,    // 数值：lo..hi 按 step 加减，clamp（SIF_WRAP 则环绕）
    SK_ACTION, // 无值项：只能长按三角触发（act 区分动作，hint 是底部提示行）
};
enum SetFmt : uint8_t { SF_PLAIN, SF_OFF0, SF_AUTO0 }; // 值 0 显示 "off"/"auto"
enum : uint8_t { ACT_NONE = 0, ACT_RESET, ACT_WIPE };
enum : uint8_t {
    SIF_WRAP    = 1u << 0, // lo..hi 环绕（枚举项）；否则 clamp
    SIF_SNAP    = 1u << 1, // 档位吸附：0 或 >=10（Inact 空闲超时）
    SIF_FLOAT10 = 1u << 2, // 字段是 float，按 ×10 处理（Hap Gain；packed 偏移不对齐，走 memcpy）
    SIF_PIN     = 1u << 3, // status_gpio：0xFF=off 且调值时跳过不可选脚
};

struct SetItem {
    const char*        label;  // 菜单行标签
    const char*        unit;   // 数值后缀（"dB"/"min"/"%"），可空
    const char*        hint;   // SK_ACTION 的长按提示行
    const char* const* names;  // 取值名表（枚举/开关用；开关留空 = off/on）
    uint8_t kind, fmt, flags, act;
    uint8_t off;               // offsetof(Config_body, 字段)
    int8_t  disp_off;          // 显示偏移（Spk Vol: -100 → dB）
    uint8_t lo, hi, step;
};

static const char* const kOnOff[]    = {"off", "on"};
static const char* const kPicoLed[]  = {"on", "off"};   // 字段是 disable_pico_led，反着显示
static const char* const kPoll[]     = {"250Hz", "500Hz", "RT"};
static const char* const kCtrl[]     = {"DS5", "DSE", "Auto"};
static const char* const kAutoHap[]  = {"Off", "Fallback", "Mix", "Replace"};
static const char* const kAHLp[]     = {"80Hz", "160Hz", "250Hz", "400Hz"};
static const char* const kMicSel[]   = {"auto", "Int", "Ext", "Off"};
static const char* const kSpkOut[]   = {"Auto", "Builtin", "Headset", "Off"};
static const char* const kStatMode[] = {"Level", "Pulse"};

#define CFG_OFF(field) ((uint8_t)offsetof(Config_body, field))
static const SetItem kSetItems[] = {
    {"Hap Gain", nullptr, nullptr, nullptr,    SK_NUM,  SF_PLAIN, SIF_FLOAT10, ACT_NONE, CFG_OFF(haptics_gain),           0, 10, 20, 1},
    {"Spk Vol",  "dB",    nullptr, nullptr,    SK_NUM,  SF_PLAIN, 0,           ACT_NONE, CFG_OFF(speaker_volume),        -100, 0, 100, 5},
    {"Inact",    "min",   nullptr, nullptr,    SK_NUM,  SF_OFF0,  SIF_SNAP,    ACT_NONE, CFG_OFF(inactive_time),         0, 0, 60, 5},
    {"Pico LED", nullptr, nullptr, kPicoLed,   SK_BOOL, SF_PLAIN, 0,           ACT_NONE, CFG_OFF(disable_pico_led),      0, 0, 1, 1},
    {"Poll",     nullptr, nullptr, kPoll,      SK_NUM,  SF_PLAIN, SIF_WRAP,    ACT_NONE, CFG_OFF(polling_rate_mode),     0, 0, 2, 1},
    {"AudBuf",   nullptr, nullptr, nullptr,    SK_NUM,  SF_PLAIN, 0,           ACT_NONE, CFG_OFF(audio_buffer_length),   0, 16, 128, 4},
    {"Ctrl",     nullptr, nullptr, kCtrl,      SK_NUM,  SF_PLAIN, SIF_WRAP,    ACT_NONE, CFG_OFF(controller_mode),       0, 0, 2, 1},
    {"AutoHap",  nullptr, nullptr, kAutoHap,   SK_NUM,  SF_PLAIN, SIF_WRAP,    ACT_NONE, CFG_OFF(auto_haptics_enable),   0, 0, 3, 1},
    {"AH Gain",  "%",     nullptr, nullptr,    SK_NUM,  SF_PLAIN, 0,           ACT_NONE, CFG_OFF(auto_haptics_gain),     0, 0, 200, 10},
    {"AH LP",    nullptr, nullptr, kAHLp,      SK_NUM,  SF_PLAIN, SIF_WRAP,    ACT_NONE, CFG_OFF(auto_haptics_lowpass),  0, 0, 3, 1},
    {"ScrDim",   "min",   nullptr, nullptr,    SK_NUM,  SF_OFF0,  0,           ACT_NONE, CFG_OFF(screen_dim_timeout),    0, 0, 250, 1},
    {"ScrOff",   "min",   nullptr, nullptr,    SK_NUM,  SF_OFF0,  0,           ACT_NONE, CFG_OFF(screen_off_timeout),    0, 0, 250, 1},
    {"CtrlWake", nullptr, nullptr, nullptr,    SK_BOOL, SF_PLAIN, 0,           ACT_NONE, CFG_OFF(controller_wakes_display), 0, 0, 1, 1},
    {"TrigRdc",  nullptr, nullptr, nullptr,    SK_NUM,  SF_AUTO0, SIF_WRAP,    ACT_NONE, CFG_OFF(trigger_reduce),        0, 0, 10, 1},
    {"SpkGain",  nullptr, nullptr, nullptr,    SK_NUM,  SF_AUTO0, SIF_WRAP,    ACT_NONE, CFG_OFF(speaker_gain),          0, 0, 7, 1},
    {"MicSel",   nullptr, nullptr, kMicSel,    SK_NUM,  SF_PLAIN, SIF_WRAP,    ACT_NONE, CFG_OFF(mic_select),            0, 0, 3, 1},
    {"LockVol",  nullptr, nullptr, nullptr,    SK_BOOL, SF_PLAIN, 0,           ACT_NONE, CFG_OFF(lock_volume),           0, 0, 1, 1},
    {"SpkOut",   nullptr, nullptr, kSpkOut,    SK_NUM,  SF_PLAIN, SIF_WRAP,    ACT_NONE, CFG_OFF(speaker_select),        0, 0, 3, 1},
    {"USB SN",   nullptr, nullptr, nullptr,    SK_BOOL, SF_PLAIN, 0,           ACT_NONE, CFG_OFF(enable_usb_sn),         0, 0, 1, 1},
    {"StatPin",  nullptr, nullptr, nullptr,    SK_NUM,  SF_PLAIN, SIF_PIN,     ACT_NONE, CFG_OFF(status_gpio_pin),       0, 0, 0, 1},
    {"StatMode", nullptr, nullptr, kStatMode,  SK_BOOL, SF_PLAIN, 0,           ACT_NONE, CFG_OFF(status_gpio_mode),      0, 0, 1, 1},
#ifdef ENABLE_WAKE_HID
    {"Wake",     nullptr, nullptr, nullptr,    SK_BOOL, SF_PLAIN, 0,           ACT_NONE, CFG_OFF(enable_wake),           0, 0, 1, 1},
    {"PS Short", nullptr, nullptr, nullptr,    SK_BOOL, SF_PLAIN, 0,           ACT_NONE, CFG_OFF(ps_shortcut_enabled),   0, 0, 1, 1},
#endif
    {"Reset to defaults", nullptr, "Hold Tri 2s = RESET", nullptr, SK_ACTION, SF_PLAIN, 0, ACT_RESET, 0, 0, 0, 0, 1},
    {"Wipe all slots",    nullptr, "Hold Tri 2s = WIPE",  nullptr, SK_ACTION, SF_PLAIN, 0, ACT_WIPE,  0, 0, 0, 0, 1},
};
#undef CFG_OFF
constexpr int kNumSettingsItems = sizeof(kSetItems) / sizeof(kSetItems[0]);
Config_body settings_local{};
int settings_sel = 0;
bool settings_dirty = false;
bool settings_init_done = false;
uint8_t settings_last_dpad = 8;  // 8 = released
uint8_t settings_last_face = 0;
const char* settings_save_status = "";     // shown on the footer after an action
uint32_t settings_status_until_us = 0;     // deadline for that message

// Factory-reset hold-Triangle-2s state. Borrowed from zurce/DS5Dongle-OLED's
// "hold to wipe" UX pattern (https://github.com/zurce/DS5Dongle-OLED).
uint32_t settings_tri_press_us = 0;
bool settings_reset_triggered = false;
constexpr uint32_t kResetHoldUs = 2000000;
constexpr uint32_t kSettingsStatusUs = 2000000; // save/reset feedback hold time

// Show a save/reset result for a couple of seconds. It renders on the footer
// row, which is the only place the long messages ("Slots wiped!", "Reset FAIL")
// fit — next to the "Settings (*)" header the old x=86 draw ran off the panel
// past ~6 characters.
void settings_set_status(const char* s) {
    settings_save_status = s;
    settings_status_until_us = (uint32_t)time_us_32() + kSettingsStatusUs;
}

uint8_t lb_r = 0, lb_g = 0, lb_b = 0;

// Lightbar mode + favorite slots: 0 = LIVE tilt preview; 1..4 = saved slots F0..F3.
// These are seeded from flash (lightbar_load_config) at boot; the defaults here
// only apply before that runs. lb_dirty tracks an unsaved mode/favorite change
// so we persist once on leaving the Lightbar screen instead of per button press.
int lb_mode = kLbModeHost;
uint8_t lb_fav_r[4] = {255, 0,   0,   255}; // Red, Green, Blue, White defaults
uint8_t lb_fav_g[4] = {0,   255, 0,   255};
uint8_t lb_fav_b[4] = {0,   0,   255, 255};
uint8_t lb_last_face = 0;
bool lb_dirty = false;

uint32_t rumble_off_at_us = 0;
bool rumble_active = false;
constexpr uint32_t kRumbleBurstUs = 250000;

int trigger_preset = 0;
const char* const kTrigPresetNames[] = {"Off", "Feedback", "Weapon", "Vibration", "Bow", "Gallop", "Machine"};

// Rising-edge trackers for the screens whose K1=cycle action moved to a
// controller button. Trigger Test uses △ (byte 7 bit 7); Lightbar uses R1
// (byte 8 bit 1) because △ is already taken on Lightbar for "save current
// RGB to favorite slot 0".
uint8_t triggers_last_face = 0;
uint8_t lb_last_buttons = 0;
constexpr int kNumTrigPresets = 7;

// 一次 SPI 事务写一个字节。注释见 flush_fb_raw()：整帧刷屏不再走这里（改批量），
// 只有 init / 对比度 / 开关屏这些零散命令用。
void cmd(uint8_t c) {
    gpio_put(kPinDC, 0);
    gpio_put(kPinCS, 0);
    spi_write_blocking(spi1, &c, 1);
    gpio_put(kPinCS, 1);
}

uint8_t reverse_byte(uint8_t b) {
    b = ((b & 0x55) << 1) | ((b & 0xAA) >> 1);
    b = ((b & 0x33) << 2) | ((b & 0xCC) >> 2);
    b = ((b & 0x0F) << 4) | ((b & 0xF0) >> 4);
    return b;
}

// 位反转查表（面板是 LSB-first），oled_init() 里用 reverse_byte() 填一次 ——
// 每帧 1024 次位运算换成查表。256 B RAM / 520 KB，不心疼。
uint8_t rev_lut[256];

// 面板 GDDRAM 当前内容的镜像。SH1107 自带显存，所以"和上一帧完全一样"的扫描线
// 根本不用发 —— 静态屏（Settings/Slots/CPU/Diag…）的 SPI 开销直接归零，dim
// 呼吸点"灭"的那一秒同样是零。
//
// 不变量：往面板显存里写数据的**唯一**出口是 flush_fb_raw()（cmd() 只发命令）。
// 以后若新增直接写 pattern/显存的功能，必须同步 fb_shadow，否则那一块会永久停在
// 旧画面上 —— 或者干脆在开头把 fb_shadow_valid 置 false 强制整屏重发一次。
uint8_t fb_shadow[kFbBytes];
bool fb_shadow_valid = false;   // false = 还没发过（上电后必须整屏发一次）

// fb 的"内容代次"：任何会把 fb 内容搞失效的事件都 +1 —— 切屏（别的屏画满了 fb）、
// 进入 dim/off 档（呼吸点把 fb 清了）、上电。做局部刷新的渲染器缓存自己上次画完时
// 的代次，发现变了就整屏重画。少了这道保险，切走再切回来时它会以为"这块还是我画的
// 那副样子"而跳过绘制，屏幕上留着上一屏的内容。
uint32_t g_fb_generation = 1;

// 输入签名（FNV-1a）：给"某个区域依赖的输入"算个 32 位指纹，变了才重画那块。
// 各处依赖的输入个数不同，混进来的值也大小不一（电量字节、ETA 分钟数…）。
inline uint32_t sig_mix(uint32_t h, uint32_t v) { return (h ^ v) * 16777619u; }

void hw_reset() {
    gpio_put(kPinRST, 1); sleep_ms(100);
    gpio_put(kPinRST, 0); sleep_ms(100);
    gpio_put(kPinRST, 1); sleep_ms(100);
}

void sh1107_set_contrast(uint8_t value) {
    if (value == current_contrast) return;
    current_contrast = value;
    cmd(0x81); cmd(value);
}

void sh1107_init() {
    cmd(0xAE);
    cmd(0x00); cmd(0x10);
    cmd(0xB0);
    cmd(0xDC); cmd(0x00);
    cmd(0x81); cmd(0x6F);
    cmd(0x21);
    cmd(0xA0);
    cmd(0xC0);
    cmd(0xA4);
    cmd(0xA6);
    cmd(0xA8); cmd(0x3F);
    cmd(0xD3); cmd(0x60);
    cmd(0xD5); cmd(0x41);
    cmd(0xD9); cmd(0x22);
    cmd(0xDB); cmd(0x35);
    cmd(0xAD); cmd(0x8A);
    sleep_ms(50);
    cmd(0xAF);
}

// Forward-declared so flush_fb can paint the per-button arrows on top of
// the rendered framebuffer just before SPI sends it to the OLED. Body
// lives near the other text-drawing helpers below.
void draw_button_chrome();

// 把 fb 推给面板。
//
// 布局：玻璃是 64 列 × 128 行、跑在纵向寻址模式（sh1107_init 里的 0x21），
// 所以「一行 fb」（16 字节 = 128 px）就是「一列玻璃」——16 个数据字节依次落在
// 该列的 page 0..15。每写一列都重发一次 0xB0，把起始 page 显式钉死，这样跳过
// 中间某几列也不会让 page 指针错位。
//
// 开销：老写法每个字节一次 CS 包裹的 SPI 事务（约 1200 次调用、~2 ms 主循环
// 停顿 —— 和喂 USB/BT 的是同一个循环）。现在每列 2 次事务（命令段 + 数据段），
// 且 CS 整帧保持拉低、只切 DC，线上字节流与老写法完全一致。再叠加"没变的列不
// 发"，静态屏每帧开销为 0，动得最凶的屏也在 1 ms 以内。
void flush_fb_raw() {
    bool any_sent = false;
    uint8_t line[kRowBytes];

    for (int j = 0; j < kH; j++) {
        const uint8_t *src = &fb[j * kRowBytes];
        uint8_t *shadow = &fb_shadow[j * kRowBytes];
        if (fb_shadow_valid && memcmp(src, shadow, kRowBytes) == 0) continue;

        if (!any_sent) {
            // 本帧第一个要发的列 —— 这时才占总线
            gpio_put(kPinCS, 0);
            const uint8_t page0 = 0xB0;
            gpio_put(kPinDC, 0);
            spi_write_blocking(spi1, &page0, 1);
            any_sent = true;
        }

        const uint8_t col = kH - 1 - j;
        const uint8_t col_cmd[2] = {(uint8_t)(0x00 + (col & 0x0F)),
                                    (uint8_t)(0x10 + (col >> 4))};
        gpio_put(kPinDC, 0);
        spi_write_blocking(spi1, col_cmd, 2);

        for (int i = 0; i < kRowBytes; i++) line[i] = rev_lut[src[i]];
        gpio_put(kPinDC, 1);
        spi_write_blocking(spi1, line, kRowBytes);

        memcpy(shadow, src, kRowBytes);
    }

    if (any_sent) gpio_put(kPinCS, 1);
    fb_shadow_valid = true;
}

void flush_fb() {
    draw_button_chrome();
    flush_fb_raw();
}

void fb_clear() { memset(fb, 0, sizeof(fb)); }

// 单像素（摇杆点、竖直细线、图标用）。保留独立的窄路径：单点走 span 反而更慢。
void px(int x, int y, bool on) {
    if (x < 0 || x >= kW || y < 0 || y >= kH) return;
    uint8_t *p = &fb[y * kRowBytes + (x >> 3)];
    const uint8_t m = (uint8_t)(1u << (7 - (x & 7)));
    if (on) *p |= m; else *p &= ~m;
}

// ---- 水平段操作 -------------------------------------------------------------
// 屏上一行 16 字节、MSB 在左（x=0 → bit7）。矩形填充/清空/反色、文字行的每一行
// 最终都落到"某一行的连续一段像素"上，这里按整字节写：中间整字节一次 memset，
// 只有首尾两个字节需要按位掩码。老实现逐像素 px()（每像素 4 次边界比较 + 移位），
// 一个 32×32 的框要 1024 次；现在上下两条边各 1 次 span。
enum SpanOp : uint8_t { SPAN_SET, SPAN_CLEAR, SPAN_XOR };

inline void span_apply(SpanOp op, uint8_t *p, uint8_t m) {
    switch (op) {
        case SPAN_SET:   *p |=  m;  break;
        case SPAN_CLEAR: *p &= ~m;  break;
        case SPAN_XOR:   *p ^=  m;  break;
    }
}

inline void span(int x, int y, int w, SpanOp op) {
    if (w <= 0 || y < 0 || y >= kH) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > kW) w = kW - x;
    if (w <= 0) return;

    uint8_t *p = &fb[y * kRowBytes + (x >> 3)];
    const int b1 = (x + w - 1) >> 3;                        // 最后一个涉及的字节
    const int full = b1 - (x >> 3);                         // 0 = 首末落在同一字节
    const uint8_t m_head = (uint8_t)(0xFF >> (x & 7));
    const uint8_t m_tail = (uint8_t)(0xFF << (7 - ((x + w - 1) & 7)));

    if (full == 0) { span_apply(op, p, (uint8_t)(m_head & m_tail)); return; }
    span_apply(op, p, m_head);
    if (full > 1) {
        uint8_t *mid = p + 1;
        const int n = full - 1;
        switch (op) {
            case SPAN_SET:   memset(mid, 0xFF, (size_t)n); break;
            case SPAN_CLEAR: memset(mid, 0x00, (size_t)n); break;
            case SPAN_XOR:   for (int i = 0; i < n; i++) mid[i] = (uint8_t)~mid[i]; break;
        }
    }
    span_apply(op, p + full, m_tail);
}

void rect_outline(int x, int y, int w, int h) {
    if (w <= 0 || h <= 0) return;
    span(x, y, w, SPAN_SET);                        // 上边
    if (h > 1) span(x, y + h - 1, w, SPAN_SET);     // 下边
    for (int j = 1; j < h - 1; j++) {               // 左右两条竖边：单像素，走 px
        px(x, y + j, true);
        px(x + w - 1, y + j, true);
    }
}

void rect_filled(int x, int y, int w, int h) {
    for (int j = 0; j < h; j++) span(x, y + j, w, SPAN_SET);
}

// 清一块矩形 —— 局部刷新重画某区域前，先把它自己的地盘擦干净。
void rect_clear(int x, int y, int w, int h) {
    for (int j = 0; j < h; j++) span(x, y + j, w, SPAN_CLEAR);
}

// XOR-invert every pixel in a region (used to flash a control "pressed").
void rect_invert(int x, int y, int w, int h) {
    for (int j = 0; j < h; j++) span(x, y + j, w, SPAN_XOR);
}

// 字形转置表：kFont5x7 是列存（一字节 = 一列的 7 个像素），逐像素画很浪费；
// 转置成"每行 5 bit"后，一个字形 = 7 次查表 + 每行 1~2 次字节 OR。
// bit4 = 最左像素。开机在 oled_init() 建一次（95×7 = 665 B RAM）。
uint8_t font_rows[95][kFontH];

void draw_char(int x, int y, char c) {
    if (c < 0x20 || c > 0x7E) return;

    // 快路径：整个字形都在屏内（本文件所有 draw_text 都满足），不用逐像素裁剪
    if (x >= 0 && y >= 0 && x + kFontW <= kW && y + kFontH <= kH) {
        const uint8_t *rows = font_rows[c - 0x20];
        uint8_t *p = &fb[y * kRowBytes + (x >> 3)];
        const int sh = x & 7;
        for (int row = 0; row < kFontH; row++) {
            const uint8_t bits = rows[row];
            if (!bits) continue;
            // 5 个像素最多跨 2 个字节：分别攒出两边的掩码，一次 OR 进去
            uint8_t m0 = 0, m1 = 0;
            for (int i = 0; i < kFontW; i++) {
                if (!(bits & (1u << (kFontW - 1 - i)))) continue;
                const int b = sh + i;
                if (b < 8) m0 |= (uint8_t)(1u << (7 - b));
                else       m1 |= (uint8_t)(1u << (7 - (b - 8)));
            }
            uint8_t *r = p + row * kRowBytes;
            r[0] |= m0;
            if (m1) r[1] |= m1;      // x+5 ≤ kW 保证 b0+1 ≤ 15，不会越出本行
        }
        return;
    }

    // 慢路径（理论上到不了）：列存字库 + 逐像素裁剪
    const uint8_t *g = kFont5x7[c - 0x20];
    for (int col = 0; col < kFontW; col++) {
        const uint8_t bits = g[col];
        for (int row = 0; row < kFontH; row++) {
            if (bits & (1 << row)) px(x + col, y + row, true);
        }
    }
}

void draw_text(int x, int y, const char *s) {
    while (*s) {
        draw_char(x, y, *s++);
        x += 6;
    }
}

// Button-chrome strip on the left edge of every screen. KEY0 (top button)
// shows '>' at y=8; KEY1 (bottom button) shows '<' at y=49. Painted by
// flush_fb() on top of the rendered framebuffer so it never gets clobbered.
// Per-screen renderers reserve x ∈ [0..5] (5-wide glyph + 1 padding) and
// start main content at kContentX.
constexpr int kContentX = 6;
void draw_button_chrome() {
    draw_char(0, 8,  '>');
    draw_char(0, 49, '<');
}

// Pixel-art icon support. Visual approach inspired by zurce/DS5Dongle-OLED
// (https://github.com/zurce/DS5Dongle-OLED) — credit to zurce for the idea
// of decorating the OLED with small bitmaps instead of bare text/shapes.
// Bitmap layout: row-major, MSB = leftmost pixel, ceil(w/8) bytes per row.
void draw_icon(int x, int y, const uint8_t *bitmap, int w, int h) {
    const int row_bytes = (w + 7) / 8;
    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            const uint8_t byte = bitmap[row * row_bytes + (col / 8)];
            const uint8_t mask = (uint8_t)(1u << (7 - (col % 8)));
            if (byte & mask) px(x + col, y + row, true);
        }
    }
}

// 8x8 "link active" filled circle (drawn when DS5 is paired)
static const uint8_t kIconLinkOn[8] = {
    0b00111100,
    0b01111110,
    0b11111111,
    0b11111111,
    0b11111111,
    0b11111111,
    0b01111110,
    0b00111100,
};
// 8x8 "link inactive" hollow circle (drawn when waiting for DS5)
static const uint8_t kIconLinkOff[8] = {
    0b00111100,
    0b01000010,
    0b10000001,
    0b10000001,
    0b10000001,
    0b10000001,
    0b01000010,
    0b00111100,
};

// Battery icon — body 52x8 + small nub on the right. Inside fill scales with pct.
void draw_battery_icon(int x, int y, int pct) {
    rect_outline(x, y, 52, 8);
    rect_filled(x + 52, y + 2, 3, 4);
    int fill = (pct * 48) / 100;
    if (fill < 0) fill = 0;
    if (fill > 48) fill = 48;
    if (fill > 0) rect_filled(x + 2, y + 2, fill, 4);
}

void send_rumble(uint8_t amplitude) {
    uint8_t pkt[78] = {};
    pkt[0] = 0x31;
    pkt[1] = 0x00;
    pkt[2] = 0x10;
    pkt[3] = 0x03;
    pkt[5] = amplitude;
    pkt[6] = amplitude;
    bt_write(pkt, sizeof(pkt));
}

void rumble_burst_tick(uint32_t now) {
    if (rumble_active && (int32_t)(now - rumble_off_at_us) >= 0) {
        send_rumble(0);
        rumble_active = false;
    }
}

// Trigger effect param format follows dualsensectl's reverse-engineering.
// Modes 0x21/0x25/0x26 use bitpacked 10-zone arrays, not raw position bytes.
void send_trigger_effect(int preset) {
    uint8_t pkt[78] = {};
    pkt[0] = 0x31;
    pkt[2] = 0x10;
    pkt[3] = 0x0C; // valid_flag0: RIGHT_TRIGGER_MOTOR_ENABLE | LEFT_TRIGGER_MOTOR_ENABLE

    uint8_t mode = 0x05; // OFF
    uint8_t p[9] = {0};

    switch (preset) {
        case 0: // Off
            mode = 0x05;
            break;
        case 1: { // Feedback — all 10 zones at max strength 8
            mode = 0x21;
            const uint16_t active = 0x03FF;
            uint32_t strength = 0;
            for (int i = 0; i < 10; i++) strength |= (uint32_t)(7u << (3 * i));
            p[0] = active & 0xFF;
            p[1] = (active >> 8) & 0xFF;
            p[2] = strength & 0xFF;
            p[3] = (strength >> 8) & 0xFF;
            p[4] = (strength >> 16) & 0xFF;
            p[5] = (strength >> 24) & 0xFF;
            break;
        }
        case 2: { // Weapon — snap between positions 3 and 5, force 8
            mode = 0x25;
            const uint16_t start_stop = (1u << 3) | (1u << 5);
            p[0] = start_stop & 0xFF;
            p[1] = (start_stop >> 8) & 0xFF;
            p[2] = 7; // force = strength - 1
            break;
        }
        case 3: { // Vibration — all 10 zones at amplitude 8, frequency 30 Hz
            mode = 0x26;
            const uint16_t active = 0x03FF;
            uint32_t strength = 0;
            for (int i = 0; i < 10; i++) strength |= (uint32_t)(7u << (3 * i));
            p[0] = active & 0xFF;
            p[1] = (active >> 8) & 0xFF;
            p[2] = strength & 0xFF;
            p[3] = (strength >> 8) & 0xFF;
            p[4] = (strength >> 16) & 0xFF;
            p[5] = (strength >> 24) & 0xFF;
            p[8] = 30;
            break;
        }
        case 4: { // Bow — drawing resistance + snap at position 6
            mode = 0x22;
            const uint16_t start_stop = (1u << 2) | (1u << 6);
            const uint8_t force_pair = 7u | (7u << 3); // strength=8, snap=8
            p[0] = start_stop & 0xFF;
            p[1] = (start_stop >> 8) & 0xFF;
            p[2] = force_pair;
            break;
        }
        case 5: { // Galloping
            mode = 0x23;
            const uint16_t start_stop = (1u << 0) | (1u << 9);
            const uint8_t ratio = (5u & 0x07) | ((1u & 0x07) << 3);
            p[0] = start_stop & 0xFF;
            p[1] = (start_stop >> 8) & 0xFF;
            p[2] = ratio;
            p[3] = 5; // frequency
            break;
        }
        case 6: { // Machine gun
            mode = 0x27;
            const uint16_t start_stop = (1u << 1) | (1u << 8);
            const uint8_t force_pair = 7u | (7u << 3);
            p[0] = start_stop & 0xFF;
            p[1] = (start_stop >> 8) & 0xFF;
            p[2] = force_pair;
            p[3] = 20; // frequency
            p[4] = 0;  // period
            break;
        }
    }

    pkt[13] = mode;
    for (int i = 0; i < 9; i++) pkt[14 + i] = p[i];
    pkt[24] = mode;
    for (int i = 0; i < 9; i++) pkt[25 + i] = p[i];

    bt_write(pkt, sizeof(pkt));
}

void send_lightbar_color(uint8_t r, uint8_t g, uint8_t b);

void handle_buttons() {
    const uint32_t now = fast_now_us();   // 每轮调用：省掉 time_us_32 的函数调用（见 fast_time.h）
    const bool k0 = gpio_get(kPinKey0);
    const bool k1 = gpio_get(kPinKey1);

    // KEY0 + KEY1 chord — both held >= kChordHoldUs triggers watchdog_reboot.
    // Pre-empts the per-key handlers so a chord cancels any armed single
    // press (whichever key gets released first won't also navigate).
    const bool chord = !k0 && !k1;
    if (chord) {
        if (chord_held_since_us == 0) chord_held_since_us = now;
        key0_armed = false;
        key1_was_pressed = false;
        if ((now - chord_held_since_us) >= kChordHoldUs) {
            watchdog_reboot(0, 0, 0);
        }
    } else {
        chord_held_since_us = 0;
    }

    // KEY0: arm on debounced rising edge, fire "next screen" on release.
    // Releasing without a chord during the hold = pure forward-nav.
    if (!k0 && key0_prev && (now - key0_t_us) > kDebounceUs) {
        key0_t_us = now;
        key0_armed = true;
        last_activity_us = time_us_64();
    }
    if (k0 && !key0_prev && key0_armed) {
        key0_armed = false;
        current_screen = (current_screen + 1) % kNumScreens;
        last_render_us = 0;
        last_activity_us = time_us_64();
    }

    // KEY1: arm on press, fire on release. Short press = back; long press
    // = brightness cycle (unchanged). Trigger-preset / lightbar-mode cycle
    // moved to the DualSense △ button — see triggers_handle_input() and
    // lightbar_handle_input(). The chord above clears key1_was_pressed so
    // a released-after-chord K1 doesn't navigate back.
    if (!k1 && key1_prev && (now - key1_t_us) > kDebounceUs) {
        key1_t_us = now;
        key1_press_us = now;
        key1_was_pressed = true;
        last_activity_us = time_us_64();
    }
    if (k1 && !key1_prev && key1_was_pressed) {
        key1_was_pressed = false;
        const uint32_t held = now - key1_press_us;
        last_activity_us = time_us_64();
        if (held > kLongPressUs) {
            bright_idx = (bright_idx + 1) % kNumBrightLevels;
            // Persist so the choice survives a power cycle (issue #9). Keep
            // settings_local in sync too, so a later Settings-screen save can't
            // clobber screen_brightness with its stale snapshot.
            Config_body b = get_config();
            b.screen_brightness = (uint8_t)bright_idx;
            set_config(b);
            config_save();
            settings_local.screen_brightness = (uint8_t)bright_idx;
        } else {
            current_screen = (current_screen - 1 + kNumScreens) % kNumScreens;
            last_render_us = 0;
        }
    }

    key0_prev = k0;
    key1_prev = k1;
}

// --- Charge ETA tracker --------------------------------------------------
// The DS5 only reports battery in 10% steps (interrupt_in_data[52] low
// nibble, 0..10; high nibble is power-state, 1 == charging). We can't read a
// finer percentage over BT, so a smooth countdown is impossible. Instead we
// time how long each 10% step takes while charging and extrapolate the
// remaining steps. Sampled once per frame from oled_loop (continuously, so
// the estimate stays current even while the panel is dimmed/off and even when
// the user is on another screen); render_screen reads g_charge_eta.
//
// Taper correction: Li-ion CC/CV charging slows sharply near the top, so a
// flat "time per step × steps left" runs optimistic in the last ~20%. Each
// measured step is normalised to a bulk-equivalent duration (divide out the
// step's taper weight); the remaining steps are then re-weighted. This makes
// the estimate consistent whether the user plugged in near-empty or near-full.
struct ChargeEta {
    bool charging;    // pstate == 1 (so the token shows only while charging)
    bool valid;       // minutes is meaningful (provisional or measured)
    bool provisional; // true until a full step is timed — using the default rate
    int  minutes;     // estimated minutes to 100%
};
ChargeEta g_charge_eta{};

// Default bulk-step duration used for a provisional estimate before any real
// step has been timed, so the token shows "~Nm?" immediately on plug-in instead
// of sitting on "~--m" for ~15-20 min. Tuned to an observed ~15 min per 10% on
// this dongle's charge current; it self-corrects to the measured rate (and drops
// the "?") as soon as the first clean step completes.
constexpr float kDefaultStepUs = 15.0f * 60.0f * 1000000.0f;

// Ceiling on a single timed step's bulk-equivalent duration. A genuine idle 10%
// step on this dongle is ~15 min; anything past ~30 min is almost always an
// anomalous/under-load sample (e.g. the controller in use while charging, or a
// battery-nibble bounce) that would otherwise balloon the projection — observed
// reading ~222m at 70% off one ~47-min step. We clamp such samples instead of
// trusting them, and pair that with a median over kRing steps so one bad reading
// can't dominate the estimate.
constexpr float kMaxStepUs = 30.0f * 60.0f * 1000000.0f;

// Relative time the step *ending* at `to_level` (10% units, 1..10) takes vs a
// bulk step. Tuned to the Li-ion CV taper: ~80% onward stretches out.
static float charge_step_weight(int to_level) {
    if (to_level >= 10) return 2.2f;  // 90→100% (constant-voltage tail)
    if (to_level == 9)  return 1.5f;  // 80→90%  (taper begins)
    return 1.0f;                      // bulk constant-current region
}

void sample_charge_eta() {
    constexpr int kRing = 5;                 // median over the last few steps
    static float    ring[kRing] = {0};       // bulk-equivalent step durations (us)
    static int      ring_count = 0;
    static int      ring_head = 0;
    static int      cur_step = -1;           // last observed 10% step
    static uint64_t step_start_us = 0;
    static bool     was_charging = false;
    static bool     first_step_pending = false;  // discard the partial step at plug-in

    const uint8_t pwr   = interrupt_in_data[52];
    int           step  = pwr & 0x0F;
    if (step > 10) step = 10;
    const uint8_t pstate = pwr >> 4;
    const bool charging = bt_is_connected() && (pstate == 1);

    if (!charging) {
        g_charge_eta = ChargeEta{};          // clears charging/valid/minutes
        ring_count = ring_head = 0;
        cur_step = -1;
        was_charging = false;
        return;
    }

    const uint64_t now = time_us_64();
    if (!was_charging) {
        // Just plugged in: start timing from here. The step in progress is
        // partial, so its duration gets discarded when it completes.
        cur_step = step;
        step_start_us = now;
        ring_count = ring_head = 0;
        first_step_pending = true;
        was_charging = true;
    } else if (step == cur_step + 1) {
        // One clean step completed. Skip the first (partial) one; otherwise
        // record its bulk-equivalent duration.
        const float dur = (float)(now - step_start_us);
        if (first_step_pending) {
            first_step_pending = false;
        } else {
            float be = dur / charge_step_weight(step);
            if (be > kMaxStepUs) be = kMaxStepUs;   // clamp under-load/anomalous outliers
            ring[ring_head] = be;
            ring_head = (ring_head + 1) % kRing;
            if (ring_count < kRing) ring_count++;
        }
        cur_step = step;
        step_start_us = now;
    } else if (step != cur_step) {
        // Multi-step jump (e.g. woke from sleep across several steps) or a
        // small dip under heavy use — can't attribute timing cleanly, so just
        // resync without polluting the ring.
        cur_step = step;
        step_start_us = now;
        first_step_pending = false;
    }

    g_charge_eta.charging = true;
    if (cur_step < 10) {
        // Use the measured rate once we have a timed step; until then fall back
        // to the default rate and flag the estimate provisional (renders "?").
        const bool measured = (ring_count > 0);
        float bulk;
        if (measured) {
            // Median of the timed steps — robust to a single slow/fast outlier
            // in a way the old mean wasn't (one 47-min under-load step used to
            // drag the whole projection up). kRing is tiny, so insertion-sort.
            float tmp[kRing];
            for (int i = 0; i < ring_count; i++) tmp[i] = ring[i];
            for (int i = 1; i < ring_count; i++) {
                const float v = tmp[i];
                int j = i - 1;
                while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
                tmp[j + 1] = v;
            }
            bulk = tmp[ring_count / 2];
        } else {
            bulk = kDefaultStepUs;
        }
        float rem_us = 0.0f;
        for (int L = cur_step + 1; L <= 10; L++) rem_us += bulk * charge_step_weight(L);
        int mins = (int)(rem_us / 60000000.0f + 0.5f);
        if (mins < 0)   mins = 0;
        if (mins > 999) mins = 999;
        g_charge_eta.valid = true;
        g_charge_eta.provisional = !measured;
        g_charge_eta.minutes = mins;
    } else {
        // cur_step == 10 → essentially full; nothing meaningful to count down.
        g_charge_eta.valid = true;
        g_charge_eta.provisional = false;
        g_charge_eta.minutes = 0;
    }
}

// ---- 状态屏的局部刷新 -------------------------------------------------------
// 这块屏是唯一"一直在动"的屏（摇杆点、扳机条、按键方块在做游戏时一直在变），
// 所以切成 5 块，各自只在自己依赖的输入变化时才"清 + 画"：
//   0 顶栏    y=0..7                    标题 + 链接图标
//   1 信息行  x≥6, y=9..25              MAC / 电量 / ETA（含 snprintf，最贵的一块）
//   2 摇杆    两个 32×32 框             最细粒度：点没动就一个像素都不碰
//   3 扳机条  x=38..41 / 92..95, y≥33
//   4 按键    x=42..91, y=30..56        十字键 + 面键 + L1/R1
// 签名没变 → 整块跳过，面板继续显示上一帧（SH1107 自带显存）。
// fb 代次变了（切屏 / 从 dim 回来 / 上电）→ fb_clear() + 全部重画。
__attribute__((noinline)) void render_screen() {
    const bool connected = bt_is_connected();

    static uint32_t seen_gen = 0;
    static uint32_t sig_last[4] = {0, 0, 0, 0};   // 顶栏 / 信息行 / 扳机条 / 按键
    static bool     prev_connected = false;
    struct Stick { int8_t x, y; bool inv, valid; };
    static Stick stick[2] = {{-1, -1, false, false}, {-1, -1, false, false}};

    // 连接/断连是两套完全不同的布局 → 也算整屏重画
    const bool layout_changed = (connected != prev_connected);
    prev_connected = connected;
    const bool force = layout_changed || (seen_gen != g_fb_generation);
    seen_gen = g_fb_generation;
    if (force) {
        fb_clear();
        stick[0].valid = stick[1].valid = false;
    }

    // 各块依赖的输入签名
    uint8_t addr[6] = {0};
    if (connected) bt_get_addr(addr);
    const uint8_t pwr = interrupt_in_data[52];
    const ChargeEta ce = g_charge_eta;

    const uint32_t h_top = sig_mix(2166136261u, connected);  // 标题是编译期常量，只有链接图标跟连接状态走
    uint32_t h_info = 2166136261u;
    for (int i = 0; i < 6; i++) h_info = sig_mix(h_info, addr[i]);
    h_info = sig_mix(h_info, pwr);
    h_info = sig_mix(h_info, (uint32_t)(ce.charging | (ce.valid << 1) | (ce.provisional << 2)));
    h_info = sig_mix(h_info, (uint32_t)ce.minutes);
    const uint32_t h_bar = sig_mix(sig_mix(2166136261u, interrupt_in_data[4]), interrupt_in_data[5]);
    const uint32_t h_btn = sig_mix(sig_mix(2166136261u, interrupt_in_data[7]),
                                   interrupt_in_data[8] & 0x03);

    // ---- 区域 0：顶栏 ----
    if (force || h_top != sig_last[0]) {
        sig_last[0] = h_top;
        rect_clear(0, 0, kW, 8);
        // FIRMWARE_VERSION is set via CMake from -DVERSION=... on the build
        // command line (release.yml passes the tag name). Local builds get
        // "dev" so a non-tagged build is visible at a glance.
        // 19-char cap keeps the title clear of the 8x8 link icon at x=120 even if a
        // future tag (FIRMWARE_VERSION) is long.
        char title[20];
        snprintf(title, sizeof(title), "DS5 Bridge %s", FIRMWARE_VERSION);
        draw_text(kContentX, 0, title);
        draw_icon(120, 0, connected ? kIconLinkOn : kIconLinkOff, 8, 8);
    }

    if (!connected) {
        // 未连接的配对提示是纯静态的：整屏重画时画一次就够
        if (force) {
            draw_text(kContentX, 14, "Pair your DualSense:");
            draw_text(kContentX, 26, "1. Hold Create + PS");
            draw_text(kContentX, 36, "2. Wait for light bar");
            draw_text(kContentX, 46, "   to flash blue");
        }
        flush_fb();
        return;
    }

    // ---- 区域 1：信息行（MAC + 电量 + ETA）----
    if (force || h_info != sig_last[1]) {
        sig_last[1] = h_info;
        rect_clear(kContentX, 9, kW - kContentX, 17);   // y=9..25 整条
        char buf[24];
        snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        draw_text(kContentX, 9, buf);

        int pct = (pwr & 0x0F) * 10;
        if (pct > 100) pct = 100;
        const uint8_t pstate = pwr >> 4;
        char marker = ' ';
        if (pstate == 1) marker = '+';      // Charging
        else if (pstate == 2) marker = '*'; // Complete
        else if (pstate >= 0xA) marker = '!'; // Error
        char bbuf[16];
        snprintf(bbuf, sizeof(bbuf), "%3d%%%c", pct, marker);
        draw_text(kContentX, 18, bbuf);
        draw_battery_icon(36, 18, pct);

        // Charge ETA, right of the battery icon (icon + nub end at x=90). Shown
        // only while charging: "~43m?" is the provisional default-rate estimate
        // shown immediately on plug-in; the "?" drops to "~43m" once a real 10%
        // step has been timed and the measured rate takes over. See
        // sample_charge_eta(). x=92 (not 94) so a 3-digit "~145m?" stays on panel.
        if (ce.charging) {
            char ebuf[8];
            if (ce.valid)
                snprintf(ebuf, sizeof(ebuf), "~%dm%s", ce.minutes,
                         ce.provisional ? "?" : "");
            else
                snprintf(ebuf, sizeof(ebuf), "~--m");
            draw_text(92, 18, ebuf);
        }
    }

    // ---- 区域 2：两个摇杆（最细粒度 —— 只动点）----
    {
        const uint8_t b8 = interrupt_in_data[8];
        for (int i = 0; i < 2; i++) {
            // Left-half visuals are shifted right by kContentX so the < button
            // chrome at (x=0, y=49) doesn't paint over the live stick dot.
            const int  bx  = i ? 96 : kContentX;
            const int  lx  = (bx + 2) + (interrupt_in_data[i * 2] * 27) / 255;
            const int  ly  = 32 + (interrupt_in_data[i * 2 + 1] * 27) / 255;
            // L3 / R3 (stick click) — invert the whole box as a pressed indicator.
            const bool inv = i ? (b8 & 0x80) : (b8 & 0x40);
            Stick &st = stick[i];

            // 反色（L3/R3 按住）时背景不是纯黑，"擦旧点画新点"的捷径会留错像素
            // —— 整框反色期间一律整框重画。点先画、后反色，和原实现同序，
            // 所以按下时点也会跟着反色。
            if (force || !st.valid || st.inv != inv || inv) {
                rect_clear(bx, 30, 32, 32);
                rect_outline(bx, 30, 32, 32);
                rect_filled(lx - 1, ly - 1, 3, 3);
                if (inv) rect_invert(bx, 30, 32, 32);
                st.inv = inv;
                st.valid = true;
            } else if (st.x != lx || st.y != ly) {
                // 点永远夹在框内（lx-1 ≥ bx+1、lx+1 ≤ bx+30），擦旧点碰不到边框
                rect_clear(st.x - 1, st.y - 1, 3, 3);
                rect_filled(lx - 1, ly - 1, 3, 3);
            } else {
                continue;   // 点没动：这块一个像素都不碰
            }
            st.x = (int8_t)lx;
            st.y = (int8_t)ly;
        }
    }

    // ---- 区域 3：L2/R2 模拟量条（垂直，从底部填充）----
    if (force || h_bar != sig_last[2]) {
        sig_last[2] = h_bar;
        rect_clear(kContentX + 32, 33, 4, 29);
        rect_outline(kContentX + 32, 33, 4, 29);
        const int l2_fill = (interrupt_in_data[4] * 27) / 255;
        if (l2_fill > 0) rect_filled(kContentX + 33, 61 - l2_fill, 2, l2_fill);
        rect_clear(92, 33, 4, 29);
        rect_outline(92, 33, 4, 29);
        const int r2_fill = (interrupt_in_data[5] * 27) / 255;
        if (r2_fill > 0) rect_filled(93, 61 - r2_fill, 2, r2_fill);
    }

    // ---- 区域 4：十字键 + 面键 + L1/R1 ----
    if (force || h_btn != sig_last[3]) {
        sig_last[3] = h_btn;
        rect_clear(42, 30, 50, 27);     // x=42..91, y=30..56（把这一整块擦干净再画）
        const uint8_t b7 = interrupt_in_data[7];
        const uint8_t b8 = interrupt_in_data[8];

        // D-pad indicator (4 directions; lit for primary + diagonals).
        // Centered between the left stick column and the face-button cluster.
        const int dp = b7 & 0x0F;
        const bool dp_n = (dp == 7 || dp == 0 || dp == 1);
        const bool dp_e = (dp == 1 || dp == 2 || dp == 3);
        const bool dp_s = (dp == 3 || dp == 4 || dp == 5);
        const bool dp_w = (dp == 5 || dp == 6 || dp == 7);
        const int dcx = 52, dcy = 46;
        auto dot = [&](int dx, int dy, bool on) {
            if (on) rect_filled(dcx + dx - 2, dcy + dy - 2, 5, 5);
            else    rect_outline(dcx + dx - 2, dcy + dy - 2, 5, 5);
        };
        dot(0,  -7, dp_n);
        dot(7,   0, dp_e);
        dot(0,   7, dp_s);
        dot(-7,  0, dp_w);

        const int fcx = 64, fcy = 46;
        auto sq = [&](int dx, int dy, bool on) {
            if (on) rect_filled(fcx + dx - 2, fcy + dy - 2, 5, 5);
            else    rect_outline(fcx + dx - 2, fcy + dy - 2, 5, 5);
        };
        // Shift face buttons right so they don't collide with the d-pad — but
        // only by 16: at 18 the ○ square's right column (x=92) merged with the
        // R2 bar's left outline (also x=92).
        const int fcx_off = 16;
        sq(fcx_off + 0,  -8, b7 & 0x80); // Triangle
        sq(fcx_off + 8,   0, b7 & 0x40); // Circle
        sq(fcx_off + 0,   8, b7 & 0x20); // Cross
        sq(fcx_off - 8,   0, b7 & 0x10); // Square

        // L1 bar shifted to sit between the L2 trigger column and the d-pad.
        if (b8 & 0x01) rect_filled(42, 30, 8, 3);  else rect_outline(42, 30, 8, 3);  // L1
        if (b8 & 0x02) rect_filled(80, 30, 12, 3); else rect_outline(80, 30, 12, 3); // R1
    }

    flush_fb();
}

__attribute__((noinline)) void render_screen_rssi() {
    fb_clear();
    draw_text(kContentX, 0, "BT Signal");
    if (bt_is_connected()) {
        int8_t rssi = 0;
        bt_rssi_request();  // ask for a fresh reading; the getter below stays pure
        bt_get_signal_strength(&rssi);
        char buf[24];
        snprintf(buf, sizeof(buf), "RSSI: %d dBm", (int)rssi);
        draw_text(kContentX, 12, buf);

        // Map RSSI range -90..-40 dBm to 0..100% bar
        int pct = ((int)rssi + 90) * 100 / 50;
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        snprintf(buf, sizeof(buf), "Quality: %d%%", pct);
        draw_text(kContentX, 22, buf);
        rect_outline(kContentX, 34, 122, 10);
        int fill = (pct * 118) / 100;
        if (fill > 0) rect_filled(kContentX + 2, 36, fill, 6);

        const char *label = "Poor";
        if (rssi > -55) label = "Excellent";
        else if (rssi > -65) label = "Good";
        else if (rssi > -75) label = "Fair";
        snprintf(buf, sizeof(buf), "Link: %s", label);
        draw_text(kContentX, 48, buf);
    } else {
        draw_text(kContentX, 30, "(no controller)");
    }
    flush_fb();
}

// Diagnostics screen state. Read-only viewport that scrolls with controller
// D-pad up/down. No cursor — there's nothing to select.
int   diag_scroll = 0;
uint8_t diag_last_dpad = 8; // edge-trigger N/E/S/W like settings_handle_input
// Rows on screen at once — used both for the render loop and for the scroll
// clamp in diag_handle_input(), which screen_input_tick() calls every loop
// iteration (the render path only runs at 10 Hz).
constexpr int kDiagVisible = 5;

// Per-second rates shown on the diag screen, shared across format_diag_row's
// rate-based rows so they stay in sync.
//
// Computed over a fixed 1 s tumbling window (not once per render): a short
// window over a bursty counter swings by whole percent per frame — USB aud at
// 48 kHz looked like it was hunting between 47k and 48k when nothing was wrong,
// and the low-rate rows (BT32 out ~tens/s) swung by tens of percent. Between
// window boundaries the last computed values stay on screen, so the rows read
// as steady per-second rates.
struct DiagRates {
    uint32_t usb_rate;
    uint32_t bt_rate;
    uint32_t mic_rate;
    uint32_t bt31_rate;
};
DiagRates g_diag_rates{};

constexpr uint32_t kRateWindowUs = 1000000u; // 1 s window length
constexpr uint32_t kRateStaleUs  = 2000000u; // no call for >2 s (diag page was left):
                                             // dt no longer means "the last second",
                                             // so re-baseline instead of computing
constexpr uint32_t kRatePrimeUs  =  200000u; // first window after entering the page:
                                             // emit an early short-window value so the
                                             // rows don't sit at 0 for a full second

void sample_diag_rates() {
    static uint32_t base_us_frames = 0, base_bt_packets = 0, base_mic_frames = 0, base_bt31 = 0;
    static uint32_t win_start_us = 0;
    static bool     have_value   = false;
    const uint32_t now_us = time_us_32();

    if (win_start_us == 0) {                 // first call since boot: baseline only
        win_start_us = now_us;
        base_us_frames  = audio_usb_frames();
        base_bt_packets = audio_bt_packets();
        base_mic_frames = audio_mic_frames();
        base_bt31       = bt_31_packet_count();
        return;
    }

    const uint32_t dt_us  = now_us - win_start_us;   // uint32 math is wrap-safe
    const bool     stale  = dt_us > kRateStaleUs;
    if (!stale && dt_us < kRateWindowUs && (have_value || dt_us < kRatePrimeUs)) {
        return;                              // window not finished — keep showing the last values
    }

    const uint32_t cur_us_frames  = audio_usb_frames();
    const uint32_t cur_bt_packets = audio_bt_packets();
    const uint32_t cur_mic_frames = audio_mic_frames();
    const uint32_t cur_bt31       = bt_31_packet_count();

    if (stale) {
        // Keep the last shown values; just restart the window from here.
        have_value = false;
    } else {
        g_diag_rates.usb_rate  = (uint32_t)(((uint64_t)(cur_us_frames  - base_us_frames)  * 1000000u) / dt_us);
        g_diag_rates.bt_rate   = (uint32_t)(((uint64_t)(cur_bt_packets - base_bt_packets) * 1000000u) / dt_us);
        g_diag_rates.mic_rate  = (uint32_t)(((uint64_t)(cur_mic_frames - base_mic_frames) * 1000000u) / dt_us);
        g_diag_rates.bt31_rate = (uint32_t)(((uint64_t)(cur_bt31       - base_bt31)       * 1000000u) / dt_us);
        have_value = true;
    }

    // Baseline and window start move together — they always describe one window.
    base_us_frames  = cur_us_frames;
    base_bt_packets = cur_bt_packets;
    base_mic_frames = cur_mic_frames;
    base_bt31       = cur_bt31;
    win_start_us    = now_us;
}

// Row list ordered by relevance: always-useful at top, parked-mic-investigation
// data at bottom. To add a row, bump kNumDiagRows and add a case.
//   0 Up   1 BT   2/3 Loop0/Loop1   4 host02   5 trig   6-9 速率  10-12 麦克风
constexpr int kNumDiagRows = 13;
__attribute__((noinline))
void format_diag_row(int idx, char* line, size_t n) {
    switch (idx) {
        case 0: {
            const uint32_t s = time_us_32() / 1000000u;
            snprintf(line, n, "Up:%luh %02lum %02lus",
                     (unsigned long)(s / 3600u),
                     (unsigned long)((s / 60u) % 60u),
                     (unsigned long)(s % 60u));
            break;
        }
        case 1:
            snprintf(line, n, "BT: %s", bt_is_connected() ? "connected" : "waiting");
            break;
        case 2:
        case 3: {
            // 主循环抖动（loop_probe.h）：最近一个 1 s 窗口内的最大轮间隔 + 循环频率。
            // Loop0 = core0 转发主循环（USB/BT/OLED/灯条），Loop1 = core1 音频自旋环。
            // "Loop0 41k/s 2100us" = 那 1 s 里最坏的一轮卡了 2.1 ms。
            // 参考值：正常空闲档 max 是个位数~几十 µs；OLED 刷屏那一拍约 1 ms；
            // config_save 的 flash 擦写、CPU 屏进屏时的频率计忙等会有几十 ms 尖峰。
            // core1 有音频时天然是"一轮一次 opus 编/解码"，最大值本来就大。
            const int      core   = idx - 2;
            const uint32_t max_us = g_loop_max_us[core];
            const uint32_t hz     = g_loop_hz[core];
            char rate[10];
            if (hz >= 1000) snprintf(rate, sizeof rate, "%luk/s", (unsigned long)(hz / 1000));
            else            snprintf(rate, sizeof rate, "%lu/s",  (unsigned long)hz);
            if (max_us >= 10000)   // ≥10 ms 用 ms 显示，保证行宽 ≤19 字符
                snprintf(line, n, "Loop%d %s %lums", core, rate, (unsigned long)(max_us / 1000));
            else
                snprintf(line, n, "Loop%d %s %luus", core, rate, (unsigned long)max_us);
            break;
        }
        case 4:
            snprintf(line, n, "host02: %lu", (unsigned long)host_out02_total());
            break;
        case 5:
            snprintf(line, n, "trig %lu / tx %lu",
                     (unsigned long)host_out02_trig_allow(),
                     (unsigned long)host_out02_to_bt());
            break;
        case 6:
            snprintf(line, n, "BT31 in: %lu/s", (unsigned long)g_diag_rates.bt31_rate);
            break;
        case 7:
            snprintf(line, n, "USB aud: %lu/s", (unsigned long)g_diag_rates.usb_rate);
            break;
        case 8:
            snprintf(line, n, "BT32 out: %lu/s", (unsigned long)g_diag_rates.bt_rate);
            break;
        case 9:
            snprintf(line, n, "Mic in: %lu/s", (unsigned long)g_diag_rates.mic_rate);
            break;
        case 10:
            snprintf(line, n, "Mic dec=%ld w=%u",
                     (long)audio_mic_last_decoded(),
                     (unsigned)audio_mic_last_wrote());
            break;
        case 11:
            snprintf(line, n, "Mic fail: %lu", (unsigned long)audio_mic_decode_failures());
            break;
        case 12: {
            // Auto-haptics real usage: peak of the derived waveform (0-127).
            // Reads 0 while Fallback yields to native haptics or audio is silent.
            // Any non-zero output counts as active.
            const uint16_t p = audio_ah_out_peak();
            const uint16_t pct = (uint16_t)((p * 100u) / 127u);
            snprintf(line, n, "AH: %u%% %s", pct, (p > 0) ? "ACT" : "idle");
            break;
        }
        default:
            line[0] = '\0';
            break;
    }
}

void diag_handle_input(int visible) {
    if (!bt_is_connected()) return;
    const uint8_t dpad = (uint8_t)(interrupt_in_data[7] & 0x0F);
    if (dpad != diag_last_dpad && dpad != 8) {
        if      (dpad == 0) diag_scroll--; // up
        else if (dpad == 4) diag_scroll++; // down
    }
    diag_last_dpad = dpad;
    const int max_top = (kNumDiagRows > visible) ? (kNumDiagRows - visible) : 0;
    if (diag_scroll < 0) diag_scroll = 0;
    if (diag_scroll > max_top) diag_scroll = max_top;
}

__attribute__((noinline)) void render_screen_diag() {
    // 局部刷新：每行缓存"上次画出来的字符串 + 那时的 y"，只有内容或位置变了的行
    // 才擦掉重画。这一屏绝大多数帧只有"运行时间"那一行在动（1 秒一次），其余行
    // 的字符串一模一样 —— 整屏重画纯属浪费。
    // 行文本区只占 x=6..118（19 字符），右侧 x=120..127 的滚动标记单独一块，所以
    // 行重画不会碰到标记。
    static uint32_t seen_gen = 0;
    static char last_row[kDiagVisible][20];
    static int  last_y[kDiagVisible] = {0};
    static bool row_valid[kDiagVisible] = {false};
    static uint8_t last_markers = 0xFF;   // bit0 = 上箭头, bit1 = 下箭头（0xFF = 未画）

    const bool force = (seen_gen != g_fb_generation);
    seen_gen = g_fb_generation;
    if (force) {
        fb_clear();
        draw_text(kContentX, 0, "Diagnostics");
        for (int i = 0; i < kDiagVisible; i++) row_valid[i] = false;
        last_markers = 0xFF;
    }

    sample_diag_rates();
    // Input is sampled by screen_input_tick() every loop iteration, not here.

    // 19 characters max (= 113 px, ends at x=118): keeps even the unbounded
    // counters ("trig 4294967295 / tx ...") from running under the scroll
    // markers at x=120. snprintf truncates; rows are informational.
    char line[20];
    for (int i = 0; i < kDiagVisible; i++) {
        const int row = diag_scroll + i;
        if (row < kNumDiagRows) format_diag_row(row, line, sizeof(line));
        else                    line[0] = '\0';
        const int y = 9 + i * 9;
        if (row_valid[i] && last_y[i] == y && strcmp(last_row[i], line) == 0) continue;
        rect_clear(kContentX, y, 113, 9);   // 行带（7 px 字形 + 行距），不碰右侧标记
        if (line[0]) draw_text(kContentX, y, line);
        memcpy(last_row[i], line, sizeof(line));
        last_y[i] = y;
        row_valid[i] = true;
    }

    // Scroll indicators along the right edge, adjacent to the visible content
    // rather than in a footer — keeps the bottom row available for content.
    // 只在"该显示哪几个箭头"变化时重画（不是只看滚动位置）。
    const uint8_t markers = (uint8_t)((diag_scroll > 0 ? 1 : 0) |
                                      (diag_scroll + kDiagVisible < kNumDiagRows ? 2 : 0));
    if (force || markers != last_markers) {
        rect_clear(120, 9, 8, 45);
        if (markers & 1) draw_text(120, 9,  "^");
        if (markers & 2) draw_text(120, 45, "v");
        last_markers = markers;
    }

    flush_fb();
}

__attribute__((noinline)) void render_screen_cpu(bool entered) {
    fb_clear();
    draw_text(kContentX, 0, "CPU / Clock");

    char buf[24];

    // Configured system clock — compile-time SYS_CLOCK_KHZ, set in main()
    // via set_sys_clock_khz(). This is the *target*.
    const uint32_t set_khz = (uint32_t)SYS_CLOCK_KHZ;
    snprintf(buf, sizeof(buf), "Set : %lu MHz", (unsigned long)(set_khz / 1000u));
    draw_text(kContentX, 12, buf);

    // Actually running clk_sys, measured by the on-chip frequency counter
    // against the crystal reference (not just what we asked for). The counter
    // busy-waits a few ms per call, so measure ONCE on screen entry and cache
    // it — clk_sys is fixed at boot and never changes, so the temperature
    // (which legitimately drifts) is the only thing worth refreshing per
    // frame. cached_real_khz==0 also forces a (re)measure as a safety net.
    static uint32_t cached_real_khz = 0;
    if (entered || cached_real_khz == 0) {
        cached_real_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    }
    const uint32_t real_khz = cached_real_khz;
    snprintf(buf, sizeof(buf), "Real: %lu.%01lu MHz",
             (unsigned long)(real_khz / 1000u),
             (unsigned long)((real_khz % 1000u) / 100u));
    draw_text(kContentX, 22, buf);

    // Core voltage actually programmed into the regulator, read back (not the
    // compile-time constant). Codes 0..15 are linear 0.05 V steps from 0.55 V.
    const int vcode = (int)vreg_get_voltage();
    if (vcode >= 0 && vcode <= 0b01111) {
        const unsigned mv = 550u + 50u * (unsigned)vcode;
        snprintf(buf, sizeof(buf), "Vcore: %u.%02u V", mv / 1000u, (mv % 1000u) / 10u);
    } else {
        snprintf(buf, sizeof(buf), "Vcore: code %d", vcode);
    }
    draw_text(kContentX, 32, buf);

    // RP2350 on-die temperature sensor. Smoothed + averaged in cmd.cpp
    // (single source of truth shared with the 0xfc web telemetry) so the
    // reading converges to the true die temp instead of chasing ADC noise.
    const uint16_t raw = cpu_temp_raw_smoothed();
    const float volts = (float)raw * 3.3f / 4096.0f;
    const float temp_c = 27.0f - (volts - 0.706f) / 0.001721f;
    const int t10 = (int)(temp_c * 10.0f + (temp_c >= 0 ? 0.5f : -0.5f));
    snprintf(buf, sizeof(buf), "Temp : %d.%d C", t10 / 10,
             (t10 < 0 ? -t10 : t10) % 10);
    draw_text(kContentX, 42, buf);

    flush_fb();
}

// △ rising edge on the Trigger Test screen cycles trigger_preset and
// re-applies the new effect to the paired controller. KEY1 used to do
// this; moving it to the controller frees K0/K1 for navigation only.
void triggers_handle_input() {
    if (!bt_is_connected()) { triggers_last_face = 0; return; }
    const uint8_t face = interrupt_in_data[7] & 0xF0;
    const bool tri_now  = (face & 0x80) != 0;
    const bool tri_prev = (triggers_last_face & 0x80) != 0;
    if (tri_now && !tri_prev) {
        trigger_preset = (trigger_preset + 1) % kNumTrigPresets;
        send_trigger_effect(trigger_preset);
    }
    triggers_last_face = face;
}

__attribute__((noinline)) void render_screen_triggers() {
    // △ cycling is sampled by screen_input_tick() every loop iteration, not here.
    fb_clear();
    draw_text(kContentX, 0, "Trigger Test");

    char buf[24];
    snprintf(buf, sizeof(buf), "Mode: %s", kTrigPresetNames[trigger_preset]);
    draw_text(kContentX, 12, buf);

    if (bt_is_connected()) {
        const uint8_t l2 = interrupt_in_data[4];
        const uint8_t r2 = interrupt_in_data[5];
        snprintf(buf, sizeof(buf), "L2:%3d  R2:%3d", l2, r2);
        draw_text(kContentX, 24, buf);

        rect_outline(kContentX, 35, 56, 9);
        int lfill = (l2 * 52) / 255;
        if (lfill > 0) rect_filled(kContentX + 2, 37, lfill, 5);
        rect_outline(72, 35, 56, 9);
        int rfill = (r2 * 52) / 255;
        if (rfill > 0) rect_filled(74, 37, rfill, 5);
    } else {
        draw_text(kContentX, 24, "(no controller)");
    }

    draw_text(kContentX, 56, "Tri=cycle");
    flush_fb();
}

// --- IMU calibration (DS5 feature report 0x05) ---------------------------
// The DualSense ships per-unit gyro/accel calibration in feature report 0x05,
// which bt.cpp already fetches and caches at connect (init_feature). Parsing it
// lets the Gyro Tilt screen and the tilt->RGB lightbar mode use bias- and
// sensitivity-corrected accel instead of raw counts, so the tilt dot recenters
// per controller. Parse + apply mirror SDL's SDL_hidapi_ps5.c (zlib-licensed)
// LoadCalibrationData/ApplyCalibrationData (credit); feature_data[0x05]'s byte
// layout matches SDL's data[] —— 注意合并后 feature_data 不含报告 ID（上游约定），
// 所以 SDL 的 data[] 里"从 1 开始的校准字"在这里就是从 0 开始。
//
// imu_apply keeps accel in the same +-8192 == 1g count space the callers already
// scale by, so existing /8192 (gyro screen) and +-8192 (lightbar) math is
// unchanged — calibration only removes the per-axis zero offset and corrects gain.
struct ImuCal { int16_t bias; float sens; };  // 0..2 gyro P/Y/R, 3..5 accel X/Y/Z
ImuCal g_imu_cal[6];
bool   g_imu_cal_valid = false;   // a plausible calibration was loaded
bool   g_imu_cal_tried = false;   // 0x05 has been seen this connection (good or bad)

constexpr float kGyroResPerDeg = 1024.0f;
constexpr float kAccelResPerG  = 8192.0f;

inline int16_t cal_ld16(const std::vector<uint8_t>& d, int i) {
    return (int16_t)((uint16_t)d[i] | ((uint16_t)d[i + 1] << 8));
}

__attribute__((noinline))
void imu_cal_parse(const std::vector<uint8_t>& d) {
    g_imu_cal_valid = false;
    if (d.size() < 35) return;                 // SDL requires >= 35 calibration bytes

    const int16_t gPB = cal_ld16(d, 0),  gYB = cal_ld16(d, 2),  gRB = cal_ld16(d, 4);
    const int16_t gPp = cal_ld16(d, 6),  gPm = cal_ld16(d, 8);
    const int16_t gYp = cal_ld16(d, 10), gYm = cal_ld16(d, 12);
    const int16_t gRp = cal_ld16(d, 14), gRm = cal_ld16(d, 16);
    const int16_t gSp = cal_ld16(d, 18), gSm = cal_ld16(d, 20);
    const int16_t aXp = cal_ld16(d, 22), aXm = cal_ld16(d, 24);
    const int16_t aYp = cal_ld16(d, 26), aYm = cal_ld16(d, 28);
    const int16_t aZp = cal_ld16(d, 30), aZm = cal_ld16(d, 32);

    const float num = (float)(gSp + gSm) * kGyroResPerDeg;
    g_imu_cal[0] = { gPB, num / (float)(gPp - gPm) };
    g_imu_cal[1] = { gYB, num / (float)(gYp - gYm) };
    g_imu_cal[2] = { gRB, num / (float)(gRp - gRm) };

    int16_t r;
    r = aXp - aXm; g_imu_cal[3] = { (int16_t)(aXp - r / 2), 2.0f * kAccelResPerG / (float)r };
    r = aYp - aYm; g_imu_cal[4] = { (int16_t)(aYp - r / 2), 2.0f * kAccelResPerG / (float)r };
    r = aZp - aZm; g_imu_cal[5] = { (int16_t)(aZp - r / 2), 2.0f * kAccelResPerG / (float)r };

    // Sanity gate (same as SDL): a wild bias or a gain off by >50% means a bad
    // factory cal or a short/garbled read — fall back to raw rather than amplify it.
    for (int i = 0; i < 6; i++) {
        const float divisor = (i < 3) ? 64.0f : 1.0f;
        const int   ab      = g_imu_cal[i].bias < 0 ? -g_imu_cal[i].bias : g_imu_cal[i].bias;
        float       gain    = 1.0f - g_imu_cal[i].sens / divisor;
        if (gain < 0) gain = -gain;
        if (ab > 1024 || gain > 0.5f) return;  // leave g_imu_cal_valid = false
    }
    g_imu_cal_valid = true;
}

// Poll once per frame from oled_loop: parse 0x05 the first time it is available
// for this controller, and reset on disconnect so the next controller re-reads.
void imu_cal_service() {
    if (!bt_is_connected()) { g_imu_cal_valid = false; g_imu_cal_tried = false; return; }
    if (g_imu_cal_tried) return;
    auto d = bt_peek_feature(0x05);
    if (d.size() < 35) return;                 // not arrived yet — retry next frame
    imu_cal_parse(d);
    g_imu_cal_tried = true;
}

// index 0..2 gyro, 3..5 accel. Returns the calibrated value in the same count
// scale the raw value used (+-8192 == 1g for accel); identity when no valid
// calibration is loaded, so behaviour matches the pre-calibration firmware.
inline int16_t imu_apply(int index, int16_t raw) {
    if (!g_imu_cal_valid) return raw;
    return (int16_t)((float)(raw - g_imu_cal[index].bias) * g_imu_cal[index].sens);
}

__attribute__((noinline)) void render_screen_gyro() {
    fb_clear();
    draw_text(kContentX, 0, "Gyro Tilt");
    if (bt_is_connected()) {
        int16_t ax, ay, az;
        memcpy(&ax, &interrupt_in_data[21], 2);
        memcpy(&ay, &interrupt_in_data[23], 2);
        memcpy(&az, &interrupt_in_data[25], 2);
        ax = imu_apply(3, ax);  // bias/sensitivity-corrected accel (identity if no cal)
        ay = imu_apply(4, ay);
        az = imu_apply(5, az);
        char buf[16];
        snprintf(buf, sizeof(buf), "X%+5d", ax); draw_text(kContentX, 10, buf);
        snprintf(buf, sizeof(buf), "Y%+5d", ay); draw_text(50, 10, buf);
        snprintf(buf, sizeof(buf), "Z%+5d", az); draw_text(94, 10, buf);

        const int bx = 44, by = 22, bw = 40, bh = 40;
        rect_outline(bx, by, bw, bh);
        for (int x = bx + 1; x < bx + bw - 1; x++) px(x, by + bh / 2, true);
        for (int y = by + 1; y < by + bh - 1; y++) px(bx + bw / 2, y, true);
        // Plot the two axes that read ~0 when the controller lies flat: X (roll,
        // left/right) and Z (pitch, fwd/back). Gravity rests on Y when flat, so
        // driving the dot from Y pegged it to the bottom edge at rest — using Z
        // keeps the dot centred flat and it tracks as you tilt. (Readout above
        // still shows all three raw axes.)
        // Negated so the dot follows the tilt direction: tilt left -> dot left,
        // tilt forward -> dot up (gravity pulls the opposite way on the axis).
        int dx = -((int)ax * (bw / 2 - 3)) / 8192;
        int dy = -((int)az * (bh / 2 - 3)) / 8192;
        int cx = bx + bw / 2 + dx;
        int cy = by + bh / 2 + dy;
        if (cx < bx + 2) cx = bx + 2;
        if (cx > bx + bw - 3) cx = bx + bw - 3;
        if (cy < by + 2) cy = by + 2;
        if (cy > by + bh - 3) cy = by + bh - 3;
        rect_filled(cx - 1, cy - 1, 3, 3);
    } else {
        draw_text(kContentX, 30, "(no controller)");
    }
    flush_fb();
}

__attribute__((noinline)) void render_screen_touchpad() {
    fb_clear();
    draw_text(kContentX, 0, "Touchpad");
    if (bt_is_connected()) {
        rect_outline(kContentX + 2, 12, 116, 30);
        int active = 0;
        for (int finger = 0; finger < 2; finger++) {
            const int off = 32 + finger * 4;
            const uint32_t f = (uint32_t)interrupt_in_data[off] |
                               ((uint32_t)interrupt_in_data[off + 1] << 8) |
                               ((uint32_t)interrupt_in_data[off + 2] << 16) |
                               ((uint32_t)interrupt_in_data[off + 3] << 24);
            const bool not_touching = (f >> 7) & 1u;
            if (not_touching) continue;
            const uint16_t fx = (f >> 8) & 0xFFFu;
            const uint16_t fy = (f >> 20) & 0xFFFu;
            int sx = (kContentX + 3) + ((int)fx * 110) / 1919;
            int sy = 13 + ((int)fy * 26) / 1079;
            if (sx < kContentX + 3)   sx = kContentX + 3;
            if (sx > 122) sx = 122;
            if (sy < 13)  sy = 13;
            if (sy > 40)  sy = 40;
            rect_filled(sx - 1, sy - 1, 3, 3);
            active++;
        }
        char buf[20];
        snprintf(buf, sizeof(buf), "Fingers: %d", active);
        draw_text(kContentX, 46, buf);
    } else {
        draw_text(kContentX, 30, "(no controller)");
    }
    flush_fb();
}

void send_lightbar_color(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t pkt[78] = {};
    pkt[0] = 0x31;
    pkt[2] = 0x10;
    pkt[4] = 0x04; // valid_flag1: LIGHTBAR_CONTROL_ENABLE (bit 2)
    pkt[47] = r;   // lightbar_red
    pkt[48] = g;   // lightbar_green
    pkt[49] = b;   // lightbar_blue
    bt_write(pkt, sizeof(pkt));
}

// Tiny 32-step sine LUT (no <cmath>). angle 0..255 → amplitude -127..127.
static const int8_t kSine32[32] = {
    0,   24,   49,   70,   90,  106,  117,  125,  127,  125,  117,  106,   90,   70,   49,   24,
    0,  -24,  -49,  -70,  -90, -106, -117, -125, -127, -125, -117, -106,  -90,  -70,  -49,  -24,
};
int sin_lut(uint8_t a) { return kSine32[(a >> 3) & 0x1F]; }

void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (h >= 360) h %= 360;
    const uint8_t region = (uint8_t)(h / 60);
    const uint16_t remainder = (uint16_t)((h - region * 60u) * 256u / 60u);
    const uint8_t p = (uint8_t)(((uint16_t)v * (255u - s)) >> 8);
    const uint8_t q = (uint8_t)(((uint16_t)v * (255u - (((uint16_t)s * remainder) >> 8))) >> 8);
    const uint8_t t = (uint8_t)(((uint16_t)v * (255u - (((uint16_t)s * (255u - remainder)) >> 8))) >> 8);
    switch (region) {
        case 0:  *r = v; *g = t; *b = p; break;
        case 1:  *r = q; *g = v; *b = p; break;
        case 2:  *r = p; *g = v; *b = t; break;
        case 3:  *r = p; *g = q; *b = v; break;
        case 4:  *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

const char* lb_mode_tag(int mode) {
    switch (mode) {
        case 0: return "[LIVE]";
        case 1: return "[FAV0]";
        case 2: return "[FAV1]";
        case 3: return "[FAV2]";
        case 4: return "[FAV3]";
        case 5: return "[BREA]";
        case 6: return "[RAIN]";
        case 7: return "[FADE]";
        case 8: return "[HOST]";
        default: return "[????]";
    }
}

// Defined next to lightbar_service() below (it needs lightbar_compute_mode);
// used here to refresh lb_r/g/b before a favorite save.
void lightbar_update_color();

// R1 rising edge on Lightbar cycles lb_mode. Used to be KEY1; that moved
// to back-nav. Triangle on this screen stays as "save current RGB to
// favorite slot 0" (the existing favorite-save UX), so R1 is the next
// free button that doesn't break a mental model.
void lightbar_handle_input() {
    if (!bt_is_connected()) { lb_last_buttons = 0; return; }
    const uint8_t btns   = interrupt_in_data[8];
    const bool r1_now    = (btns & 0x02) != 0;
    const bool r1_prev   = (lb_last_buttons & 0x02) != 0;
    if (r1_now && !r1_prev) {
        lb_mode = (lb_mode + 1) % kNumLbModes;
        lb_dirty = true; // persisted on leaving the Lightbar screen
    }
    lb_last_buttons = btns;

    // Face button rising edge -> save current color to slot 0..3. Lived in
    // render_screen_lightbar() until the per-screen input moved out of the
    // 10 Hz render path (see screen_input_tick).
    const uint8_t face = interrupt_in_data[7] & 0xF0;
    const uint8_t pressed = face & ~lb_last_face;
    lb_last_face = face;
    int save_slot = -1;
    if      (pressed & 0x80) save_slot = 0; // Triangle
    else if (pressed & 0x40) save_slot = 1; // Circle
    else if (pressed & 0x20) save_slot = 2; // Cross
    else if (pressed & 0x10) save_slot = 3; // Square
    if (save_slot >= 0) {
        // This runs before lightbar_service() in oled_loop, so lb_r/g/b can be
        // up to one frame stale — recompute before capturing them.
        lightbar_update_color();
        lb_fav_r[save_slot] = lb_r;
        lb_fav_g[save_slot] = lb_g;
        lb_fav_b[save_slot] = lb_b;
        lb_dirty = true; // persisted on leaving the Lightbar screen
    }
}

__attribute__((noinline)) void render_screen_lightbar() {
    // Input (R1 mode cycle, face-button favorite save) is sampled by
    // screen_input_tick() every loop iteration, not here.
    fb_clear();
    draw_text(kContentX, 0, "Lightbar");
    draw_text(86, 0, lb_mode_tag(lb_mode));

    if (bt_is_connected()) {
        // lb_r/lb_g/lb_b are computed every frame by lightbar_service() (which
        // runs ahead of this render in oled_loop), so here we only display them.
        char buf[16];
        snprintf(buf, sizeof(buf), "R:%3u", lb_r); draw_text(kContentX, 12, buf);
        snprintf(buf, sizeof(buf), "G:%3u", lb_g); draw_text(48, 12, buf);
        snprintf(buf, sizeof(buf), "B:%3u", lb_b); draw_text(90, 12, buf);

        const int by = 22, bh = 8;
        rect_outline(kContentX,  by, 38, bh); int rf = (lb_r * 34) / 255; if (rf > 0) rect_filled(kContentX + 2,  by + 2, rf, bh - 4);
        rect_outline(48, by, 38, bh); int gf = (lb_g * 34) / 255; if (gf > 0) rect_filled(50, by + 2, gf, bh - 4);
        rect_outline(90, by, 38, bh); int bf = (lb_b * 34) / 255; if (bf > 0) rect_filled(92, by + 2, bf, bh - 4);

        draw_text(kContentX, 38, "Sv:T=0 C=1 X=2 S=3");
        const char* hint =
            (lb_mode == 0)           ? "Tilt = R/G/B"   :
            (lb_mode == 5)           ? "Breathing FAV0" :
            (lb_mode == 6)           ? "Rainbow sweep"  :
            (lb_mode == 7)           ? "Fade thru FAVs" :
            (lb_mode == kLbModeHost) ? "Host controls"  :
                                       "Locked to fav";
        draw_text(kContentX, 48, hint);
        // No send here: lightbar_service() owns pushing the color to the
        // controller every frame, on this screen and every other.
    } else {
        draw_text(kContentX, 30, "(no controller)");
    }
    draw_text(kContentX, 56, "R1=mode");
    flush_fb();
}

// Compute lb_r/lb_g/lb_b for an OLED lightbar mode (0..7). HOST (8) is handled
// by the caller (no firmware color). noinline keeps the float/HSV literals out
// of lightbar_service's / oled_loop's literal pool (same Thumb reach constraint
// the render_screen_* functions hit).
__attribute__((noinline))
void lightbar_compute_mode(int mode, uint32_t now_ms) {
    if (mode == 0) {
        // LIVE: tilt -> RGB
        int16_t ax, ay, az;
        memcpy(&ax, &interrupt_in_data[21], 2);
        memcpy(&ay, &interrupt_in_data[23], 2);
        memcpy(&az, &interrupt_in_data[25], 2);
        ax = imu_apply(3, ax);  // calibrated accel keeps the +-8192 == 1g scale below
        ay = imu_apply(4, ay);
        az = imu_apply(5, az);
        const int rr = ((int)ax + 8192) * 255 / 16384;
        const int gg = ((int)ay + 8192) * 255 / 16384;
        const int bb = ((int)az + 8192) * 255 / 16384;
        lb_r = (uint8_t)(rr < 0 ? 0 : rr > 255 ? 255 : rr);
        lb_g = (uint8_t)(gg < 0 ? 0 : gg > 255 ? 255 : gg);
        lb_b = (uint8_t)(bb < 0 ? 0 : bb > 255 ? 255 : bb);
    } else if (mode <= 4) {
        // FAV slot: fixed color
        const int slot = mode - 1;
        lb_r = lb_fav_r[slot];
        lb_g = lb_fav_g[slot];
        lb_b = lb_fav_b[slot];
    } else if (mode == 5) {
        // BREATHING: modulate FAV0 brightness with a sine wave (~3 s cycle)
        const uint8_t phase = (uint8_t)(now_ms / 12);
        const int s = sin_lut(phase); // -127..127
        const uint16_t scale = (uint16_t)(32 + (s + 127) / 2); // 32..191
        lb_r = (uint8_t)((lb_fav_r[0] * scale) / 255);
        lb_g = (uint8_t)((lb_fav_g[0] * scale) / 255);
        lb_b = (uint8_t)((lb_fav_b[0] * scale) / 255);
    } else if (mode == 6) {
        // RAINBOW: hue sweep over ~6 s
        const uint16_t hue = (uint16_t)((now_ms / 17) % 360);
        hsv_to_rgb(hue, 255, 255, &lb_r, &lb_g, &lb_b);
    } else {
        // FADE between FAV slots, 2 s per slot
        const uint32_t kSlotMs = 2000;
        const uint32_t total = now_ms % (4 * kSlotMs);
        const int slot = (int)(total / kSlotMs);
        const int next = (slot + 1) & 3;
        const uint16_t blend = (uint16_t)(((total - slot * kSlotMs) * 256u) / kSlotMs);
        lb_r = (uint8_t)((lb_fav_r[slot] * (255 - blend) + lb_fav_r[next] * blend) / 255);
        lb_g = (uint8_t)((lb_fav_g[slot] * (255 - blend) + lb_fav_g[next] * blend) / 255);
        lb_b = (uint8_t)((lb_fav_b[slot] * (255 - blend) + lb_fav_b[next] * blend) / 255);
    }
}

// The single owner of the controller LED. Runs every frame (~10 Hz) from
// oled_loop, on every screen, so a chosen mode "sticks" everywhere instead of
// only while the Lightbar screen renders. Priority:
//   1. Charging  -> amber-orange breathing pulse (status indicator).
//   2. lb_mode != HOST -> the selected OLED mode/color.
//   3. HOST (or disconnected) -> hand the LED back to the host/game.
// When the firmware owns the LED it actively pushes the color via
// send_lightbar_color (idle: every frame; during audio: ~10 Hz), and turns on
// g_lightbar_override — main.cpp's 0x02 forward then clears AllowLedColor+RGB
// so the host's writes can't stomp it.
// Recompute lb_r/lb_g/lb_b (and g_lightbar_override) for the current state —
// no BT traffic. Split out of lightbar_service() so lightbar_handle_input(),
// which runs earlier in oled_loop, can capture a fresh color when saving a
// favorite instead of the previous frame's.
__attribute__((noinline))
void lightbar_update_color() {
    if (!bt_is_connected()) { g_lightbar_override = false; return; }
    const uint32_t now_ms = time_us_32() / 1000;

    if (g_charge_eta.charging) {
        // ~4.6 s breathing cycle (256 phase steps × 18 ms). Base amber
        // (255,100,0) sine-enveloped from dim (24) to bright (240).
        const uint8_t  phase = (uint8_t)(now_ms / 18);
        const int      s     = sin_lut(phase);                          // -127..127
        const uint16_t scale = (uint16_t)(24 + ((s + 127) * 216) / 254); // 24..240
        lb_r = (uint8_t)((255u * scale) / 255u);
        lb_g = (uint8_t)((100u * scale) / 255u);
        lb_b = 0;
    } else if (lb_mode == kLbModeHost) {
        // Reflect the host's current LED on the OLED bars, then stand down.
        lb_r = g_host_led[0];   // 主机当前颜色（main.cpp 记录）
        lb_g = g_host_led[1];
        lb_b = g_host_led[2];
        g_lightbar_override = false;
        return;
    } else {
        lightbar_compute_mode(lb_mode, now_ms);
    }

    g_lightbar_override = true;
}

// Pushes the color computed by lightbar_update_color() to the controller.
// Runs once per rendered frame from oled_loop.
__attribute__((noinline))
void lightbar_service() {
    lightbar_update_color();
    if (!g_lightbar_override) return; // HOST mode / no controller: LED isn't ours
    // 反向移植后音频帧不再携带 state[]（上游 0x39 双包只装 haptics+speaker），
    // 固件持有的灯条颜色改为自己直发：空闲时按帧率直发；音频播放中降到 ~10 Hz，
    // 避免挤占音频包节奏（0x39 双包比旧 0x36 单包省一半包量，10 Hz 的额外 0x31
    // 在链路余量内）。
    {
        static uint32_t lb_last_push_us = 0;
        const uint32_t now_us = time_us_32();
        const uint32_t min_gap = spk_active ? 100000u : 0u;
        if (lb_last_push_us == 0 || (uint32_t)(now_us - lb_last_push_us) >= min_gap) {
            lb_last_push_us = now_us;
            send_lightbar_color(lb_r, lb_g, lb_b);
        }
    }
}

void lightbar_load_config() {
    const Config_body& c = get_config();
    lb_mode = c.lightbar_mode;
    if (lb_mode < 0 || lb_mode >= kNumLbModes) lb_mode = kLbModeHost;
    for (int i = 0; i < 4; i++) {
        lb_fav_r[i] = c.lb_fav_r[i];
        lb_fav_g[i] = c.lb_fav_g[i];
        lb_fav_b[i] = c.lb_fav_b[i];
    }
    lb_dirty = false;
}

void lightbar_save_config() {
    Config_body b = get_config();
    b.lightbar_mode = (uint8_t)lb_mode;
    for (int i = 0; i < 4; i++) {
        b.lb_fav_r[i] = lb_fav_r[i];
        b.lb_fav_g[i] = lb_fav_g[i];
        b.lb_fav_b[i] = lb_fav_b[i];
    }
    set_config(b);
    config_save();
    lb_dirty = false;
}

__attribute__((noinline)) void render_screen_vu() {
    fb_clear();
    draw_text(kContentX, 0, "Audio Meters");
    if (bt_is_connected()) {
        const uint8_t spk = audio_peak_speaker();
        const uint8_t hap = audio_peak_haptic();
        char buf[16];
        snprintf(buf, sizeof(buf), "SPK %3u", spk);
        draw_text(kContentX, 14, buf);
        rect_outline(48, 14, 80, 8);
        int sfill = (spk * 76) / 255;
        if (sfill > 0) rect_filled(50, 16, sfill, 4);

        snprintf(buf, sizeof(buf), "HAP %3u", hap);
        draw_text(kContentX, 28, buf);
        rect_outline(48, 28, 80, 8);
        int hfill = (hap * 76) / 255;
        if (hfill > 0) rect_filled(50, 30, hfill, 4);

        draw_text(kContentX, 42, "Live USB audio peaks");
    } else {
        draw_text(kContentX, 30, "(no controller)");
    }
    flush_fb();
}

// 上游 status_gpio 的候选引脚：status_gpio_pin_valid() 已排除 SDK 保留脚
// (UART/VSYS/VBUS/SMPS/CYW43)，这里再排除 OLED 自己占用的脚
// (SPI CLK/MOSI/CS/DC/RST + 两个按键)，否则用户可能选到屏幕正在用的脚。
static bool status_pin_selectable(uint8_t pin) {
    if (!status_gpio_pin_valid(pin)) return false;
    switch (pin) {
        case 8: case 9: case 10: case 11: case 12: case 15: case 17: return false;
        default: return true;
    }
}

void settings_adjust(int delta) {
    const SetItem& it = kSetItems[settings_sel];
    settings_dirty = true; // 与旧实现一致：动作项上按左右也置 dirty（历史行为，保持零差异）
    if (it.kind == SK_ACTION) return; // Reset / Wipe 只认长按（见 settings_handle_input）
    uint8_t* p = reinterpret_cast<uint8_t*>(&settings_local) + it.off;

    if (it.kind == SK_BOOL) {
        *p ^= 1;
        return;
    }
    if (it.flags & SIF_FLOAT10) { // Hap Gain：float 字段（packed 偏移不对齐，走 memcpy）
        float f;
        memcpy(&f, p, sizeof f);
        int v = (int)(f * 10.0f + 0.5f) + delta;
        if (v < it.lo) v = it.lo;
        if (v > it.hi) v = it.hi;
        f = v / 10.0f;
        memcpy(p, &f, sizeof f);
        return;
    }

    int v = (int)*p + delta * (it.step ? it.step : 1);
    if (it.flags & SIF_PIN) { // StatPin：只停在合法且未被 OLED 占用的脚（0xFF=off 也是档位）
        for (int tries = 0; tries < 64; tries++) {
            if (v < 0) v = 255;
            if (v > 255) v = 0;
            if (v == STATUS_GPIO_DISABLED || status_pin_selectable((uint8_t)v)) break;
            v += delta;
        }
        *p = (uint8_t)v;
        return;
    }

    if (it.flags & SIF_WRAP) { // 枚举环绕（lo<->hi）
        if (v < it.lo) v = it.hi;
        if (v > it.hi) v = it.lo;
    } else {
        if (v < it.lo) v = it.lo;
        if (v > it.hi) v = it.hi;
    }
    if ((it.flags & SIF_SNAP) && v > 0 && v < 10) {
        v = (delta < 0) ? 0 : 10; // 空闲超时档位：0 / 10 / 15 / … / 60
    }
    *p = (uint8_t)v;
}

void settings_handle_input() {
    if (!bt_is_connected()) return;
    const uint8_t dpad = (uint8_t)(interrupt_in_data[7] & 0x0F);
    const uint8_t face = (uint8_t)(interrupt_in_data[7] & 0xF0);

    // Edge-trigger on D-pad direction CHANGE; only pure N/E/S/W to avoid diagonals
    if (dpad != settings_last_dpad && dpad != 8) {
        if      (dpad == 0) settings_sel = (settings_sel - 1 + kNumSettingsItems) % kNumSettingsItems;
        else if (dpad == 4) settings_sel = (settings_sel + 1) % kNumSettingsItems;
        else if (dpad == 6) settings_adjust(-1);
        else if (dpad == 2) settings_adjust(+1);
    }
    settings_last_dpad = dpad;

    // Triangle handling — Reset and Wipe-slots items both require a 2 s hold;
    // every other item saves edits on a normal short press.
    const bool tri_now = (face & 0x80) != 0;
    const bool tri_prev = (settings_last_face & 0x80) != 0;
    const bool is_hold_item = (kSetItems[settings_sel].kind == SK_ACTION);
    if (tri_now && !tri_prev) {
        settings_tri_press_us = (uint32_t)time_us_32();
        settings_reset_triggered = false;
    }
    if (is_hold_item && tri_now && !settings_reset_triggered
        && ((uint32_t)time_us_32() - settings_tri_press_us) >= kResetHoldUs) {
        settings_reset_triggered = true;
        if (kSetItems[settings_sel].act == ACT_RESET) {
            config_default();
            if (config_save()) {
                settings_local = get_config();
                lightbar_load_config(); // refresh RAM lightbar state (no reboot here)
                settings_dirty = false;
                settings_set_status("Reset!");
            } else {
                settings_set_status("Reset FAIL");
            }
        } else {
            bt_wipe_all_slots();
            settings_set_status("Slots wiped!");
        }
    }
    if (!tri_now && tri_prev) {
        if (!is_hold_item && !settings_reset_triggered) {
            const bool sn_changed = ((get_config().enable_usb_sn != 0)
                                     != (settings_local.enable_usb_sn != 0));
#ifdef ENABLE_WAKE_HID
            // enable_wake / ps_shortcut_enabled 改变 USB 枚举面（bcdUSB 2.1 + BOS +
            // 键盘接口 / remote-wakeup 位），保存后必须重新枚举才生效。
            const bool wake_changed =
                (get_config().enable_wake != settings_local.enable_wake)
                || (get_config().ps_shortcut_enabled != settings_local.ps_shortcut_enabled);
#endif
            set_config(settings_local);
            // 原来靠比较状态串前缀（"Sa..."）判断成功 —— 但 "Save FAIL" 也匹配，
            // 保存失败时同样会清 dirty 并触发重挂载。改成直接用返回值。
            const bool saved = config_save();
            settings_set_status(saved ? "Saved!" : "Save FAIL");
            if (saved) {
                settings_dirty = false;
                // USB 序列号开关只在主机重新枚举时被读到 → 保存后主动重挂载一次
                // （与 web 工具 0x03 指令同一条路径；会给主机一个正常的重连脉冲）
#ifdef ENABLE_WAKE_HID
                if (wake_changed) {
                    // 模式选择与 bt.cpp 的断连处理相同：wake 开且手柄没连 → 键盘专属
                    // 模式；否则正常全功能配置（保存时手柄必然连着，等效 false）。
                    usb_reconnect(get_config().enable_wake && !bt_is_connected());
                } else if (sn_changed) {
                    usb_reconnect(false);
                }
#else
                if (sn_changed) usb_reconnect(false);
#endif
            }
        }
        settings_reset_triggered = false;
    }
    settings_last_face = face;
}

__attribute__((noinline)) void format_settings_item(int idx, char* line, size_t n) {
    const SetItem& it = kSetItems[idx];
    const char* cur = (idx == settings_sel) ? ">" : " ";

    if (it.kind == SK_ACTION) { // 动作项只有标签（长按提示单独一行）
        snprintf(line, n, "%s %s", cur, it.label);
        return;
    }

    const uint8_t* base = reinterpret_cast<const uint8_t*>(&settings_local);
    const uint8_t v = base[it.off];
    char val[16];
    if (it.kind == SK_BOOL) {
        const char* const* nm = it.names ? it.names : kOnOff;
        snprintf(val, sizeof val, "%s", nm[v & 1]);
    } else if (it.flags & SIF_FLOAT10) {
        float f;
        memcpy(&f, base + it.off, sizeof f);
        const int g = (int)(f * 10.0f + 0.5f);
        snprintf(val, sizeof val, "%d.%dx", g / 10, g % 10);
    } else if (it.flags & SIF_PIN) {
        if (v == STATUS_GPIO_DISABLED) snprintf(val, sizeof val, "off");
        else snprintf(val, sizeof val, "GP%u", v);
    } else if (it.names) { // 枚举：显示名表
        snprintf(val, sizeof val, "%s", it.names[v <= it.hi ? v : it.hi]);
    } else if (it.fmt == SF_OFF0 && v == 0) {
        snprintf(val, sizeof val, "off");
    } else if (it.fmt == SF_AUTO0 && v == 0) {
        snprintf(val, sizeof val, "auto");
    } else {
        snprintf(val, sizeof val, "%d%s", (int)v + it.disp_off, it.unit ? it.unit : "");
    }
    snprintf(line, n, "%s %s %s", cur, it.label, val);
}

__attribute__((noinline)) void render_screen_settings() {
    if (!settings_init_done) {
        settings_local = get_config();
        settings_init_done = true;
    }
    // Input is sampled by screen_input_tick() every loop iteration, not here.

    fb_clear();
    char buf[24];
    snprintf(buf, sizeof(buf), "Settings %s", settings_dirty ? "(*)" : "   ");
    draw_text(kContentX, 0, buf);

    constexpr int kVisible = 5;
    int top = 0;
    if (settings_sel >= kVisible) top = settings_sel - kVisible + 1;
    char line[28];
    for (int i = 0; i < kVisible && top + i < kNumSettingsItems; i++) {
        format_settings_item(top + i, line, sizeof(line));
        draw_text(kContentX, 9 + i * 9, line);
    }

    // Footer row: a recent save/reset result takes over while live, otherwise
    // the selected item's hint (or the generic nav hint).
    const char* footer = kSetItems[settings_sel].hint;
    if (settings_save_status[0]
        && (int32_t)((uint32_t)time_us_32() - settings_status_until_us) < 0) {
        footer = settings_save_status;
    } else if (!footer) {
        footer = "DP nav/adj  Tri=save";
    }
    draw_text(kContentX, 56, footer);
    flush_fb();
}

// ---- Slots screen (Phase G) ----------------------------------------------
// Multi-slot persistent pairing UI. Modeled on zurce/DS5Dongle-OLED.
// Credit to zurce.

int slots_cursor = -1;             // initialized to active slot on first entry
uint8_t slots_last_dpad = 8;
uint8_t slots_last_face = 0;
uint32_t slots_sq_press_us = 0;
bool slots_wipe_triggered = false;
const char* slots_status = "";
uint32_t slots_status_until_us = 0;
constexpr uint32_t kSlotsWipeHoldUs = 1500000;  // 1.5 s

void slots_handle_input() {
    if (slots_cursor < 0) slots_cursor = bt_get_slot();
    if (!bt_is_connected()) {
        // Even without a DS5 connected we still want to navigate / wipe;
        // we just can't read the controller's D-pad / face inputs. Return
        // here and require KEY0/KEY1 for screen switching.
        slots_last_dpad = 8;
        slots_last_face = 0;
        return;
    }
    const uint8_t dpad = (uint8_t)(interrupt_in_data[7] & 0x0F);
    const uint8_t face = (uint8_t)(interrupt_in_data[7] & 0xF0);

    if (dpad != slots_last_dpad && dpad != 8) {
        if      (dpad == 0) slots_cursor = (slots_cursor - 1 + kNumSlots) % kNumSlots;
        else if (dpad == 4) slots_cursor = (slots_cursor + 1) % kNumSlots;
    }
    slots_last_dpad = dpad;

    // Triangle rising edge: switch to cursor slot if different from active
    const bool tri_now = (face & 0x80) != 0;
    const bool tri_prev = (slots_last_face & 0x80) != 0;
    if (tri_now && !tri_prev) {
        if (slots_cursor != bt_get_slot()) {
            bt_set_slot(slots_cursor);
            slots_status = "Switched!";
            slots_status_until_us = (uint32_t)time_us_32() + 1500000;
        }
    }

    // Square hold 1.5 s: wipe cursor slot
    const bool sq_now = (face & 0x10) != 0;
    const bool sq_prev = (slots_last_face & 0x10) != 0;
    if (sq_now && !sq_prev) {
        slots_sq_press_us = (uint32_t)time_us_32();
        slots_wipe_triggered = false;
    }
    if (sq_now && !slots_wipe_triggered
        && ((uint32_t)time_us_32() - slots_sq_press_us) >= kSlotsWipeHoldUs) {
        slots_wipe_triggered = true;
        bt_forget_slot(slots_cursor);
        slots_status = "Wiped!";
        slots_status_until_us = (uint32_t)time_us_32() + 1500000;
    }
    if (!sq_now && sq_prev) slots_wipe_triggered = false;

    slots_last_face = face;
}

__attribute__((noinline)) void render_screen_slots() {
    // Input is sampled by screen_input_tick() every loop iteration, not here.
    if (slots_cursor < 0) slots_cursor = bt_get_slot();

    fb_clear();
    char hdr[24];
    const int active = bt_get_slot();
    const bool conn = bt_is_connected();
    // 20 chars = ends at x=124. The old spacing made the header 21 chars, which
    // clipped the trailing ']' — and the status draw at x=80 painted over it.
    snprintf(hdr, sizeof(hdr), "Slots        [s%d %s]", active, conn ? "ON" : "--");
    draw_text(kContentX, 0, hdr);

    for (int i = 0; i < kNumSlots; i++) {
        char line[28];
        const char *cursor_mark = (i == slots_cursor) ? ">" : " ";
        const char *active_mark = (i == active) ? "*" : " ";
        if (slot_occupied(i)) {
            uint8_t a[6];
            slot_get_addr(i, a);
            snprintf(line, sizeof(line), "%s%d%s %02X:%02X:%02X:%02X:%02X:%02X",
                     cursor_mark, i, active_mark, a[0], a[1], a[2], a[3], a[4], a[5]);
        } else {
            snprintf(line, sizeof(line), "%s%d%s (empty)", cursor_mark, i, active_mark);
        }
        draw_text(kContentX, 9 + i * 9, line);
    }

    // Footer row carries the switch/wipe result while it is live; the hint it
    // replaces was also 2 chars too wide for the panel.
    const bool status_live = slots_status[0]
        && (int32_t)((uint32_t)time_us_32() - slots_status_until_us) < 0;
    draw_text(kContentX, 56, status_live ? slots_status : "Tri=sw Sq hold=wipe");
    flush_fb();
}

void boot_splash() {
    fb_clear();
    auto cx_for = [](const char* s) {
        int n = 0; while (s[n]) n++;
        return (128 - (n * 6 - 1)) / 2;
    };
    const char* l1 = "DS5 Bridge";
    // Follow the build version (-DVERSION from release.yml tag) instead of a
    // hard-coded string — the splash previously showed a stale "v0.6.0" even
    // on newer releases. Local builds without -DVERSION show "dev".
    const char* l2 = FIRMWARE_VERSION;
    const char* l3 = "Pico2W + OLED";
    draw_text(cx_for(l1), 16, l1);
    draw_text(cx_for(l2), 30, l2);
    draw_text(cx_for(l3), 44, l3);
    flush_fb();
    // Hold the splash for ~1.5 s WITHOUT blocking: oled_init() runs before the
    // main loop, so a sleep here delayed tud_task() — and with it USB
    // enumeration — by the same amount. The SH1107 keeps the image in its own
    // GDDRAM, so oled_loop() just skips rendering until this deadline.
    splash_deadline_us = fast_now_us() + 1500000u;   // 32 位 µs，见 fast_time.h
    splash_done = false;
}

// Per-screen input sampling. Called every oled_loop() iteration — NOT from the
// renderers, which only run once per kFrameUs (10 Hz): sampling there dropped
// any controller tap shorter than a frame (a quick D-pad nudge, a △ press).
// Each handler keeps its own edge-detection state, so a higher sample rate only
// makes them more reliable. Only the screen on display is sampled.
void screen_input_tick() {
    // Only while the panel is actually awake. With "CtrlWake" off the dim/off
    // tiers deliberately ignore controller input (only KEY0/KEY1 wake it), and
    // running the handlers there would let a button press silently change a
    // setting or switch a slot behind a blank panel. oled_power_state lags by
    // at most one frame, so a press that wakes the panel still registers — the
    // same one-frame latency the old render-path sampling had.
    if (oled_power_state != OLED_ACTIVE) return;
    switch (current_screen) {
        case kScreenSlots:    slots_handle_input();            break;
        case kScreenLightbar: lightbar_handle_input();         break;
        case kScreenTriggers: triggers_handle_input();         break;
        case kScreenDiag:     diag_handle_input(kDiagVisible); break;
        case kScreenSettings: settings_handle_input();         break;
        default: break;
    }
}

} // namespace

void oled_init() {
    spi_init(spi1, 10 * 1000 * 1000);
    gpio_set_function(kPinCLK, GPIO_FUNC_SPI);
    gpio_set_function(kPinMOSI, GPIO_FUNC_SPI);

    gpio_init(kPinCS);   gpio_set_dir(kPinCS, GPIO_OUT);  gpio_put(kPinCS, 1);
    gpio_init(kPinDC);   gpio_set_dir(kPinDC, GPIO_OUT);  gpio_put(kPinDC, 0);
    gpio_init(kPinRST);  gpio_set_dir(kPinRST, GPIO_OUT); gpio_put(kPinRST, 1);

    gpio_init(kPinKey0); gpio_set_dir(kPinKey0, GPIO_IN); gpio_pull_up(kPinKey0);
    gpio_init(kPinKey1); gpio_set_dir(kPinKey1, GPIO_IN); gpio_pull_up(kPinKey1);

    // 位反转查表（见 rev_lut）；同时让显存镜像失效 —— 上电时 GDDRAM 内容未定义，
    // 第一帧必须整屏发一次。此后 0xAE/0xAF 开关屏不动显存，无需再失效。
    for (int i = 0; i < 256; i++) rev_lut[i] = reverse_byte((uint8_t)i);
    fb_shadow_valid = false;

    // 字形转置表（见 font_rows）：列存字库 → 每行 5 bit
    for (int c = 0; c < 95; c++) {
        for (int row = 0; row < kFontH; row++) {
            uint8_t bits = 0;
            for (int col = 0; col < kFontW; col++)
                if (kFont5x7[c][col] & (1u << row)) bits |= (uint8_t)(1u << (kFontW - 1 - col));
            font_rows[c][row] = bits;
        }
    }

    hw_reset();
    sh1107_init();
    fb_clear();
    boot_splash();

    // Restore the persisted lightbar mode + favorites (config_load() already ran
    // in main() before this). Defaults to HOST passthrough on a fresh flash.
    lightbar_load_config();

    // Restore the persisted OLED brightness (KEY1-long-press choice). config_valid
    // clamps screen_brightness to a legal kBrightLevels index, so this is safe to
    // use directly. Fresh flash → 0 (full brightness). Issue #9.
    bright_idx = get_config().screen_brightness;
}

// Dim-tier renderer: blank the panel and draw a tiny "I'm alive" dot that
// breathes (1s on / 1s off) and walks through 8 evenly-spaced positions every
// ~30 s. Two goals: (1) reduce total pixel-on-time to a tiny fraction so the
// panel barely glows even with the contrast register pinned, (2) prevent any
// single pixel from accumulating wear. noinline keeps oled_loop's literal pool
// in Thumb's 4 KB reach (same constraint the other render_screen_* hit).
__attribute__((noinline))
void render_dim_pulse(uint32_t dim_elapsed_us) {
    fb_clear();
    constexpr uint32_t kPulsePeriodUs = 2UL * 1000000UL; // 2 s blink cycle
    constexpr uint32_t kPulseOnUs     = 1UL * 1000000UL; // 1 s on, 1 s off
    constexpr uint32_t kPosStepUs     = 30UL * 1000000UL; // 30 s per position
    constexpr int kPositions[][2] = {
        { 16,  8}, { 64,  8}, {112,  8},
        {112, 32},
        {112, 56}, { 64, 56}, { 16, 56},
        { 16, 32},
    };
    constexpr int kNumPositions = sizeof(kPositions) / sizeof(kPositions[0]);
    const bool dot_on = (dim_elapsed_us % kPulsePeriodUs) < kPulseOnUs;
    if (dot_on) {
        const int idx = (int)((dim_elapsed_us / kPosStepUs) % (uint32_t)kNumPositions);
        const int cx = kPositions[idx][0];
        const int cy = kPositions[idx][1];
        // 2x2 dot — small enough to barely register, big enough to see across a desk.
        rect_filled(cx, cy, 2, 2);
    }
    flush_fb_raw(); // skip chrome arrows; nothing to navigate to from sleep
}

void oled_loop() {
    handle_buttons();
    const uint32_t now = fast_now_us();   // 每轮调用：省掉 time_us_32 的函数调用（见 fast_time.h）
    rumble_burst_tick(now);
    // Boot splash still on the panel: keep the main loop (USB / BT / audio)
    // running and skip everything else until the deadline passes.
    // （常见路径下 splash_done 已经是 true，只剩一次 bool 判断 —— 见 fast_time.h）
    if (!splash_done) {
        if ((int32_t)(fast_now_us() - splash_deadline_us) < 0) return;
        splash_done = true;
    }
    // Controller input for the current screen is sampled every iteration;
    // only rendering is paced by kFrameUs below.
    screen_input_tick();
    if ((now - last_render_us) < kFrameUs) return;
    last_render_us = now;
    // Track charge progress every frame — before the power-ladder early-returns
    // below, so step timing stays correct even while the panel is dimmed/off.
    sample_charge_eta();
    // Parse the DS5's per-unit IMU calibration once it lands (no-op until then),
    // so the tilt screen + tilt->RGB lightbar use corrected accel. See imu_apply().
    imu_cal_service();
    // Drive the controller LED every frame (any screen / power state): charging
    // pulse, selected OLED mode, or hand-off to the host. See lightbar_service().
    lightbar_service();
    // Bump activity on controller input changes (cheap rolling hash over input
    // bytes). Mirror bt.cpp's inactivity heuristic so resting-controller noise
    // doesn't read as activity: the analog sticks (idata[0..3]) jitter by ±1 LSB
    // at rest, so collapse their rest band [120,140] to a constant, and skip
    // idata[6] (the volatile counter byte bt.cpp's idle check also ignores).
    // Without this the dot/dim tier never engages while a controller is
    // connected, because a stick flicker resets the idle timer every few frames.
    uint32_t hash = 0;
    for (int i = 0; i < 10; i++) {
        if (i == 6) continue;
        uint8_t b = interrupt_in_data[i];
        if (i < 4 && b >= 120 && b <= 140) b = 128; // stick deadzone
        hash = hash * 31u + b;
    }
    if (hash != last_input_hash) {
        last_input_hash = hash;
        // Controller input only keeps the panel awake when the user has left
        // "CtrlWake" on (the default). With it off, the dim/off timers count
        // down during gameplay and only KEY0/KEY1 wake the screen — see
        // handle_buttons(), which bumps last_activity_us unconditionally.
        // Issues #8 / #9.
        if (get_config().controller_wakes_display) last_activity_us = time_us_64();
    }
    // Rising-edge: BT-connect itself counts as activity, so the screen wakes
    // the moment a controller pairs rather than waiting for the first input.
    const bool bt_connected_now = bt_is_connected();
    if (bt_connected_now && !prev_bt_connected) last_activity_us = time_us_64();
    prev_bt_connected = bt_connected_now;

    // Power-state ladder: Active → Dim (breathing dot) → Off based on idle time.
    // Thresholds are user-configurable (minutes; 0 = that tier disabled) — #5.
    // While charging we cap the ladder at Dim — the panel keeps doing the
    // low-power breathing dot but never fully sleeps. This stops the user from
    // unplugging the controller just to "wake" the dongle (which would reset the
    // charge-ETA calibration). The dot tier already draws ~no current, so this
    // costs little; sample_charge_eta() runs before this block regardless.
    const uint64_t idle   = time_us_64() - last_activity_us;
    const uint64_t dim_us = (uint64_t)get_config().screen_dim_timeout * 60ULL * 1000000ULL;
    const uint64_t off_us = (uint64_t)get_config().screen_off_timeout * 60ULL * 1000000ULL;
    const bool off_enabled = get_config().screen_off_timeout != 0;
    const bool dim_enabled = get_config().screen_dim_timeout != 0;
    if (off_enabled && idle > off_us && !g_charge_eta.charging) {
        if (oled_power_state != OLED_OFF) {
            cmd(0xAE);
            oled_power_state = OLED_OFF;
            g_fb_generation++; // 面板要黑了：醒来时局部刷新的屏必须整屏重画
        }
        return; // panel is off, nothing to draw
    }
    if (oled_power_state == OLED_OFF) cmd(0xAF); // wake panel before drawing
    if (dim_enabled && idle > dim_us) {
        sh1107_set_contrast(kDimContrast);
        if (oled_power_state != OLED_DIM) g_fb_generation++; // 呼吸点会清掉 fb
        oled_power_state = OLED_DIM;
        render_dim_pulse((uint32_t)(idle - dim_us));
        return; // skip the regular per-screen render path
    }
    sh1107_set_contrast(kBrightLevels[bright_idx]);
    oled_power_state = OLED_ACTIVE;

    // True on the first render after navigating to a different screen.
    // Lets a screen do expensive one-shot work on entry (the CPU screen
    // caches its frequency-counter measurement here).
    static int last_rendered_screen = -1;
    const bool screen_entered = (current_screen != last_rendered_screen);
    // 切屏 = fb 被别的屏画满 → 做局部刷新的屏下次必须整屏重画
    if (screen_entered) g_fb_generation++;

    // Leaving Trigger Test in either direction → reset the adaptive
    // trigger preset to OFF and push it to the controller. Otherwise
    // the last-cycled effect (Weapon snap, Galloping pulse, etc.)
    // stays active on the DS5 indefinitely, which surprised users
    // who'd just navigated away expecting a clean slate.
    if (last_rendered_screen == kScreenTriggers
        && current_screen != kScreenTriggers) {
        trigger_preset = 0;
        send_trigger_effect(0);
    }

    // Leaving the Lightbar screen → persist mode/favorite changes made there,
    // batched into a single flash write instead of one per button press.
    if (last_rendered_screen == kScreenLightbar
        && current_screen != kScreenLightbar
        && lb_dirty) {
        lightbar_save_config();
    }

    last_rendered_screen = current_screen;

    switch (current_screen) {
        case kScreenStatus:   render_screen();           break;
        case kScreenSlots:    render_screen_slots();     break;
        case kScreenLightbar: render_screen_lightbar();  break;
        case kScreenTriggers: render_screen_triggers();  break;
        case kScreenGyro:     render_screen_gyro();      break;
        case kScreenTouchpad: render_screen_touchpad();  break;
        case kScreenDiag:     render_screen_diag();      break;
        case kScreenCpu:      render_screen_cpu(screen_entered); break;
        case kScreenRssi:     render_screen_rssi();      break;
        case kScreenVU:       render_screen_vu();        break;
        case kScreenSettings: render_screen_settings();  break;
    }
}
