//
// Created by awalol on 2026/3/5.
//

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <malloc.h>
#include <unistd.h>
#include "audio.h"
#include "bt.h"
#if ENABLE_DEBUG
#include "debug.h"
#endif
#include "resample.h"
#include "tusb.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "opus.h"
#include "utils.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "pico/util/queue.h"
#include "pico/time.h"
#include "pico/platform.h"
#include "config.h"
#include "auto_haptics.h"
#include "loop_probe.h"   // 主循环抖动探针（Diag 屏 "Loop1" 行）

#define INPUT_CHANNELS    4
#define OUTPUT_CHANNELS   2
#define SAMPLE_SIZE       64
#define SPEAKER_OPUS_SIZE 200
#define REPORT_SIZE       547
#define REPORT_ID         0x39
// #define VOLUME_GAIN       2
// #define BUFFER_LENGTH     48
#define MIC_CHANNELS      1
#define MIC_FRAMES        480
#define MIC_OPUS_SIZE     71   // bytes per opus-encoded mic frame from the DualSense

using std::clamp;
using std::max;

static WDL_Resampler resampler;
extern uint8_t reportSeqCounter;
extern uint8_t packetCounter;
static bool plug_headset = false;
static volatile uint32_t mic_epoch = 0; // close/reopen/disconnect flush generation
static volatile bool mic_active = false; // host has opened the mic IN interface (alt != 0)
alignas(8) static uint32_t audio_core1_stack[7000];
queue_t audio_fifo; // raw pcm data
queue_t mic_fifo;
queue_t mic_decode_fifo;
queue_t audio_spk_fifo; // opus data
queue_t haptics_fifo;

struct audio_raw_element {
    float data[512 * 2];
};

struct mic_element {
    uint8_t data[MIC_OPUS_SIZE];
    uint32_t epoch;
};

struct mic_decode_element {
    int16_t data[MIC_FRAMES * MIC_CHANNELS];
    uint32_t epoch;
};

struct audio_spk_element {
    uint8_t data[SPEAKER_OPUS_SIZE];
};

struct haptics_element {
    uint8_t data[64];
};

// Mic diag counters surfaced on the OLED Diagnostics screen / web emulator
// (defined here with the decoder; guarded by our own accessors in audio.h).
static OpusDecoder *mic_decoder = nullptr; // created + owned by core1 (mic_proc)
static volatile uint32_t g_mic_frames = 0;
static volatile int32_t  g_mic_last_decoded = 0;  // opus_decode return value (core1)
static volatile uint16_t g_mic_last_want = 0;     // bytes we asked TinyUSB to send
static volatile uint16_t g_mic_last_wrote = 0;    // bytes TinyUSB accepted
uint32_t audio_mic_frames() { return g_mic_frames; }
int32_t  audio_mic_last_decoded() { return g_mic_last_decoded; }
uint16_t audio_mic_last_want()    { return g_mic_last_want; }
uint16_t audio_mic_last_wrote()   { return g_mic_last_wrote; }
static volatile uint32_t g_mic_decode_failures = 0;  // opus_decode returns <= 0 (bad/missing packets)
uint32_t audio_mic_decode_failures() { return g_mic_decode_failures; }

static constexpr int      MIC_DECODE_DEPTH  = 8;       // jitter-buffer capacity (frames)
static constexpr int      MIC_PLAYOUT_START = 3;       // pre-buffer before playout begins
static constexpr uint32_t MIC_FRAME_US      = 10000;  // 10 ms per Opus frame @ 48 kHz
static constexpr uint32_t MIC_SESSION_US    = 300000; // no real frame this long → session over
// 1 ms of 48 kHz stereo int16 = 48 frames = 192 bytes. The mic IN FIFO is only
// 4 USB packets deep (~784 B), so the old whole-frame 1920 B write was always
// truncated by TinyUSB; feeding 1 ms slices as space frees up slaves the mic to
// the host clock (upstream 370032e, written for macOS CoreAudio).
static constexpr uint32_t MIC_SLICE_FRAMES  = 48;
static constexpr uint32_t MIC_SLICE_BYTES   = MIC_SLICE_FRAMES * 2 * sizeof(int16_t);
static volatile uint32_t g_mic_last_real_us = 0;      // core1 writes, core0 reads (32-bit: no tearing)
static volatile bool     g_mic_session      = false;  // real frames arrived within MIC_SESSION_US
static volatile uint32_t g_mic_plc_frames = 0;        // concealed frames generated (Diag)
uint32_t audio_mic_plc_frames() { return g_mic_plc_frames; }


static volatile uint32_t g_drop_raw = 0, g_drop_enc = 0, g_drop_hap = 0;
static volatile uint32_t g_c1_loops = 0, g_c1_enc = 0, g_raw_push = 0, g_hap_push = 0;
uint32_t audio_fifo_drops() { return g_drop_raw; }
uint32_t opus_fifo_drops() { return g_drop_enc; }
uint32_t audio_hap_drops() { return g_drop_hap; }
uint32_t audio_c1_loops() { return g_c1_loops; }
uint32_t audio_c1_enc() { return g_c1_enc; }

extern "C" char __HeapLimit;
static uint32_t free_heap_bytes() {
    const struct mallinfo info = mallinfo();
    const char *top = static_cast<const char *>(sbrk(0));
    return static_cast<uint32_t>(info.fordblks) + static_cast<uint32_t>(&__HeapLimit - top);
}
static volatile uint32_t g_c1_flags = 0, g_c1_free_heap = 0;
static volatile uint8_t g_c1_stage = 0;
static volatile bool g_spk_ready = false;
uint32_t audio_c1_flags() { return g_c1_flags; }
uint32_t audio_c1_free_heap() { return g_c1_free_heap; }
uint32_t audio_c1_stage() { return g_c1_stage; }
uint32_t audio_raw_push() { return g_raw_push; }
uint32_t audio_hap_push() { return g_hap_push; }


// Monotonic byte-flow counters for the OLED Diagnostics screen and the web
// emulator's USB / BT rate display. Updated below.
static volatile uint32_t g_usb_frames = 0;
static volatile uint32_t g_bt_packets = 0;
uint32_t audio_usb_frames() { return g_usb_frames; }
uint32_t audio_bt_packets() { return g_bt_packets; }

// Rolling-peak meters for the OLED VU screen. Updated in audio_loop once per USB
// frame (written by core1, read by core0 — hence volatile).
//
// The release is a pure function of elapsed time (~256 ms to zero) instead of
// "12.5 % per read". Decay-on-read meant a stale peak sat frozen while nobody was looking
// at the meter — the VU screen opened showing a spike from minutes ago — and every
// extra reader (the web emulator's 0xFB diag payload reads these too) silently
// doubled the fall rate. Now the value on screen is decay(stored peak, age), so it
// expires on its own and all readers see the same number.
static volatile uint16_t g_peak_spk   = 0;
static volatile uint16_t g_peak_hap   = 0;
static volatile uint32_t g_peak_spk_t = 0;  // µs when g_peak_spk was recorded
static volatile uint32_t g_peak_hap_t = 0;

// Linear release: ~1/256 less per ~1 ms unit, fully released at ~256 ms —
// anything older reads 0, so a peak cannot survive a page that was left closed.
// Integer only (no divide): a shift, a multiply, a shift. The u >= 256 guard is
// what keeps the subtraction from going negative (and wrapping in the uint16).
static inline uint16_t peak_decay(uint16_t v, uint32_t dt_us) {
    const uint32_t u = dt_us >> 10;                 // ~1 ms units
    if (u >= 256u) return 0;                        // ≥ ~256 ms old → fully released
    return (uint16_t)(v - (uint32_t)v * u / 256u);  // u < 256 → never negative
}

uint8_t audio_peak_speaker() {
    return (uint8_t)(peak_decay(g_peak_spk, time_us_32() - g_peak_spk_t) >> 7);
}
uint8_t audio_peak_haptic() {
    return (uint8_t)(peak_decay(g_peak_hap, time_us_32() - g_peak_hap_t) >> 7);
}


void set_headset(bool state) {
    plug_headset = state;
}

// Called from tud_audio_set_itf_cb when the host opens/closes the mic IN
// interface. Gates controller-mic streaming so it only runs while recording.
void set_mic_active(bool active) {
    if (mic_active != active) ++mic_epoch;
    mic_active = active;
    update_mic_status();
}

void audio_on_bt_disconnected() {
    // Keep the host's open interface state, but discard the previous controller's audio.
    ++mic_epoch;
}

bool audio_mic_active() {
    return mic_active;
}

void update_mic_status() {
    uint8_t pkt[142]{};
    pkt[0] = 0x32;
    pkt[1] = reportSeqCounter << 4;
    reportSeqCounter = (reportSeqCounter + 1) & 0x0F;
    pkt[2] = 0x11 | 0 << 6 | 1 << 7;
    pkt[3] = 1;
    pkt[4] = (mic_active && get_config().mic_select != 3) ? 0b00000011 : 0b00000010;
    bt_write(pkt, sizeof(pkt));
}

// Re-assert the DS5 mic-enable (pkt[4] bit 0 of the 0x32 status packet) roughly
// 4x/sec while the host is recording but no mic frames have arrived yet. The
// enable is sticky, so this only runs until the stream starts (then it stops,
// saving BT traffic + DS5 battery) and resumes if the stream stalls. Mirrors the
// OLED Edition fix: without it, mic only works while something also plays audio
// (the enable otherwise only rides the audio packets, which need 2+2 queued frames).
static void mic_enable_keepalive() {
    if (!bt_is_connected() || get_config().mic_select == 3 || !mic_active) return;
    const uint64_t now = time_us_64();
    static uint32_t last_frames = 0;
    static uint64_t last_frame_us = 0;
    static uint64_t last_send_us = 0;
    const uint32_t frames = g_mic_frames;
    if (frames != last_frames) { last_frames = frames; last_frame_us = now; }
    if (last_frame_us != 0 && (now - last_frame_us) < 1000000ULL) return; // streaming -> sticky
    if (last_send_us != 0 && (now - last_send_us) < 250000ULL) return;    // ~4 Hz while arming
    last_send_us = now;
    update_mic_status();
}

void __not_in_flash_func(audio_bt_task)() {
    const Config_body &cfg = get_config();
    const bool mic_enabled = mic_active && cfg.mic_select != 3;
#if !DISABLE_SPEAKER_PROC
    const bool speaker_enabled = g_spk_ready && cfg.speaker_select != 3;
#endif

    if (queue_get_level(&haptics_fifo) < 2) {
        return;
    }
#if !DISABLE_SPEAKER_PROC
    if (speaker_enabled && queue_get_level(&audio_spk_fifo) < 2) {
        return;
    }
#endif

    uint8_t pkt[REPORT_SIZE]{};
    pkt[0] = REPORT_ID;
    pkt[1] = reportSeqCounter << 4;
    reportSeqCounter = (reportSeqCounter + 1) & 0x0F;
    pkt[2] = 0x11 | 0 << 6 | 1 << 7;
    pkt[3] = 6;
    pkt[4] = mic_enabled ? 0b01111111 : 0b01111110;
    // byte 4 研究
    // bit 6 是必须的
    // 其余 bit 每多设置一个为0，就需要将pkt[3] - 1，然后将下面这些缩短一个字节的数据。
    // 最终实测，可以只保留一个 buf_len + packetCounter
    // pkt 5-7 的注释是根据 Nielk1 采样到的数据进行猜测。但是实际上修改还是发现有任何效果
    const auto buf_len = cfg.audio_buffer_length;
    pkt[5] = buf_len; // VolumeHeadphones - guess but no work
    pkt[6] = buf_len; // VolumeMic - guess but no work
    pkt[7] = buf_len; // VolumeSpeaker - guess but no work
    pkt[8] = buf_len; // AudioBufferLength
    pkt[9] = packetCounter += 2;
    pkt[10] = 0x12 | 1 << 6 | 1 << 7;
    pkt[11] = SAMPLE_SIZE;
    static haptics_element haptics_pb{};
    if (queue_get_level(&haptics_fifo) >= 2) {
        if (queue_try_remove(&haptics_fifo, &haptics_pb)) {
            memcpy(pkt + 12, haptics_pb.data,SAMPLE_SIZE);
        } else {
            printf("[Audio] Warning: Haptics queue remove failed\n");
        }
        if (queue_try_remove(&haptics_fifo, &haptics_pb)) {
            memcpy(pkt + 12 + SAMPLE_SIZE, haptics_pb.data,SAMPLE_SIZE);
        } else {
            printf("[Audio] Warning: Haptics queue remove failed\n");
        }
    }
#if !DISABLE_SPEAKER_PROC
    if (speaker_enabled) {
        pkt[140] = ((
            cfg.speaker_select == 2 || // lock headphone
            (cfg.speaker_select == 0 && plug_headset) // auto
            ) ? 0x16 : 0x13) | 1 << 6 | 1 << 7;
        pkt[141] = SPEAKER_OPUS_SIZE;
        static audio_spk_element spk_pb{};
        if (queue_get_level(&audio_spk_fifo) >= 2) {
            if (queue_try_remove(&audio_spk_fifo, &spk_pb)) {
                memcpy(pkt + 142, spk_pb.data,SPEAKER_OPUS_SIZE);
            } else {
                printf("[Audio] Warning: Speaker queue remove failed\n");
            }
            if (queue_try_remove(&audio_spk_fifo, &spk_pb)) {
                memcpy(pkt + 142 + SPEAKER_OPUS_SIZE, spk_pb.data,SPEAKER_OPUS_SIZE);
            } else {
                printf("[Audio] Warning: Speaker queue remove failed\n");
            }
        }
    }
#endif
    bt_write(pkt, sizeof(pkt));
    g_bt_packets++;
}

// Mic out: feed decoded PCM to the USB IN FIFO in 1 ms slices whenever it has
// room, so delivery follows the host's clock instead of our own timer. Pre-
// buffers MIC_PLAYOUT_START frames per session to absorb BT jitter; on underrun
// writes a silent slice so the host stream never starves (CoreAudio resets on
// starvation). Silence also covers BT Mic off while the host has the mic open.
static void __not_in_flash_func(mic_drain)() {
    static bool started = false;
    static bool have = false;
    static uint32_t offset = 0;
    static mic_decode_element frame{};

    static uint32_t seen_epoch = 0;
    const uint32_t epoch = mic_epoch;
    if (seen_epoch != epoch) { started = false; have = false; seen_epoch = epoch; }
    if (!mic_active || get_config().mic_select == 3) { started = false; have = false; return; }
    tu_fifo_t *ff = tud_audio_get_ep_in_ff();
    if (ff == nullptr) return;

    if (!started && queue_get_level(&mic_decode_fifo) >= MIC_PLAYOUT_START) started = true;
    if (started && !g_mic_session && queue_is_empty(&mic_decode_fifo) && !have) {
        started = false; // session over — re-buffer when the mic comes back
    }

    while (tu_fifo_remaining(ff) >= MIC_SLICE_BYTES) {
        static int16_t slice[MIC_SLICE_FRAMES * 2];
        if (!have) {
            if (started && queue_try_remove(&mic_decode_fifo, &frame) && frame.epoch == epoch) {
                have = true;
                offset = 0;
            } else {
                memset(slice, 0, sizeof(slice));
                tud_audio_write(slice, MIC_SLICE_BYTES);
                break;
            }
        }
        for (uint32_t i = 0; i < MIC_SLICE_FRAMES; i++) {
            const int16_t s = frame.data[offset + i];
            slice[i * 2]     = s; // mono duplicated to L/R — our UAC1 mic is 2-ch like a real DS5
            slice[i * 2 + 1] = s;
        }
        g_mic_last_wrote = tud_audio_write(slice, MIC_SLICE_BYTES);
        g_mic_last_want  = MIC_SLICE_BYTES;
        offset += MIC_SLICE_FRAMES;
        if (offset >= MIC_FRAMES) have = false;
    }
}


void __not_in_flash_func(audio_loop)() {
    const Config_body &cfg = get_config();
    const bool speaker_enabled = g_spk_ready && cfg.speaker_select != 3;

    mic_drain();

    audio_bt_task();

    // 1. 读取 USB 音频数据
    if (!tud_audio_available()) {
        // Keep the DS5 mic streaming even without output audio — but ONLY once the
        // host has enumerated us (tud_mounted). Running it during the fresh-pair
        // feature handshake floods BT TX and delays controller-type detection past
        // the connection watchdog's timeout (~10-15s "shutdown" on fresh pair).
        if (tud_mounted()) mic_enable_keepalive();
        return;
    }

    int16_t raw[192];
    uint32_t bytes_read = tud_audio_read(raw, sizeof(raw)); // 每次读入 384 bytes
    int frames = bytes_read / (INPUT_CHANNELS * sizeof(int16_t));
    if (frames == 0) {
        return;
    }
    g_usb_frames += (uint32_t) frames;

    static float audio_buf[512 * 2];
    static uint audio_buf_pos = 0;
    WDL_ResampleSample *in_buf;
    int nframes = resampler.ResamplePrepare(frames, OUTPUT_CHANNELS, &in_buf);

    // OLED 音量（Spk Vol）走软件增益 —— fork 的原有做法（上游没有这个菜单，这行在他们那儿
    // 是注释的）。字段与上游同单位（[0,127]，100 = 0dB），这里换算成线性增益；主机静音时归零。
    // 主机自己的音量走另一条路（SET_CUR → 0x32 推给手柄），两条互不干扰 —— 与 fork 一致。
    extern uint8_t mute[2]; // usb.cpp 定义（主机 SET_CUR 的静音状态）
    const float audio_gain = mute[0] ? 0.0f
                                     : powf(10.0f, ((int) cfg.speaker_volume - 100) / 20.0f);
    const float haptics_gain = cfg.haptics_gain;
    const float haptics_gain_right = cfg.haptics_gain_right;
#if !DISABLE_SPEAKER_PROC
    if (!speaker_enabled) {
        audio_buf_pos = 0;
        while (queue_try_remove(&audio_fifo, NULL)) {
        }
    }
#endif
    // Peak meters: start from the *decayed* stored peak (not the raw stored one),
    // so a new, smaller peak can still take over once the old big one has aged out.
    const uint32_t peak_now = time_us_32();
    const uint16_t peak_base_spk = peak_decay(g_peak_spk, peak_now - g_peak_spk_t);
    const uint16_t peak_base_hap = peak_decay(g_peak_hap, peak_now - g_peak_hap_t);
    uint16_t spk_max = peak_base_spk;
    uint16_t hap_max = peak_base_hap;
    uint16_t native_max = 0;  // 本帧 ch3/ch4 实际峰值（Fallback 静默判断用，不继承 VU 显示缓存）

    // ---- Audio Auto Haptics（实现见 auto_haptics.cpp；sample() 头内联保热路径）----
    g_auto_haptics.begin();
    for (int i = 0; i < nframes; i++) {
        // VU peak tracking
        {
            int16_t sl = raw[i * INPUT_CHANNELS];
            int16_t sr = raw[i * INPUT_CHANNELS + 1];
            int16_t hl = raw[i * INPUT_CHANNELS + 2];
            int16_t hr = raw[i * INPUT_CHANNELS + 3];
            uint16_t a = (uint16_t)(sl < 0 ? -sl : sl);
            uint16_t b = (uint16_t)(sr < 0 ? -sr : sr);
            if (a > spk_max) spk_max = a;
            if (b > spk_max) spk_max = b;
            a = (uint16_t)(hl < 0 ? -hl : hl);
            b = (uint16_t)(hr < 0 ? -hr : hr);
            if (a > hap_max) hap_max = a;
            if (b > hap_max) hap_max = b;
            if (a > native_max) native_max = a;
            if (b > native_max) native_max = b;
        }

#if !DISABLE_SPEAKER_PROC
        if (speaker_enabled) {
            audio_buf[audio_buf_pos++] = clamp(raw[i * INPUT_CHANNELS] / 32768.0f * audio_gain, -1.0f, 1.0f);
            audio_buf[audio_buf_pos++] = clamp(raw[i * INPUT_CHANNELS + 1] / 32768.0f * audio_gain, -1.0f, 1.0f);
            if (audio_buf_pos == 512 * 2) {
                static audio_raw_element element{};
                memcpy(element.data, audio_buf, 512 * 2 * 4);
                if (queue_is_full(&audio_fifo)) {
                    queue_try_remove(&audio_fifo, NULL);
                    g_drop_raw++;
                }
                g_raw_push++;
                if (!queue_try_add(&audio_fifo, &element)) {
                    printf("[Audio] Warning: audio_fifo add failed\n");
                }
                audio_buf_pos = 0;
            }
        }
#endif
        // 上游 a55fd46：haptics 增益改到重采样之后再施加（见下面的 int8 转换）。
        float h_l = raw[i * INPUT_CHANNELS + 2] / 32768.0f;
        float h_r = raw[i * INPUT_CHANNELS + 3] / 32768.0f;

        g_auto_haptics.sample(raw[i * INPUT_CHANNELS] / 32768.0f,
                              raw[i * INPUT_CHANNELS + 1] / 32768.0f, h_l, h_r);

        in_buf[i * 2]     = static_cast<WDL_ResampleSample>(clamp(h_l, -1.0f, 1.0f));
        in_buf[i * 2 + 1] = static_cast<WDL_ResampleSample>(clamp(h_r, -1.0f, 1.0f));
    }
    // 只有真的出现更高峰值才刷新（并盖时间戳）；否则保持原值+原时间戳不动，
    // 让它按年龄自然过期——避免"旧的大峰值"永久压制后续的小峰值。
    if (spk_max > peak_base_spk) { g_peak_spk = spk_max; g_peak_spk_t = peak_now; }
    if (hap_max > peak_base_hap) { g_peak_hap = hap_max; g_peak_hap_t = peak_now; }
    g_auto_haptics.end(native_max);

    // 3. 48kHz -> 3kHz 重采样
    static WDL_ResampleSample out_buf[SAMPLE_SIZE]; // 64 floats = 32帧 × 2ch
    const int out_frames = resampler.ResampleOut(out_buf, nframes, nframes / 4, OUTPUT_CHANNELS);

    static int8_t haptic_buf[SAMPLE_SIZE];
    static int haptic_buf_pos = 0;

    // 4. 转换为int8并缓冲，满64字节即组包发送
    for (int i = 0; i < out_frames; i++) {
        int val_l = static_cast<int>(out_buf[i * 2] * 127.0f * haptics_gain);
        int val_r = static_cast<int>(out_buf[i * 2 + 1] * 127.0f * haptics_gain_right);
        haptic_buf[haptic_buf_pos++] = static_cast<int8_t>(clamp(val_l, -128, 127));
        haptic_buf[haptic_buf_pos++] = static_cast<int8_t>(clamp(val_r, -128, 127));

        if (haptic_buf_pos != SAMPLE_SIZE) {
            continue;
        }
        static haptics_element element{};
        memcpy(element.data, haptic_buf,SAMPLE_SIZE);
        if (queue_is_full(&haptics_fifo)) {
            queue_try_remove(&haptics_fifo, NULL);
            g_drop_hap++;
        }
        g_hap_push++;
        if (!queue_try_add(&haptics_fifo, &element)) {
            printf("[Audio] Warning: haptics_fifo add failed\n");
        }
        haptic_buf_pos = 0;
    }
}

void audio_init() {
    resampler.SetMode(true, 0, false);
    // 在有线连接的 DS5，其内部的 hd 震动也是工作在 3000Hz 的音频
    // 怎么发现的呢？打开频率发生器，发现在 0-3000 以及 3000-6000
    // 以及往后相同范围的区域，声音都是先升后降
    // 因此推断，DS5内部也是 3000Hz 的工作频率，并且没有低通滤波
    // 但是SetStateData有一个开关？还没进行测试
    // ------
    // resampler.SetMode(true, 2, false);
    // resampler.SetFilterParms(0.85f, 0.707f);
    // ------
    resampler.SetRates(48000, 3000);
    resampler.SetFeedMode(true);
    resampler.Prealloc(2, 48, 4);
    queue_init(&haptics_fifo, sizeof(haptics_element), 2);
    // Mic queues are read from audio_loop on core0 every iteration, so they
    // must exist regardless of the speaker-proc build flag.
    //
    // BUGFIX: ELASTIC BUFFER EXTENSION:
    // Increased microphone queues to depth 8 (~80ms buffer size).
    // This absorbs initial Opus encoder/decoder warm-up delays and mitigates
    // startup crackling/stuttering under Core 1 task schedulers.
    // queue_init(&mic_fifo, sizeof(mic_element), 2);
    // queue_init(&mic_decode_fifo, sizeof(mic_decode_element), 2);
    queue_init(&mic_fifo, sizeof(mic_element), 8);
    queue_init(&mic_decode_fifo, sizeof(mic_decode_element), MIC_DECODE_DEPTH);
#if !DISABLE_SPEAKER_PROC
    queue_init(&audio_fifo, sizeof(audio_raw_element), 2);
    queue_init(&audio_spk_fifo, sizeof(audio_spk_element), 2);
#endif
#if ENABLE_DEBUG
    // 通常 stack 最大使用 25836 bytes 即 stack[6459]
    debug_fill_core1_stack_watermark(audio_core1_stack,
                                     sizeof(audio_core1_stack) / sizeof(audio_core1_stack[0]));
#endif
    // The microphone decoder needs core1 even when speaker encoding is disabled.
    multicore_launch_core1_with_stack(core1_entry, audio_core1_stack, sizeof(audio_core1_stack));
}

#if !DISABLE_SPEAKER_PROC
static OpusEncoder *encoder;
static WDL_Resampler resampler_audio;

// Speaker path: USB OUT PCM (core0 audio_fifo) -> resample -> opus encode ->
// opus_buf for the haptics/speaker BT report. Non-blocking so core1 can also
// service the mic path. Kept in RAM to remove XIP miss latency from the loop.
static void __not_in_flash_func(speaker_proc)() {
    static audio_raw_element audio_element{};
    if (!queue_try_remove(&audio_fifo, &audio_element)) {
        return;
    }
    if (get_config().speaker_select == 3) {
        return;
    }
    // 将 512 frames 重采样成 480 frames 以解决噪音问题。感谢 @Junhoo
    WDL_ResampleSample *in_buf;
    int nframes = resampler_audio.ResamplePrepare(512, 2, &in_buf);
    for (int i = 0; i < nframes * 2; i++) {
        in_buf[i] = audio_element.data[i];
    }
    static WDL_ResampleSample out_buf[480 * 2];
    resampler_audio.ResampleOut(out_buf, nframes, 480, 2);

    static uint8_t out[SPEAKER_OPUS_SIZE];
    const int encoded_len = opus_encode_float(encoder, out_buf, 480, out, sizeof(out));
    if (encoded_len <= 0) {
#if ENABLE_VERBOSE
        printf("[Audio] OpusEncoder encode failed: %d\n", encoded_len);
#endif
        return;
    }

    static audio_spk_element spk_ele{};
    g_c1_enc++;
    memcpy(spk_ele.data, out, encoded_len);
    if (encoded_len < (int) sizeof(spk_ele.data)) {
        memset(spk_ele.data + encoded_len, 0, sizeof(spk_ele.data) - encoded_len);
    }
    if (queue_is_full(&audio_spk_fifo)) {
        queue_try_remove(&audio_spk_fifo, NULL);
        g_drop_enc++;
    }
    if (!queue_try_add(&audio_spk_fifo, &spk_ele)) {
        printf("[Audio] Warning: audio_spk_fifo add failed\n");
    }
}

#endif // !DISABLE_SPEAKER_PROC

static void __not_in_flash_func(mic_push_decoded)(const mic_decode_element &dec) {
    if (queue_is_full(&mic_decode_fifo)) queue_try_remove(&mic_decode_fifo, NULL);
    queue_try_add(&mic_decode_fifo, &dec);
}

// Mic path: Opus from the controller (mic_fifo) -> decode -> jitter buffer.
// If real packets stop mid-session, conceal one frame per missed 10 ms slot
// via Opus PLC; after MIC_SESSION_US of silence the session is over.
static bool __not_in_flash_func(mic_proc)() {
    static mic_element pkt{};
    static mic_decode_element dec{};
    static uint32_t last_out_us = 0;
    static uint32_t seen_epoch = 0;
    const uint32_t epoch = mic_epoch;
    if (seen_epoch != epoch) {
        opus_decoder_ctl(mic_decoder, OPUS_RESET_STATE);
        seen_epoch = epoch;
        g_mic_session = false;
        while (queue_try_remove(&mic_decode_fifo, nullptr)) {}
    }
    if (!mic_active || get_config().mic_select == 3 || !bt_is_connected()) {
        g_mic_session = false;
        while (queue_try_remove(&mic_fifo, nullptr)) {}
        while (queue_try_remove(&mic_decode_fifo, nullptr)) {}
        return false;
    }
    const uint32_t now = time_us_32();

    if (queue_try_remove(&mic_fifo, &pkt)) {
        if (pkt.epoch != epoch) return true;
        const int n = opus_decode(mic_decoder, pkt.data, MIC_OPUS_SIZE, dec.data, MIC_FRAMES, 0);
        g_mic_last_decoded = n; // observed in OLED Diag
        if (n > 0) {
            if (n < MIC_FRAMES) memset(dec.data + n, 0, (MIC_FRAMES - n) * sizeof(int16_t));
            dec.epoch = epoch;
            mic_push_decoded(dec);
            g_mic_last_real_us = now;
            g_mic_session = true;
            g_mic_frames++;
            last_out_us = now;
        } else {
            g_mic_decode_failures++;
        }
        return true;
    }
    if (!g_mic_session) return false;
    if (now - g_mic_last_real_us >= MIC_SESSION_US) {
        g_mic_session = false;
        return false;
    }
    // Half a frame of slack before concealing, so normal BT jitter isn't
    // mistaken for loss.
    if (now - last_out_us >= MIC_FRAME_US + MIC_FRAME_US / 2) {
        const int n = opus_decode(mic_decoder, nullptr, 0, dec.data, MIC_FRAMES, 0); // PLC
        if (n > 0) {
            if (n < MIC_FRAMES) memset(dec.data + n, 0, (MIC_FRAMES - n) * sizeof(int16_t));
            dec.epoch = epoch;
            mic_push_decoded(dec);
            g_mic_plc_frames++;
        }
        last_out_us += MIC_FRAME_US;
        return true;
    }
    return false;
}

void __not_in_flash_func(core1_entry)() {
    g_c1_stage = 1;
    flash_safe_execute_core_init();
    g_c1_stage = 2;
    g_c1_flags |= 0x01;
    // Preserve this port's Bluetooth/USB startup delay.
    sleep_ms(300);
    g_c1_stage = 3; // startup delay complete; this port has no perf cycle counter
    g_c1_free_heap = free_heap_bytes();
    int error = 0;
#if !DISABLE_SPEAKER_PROC
    g_c1_stage = 4;
    encoder = opus_encoder_create(48000, 2, OPUS_APPLICATION_AUDIO, &error);
    g_c1_stage = 5;
    if (error != 0 || encoder == nullptr) {
        printf("[Audio] OpusEncoder create failed (err=%d): speaker off\n", error);
        encoder = nullptr;
        g_c1_flags |= 0x02;
    } else {
        opus_encoder_ctl(encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
        opus_encoder_ctl(encoder, OPUS_SET_BITRATE(200 * 8 * 100));
        opus_encoder_ctl(encoder, OPUS_SET_VBR(false));
        opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(0));
        resampler_audio.SetMode(true, 0, false);
        resampler_audio.SetRates(51200, 48000);
        resampler_audio.SetFeedMode(true);
        resampler_audio.Prealloc(2, 512, 480);
        g_spk_ready = true;
    }
    g_c1_stage = 6;
#endif
    g_c1_stage = 7;
    mic_decoder = opus_decoder_create(48000, MIC_CHANNELS, &error);
    g_c1_stage = 8;
    if (error != 0 || mic_decoder == nullptr) {
        printf("[Audio] OpusDecoder create failed\n");
        mic_decoder = nullptr;
        g_c1_flags |= 0x04;
    }
    // A stopped core1 cannot participate in flash lockout: always keep looping.
    g_c1_stage = 9;

    static LoopProbe probe_core1;

    while (true) {
        // 抖动探针（core1）：量相邻两轮之间最大间隔。空闲时是 ~10 µs 的自旋，
        // 有音频时一轮 = 一次 opus 编/解码，被 core0 的 flash 擦写停车则会出现
        // 几十 ms 的尖峰 —— 这些都会体现在 Diag 屏的 Loop1 行。见 loop_probe.h。
        loop_probe_tick(probe_core1, 1);
        g_c1_loops++;

        bool work_done = false;

        // Only enter processing if data is actually waiting.
        // This avoids constantly acquiring queue locks (spinlocks) when idle,
        // which would otherwise thrash the RP2350 system bus and starve Core 0.
#if !DISABLE_SPEAKER_PROC
        if (g_spk_ready && queue_get_level(&audio_fifo) > 0) {
            speaker_proc();
            work_done = true;
        }
#endif
        if (mic_decoder != nullptr && mic_proc()) work_done = true;
        
        // If both queues are empty, we can safely sleep.
        // This prevents 100% CPU usage while maintaining sub-millisecond response times.
        if (!work_done) {
            sleep_us(10); 
        }
    }
}

// data points at the opus mic payload, len is the bytes available there.
// In RAM (consistent with the BT-receive path) and validates len so a short
// or malformed report can't over-read past the packet buffer.
void __not_in_flash_func(mic_add_queue)(uint8_t *data, uint16_t len) {
    if (!mic_active || get_config().mic_select == 3) return;
    if (len < MIC_OPUS_SIZE) return;
    static mic_element mic_packet{};
    mic_packet.epoch = mic_epoch;
    memcpy(mic_packet.data, data, MIC_OPUS_SIZE);
    if (queue_is_full(&mic_fifo)) {
        queue_try_remove(&mic_fifo, NULL);
    }
    queue_try_add(&mic_fifo, &mic_packet);
}
