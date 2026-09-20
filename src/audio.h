//
// Created by awalol on 2026/3/5.
//

#ifndef DS5_BRIDGE_AUDIO_H
#define DS5_BRIDGE_AUDIO_H

#include <cstdint>

void audio_init();
void audio_loop();
void core1_entry();
void set_headset(bool state);
void set_mic_active(bool active);
bool audio_mic_active();
void mic_add_queue(uint8_t *data, uint16_t len);
void update_mic_status();

// Accessors used by the optional OLED add-on (diag + VU meter screens).
uint8_t  audio_peak_speaker();   // 0..255; time-based release, ~256 ms to zero (pure getter)
uint8_t  audio_peak_haptic();    // 0..255; time-based release, ~256 ms to zero (pure getter)

// Byte-flow counters for the Diagnostics screen + web emulator.
uint32_t audio_usb_frames();
uint32_t audio_bt_packets();
uint32_t audio_mic_frames();   // count of mic Opus frames decoded + written
int32_t  audio_mic_last_decoded(); // last opus_decode return — neg = error, 480 = OK
uint16_t audio_mic_last_want();    // bytes asked of tud_audio_write
uint16_t audio_mic_last_wrote();   // bytes TinyUSB FIFO actually accepted
uint32_t audio_mic_decode_failures(); // opus_decode <= 0 count (bad/missing packets)
uint16_t audio_ah_out_peak();   // auto-haptics actual-output peak (0-127), 0 = not contributing

#endif //DS5_BRIDGE_AUDIO_H
