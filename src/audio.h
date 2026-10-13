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
void audio_on_bt_disconnected();

// Accessors used by the optional OLED add-on (diag + VU meter screens).
uint8_t  audio_peak_speaker();   // 0..255; time-based release, ~256 ms to zero (pure getter)
uint8_t  audio_peak_haptic();    // 0..255; time-based release, ~256 ms to zero (pure getter)

// Byte-flow counters for the Diagnostics screen + web emulator.
uint32_t audio_usb_frames();
uint32_t audio_bt_packets();
uint32_t audio_mic_frames();   // count of real mic Opus frames decoded
int32_t  audio_mic_last_decoded(); // last opus_decode return — neg = error, 480 = OK
uint16_t audio_mic_last_want();    // bytes asked of tud_audio_write
uint16_t audio_mic_last_wrote();   // bytes TinyUSB FIFO actually accepted
uint32_t audio_mic_decode_failures(); // opus_decode <= 0 count (bad/missing packets)
uint16_t audio_ah_out_peak();   // auto-haptics actual-output peak (0-127), 0 = not contributing

uint32_t audio_mic_plc_frames();
uint32_t audio_fifo_drops();
uint32_t opus_fifo_drops();
uint32_t audio_hap_drops();
uint32_t audio_c1_loops();
uint32_t audio_c1_enc();
uint32_t audio_c1_flags();     // bit0 started, bit1 encoder failed, bit2 decoder failed
uint32_t audio_c1_free_heap(); // startup free heap, bytes
uint32_t audio_c1_stage();     // 0..9; stage 3 is the port's startup delay complete
uint32_t audio_raw_push();
uint32_t audio_hap_push();

#endif //DS5_BRIDGE_AUDIO_H
