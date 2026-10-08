#include "xbox_protocol.h"
#include <algorithm>
#include <cstring>

namespace xbox {
namespace {
void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
uint16_t get16(const uint8_t *p) { return p[0] | (uint16_t(p[1]) << 8); }
bool varint(const uint8_t *data, size_t len, size_t &pos, uint32_t &value) {
    value = 0;
    for (unsigned shift = 0; shift < 28 && pos < len; shift += 7) {
        const uint8_t b = data[pos++];
        value |= uint32_t(b & 0x7f) << shift;
        if (!(b & 0x80)) return true;
    }
    return false;
}
// Build the MS-GIPUSB gamepad metadata, with the Share interface extension.
// Firmware version 1.0 deliberately differs from retail Series metadata caches.
struct Metadata {
    std::array<uint8_t, 256> data{};
    size_t size = 38; // 16-byte header + offsets (16) + reserved (6)
    Metadata() {
        put16(data.data(), 16);
        data[2] = 1;
        auto offset = [&](unsigned field) { put16(data.data() + 16 + field * 2, size - 16); };
        auto byte = [&](uint8_t b) { data[size++] = b; };
        offset(1); byte(1); byte(1); byte(0); byte(0); byte(0); // firmware 1.0
        offset(2); byte(0); // no audio
        offset(3); for (uint8_t b : {6, 1, 2, 3, 4, 6, 7}) byte(b);
        offset(4); for (uint8_t b : {5, 1, 4, 5, 6, 10}) byte(b);
        offset(5); byte(1);
        constexpr char name[] = "Windows.Xbox.Input.Gamepad";
        byte(sizeof(name) - 1); byte(0);
        for (size_t i = 0; i < sizeof(name) - 1; ++i) byte(name[i]);
        offset(6); byte(5);
        constexpr uint8_t guids[][16] = {
            {0x56,0xff,0x76,0x97,0xfd,0x9b,0x81,0x45,0xad,0x45,0xb6,0x45,0xbb,0xa5,0x26,0xd6},
            {0x2c,0x40,0x2e,0x08,0xdf,0x07,0xe1,0x45,0xa5,0xab,0xa3,0x12,0x7a,0xf1,0x97,0xb5},
            {0xe7,0x1f,0xf3,0xb8,0x86,0x73,0xe9,0x40,0xa9,0xf8,0x2f,0x21,0x26,0x3a,0xcf,0xb7},
            {0xfe,0xd2,0xdd,0xec,0x87,0xd3,0x94,0x42,0xbd,0x96,0x1a,0x71,0x2e,0x3d,0xc7,0x7d},
            // MS-GIPUSB 5.1: Windows USB security opt-out. Required for this
            // PC-only controller to enumerate without an accessory security chip.
            {0x77,0xce,0x34,0x7a,0xe2,0x7d,0xc6,0x45,0x8c,0xa4,0x00,0x42,0xc0,0x8b,0xd9,0x4a},
        };
        for (const auto &guid : guids) for (uint8_t b : guid) byte(b);
        offset(0); byte(2);
        for (uint8_t type : {0x20, 0x09}) {
            const size_t start = size;
            byte(23); byte(0); byte(type); byte(type == 0x20 ? input_size : 9);
            byte(0); byte(1); byte(0); byte(type == 0x20 ? 0x10 : 0x08);
            while (size < start + 23) byte(0);
        }
        put16(data.data() + 14, size); // metadata binary blob size, per Microsoft example
    }
};
const Metadata meta;
}

void make_device_id(uint32_t boot_random, uint8_t out[8]) {
    // MS-GIPUSB 2.2.1.3: 0x0000FFFB followed by a random 32-bit boot ID.
    // Hello and USB serial use this same ID, encoded little endian on the wire.
    for (unsigned i = 0; i < 4; ++i) out[i] = boot_random >> (8 * i);
    out[4] = 0xfb; out[5] = 0xff; out[6] = out[7] = 0;
}

Input map_input(const uint8_t *ds, size_t len) {
    Input out{};
    if (len < 10) return out;
    // Cross/Circle/Square/Triangle -> A/B/X/Y. Create -> View, Mute -> Share.
    out[0] = ((ds[7] & 0x20) >> 1) | ((ds[7] & 0x40) >> 1)
           | ((ds[7] & 0x10) << 2) | (ds[7] & 0x80)
           | ((ds[8] & 0x10) >> 1) | ((ds[8] & 0x20) >> 3);
    constexpr uint8_t hats[] = {1,9,8,10,2,6,4,5,0};
    out[1] = ((ds[8] & 3) << 4) | (ds[8] & 0xc0);
    const uint8_t hat = ds[7] & 0x0f;
    if (hat < sizeof(hats)) out[1] |= hats[hat];
    put16(out.data() + 2, (uint32_t(ds[4]) * 1023 + 127) / 255);
    put16(out.data() + 4, (uint32_t(ds[5]) * 1023 + 127) / 255);
    auto axis = [](uint8_t b, bool invert) -> int16_t {
        // Preserve exact zero at 128 and both signed endpoints.
        int32_t v = b < 128 ? (int32_t(b) - 128) * 256 : (int32_t(b) - 128) * 32767 / 127;
        if (invert) v = v == -32768 ? 32767 : (v == 32767 ? -32768 : -v);
        return int16_t(v);
    };
    put16(out.data() + 6, axis(ds[0], false));
    put16(out.data() + 8, axis(ds[1], true));
    put16(out.data() + 10, axis(ds[2], false));
    put16(out.data() + 12, axis(ds[3], true));
    out[14] = (ds[9] & 4) ? 1 : 0;
    return out;
}

void trigger_effect(uint8_t percent, uint8_t out[11]) {
    std::memset(out, 0, 11);
    if (!percent) { out[0] = 0x05; return; }
    const uint8_t amplitude = (std::min<unsigned>(percent, 100) * 8 + 99) / 100;
    uint32_t zones = 0;
    for (unsigned i = 0; i < 10; ++i) zones |= uint32_t(amplitude - 1) << (3 * i);
    out[0] = 0x26; out[1] = 0xff; out[2] = 3; // vibrate throughout trigger travel
    for (unsigned i = 0; i < 4; ++i) out[3 + i] = zones >> (8 * i);
    out[9] = 40; // Xbox PWM has no frequency field; use a fixed 40 Hz carrier
}

bool Rumble::command(const uint8_t *p, size_t len, uint32_t now) {
    if (len != 9 || p[0] != 0 || (p[1] & 0xf0)) return false;
    if (!p[6]) { stop(); return true; } // duration zero cancels ALL motors
    constexpr uint8_t masks[] = {8,4,2,1};
    for (unsigned i = 0; i < 4; ++i) if (p[1] & masks[i]) {
        motors_[i] = {uint8_t(std::min<unsigned>(p[2 + i], 100)), now,
                      uint16_t(p[6] * 10), uint16_t(p[7] * 10), uint16_t(p[8] + 1)};
    }
    return true;
}
Motors Rumble::tick(uint32_t now) {
    Motors out{};
    for (unsigned i = 0; i < 4; ++i) {
        auto &m = motors_[i];
        if (!m.cycles) continue;
        const uint32_t elapsed = now - m.start;
        const uint32_t period = m.duration + m.delay;
        if (elapsed >= period * m.cycles) { m = {}; continue; }
        if (elapsed % period < m.duration) out[i] = m.level;
    }
    return out;
}
void Rumble::stop() { motors_ = {}; }

const std::array<uint8_t,256> &Protocol::metadata() { return meta.data; }
size_t Protocol::metadata_size() { return meta.size; }
uint8_t Protocol::next(uint8_t &seq) { if (++seq == 0) ++seq; return seq; }
bool Protocol::send(uint8_t type, uint8_t flags, uint8_t &counter, const uint8_t *p, size_t len) {
    std::array<uint8_t,64> packet{};
    if (len > 60) return false;
    uint8_t seq = counter;
    next(seq);
    packet[0] = type; packet[1] = flags; packet[2] = seq; packet[3] = len;
    if (len) std::memcpy(packet.data() + 4, p, len);
    if (!send_(context_, packet.data(), len + 4)) return false;
    counter = seq;
    return true;
}
void Protocol::stop_motors() {
    rumble_.stop();
    // Always send a stop, including when leaving DS mode with a native effect active.
    applied_ = {};
    apply_(context_, applied_);
}
void Protocol::reset(const uint8_t device_id[8], uint32_t now) {
    stop_motors();
    std::memcpy(id_.data(), device_id, 8);
    state_ = State::Arrival;
    command_seq_ = input_seq_ = metadata_seq_ = 0;
    metadata_offset_ = metadata_sent_ = 0;
    metadata_wait_ = metadata_complete_ = false;
    ack_head_ = ack_count_ = 0;
    hello_sent_ = input_sent_ = guide_ = false;
    guide_wait_ = false; guide_retries_ = guide_seq_ = 0; guide_time_ = now;
    status_pending_ = false;
    hello_time_ = idle_time_ = status_time_ = started_time_ = input_time_ = now;
}

void Protocol::receive(const uint8_t *data, size_t len, uint32_t now) {
    // USB may concatenate messages. Decode bounded LEB128 fields before accessing payloads.
    while (len >= 4) {
        size_t pos = 3;
        uint32_t size, offset = 0;
        if (!varint(data, len, pos, size)) return;
        if ((data[1] & 0x80) && !varint(data, len, pos, offset)) return;
        if (size > len - pos) return;
        const uint8_t *p = data + pos;
        const uint8_t flags = data[1];
        // Only client zero; no audio, expansion devices or security passthrough.
        if ((flags & 0x0f) == 0 && !(flags & 0x80)) {
            if ((flags & 0x10) && ack_count_ < acks_.size()) {
                auto &a = acks_[(ack_head_ + ack_count_++) % acks_.size()].data;
                a = {1,0x20,data[2],9,0,data[0],uint8_t(flags & 0x20),0,0,0,0,0,0};
                put16(a.data() + 7, size);
            }
            if (data[0] == 1 && (flags & 0x20) && size == 9 && p[0] == 0 && p[1] == 7
                && data[2] == guide_seq_) {
                guide_wait_ = false;
            } else if (data[0] == 1 && (flags & 0x20) && size == 9 && p[0] == 0 && p[1] == 4
                && (p[2] & 0x20) && data[2] == metadata_seq_ && state_ == State::Metadata) {
                const size_t received = get16(p + 3);
                if (received <= metadata_sent_) {
                    metadata_offset_ = received;
                    metadata_wait_ = false;
                    metadata_complete_ = received == meta.size;
                    metadata_time_ = now;
                }
            } else if (data[0] == 4 && (flags & 0x20) && size == 0) {
                stop_motors();
                state_ = State::Metadata;
                metadata_seq_ = next(command_seq_);
                metadata_offset_ = metadata_sent_ = 0;
                metadata_wait_ = metadata_complete_ = false;
                metadata_time_ = now;
            } else if (data[0] == 5 && (flags & 0x20) && size >= 1) {
                switch (p[0]) {
                    case 0:
                        state_ = State::Active;
                        status_pending_ = true; input_sent_ = false; guide_ = false;
                        started_time_ = status_time_ = now; guide_wait_ = false;
                        break;
                    case 1: state_ = State::Idle; metadata_complete_ = false; stop_motors(); break;
                    case 4: state_ = State::Off; status_pending_ = true; stop_motors(); break;
                    case 5: stop_motors(); break;
                    case 7: reset(id_.data(), now); break;
                    default: break;
                }
            } else if (data[0] == 9 && !(flags & 0x20) && active()) {
                rumble_.command(p, size, now);
            }
        }
        const size_t consumed = pos + size;
        data += consumed; len -= consumed;
    }
}

bool Protocol::metadata_packet(uint32_t now) {
    if (metadata_wait_) {
        if (now - metadata_time_ >= 1000) {
            state_ = State::Arrival; hello_sent_ = false;
        }
        return false;
    }
    std::array<uint8_t,64> packet{};
    const size_t offset = metadata_offset_;
    const size_t count = std::min<size_t>(58, meta.size - offset);
    const bool first = offset == 0;
    const bool last = offset + count == meta.size;
    packet[0] = 4;
    packet[1] = metadata_complete_ ? 0xa0 : (first ? 0xf0 : (last ? 0xb0 : 0xa0));
    packet[2] = metadata_seq_;
    packet[3] = metadata_complete_ ? 0 : count;
    const size_t tlo = first || metadata_complete_ ? meta.size : offset;
    packet[4] = (tlo & 0x7f) | 0x80; packet[5] = tlo >> 7;
    if (!metadata_complete_) std::memcpy(packet.data() + 6, meta.data.data() + offset, count);
    if (!send_(context_, packet.data(), metadata_complete_ ? 6 : count + 6)) return false;
    if (metadata_complete_) {
        state_ = State::Idle; idle_time_ = now; input_sent_ = false;
    } else {
        metadata_offset_ += count;
        metadata_sent_ = metadata_offset_;
        metadata_wait_ = first || last;
        metadata_time_ = now;
    }
    return true;
}

void Protocol::tick(uint32_t now, const uint8_t *ds, size_t len) {
    const Motors motors = active() ? rumble_.tick(now) : Motors{};
    if (motors != applied_) { apply_(context_, motors); applied_ = motors; }
    if (ack_count_) {
        const auto &a = acks_[ack_head_].data;
        if (send_(context_, a.data(), a.size())) {
            ack_head_ = (ack_head_ + 1) % acks_.size(); --ack_count_;
        }
        return;
    }
    if (state_ == State::Arrival) {
        if (hello_sent_ && now - hello_time_ < 500) return;
        uint8_t hello[28]{};
        std::memcpy(hello, id_.data(), 8);
        put16(hello + 8, vendor_id); put16(hello + 10, product_id);
        put16(hello + 12, 1); // metadata firmware 1.0
        hello[20] = 1; hello[22] = 1; hello[24] = 1; hello[26] = 1;
        if (send(2,0x20,command_seq_,hello,sizeof(hello))) {
            hello_sent_ = true; hello_time_ = now;
        }
        return;
    }
    if (state_ == State::Metadata) { metadata_packet(now); return; }
    if (state_ == State::Idle && now - idle_time_ >= 500 && metadata_complete_) {
        state_ = State::Active; status_pending_ = true; started_time_ = now;
    }
    if (status_pending_ || (active() && now - status_time_ >=
                           (now - started_time_ < 10000 ? 1000u : 20000u))) {
        const uint8_t status[4] = {uint8_t(state_ == State::Off ? 0 : 0x80),0,0,0};
        if (send(3,0x20,command_seq_,status,sizeof(status))) {
            status_pending_ = false; status_time_ = now;
        }
        return;
    }
    if (!active() || len < 10) return;
    const bool guide = ds[9] & 1;
    if (guide != guide_) {
        const uint8_t key[2] = {uint8_t(guide),0x5b};
        if (send(7,0x30,command_seq_,key,sizeof(key))) {
            guide_ = guide; guide_seq_ = command_seq_; guide_wait_ = true;
            guide_time_ = now; guide_retries_ = 0;
        }
        return;
    }
    if (guide_wait_ && guide_retries_ < 8 && now - guide_time_ >= 100) {
        const uint8_t packet[] = {7,0x30,guide_seq_,2,uint8_t(guide_),0x5b};
        if (send_(context_,packet,sizeof(packet))) { guide_time_ = now; ++guide_retries_; }
        return;
    }
    const Input input = map_input(ds, len);
    // Periodic keepalive also releases stale input after a reconnect.
    if (!input_sent_ || input != last_input_ || now - input_time_ >= 1000) {
        Input payload = input;
        if (input_sent_ && input == last_input_) payload[0] |= 2;
        if (send(0x20,0,input_seq_,payload.data(),payload.size())) {
            last_input_ = input; input_sent_ = true; input_time_ = now;
        }
    }
}
} // namespace xbox
