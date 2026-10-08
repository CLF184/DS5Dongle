// Xbox Series USB/GIP gamepad bridge. Wire formats: Microsoft MS-GIPUSB.
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace xbox {
constexpr uint16_t vendor_id = 0x045e;
constexpr uint16_t product_id = 0x0b12;
constexpr size_t input_size = 32; // standard input + Console Function Map (Share)
using Input = std::array<uint8_t, input_size>;
using Motors = std::array<uint8_t, 4>; // LT, RT, low frequency, high frequency; 0..100
void make_device_id(uint32_t boot_random, uint8_t out[8]);
Input map_input(const uint8_t *ds, size_t len);
void trigger_effect(uint8_t percent, uint8_t out[11]);

class Rumble {
public:
    bool command(const uint8_t *payload, size_t len, uint32_t now);
    Motors tick(uint32_t now);
    void stop();
private:
    struct Motor {
        uint8_t level = 0;
        uint32_t start = 0;
        uint16_t duration = 0, delay = 0, cycles = 0;
    };
    std::array<Motor, 4> motors_{};
};

class Protocol {
public:
    using Send = bool (*)(void *, const uint8_t *, size_t);
    using Apply = void (*)(void *, const Motors &);
    Protocol(Send send, Apply apply, void *context) : send_(send), apply_(apply), context_(context) {}
    void reset(const uint8_t device_id[8], uint32_t now);
    void receive(const uint8_t *data, size_t len, uint32_t now);
    void tick(uint32_t now, const uint8_t *ds, size_t len);
    void stop_motors();
    bool active() const { return state_ == State::Active; }
    static const std::array<uint8_t, 256> &metadata();
    static size_t metadata_size();
private:
    enum class State { Arrival, Metadata, Idle, Active, Off };
    struct Ack { std::array<uint8_t, 13> data{}; };
    bool send(uint8_t type, uint8_t flags, uint8_t &counter, const uint8_t *p, size_t len);
    bool metadata_packet(uint32_t now);
    static uint8_t next(uint8_t &seq);
    Send send_;
    Apply apply_;
    void *context_;
    State state_ = State::Arrival;
    std::array<uint8_t, 8> id_{};
    uint8_t command_seq_ = 0, input_seq_ = 0, metadata_seq_ = 0;
    size_t metadata_offset_ = 0, metadata_sent_ = 0;
    bool metadata_wait_ = false, metadata_complete_ = false;
    uint32_t metadata_time_ = 0, hello_time_ = 0, idle_time_ = 0;
    uint32_t status_time_ = 0, started_time_ = 0, input_time_ = 0;
    bool hello_sent_ = false, status_pending_ = false, input_sent_ = false, guide_ = false;
    bool guide_wait_ = false;
    uint8_t guide_seq_ = 0, guide_retries_ = 0;
    uint32_t guide_time_ = 0;
    std::array<Ack, 8> acks_{};
    uint8_t ack_head_ = 0, ack_count_ = 0;
    Input last_input_{};
    Rumble rumble_;
    Motors applied_{};
};
} // namespace xbox
