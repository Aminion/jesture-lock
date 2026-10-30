#pragma once

// Message payloads, docs/protocol.md "Payload header" and "Messages". Header included;
// pass the result to frame_encode().

#include "gesture/proto/frame.hpp"
#include "gesture/proto/le.hpp"

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

namespace proto {

enum class MsgType : std::uint8_t {
    hello = 1,
    sample = 2,
    status = 3,
    event = 4, // reserved, layout TBD
};

// Hello.fw
enum class Firmware : std::uint8_t {
    stream = 1,
    recognize = 2,
};

// Hello.imu
enum class ImuState : std::uint8_t {
    none = 0, // no IMU answered
    ok = 1,
    unknown_who_am_i = 2,
    init_failed = 3,
};

struct Hello {
    Firmware fw;
    ImuState imu;
    std::uint8_t imu_addr; // 7-bit, 0 if none answered
    std::uint8_t who_am_i; // 0 if none answered
    std::uint8_t ctrl1_xl; // as written, 0 if not configured
    std::uint8_t ctrl2_g;  // as written, 0 if not configured
};

using Axes = std::array<std::int16_t, 3>; // x, y, z, raw LSB

struct Sample {
    std::uint16_t seq; // +1 per Sample, wraps
    std::uint32_t t_us;
    Axes gyro;
    Axes accel;
};

// Counters since reset, wrapping.
struct Status {
    std::uint32_t t_us;
    std::uint32_t missed;
    std::uint32_t stuck;
    std::uint32_t i2c_err;
    std::uint32_t rx_err;
};

inline constexpr std::size_t hello_len = 8;
inline constexpr std::size_t sample_len = 20;
inline constexpr std::size_t status_len = 22;
static_assert(status_len <= max_payload);

namespace detail {

// Appends little-endian fields after the header. proto_test checks each layout byte by byte.
template <std::size_t N> struct Writer {
    std::array<std::uint8_t, N> buf{};
    std::size_t pos = 0;

    constexpr explicit Writer(MsgType type) {
        put(std::to_underlying(type));
        put(version);
    }
    template <std::integral T> constexpr void put(T v) {
        store_le(std::span{buf}.subspan(pos).template first<sizeof(T)>(), v);
        pos += sizeof(T);
    }
    constexpr void put(const Axes& v) {
        for (const std::int16_t a : v) {
            put(a);
        }
    }
};

} // namespace detail

[[nodiscard]] constexpr std::array<std::uint8_t, hello_len> encode(const Hello& m) {
    detail::Writer<hello_len> w{MsgType::hello};
    w.put(std::to_underlying(m.fw));
    w.put(std::to_underlying(m.imu));
    w.put(m.imu_addr);
    w.put(m.who_am_i);
    w.put(m.ctrl1_xl);
    w.put(m.ctrl2_g);
    return w.buf;
}

[[nodiscard]] constexpr std::array<std::uint8_t, sample_len> encode(const Sample& m) {
    detail::Writer<sample_len> w{MsgType::sample};
    w.put(m.seq);
    w.put(m.t_us);
    w.put(m.gyro);
    w.put(m.accel);
    return w.buf;
}

[[nodiscard]] constexpr std::array<std::uint8_t, status_len> encode(const Status& m) {
    detail::Writer<status_len> w{MsgType::status};
    w.put(m.t_us);
    w.put(m.missed);
    w.put(m.stuck);
    w.put(m.i2c_err);
    w.put(m.rx_err);
    return w.buf;
}

} // namespace proto
