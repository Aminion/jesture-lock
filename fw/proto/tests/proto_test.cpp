// Host tests for fw/proto. Returns non-zero on failure (ctest). Everything is constexpr, so
// most checks are static_asserts; only sweeps too long for constexpr evaluation run.
#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <print>
#include <source_location>
#include <span>

#include "proto/cobs.hpp"
#include "proto/crc16.hpp"
#include "proto/frame.hpp"
#include "proto/le.hpp"

namespace {

using Bytes = std::span<const std::uint8_t>;

template <std::size_t N>
consteval std::array<std::uint8_t, N> bytes_from(std::uint8_t first) {
    std::array<std::uint8_t, N> a{};
    for (std::size_t i = 0; i < N; ++i) {
        a[i] = static_cast<std::uint8_t>(first + i);
    }
    return a;
}

template <std::size_t N, std::size_t M>
consteval std::array<std::uint8_t, N + M> cat(const std::array<std::uint8_t, N>& a,
                                              const std::array<std::uint8_t, M>& b) {
    std::array<std::uint8_t, N + M> r{};
    std::ranges::copy(a, r.begin());
    std::ranges::copy(b, std::span{r}.subspan(N).begin());
    return r;
}

// --- CRC16 ----------------------------------------------------------------------

constexpr std::array<std::uint8_t, 9> check_string{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
static_assert(proto::crc16(check_string) == 0x29B1); // catalogue check value
static_assert(proto::crc16(Bytes{}) == proto::crc16_init);
static_assert(proto::crc16(Bytes{check_string}.subspan(4),
                           proto::crc16(Bytes{check_string}.first(4))) == 0x29B1); // continued
// init 0xFFFF: leading zeros change the result.
static_assert(proto::crc16(std::array<std::uint8_t, 1>{0}) !=
              proto::crc16(std::array<std::uint8_t, 2>{0, 0}));

// --- Little endian ----------------------------------------------------------------

template <std::integral T>
consteval std::array<std::uint8_t, sizeof(T)> le_bytes(T v) {
    std::array<std::uint8_t, sizeof(T)> a{};
    proto::store_le(std::span{a}, v);
    return a;
}
static_assert(le_bytes(std::uint16_t{0x1234}) == std::array<std::uint8_t, 2>{0x34, 0x12});
static_assert(le_bytes(std::uint32_t{0x12345678}) ==
              std::array<std::uint8_t, 4>{0x78, 0x56, 0x34, 0x12});
static_assert(le_bytes(std::int16_t{-2}) == std::array<std::uint8_t, 2>{0xFE, 0xFF});
static_assert(le_bytes(std::uint8_t{0xAB}) == std::array<std::uint8_t, 1>{0xAB});
static_assert(proto::load_le<std::int16_t>(std::array<std::uint8_t, 2>{0x00, 0x80}) == -32768);
static_assert(proto::load_le<std::int16_t>(std::array<std::uint8_t, 2>{0xFF, 0x7F}) == 32767);
static_assert(proto::load_le<std::uint32_t>(std::array<std::uint8_t, 4>{0x78, 0x56, 0x34, 0x12}) ==
              0x12345678);
static_assert(proto::load_le<std::int32_t>(le_bytes(std::int32_t{-123456789})) == -123456789);

// --- COBS: known vectors (Wikipedia, "Consistent Overhead Byte Stuffing") ---------

// Encodes to exactly `enc` and decodes back to `raw`.
template <std::size_t N, std::size_t M>
consteval bool cobs_vector(const std::array<std::uint8_t, N>& raw,
                           const std::array<std::uint8_t, M>& enc) {
    std::array<std::uint8_t, proto::cobs_max_encoded(N)> out{};
    const auto n = proto::cobs_encode(raw, out);
    if (!n || *n != M || !std::ranges::equal(std::span{out}.first(M), enc)) {
        return false;
    }
    std::array<std::uint8_t, N + 1> back{}; // + 1: no zero-size array for N = 0
    const auto d = proto::cobs_decode(enc, back);
    return d && *d == N && std::ranges::equal(std::span{back}.first(N), raw);
}

using A0 = std::array<std::uint8_t, 0>;
template <std::size_t N>
using A = std::array<std::uint8_t, N>;

static_assert(cobs_vector(A0{}, A<1>{0x01}));
static_assert(cobs_vector(A<1>{0x00}, A<2>{0x01, 0x01}));
static_assert(cobs_vector(A<2>{0x00, 0x00}, A<3>{0x01, 0x01, 0x01}));
static_assert(cobs_vector(A<3>{0x00, 0x11, 0x00}, A<4>{0x01, 0x02, 0x11, 0x01}));
static_assert(cobs_vector(A<4>{0x11, 0x22, 0x00, 0x33}, A<5>{0x03, 0x11, 0x22, 0x02, 0x33}));
static_assert(cobs_vector(A<4>{0x11, 0x22, 0x33, 0x44}, A<5>{0x05, 0x11, 0x22, 0x33, 0x44}));
static_assert(cobs_vector(A<4>{0x11, 0x00, 0x00, 0x00},
                          A<5>{0x02, 0x11, 0x01, 0x01, 0x01}));
// 254 non-zero bytes: one full block, no empty block after it.
static_assert(cobs_vector(bytes_from<254>(0x01), cat(A<1>{0xFF}, bytes_from<254>(0x01))));
static_assert(cobs_vector(cat(A<1>{0x00}, bytes_from<254>(0x01)),
                          cat(A<2>{0x01, 0xFF}, bytes_from<254>(0x01))));
// 255 non-zero bytes: a full block, then a block of one.
static_assert(cobs_vector(bytes_from<255>(0x01),
                          cat(cat(A<1>{0xFF}, bytes_from<254>(0x01)), A<2>{0x02, 0xFF})));
static_assert(cobs_vector(cat(bytes_from<254>(0x02), A<1>{0x00}),
                          cat(cat(A<1>{0xFF}, bytes_from<254>(0x02)), A<2>{0x01, 0x01})));
static_assert(cobs_vector(cat(bytes_from<253>(0x03), A<2>{0x00, 0x01}),
                          cat(cat(A<1>{0xFE}, bytes_from<253>(0x03)), A<2>{0x02, 0x01})));

constexpr std::size_t sweep_max_len = 600; // past two full blocks

// Encodes within the bound, without 0x00, and decodes back to itself.
bool cobs_round_trip(std::span<const std::uint8_t> raw) {
    std::array<std::uint8_t, proto::cobs_max_encoded(sweep_max_len)> enc{};
    const auto n = proto::cobs_encode(raw, enc);
    if (!n || *n > proto::cobs_max_encoded(raw.size()) ||
        std::ranges::contains(std::span{enc}.first(*n), std::uint8_t{0})) {
        return false;
    }
    std::array<std::uint8_t, sweep_max_len> back{};
    const auto d = proto::cobs_decode(std::span{enc}.first(*n), back);
    return d && *d == raw.size() && std::ranges::equal(std::span{back}.first(*d), raw);
}

// --- COBS: errors ---------------------------------------------------------------

template <std::size_t N>
consteval auto cobs_decode_into(const std::array<std::uint8_t, N>& enc, std::size_t out_size) {
    std::array<std::uint8_t, 8> out{};
    return proto::cobs_decode(enc, std::span{out}.first(out_size));
}
static_assert(cobs_decode_into(A<2>{0x00, 0x01}, 8).error() == proto::CobsError::zero_byte);
static_assert(cobs_decode_into(A<3>{0x03, 0x11, 0x00}, 8).error() ==
              proto::CobsError::zero_byte);
static_assert(cobs_decode_into(A<2>{0x03, 0x11}, 8).error() == proto::CobsError::truncated);
static_assert(cobs_decode_into(A<5>{0x05, 0x11, 0x22, 0x33, 0x44}, 3).error() ==
              proto::CobsError::overflow);
// The implicit zero between blocks needs room too.
static_assert(cobs_decode_into(A<4>{0x02, 0x11, 0x02, 0x22}, 1).error() ==
              proto::CobsError::overflow);
static_assert(cobs_decode_into(A0{}, 0).value() == 0);

consteval bool cobs_encode_needs_room() {
    std::array<std::uint8_t, 4> out{};
    return !proto::cobs_encode(A<4>{1, 2, 3, 4}, out) &&
           proto::cobs_encode(A<3>{1, 2, 3}, out) == 4;
}
static_assert(cobs_encode_needs_room());

// --- Frames: docs/protocol.md "Test vector" ---------------------------------------

static_assert(proto::max_frame == 36);

// Sample: seq 258, t_us 10 000 000, gyro (250, -630, -55), accel (-175, 50, 16900).
constexpr A<20> sample_payload{0x02, 0x01, 0x02, 0x01, 0x80, 0x96, 0x98, 0x00, 0xfa, 0x00,
                               0x8a, 0xfd, 0xc9, 0xff, 0x51, 0xff, 0x32, 0x00, 0x04, 0x42};
constexpr A<24> sample_wire{0x08, 0x02, 0x01, 0x02, 0x01, 0x80, 0x96, 0x98,
                            0x02, 0xfa, 0x08, 0x8a, 0xfd, 0xc9, 0xff, 0x51,
                            0xff, 0x32, 0x05, 0x04, 0x42, 0x8d, 0xe9, 0x00};
static_assert(proto::crc16(sample_payload) == 0xE98D);

// The documented field values sit at the documented offsets.
template <std::integral T>
consteval T sample_field(std::size_t offset) {
    return proto::load_le<T>(std::span{sample_payload}.subspan(offset).template first<sizeof(T)>());
}
static_assert(sample_field<std::uint16_t>(2) == 258);
static_assert(sample_field<std::uint32_t>(4) == 10'000'000);
static_assert(sample_field<std::int16_t>(8) == 250 && sample_field<std::int16_t>(10) == -630 &&
              sample_field<std::int16_t>(12) == -55);
static_assert(sample_field<std::int16_t>(14) == -175 && sample_field<std::int16_t>(16) == 50 &&
              sample_field<std::int16_t>(18) == 16900);

consteval bool frame_encodes_test_vector() {
    std::array<std::uint8_t, proto::max_frame> out{};
    const auto n = proto::frame_encode(sample_payload, out);
    return n == sample_wire.size() && std::ranges::equal(std::span{out}.first(*n), sample_wire);
}
static_assert(frame_encodes_test_vector());

// Decodes the wire bytes without the delimiter.
template <std::size_t N>
consteval auto frame_decode_wire(const std::array<std::uint8_t, N>& wire) {
    std::array<std::uint8_t, proto::max_payload + proto::crc_bytes> out{};
    return proto::frame_decode(std::span{wire}.first(N - 1), out);
}

consteval bool frame_decodes_test_vector() {
    std::array<std::uint8_t, proto::max_payload + proto::crc_bytes> out{};
    const auto n = proto::frame_decode(std::span{sample_wire}.first(sample_wire.size() - 1), out);
    return n == sample_payload.size() &&
           std::ranges::equal(std::span{out}.first(*n), sample_payload);
}
static_assert(frame_decodes_test_vector());

// Every single-bit error in the encoded bytes is caught (as COBS or CRC error).
consteval bool frame_detects_bit_flips() {
    for (std::size_t i = 0; i + 1 < sample_wire.size(); ++i) {
        for (int bit = 0; bit < 8; ++bit) {
            auto wire = sample_wire;
            wire[i] = static_cast<std::uint8_t>(wire[i] ^ (1U << bit));
            if (frame_decode_wire(wire)) {
                return false;
            }
        }
    }
    return true;
}
static_assert(frame_detects_bit_flips());

// Payload limits: max_payload is accepted, one more is not; too short to hold a header.
consteval bool frame_limits() {
    std::array<std::uint8_t, proto::max_frame> wire{};
    std::array<std::uint8_t, proto::max_payload + 1> payload{};
    payload[0] = 3;
    if (proto::frame_encode(payload, wire)) {
        return false;
    }
    const auto n = proto::frame_encode(std::span{payload}.first(proto::max_payload), wire);
    std::array<std::uint8_t, proto::max_payload + proto::crc_bytes> out{};
    if (!n || proto::frame_decode(std::span{wire}.first(*n - 1), out) != proto::max_payload) {
        return false;
    }
    std::array<std::uint8_t, proto::max_frame - 1> small{};
    if (proto::frame_encode(sample_payload, small)) {
        return false;
    }
    const auto m = proto::frame_encode(std::span{payload}.first(1), wire);
    return m && proto::frame_decode(std::span{wire}.first(*m - 1), out).error() ==
                    proto::FrameError::too_short;
}
static_assert(frame_limits());

// One byte longer than any valid frame decodes: too_long, not a buffer overrun.
consteval bool frame_rejects_long() {
    std::array<std::uint8_t, proto::max_payload + proto::crc_bytes + 1> raw{};
    raw.fill(0x55);
    std::array<std::uint8_t, proto::cobs_max_encoded(raw.size())> enc{};
    const auto n = proto::cobs_encode(raw, enc);
    std::array<std::uint8_t, proto::max_payload + proto::crc_bytes> out{};
    return n && proto::frame_decode(std::span{enc}.first(*n), out).error() ==
                    proto::FrameError::too_long;
}
static_assert(frame_rejects_long());

static_assert(frame_decode_wire(A<1>{0x00}).error() == proto::FrameError::too_short); // empty

// --- Run time: sweeps too long for constexpr evaluation ----------------------------

struct Tests {
    int failed = 0;
};

void check(Tests& t, bool ok, std::source_location loc = std::source_location::current()) {
    if (!ok) {
        std::println(stderr, "{}:{}: check failed", loc.file_name(), loc.line());
        ++t.failed;
    }
}

// Every length 0..600, with no zeros, all zeros, and zeros at block-sized spacing.
void cobs_round_trips(Tests& t) {
    constexpr std::array<std::size_t, 5> zero_spacings{0, 1, 7, 254, 255}; // 0: no zeros
    for (const std::size_t zero_every : zero_spacings) {
        std::array<std::uint8_t, sweep_max_len> raw{};
        for (std::size_t i = 0; i < raw.size(); ++i) {
            const bool zero = zero_every != 0 && i % zero_every == 0;
            raw.at(i) = zero ? 0 : static_cast<std::uint8_t>(1 + ((i * 37) % 255));
        }
        for (std::size_t len = 0; len <= sweep_max_len; ++len) {
            check(t, cobs_round_trip(std::span{raw}.first(len)));
        }
    }
}

} // namespace

int main() {
    Tests t;
    cobs_round_trips(t);
    if (t.failed != 0) {
        std::println(stderr, "proto_test: {} checks failed", t.failed);
        return 1;
    }
    return 0;
}
