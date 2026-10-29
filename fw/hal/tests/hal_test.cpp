// Host tests for the register-free parts of fw/hal. Returns non-zero on failure (ctest).
// Only headers are used: the drivers themselves need the MCU and are checked on hardware.
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "gesture_test/check.hpp"
#include "hal/i2c_timing.hpp"
#include "hal/lsm6ds3.hpp"
#include "hal/ring.hpp"
#include "hal/uart.hpp"

namespace {

// --- UART baud rate (compile time) ---------------------------------------------

static_assert(hal::uart_brr(115'200) == 139); // 16e6 / 115200 = 138.9
static_assert(hal::uart_brr(9'600) == 1667);
static_assert(hal::uart_baud_error_ppm(115'200) < 1'000); // 115108 baud: -0.08%
static_assert(hal::uart_baud_error_ppm(1'000'000) == 0);  // BRR = 16 exactly
static_assert(hal::uart_baud_ok(9'600));
static_assert(hal::uart_baud_ok(115'200));
static_assert(hal::uart_baud_ok(460'800));    // BRR 35: -0.79%
static_assert(hal::uart_baud_ok(1'000'000));  // smallest legal BRR
static_assert(!hal::uart_baud_ok(921'600));   // BRR 17: +2.1%, needs OVER8
static_assert(!hal::uart_baud_ok(1'100'000)); // BRR 15 < 16
static_assert(!hal::uart_baud_ok(200));       // BRR 80000 does not fit 16 bits
static_assert(!hal::uart_baud_ok(0));
static_assert(!hal::uart_baud_ok(40'000'000)); // BRR rounds to 0

// --- I2C timing (compile time) -------------------------------------------------

// RM0444 Rev 6 Table 173 (16 MHz) and Table 172 (8 MHz), standard mode.
constexpr hal::I2cTiming sm100k_16mhz = hal::i2c_timing_100khz_16mhz;
constexpr hal::I2cTiming sm10k_16mhz{
    .presc = 3, .scldel = 4, .sdadel = 2, .sclh = 0xC3, .scll = 0xC7};
constexpr hal::I2cTiming sm100k_8mhz{
    .presc = 1, .scldel = 4, .sdadel = 2, .sclh = 0x0F, .scll = 0x13};

static_assert(hal::i2c_timingr(sm100k_16mhz) == 0x3042'0F13);
static_assert(hal::i2c_timingr(sm10k_16mhz) == 0x3042'C3C7);
static_assert(hal::i2c_timing_standard_mode_ok(sm100k_16mhz, 16'000'000));
static_assert(hal::i2c_timing_standard_mode_ok(sm10k_16mhz, 16'000'000));
static_assert(hal::i2c_timing_standard_mode_ok(sm100k_8mhz, 8'000'000));

consteval hal::I2cTiming with(hal::I2cTiming t, auto field, std::uint8_t v) {
    t.*field = v;
    return t;
}
// Each limit on its own: one step past it fails.
static_assert(!hal::i2c_timing_standard_mode_ok(sm100k_16mhz, 32'000'000)); // all halved
static_assert(!hal::i2c_timing_standard_mode_ok(with(sm100k_16mhz, &hal::I2cTiming::scldel, 3),
                                                16'000'000)); // setup 1000 ns < 1250
static_assert(!hal::i2c_timing_standard_mode_ok(with(sm100k_16mhz, &hal::I2cTiming::sdadel, 0),
                                                16'000'000)); // hold 0 < 62.5 ns
static_assert(!hal::i2c_timing_standard_mode_ok(with(sm100k_16mhz, &hal::I2cTiming::sdadel, 8),
                                                16'000'000)); // hold 2 us > 1.94 us
static_assert(!hal::i2c_timing_standard_mode_ok(with(sm100k_16mhz, &hal::I2cTiming::scll, 0x11),
                                                16'000'000)); // tLOW 4.5 us < 4.7
static_assert(!hal::i2c_timing_standard_mode_ok(with(sm100k_16mhz, &hal::I2cTiming::sclh, 0x0E),
                                                16'000'000)); // tHIGH 3.75 us < 4.0
static_assert(!hal::i2c_timing_standard_mode_ok(with(sm100k_16mhz, &hal::I2cTiming::presc,
                                                     hal::i2c_timing_field4_max + 1),
                                                16'000'000)); // PRESC is 4 bits
static_assert(!hal::i2c_timing_standard_mode_ok(sm100k_16mhz, 0));

// --- LSM6DS3 WHO_AM_I (compile time) -------------------------------------------

static_assert(hal::lsm6ds3_chip(0x69) == hal::Lsm6ds3Chip::lsm6ds3);
static_assert(hal::lsm6ds3_chip(0x6A) == hal::Lsm6ds3Chip::lsm6ds3trc);
static_assert(hal::lsm6ds3_chip(0x6C) == hal::Lsm6ds3Chip::lsm6dso);
consteval int known_who_am_i_values() {
    int n = 0;
    for (int v = 0; v <= 0xFF; ++v) {
        n += hal::lsm6ds3_chip(static_cast<std::uint8_t>(v)).has_value() ? 1 : 0;
    }
    return n;
}
static_assert(known_who_am_i_values() == 3); // 0x00/0xFF (bus stuck) and the rest: unknown

// Little endian per axis, gyro x, y, z then accel x, y, z; full int16 range.
constexpr std::array<std::uint8_t, hal::lsm6ds3_sample_bytes> raw_sample{
    0x01, 0x00, 0xFF, 0xFF, 0x00, 0x80, 0xFF, 0x7F, 0x34, 0x12, 0x11, 0x40};
constexpr hal::Lsm6ds3Sample decoded = hal::lsm6ds3_decode(raw_sample);
static_assert(hal::lsm6ds3_sample_bytes == 12);
static_assert(decoded.gyro == hal::Lsm6ds3Axes{1, -1, -32768});
static_assert(decoded.accel == hal::Lsm6ds3Axes{32767, 0x1234, 16401}); // ~1 g

// --- Ring buffer (run time: volatile indices are not constexpr) ------------------

using gesture_test::check;
using gesture_test::Tests;

void ring_empty(Tests& t) {
    hal::Ring<8> r;
    check(t, !hal::ring_pop(r).has_value());
}

void ring_fifo_order(Tests& t) {
    hal::Ring<8> r;
    check(t, hal::ring_push(r, 1));
    check(t, hal::ring_push(r, 2));
    check(t, hal::ring_push(r, 3));
    check(t, hal::ring_pop(r) == 1);
    check(t, hal::ring_pop(r) == 2);
    check(t, hal::ring_pop(r) == 3);
    check(t, !hal::ring_pop(r).has_value());
}

// Holds N - 1 bytes; a push into a full ring drops the byte and keeps the rest intact.
template <std::size_t N>
void ring_full(Tests& t) {
    hal::Ring<N> r;
    for (std::size_t i = 0; i < N - 1; ++i) {
        check(t, hal::ring_push(r, static_cast<std::uint8_t>(i)));
    }
    check(t, !hal::ring_push(r, 0xEE));
    check(t, hal::ring_pop(r) == 0);
    check(t, hal::ring_push(r, 0xAB)); // the freed slot is usable again
    check(t, !hal::ring_push(r, 0xEE));
    for (std::size_t i = 1; i < N - 1; ++i) {
        check(t, hal::ring_pop(r) == static_cast<std::uint8_t>(i));
    }
    check(t, hal::ring_pop(r) == 0xAB);
    check(t, !hal::ring_pop(r).has_value());
}

// Indices wrap many times with the ring at every fill level from 0 to N - 1.
template <std::size_t N>
void ring_wrap(Tests& t) {
    hal::Ring<N> r;
    std::uint8_t in = 0;
    std::uint8_t out = 0;
    for (int round = 0; round < 10; ++round) {
        for (std::size_t fill = 1; fill < N; ++fill) {
            for (std::size_t i = 0; i < fill; ++i) {
                check(t, hal::ring_push(r, in++));
            }
            for (std::size_t i = 0; i < fill; ++i) {
                check(t, hal::ring_pop(r) == out++);
            }
            check(t, !hal::ring_pop(r).has_value());
        }
    }
}

} // namespace

int main() {
    Tests t;
    ring_empty(t);
    ring_fifo_order(t);
    ring_full<2>(t);
    ring_full<64>(t);
    ring_full<256>(t);
    ring_wrap<2>(t);
    ring_wrap<64>(t);
    ring_wrap<256>(t);
    return gesture_test::finish(t, "hal_test");
}
