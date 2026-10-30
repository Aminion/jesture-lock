// Host tests for the register-free parts of firmware/board. Returns non-zero on failure (ctest).
// Only headers are used: the drivers themselves need the MCU and are checked on hardware.
#include "gesture/board/i2c_timing.hpp"
#include "gesture/board/lsm6ds3.hpp"
#include "gesture/board/ring.hpp"
#include "gesture/board/uart.hpp"
#include "gesture/test/check.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <ranges>

namespace {

// --- UART baud rate (compile time) ---------------------------------------------

static_assert(board::uart_brr(115'200) == 139); // 16e6 / 115200 = 138.9
static_assert(board::uart_brr(9'600) == 1667);
static_assert(board::uart_baud_error_ppm(115'200) < 1'000); // 115108 baud: -0.08%
static_assert(board::uart_baud_error_ppm(1'000'000) == 0);  // BRR = 16 exactly
static_assert(board::uart_baud_ok(9'600));
static_assert(board::uart_baud_ok(115'200));
static_assert(board::uart_baud_ok(460'800));    // BRR 35: -0.79%
static_assert(board::uart_baud_ok(1'000'000));  // smallest legal BRR
static_assert(!board::uart_baud_ok(921'600));   // BRR 17: +2.1%, needs OVER8
static_assert(!board::uart_baud_ok(1'100'000)); // BRR 15 < 16
static_assert(!board::uart_baud_ok(200));       // BRR 80000 does not fit 16 bits
static_assert(!board::uart_baud_ok(0));
static_assert(!board::uart_baud_ok(40'000'000)); // BRR rounds to 0

// --- I2C timing (compile time) -------------------------------------------------

using board::i2c_fast_mode;
using board::i2c_standard_mode;
using board::i2c_timing_ok;
using board::I2cTiming;

// RM0444 Rev 6 Table 173 (16 MHz) and Table 172 (8 MHz).
constexpr I2cTiming sm100k_16mhz{.presc = 3, .scldel = 4, .sdadel = 2, .sclh = 0x0F, .scll = 0x13};
constexpr I2cTiming sm10k_16mhz{.presc = 3, .scldel = 4, .sdadel = 2, .sclh = 0xC3, .scll = 0xC7};
constexpr I2cTiming sm100k_8mhz{.presc = 1, .scldel = 4, .sdadel = 2, .sclh = 0x0F, .scll = 0x13};
constexpr I2cTiming fm400k_16mhz = board::i2c_timing_400khz_16mhz;
constexpr I2cTiming fm400k_8mhz{.presc = 0, .scldel = 3, .sdadel = 1, .sclh = 0x03, .scll = 0x09};

static_assert(i2c_timing_ok(sm100k_16mhz, 16'000'000, i2c_standard_mode));
static_assert(i2c_timing_ok(sm10k_16mhz, 16'000'000, i2c_standard_mode));
static_assert(i2c_timing_ok(sm100k_8mhz, 8'000'000, i2c_standard_mode));
static_assert(i2c_timing_ok(fm400k_16mhz, 16'000'000, i2c_fast_mode));
static_assert(i2c_timing_ok(fm400k_8mhz, 8'000'000, i2c_fast_mode));
// Fast-mode timing is too fast for standard mode, and standard-mode hold too long for fast.
static_assert(!i2c_timing_ok(fm400k_16mhz, 16'000'000, i2c_standard_mode));
static_assert(!i2c_timing_ok(sm100k_16mhz, 16'000'000, i2c_fast_mode));

consteval I2cTiming with(I2cTiming t, auto field, std::uint8_t v) {
    t.*field = v;
    return t;
}
// Each limit on its own: one step past it fails.
// Standard mode, 16 MHz: t_I2CCLK 62.5 ns, t_PRESC 250 ns, tSYNC credit 125 ns.
static_assert(!i2c_timing_ok(sm100k_16mhz, 32'000'000, i2c_standard_mode)); // all halved
static_assert(!i2c_timing_ok(with(sm100k_16mhz, &I2cTiming::scldel, 3), 16'000'000,
                             i2c_standard_mode)); // setup 1000 ns < 1250
static_assert(!i2c_timing_ok(with(sm100k_16mhz, &I2cTiming::sdadel, 0), 16'000'000,
                             i2c_standard_mode)); // hold 0 < 62.5 ns
static_assert(i2c_timing_ok(with(sm100k_16mhz, &I2cTiming::sdadel, 11), 16'000'000,
                            i2c_standard_mode)); // hold 2.75 us <= 2.94 us
static_assert(!i2c_timing_ok(with(sm100k_16mhz, &I2cTiming::sdadel, 12), 16'000'000,
                             i2c_standard_mode)); // hold 3 us > 2.94 us
static_assert(!i2c_timing_ok(with(sm100k_16mhz, &I2cTiming::scll, 0x11), 16'000'000,
                             i2c_standard_mode)); // tLOW 4.625 us < 4.7
static_assert(!i2c_timing_ok(with(sm100k_16mhz, &I2cTiming::sclh, 0x0E), 16'000'000,
                             i2c_standard_mode)); // tHIGH 3.875 us < 4.0
static_assert(!i2c_timing_ok(with(sm100k_16mhz, &I2cTiming::presc,
                                  board::i2c_timing_field4_max + 1),
                             16'000'000, i2c_standard_mode)); // PRESC is 4 bits
static_assert(!i2c_timing_ok(sm100k_16mhz, 0, i2c_standard_mode));
// Fast mode, 16 MHz: t_PRESC 125 ns.
static_assert(!i2c_timing_ok(fm400k_16mhz, 32'000'000, i2c_fast_mode)); // all halved
static_assert(!i2c_timing_ok(with(fm400k_16mhz, &I2cTiming::scldel, 2), 16'000'000,
                             i2c_fast_mode)); // setup 375 ns < 400
static_assert(!i2c_timing_ok(with(fm400k_16mhz, &I2cTiming::sdadel, 0), 16'000'000,
                             i2c_fast_mode)); // hold 0 < 62.5 ns
static_assert(i2c_timing_ok(with(fm400k_16mhz, &I2cTiming::sdadel, 3), 16'000'000,
                            i2c_fast_mode)); // hold 375 ns <= 390 ns
static_assert(!i2c_timing_ok(with(fm400k_16mhz, &I2cTiming::sdadel, 4), 16'000'000,
                             i2c_fast_mode)); // hold 500 ns > 390 ns
static_assert(!i2c_timing_ok(with(fm400k_16mhz, &I2cTiming::scll, 0x08), 16'000'000,
                             i2c_fast_mode)); // tLOW 1.25 us < 1.3
static_assert(!i2c_timing_ok(with(fm400k_16mhz, &I2cTiming::sclh, 0x02), 16'000'000,
                             i2c_fast_mode)); // tHIGH 500 ns < 600

// --- LSM6DS3 WHO_AM_I (compile time) -------------------------------------------

static_assert(board::lsm6ds3_chip(0x69) == board::Lsm6ds3Chip::lsm6ds3);
static_assert(board::lsm6ds3_chip(0x6A) == board::Lsm6ds3Chip::lsm6ds3trc);
static_assert(board::lsm6ds3_chip(0x6C) == board::Lsm6ds3Chip::lsm6dso);
static_assert(std::ranges::count_if(std::views::iota(0, 0x100), [](int v) {
                  return board::lsm6ds3_chip(static_cast<std::uint8_t>(v)).has_value();
              }) == 3); // 0x00/0xFF (bus stuck) and the rest: unknown

// Little endian per axis, gyro x, y, z then accel x, y, z; full int16 range.
constexpr std::array<std::uint8_t, board::lsm6ds3_sample_bytes> raw_sample{
    0x01, 0x00, 0xFF, 0xFF, 0x00, 0x80, 0xFF, 0x7F, 0x34, 0x12, 0x11, 0x40};
constexpr board::Lsm6ds3Sample decoded = board::lsm6ds3_decode(raw_sample);
static_assert(board::lsm6ds3_sample_bytes == 12);
static_assert(decoded.gyro == board::Lsm6ds3Axes{1, -1, -32768});
static_assert(decoded.accel == board::Lsm6ds3Axes{32767, 0x1234, 16401}); // ~1 g

// --- Ring buffer (run time: volatile indices are not constexpr) ------------------

using gesture_test::check;
using gesture_test::Tests;

void ring_empty(Tests& t) {
    board::Ring<8> r;
    check(t, !board::ring_pop(r).has_value());
}

void ring_fifo_order(Tests& t) {
    board::Ring<8> r;
    check(t, board::ring_push(r, 1));
    check(t, board::ring_push(r, 2));
    check(t, board::ring_push(r, 3));
    check(t, board::ring_pop(r) == 1);
    check(t, board::ring_pop(r) == 2);
    check(t, board::ring_pop(r) == 3);
    check(t, !board::ring_pop(r).has_value());
}

// Holds N - 1 bytes; a push into a full ring drops the byte and keeps the rest intact.
template <std::size_t N> void ring_full(Tests& t) {
    board::Ring<N> r;
    for (std::size_t i = 0; i < N - 1; ++i) {
        check(t, board::ring_push(r, static_cast<std::uint8_t>(i)));
    }
    check(t, !board::ring_push(r, 0xEE));
    check(t, board::ring_pop(r) == 0);
    check(t, board::ring_push(r, 0xAB)); // the freed slot is usable again
    check(t, !board::ring_push(r, 0xEE));
    for (std::size_t i = 1; i < N - 1; ++i) {
        check(t, board::ring_pop(r) == static_cast<std::uint8_t>(i));
    }
    check(t, board::ring_pop(r) == 0xAB);
    check(t, !board::ring_pop(r).has_value());
}

// Indices wrap many times with the ring at every fill level from 0 to N - 1.
template <std::size_t N> void ring_wrap(Tests& t) {
    board::Ring<N> r;
    std::uint8_t in = 0;
    std::uint8_t out = 0;
    for (int round = 0; round < 10; ++round) {
        for (std::size_t fill = 1; fill < N; ++fill) {
            for (std::size_t i = 0; i < fill; ++i) {
                check(t, board::ring_push(r, in++));
            }
            for (std::size_t i = 0; i < fill; ++i) {
                check(t, board::ring_pop(r) == out++);
            }
            check(t, !board::ring_pop(r).has_value());
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
    return gesture_test::finish(t, "board_test");
}
