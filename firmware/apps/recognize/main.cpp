// recognize firmware: IMU -> core/ on the MCU -> recognition events over UART.
//
// For now a board self-test of core/: at boot it replays the embedded vector
// (core/contract/vectors/v1/, written by `gesture-cli vectors`) through gc_feed and checks that the
// MCU build gives the same events, strokes and live points as the host build, bit for bit (the
// digest is defined in host/crates/gesture-cli/src/vector.rs). It also times every gc_feed against
// the sample period and measures the stack peak. The result is in `bench_result`, read over SWD
// (core/contract/vectors/v1/README.md); the LED blinks slowly on a match, fast otherwise.
//
// It also embeds the board's calibration file (docs/calibration.md) and checks it at build
// time; the config made from it waits for the live IMU feed.
#include "gesture/board/clock.hpp"
#include "gesture/board/gpio.hpp"
#include "gesture/board/lsm6ds3.hpp"
#include "gesture/core.h"
#include "gesture/proto/crc16.hpp"
#include "gesture/proto/le.hpp"
#include "stm32g0xx.h"

#include <algorithm>
#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

// Linker script symbols: end of .bss, top of RAM.
extern "C" {
// NOLINTNEXTLINE(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp): linker names
extern std::uint32_t _ebss;
// NOLINTNEXTLINE(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp): linker names
extern std::uint32_t _estack;
}

// The self-test result, read over SWD. Fields are u32 in this order (README).
struct Bench {
    std::uint32_t magic;         // bench_magic once the run is over
    std::uint32_t result;        // Result
    std::uint32_t samples;       // fed so far
    std::uint32_t events;        // non-NONE events seen
    std::uint32_t digest;        // over the run, as vector.rs
    std::uint32_t expect_events; // from the vector (host build)
    std::uint32_t expect_digest;
    std::uint32_t max_us;        // slowest gc_feed
    std::uint32_t max_at;        // its sample index
    std::uint32_t max_moving_us; // slowest gc_feed returning MOVING (the ring replay)
    std::uint32_t max_end_us;    // slowest gc_feed returning STROKE or REJECTED
    std::uint32_t over_period;   // gc_feed calls longer than a sample period
    std::uint32_t total_us;      // sum of all gc_feed times
    std::uint32_t stack_peak;    // bytes below _estack ever written
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables): read over SWD
Bench bench_result{};

namespace {

constexpr std::uint32_t bench_magic = 0x31424347; // "GCB1"

enum class Result : std::uint8_t { running = 0, match = 1, mismatch = 2, init_failed = 3 };

// --- The vector (layout: vector.rs) -------------------------------------------------

// GESTURE_BOARD_VECTOR is set in firmware/apps/CMakeLists.txt. #embed is C23; GCC 15 takes
// it in C++ silently, clang (clang-tidy) as an extension that -Wpedantic flags.
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc23-extensions"
#endif
constexpr auto vector = std::to_array<std::uint8_t>({
#embed GESTURE_BOARD_VECTOR
});
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// Header field offsets.
constexpr std::size_t magic_at = 0;
constexpr std::size_t samples_at = 4;
constexpr std::size_t events_at = 8;
constexpr std::size_t digest_at = 12;
constexpr std::size_t config_at = 16;
constexpr std::size_t header_size = config_at + sizeof(gc_config);
static_assert(header_size == 84);

template <typename T> constexpr T read_at(std::size_t at) {
    std::array<std::uint8_t, sizeof(T)> b{};
    std::ranges::copy(std::span{vector}.subspan(at, sizeof(T)), b.begin());
    return std::bit_cast<T>(b);
}

constexpr std::uint32_t vector_samples = read_at<std::uint32_t>(samples_at);
static_assert(read_at<std::uint32_t>(magic_at) == 0x31564347, "not a GCV1 vector"); // "GCV1"
static_assert(vector.size() == header_size + (vector_samples * sizeof(gc_sample)),
              "vector size does not match its sample count");
static_assert(std::endian::native == std::endian::little, "the vector is little-endian");

// --- The board calibration (docs/calibration.md) -----------------------------------

// GESTURE_BOARD_CAL is data/calibration/board.cal, written by `gesture-cli calibrate`. A
// file that is damaged, or measured at other scales than the driver sets, does not build.
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc23-extensions"
#endif
constexpr auto cal = std::to_array<std::uint8_t>({
#embed GESTURE_BOARD_CAL
});
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// File layout.
constexpr std::size_t cal_size = 44;
constexpr std::size_t cal_version_at = 4;
constexpr std::size_t cal_ctrl1_xl_at = 5;
constexpr std::size_t cal_ctrl2_g_at = 6;
constexpr std::size_t cal_reserved0_at = 7;
constexpr std::size_t cal_offset_at = 8;
constexpr std::size_t cal_reserved1_at = 14;
constexpr std::size_t cal_gain_at = 16;
constexpr std::size_t cal_bias_at = 28;
constexpr std::size_t cal_reserved2_at = 40;
constexpr std::size_t cal_crc_at = 42;
constexpr std::uint8_t cal_version = 1;
// The full-scale bits: FS_XL in CTRL1_XL; FS_G and FS_125 in CTRL2_G.
constexpr std::uint8_t fs_xl_mask = 0x0C;
constexpr std::uint8_t fs_g_mask = 0x0E;
// accel_gain_q30 that gc_init accepts: 0.9..1.1 (docs/core.md).
constexpr std::int32_t gain_q30_min = 966'367'642;
constexpr std::int32_t gain_q30_max = 1'181'116'006;

static_assert(cal.size() == cal_size, "board.cal: not 44 bytes");

template <std::integral T> constexpr T cal_at(std::size_t at) {
    return proto::load_le<T>(std::span{cal}.subspan(at).first<sizeof(T)>());
}

template <std::integral T> constexpr std::array<T, 3> cal_axes(std::size_t at) {
    return {cal_at<T>(at), cal_at<T>(at + sizeof(T)), cal_at<T>(at + (2 * sizeof(T)))};
}

constexpr auto cal_offset = cal_axes<std::int16_t>(cal_offset_at);
constexpr auto cal_gain = cal_axes<std::int32_t>(cal_gain_at);
constexpr auto cal_bias = cal_axes<std::int32_t>(cal_bias_at);

static_assert(cal_at<std::uint32_t>(0) == 0x4c41'4347, "board.cal: not a GCAL file"); // "GCAL"
static_assert(cal[cal_version_at] == cal_version, "board.cal: unknown version");
static_assert(proto::crc16(std::span{cal}.first(cal_crc_at)) == cal_at<std::uint16_t>(cal_crc_at),
              "board.cal: CRC mismatch");
static_assert(cal[cal_reserved0_at] == 0 && cal_at<std::uint16_t>(cal_reserved1_at) == 0 &&
                  cal_at<std::uint16_t>(cal_reserved2_at) == 0,
              "board.cal: a reserved field is not 0");
static_assert(std::ranges::all_of(cal_gain,
                                  [](std::int32_t g) {
                                      return g >= gain_q30_min && g <= gain_q30_max;
                                  }),
              "board.cal: a gain is outside 0.9..1.1");
static_assert(((cal[cal_ctrl1_xl_at] ^ board::lsm6ds3_ctrl1_xl) & fs_xl_mask) == 0 &&
                  ((cal[cal_ctrl2_g_at] ^ board::lsm6ds3_ctrl2_g) & fs_g_mask) == 0,
              "board.cal: measured at other scales than the driver sets");

// The thresholds and limits below are the table in docs/calibration.md: the hand preset
// in LSB of +-4 g (FS_XL 10) and 500 dps (FS_G 01). Another scale needs other numbers.
static_assert((board::lsm6ds3_ctrl1_xl & fs_xl_mask) == 0x08 &&
                  (board::lsm6ds3_ctrl2_g & fs_g_mask) == 0x04,
              "the still thresholds are for +-4 g and 500 dps");

// The config for the live IMU: the calibration file, the thresholds, the limits. The host
// builds the same 68 bytes for a stream of this board (session::reference). Not fed to
// the core yet: recognize does not read the IMU so far.
[[maybe_unused]] constexpr gc_config live_config{
    .abi_version = GC_ABI_VERSION,
    .ctrl1_xl = board::lsm6ds3_ctrl1_xl,
    .ctrl2_g = board::lsm6ds3_ctrl2_g,
    .reserved0 = 0,
    .accel_offset_lsb = {cal_offset[0], cal_offset[1], cal_offset[2]},
    .reserved1 = 0,
    .accel_gain_q30 = {cal_gain[0], cal_gain[1], cal_gain[2]},
    .gyro_bias_q8 = {cal_bias[0], cal_bias[1], cal_bias[2]},
    .still_gyro_std_lsb = 171,
    .still_accel_std_lsb = 123,
    .still_gyro_rate_lsb = 229,
    .hold_n = 20,
    .bias_gyro_std_lsb = 11,
    .bias_accel_std_lsb = 16,
    .clip_lsb = 32700,
    .max_dt_us = 14423,
    .min_stroke_us = 150'000,
    .max_stroke_us = 4'000'000,
    .min_extent_mm = 20,
    .max_v_end_mm_s = 300,
};
static_assert(sizeof(gc_config) == 68);

// --- Digest: FNV-1a 32 over the C layouts, as vector.rs ----------------------------

constexpr std::uint32_t fnv_offset_basis = 0x811c'9dc5;
constexpr std::uint32_t fnv_prime = 0x0100'0193;

struct Fnv {
    std::uint32_t h = fnv_offset_basis;
};

template <typename T> void hash(Fnv& f, const T& v) {
    for (const std::uint8_t b : std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(v)) {
        f.h = (f.h ^ b) * fnv_prime;
    }
}

constexpr std::uint16_t points_cap = 64;

// --- Stack high-water mark --------------------------------------------------------

constexpr std::uint32_t paint = 0xa5a5'a5a5;
// Left alone below the stack pointer while painting: the frames of paint_stack itself.
constexpr std::uintptr_t paint_margin = 64;

std::uintptr_t stack_pointer() {
    return __get_MSP(); // thread mode runs on MSP: nothing switches to PSP
}

std::span<volatile std::uint32_t> free_ram(std::uintptr_t top) {
    const auto lo = std::bit_cast<std::uintptr_t>(&_ebss);
    return {static_cast<volatile std::uint32_t*>(&_ebss), (top - lo) / sizeof(std::uint32_t)};
}

// Fills the RAM between .bss and the current stack (less a margin) with `paint`.
[[gnu::noinline]] void paint_stack() {
    for (volatile std::uint32_t& w : free_ram(stack_pointer() - paint_margin)) {
        w = paint;
    }
}

// Bytes from the lowest word no longer painted up to the top of RAM.
std::uint32_t stack_peak() {
    const auto ram = free_ram(stack_pointer());
    const auto first = std::ranges::find_if(ram, [](std::uint32_t w) { return w != paint; });
    const auto low = std::bit_cast<std::uintptr_t>(&_ebss) +
                     (static_cast<std::uintptr_t>(first - ram.begin()) * sizeof(std::uint32_t));
    return static_cast<std::uint32_t>(std::bit_cast<std::uintptr_t>(&_estack) - low);
}

// --- The run ----------------------------------------------------------------------

constexpr std::uint32_t sample_period_us = 9600;

// gc_feed alone, interrupts masked, timed with TIM2 (1 us).
gc_event timed_feed(gc_state& s, const gc_sample& in, std::uint32_t& us) {
    __disable_irq();
    const std::uint32_t t0 = board::micros();
    const gc_event ev = gc_feed(&s, &in);
    us = board::micros() - t0;
    __enable_irq();
    return ev;
}

[[gnu::noinline]] Result run(Bench& b) {
    static gc_state state{};
    static std::array<gc_point, points_cap> points{};
    constexpr auto cfg = read_at<gc_config>(config_at);
    if (gc_init(&state, &cfg) != GC_OK) {
        return Result::init_failed;
    }
    Fnv f;
    for (std::uint32_t k = 0; k < vector_samples; ++k) {
        const auto in = read_at<gc_sample>(header_size + (k * sizeof(gc_sample)));
        std::uint32_t us = 0;
        const gc_event ev = timed_feed(state, in, us);

        b.total_us += us;
        if (us > b.max_us) {
            b.max_us = us;
            b.max_at = k;
        }
        if (ev.kind == GC_EV_MOVING) {
            b.max_moving_us = std::max(b.max_moving_us, us);
        }
        if (ev.kind == GC_EV_STROKE || ev.kind == GC_EV_REJECTED) {
            b.max_end_us = std::max(b.max_end_us, us);
        }
        b.over_period += us > sample_period_us ? 1U : 0U;

        if (ev.kind != GC_EV_NONE) {
            ++b.events;
            hash(f, k);
            hash(f, ev);
        }
        if (ev.kind == GC_EV_STROKE) {
            gc_stroke_info info{};
            const std::uint16_t n = gc_stroke(&state, &info, points.data(), points_cap);
            hash(f, n);
            hash(f, info);
            for (const gc_point& p : std::span{points}.first(n)) {
                hash(f, p);
            }
        }
        gc_point live{};
        hash(f, gc_live(&state, &live));
        hash(f, live);
        b.samples = k + 1;
    }
    b.digest = f.h;
    return b.digest == b.expect_digest && b.events == b.expect_events ? Result::match
                                                                      : Result::mismatch;
}

} // namespace

int main() {
    board::clock_init();
    board::led_init();
    paint_stack();

    Bench b{};
    b.expect_events = read_at<std::uint32_t>(events_at);
    b.expect_digest = read_at<std::uint32_t>(digest_at);
    b.result = std::to_underlying(run(b));
    b.stack_peak = stack_peak();
    b.magic = bench_magic;
    bench_result = b;

    // Slow blink (toggle every 0.5 s) = the MCU matches the host, fast = it does not.
    const std::uint32_t blink_ms = b.result == std::to_underlying(Result::match) ? 500 : 60;
    for (;;) {
        board::led_toggle();
        board::delay_ms(blink_ms);
    }
}
