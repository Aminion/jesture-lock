// recognize firmware: IMU -> core/ on the MCU -> recognition events over UART.
//
// For now a board self-test of core/: at boot it replays the embedded vector
// (data/vectors/, written by `gesture-cli vector`) through gc_feed and checks that the MCU
// build gives the same events, strokes and live points as the host build, bit for bit
// (the digest is defined in host/crates/gesture-cli/src/vector.rs). It also times every
// gc_feed against the sample period and measures the stack peak. The result is in
// `bench_result`, read over SWD (data/vectors/README.md); the LED blinks slowly on a
// match, fast otherwise.
#include "gesture/board/clock.hpp"
#include "gesture/board/gpio.hpp"
#include "gesture/core.h"

#include <stm32g0xx.h>

#include <algorithm>
#include <array>
#include <bit>
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

constexpr std::size_t header_size = 84;
constexpr std::size_t config_at = 16;

template <typename T> constexpr T read_at(std::size_t at) {
    std::array<std::uint8_t, sizeof(T)> b{};
    std::ranges::copy(std::span{vector}.subspan(at, sizeof(T)), b.begin());
    return std::bit_cast<T>(b);
}

constexpr std::uint32_t vector_samples = read_at<std::uint32_t>(4);
static_assert(read_at<std::uint32_t>(0) == 0x31564347, "not a GCV1 vector"); // "GCV1"
static_assert(vector.size() == header_size + (vector_samples * sizeof(gc_sample)),
              "vector size does not match its sample count");
static_assert(std::endian::native == std::endian::little, "the vector is little-endian");

// --- Digest: FNV-1a 32 over the C layouts, as vector.rs ----------------------------

struct Fnv {
    std::uint32_t h = 0x811c'9dc5;
};

template <typename T> void hash(Fnv& f, const T& v) {
    for (const std::uint8_t b : std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(v)) {
        f.h = (f.h ^ b) * 0x0100'0193U;
    }
}

constexpr std::uint16_t points_cap = 64;

// --- Stack high-water mark --------------------------------------------------------

constexpr std::uint32_t paint = 0xa5a5'a5a5;
// Left alone below the stack pointer while painting: the frames of paint_stack itself.
constexpr std::uintptr_t paint_margin = 64;

std::uintptr_t stack_pointer() {
    std::uintptr_t sp = 0;
    asm volatile("mov %0, sp" : "=r"(sp));
    return sp;
}

std::span<volatile std::uint32_t> free_ram(std::uintptr_t top) {
    const auto lo = std::bit_cast<std::uintptr_t>(&_ebss);
    return {static_cast<volatile std::uint32_t*>(&_ebss),
            (top - lo) / sizeof(std::uint32_t)};
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
    b.expect_events = read_at<std::uint32_t>(8);
    b.expect_digest = read_at<std::uint32_t>(12);
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
