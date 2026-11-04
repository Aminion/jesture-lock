// Host tests for core/. Returns non-zero on failure (ctest). Internal helpers are
// constexpr and checked by static_assert; the C ABI is checked at run time. Conformance
// vectors (docs/core.md, through core.h only) are separate.
#include "arith.hpp"
#include "config.hpp"
#include "gesture/core.h"
#include "gesture/test/check.hpp"
#include "ring.hpp"
#include "state.hpp"
#include "still.hpp"
#include "stroke.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <utility>

namespace {

using gcore::rdiv;

// --- rdiv: to nearest, halves away from zero ------------------------------------

static_assert(rdiv(0, 5) == 0);
static_assert(rdiv(6, 3) == 2 && rdiv(-6, 3) == -2);
static_assert(rdiv(1, 3) == 0 && rdiv(-1, 3) == 0);
static_assert(rdiv(2, 3) == 1 && rdiv(-2, 3) == -1);
static_assert(rdiv(4, 3) == 1 && rdiv(5, 3) == 2 && rdiv(-5, 3) == -2);
static_assert(rdiv(1, 2) == 1 && rdiv(-1, 2) == -1); // halves
static_assert(rdiv(5, 2) == 3 && rdiv(-5, 2) == -3);
static_assert(rdiv(1, 4) == 0 && rdiv(2, 4) == 1 && rdiv(3, 4) == 1);
static_assert(rdiv(-2, 4) == -1 && rdiv(-3, 4) == -1 && rdiv(-6, 4) == -2);
static_assert(rdiv(7, 1) == 7 && rdiv(-7, 1) == -7);
static_assert(rdiv(rdiv(-5, 2), 1) == -rdiv(5, 2)); // odd: rdiv(-a, b) == -rdiv(a, b)
// Large operands, as in the Q30 products.
static_assert(rdiv((3LL << 61) + (1LL << 29), 1LL << 30) == (3LL << 31) + 1);
static_assert(rdiv((3LL << 61) + (1LL << 29) - 1, 1LL << 30) == 3LL << 31);
static_assert(rdiv(-(3LL << 61) - (1LL << 29), 1LL << 30) == -(3LL << 31) - 1);

// --- saturation ------------------------------------------------------------------

static_assert(gcore::sat16(32767) == 32767 && gcore::sat16(32768) == 32767);
static_assert(gcore::sat16(-32768) == -32768 && gcore::sat16(-32769) == -32768);
static_assert(gcore::sat16(std::numeric_limits<std::int64_t>::min()) == -32768);
static_assert(gcore::satu16(-1) == 0 && gcore::satu16(65535) == 65535);
static_assert(gcore::satu16(65536) == 65535);

// --- scales: every FS code, ODR bits ignored -------------------------------------

static_assert(gcore::accel_ug_per_lsb(0x40) == 61);     // 2 g
static_assert(gcore::accel_ug_per_lsb(0x44) == 488);    // 16 g
static_assert(gcore::accel_ug_per_lsb(0x48) == 122);    // 4 g
static_assert(gcore::accel_ug_per_lsb(0x4C) == 244);    // 8 g
static_assert(gcore::accel_ug_per_lsb(0xA8) == 122);    // other ODR, low bits
static_assert(gcore::gyro_udps_per_lsb(0x40) == 8750);  // 250 dps
static_assert(gcore::gyro_udps_per_lsb(0x44) == 17500); // 500 dps
static_assert(gcore::gyro_udps_per_lsb(0x48) == 35000); // 1000 dps
static_assert(gcore::gyro_udps_per_lsb(0x4C) == 70000); // 2000 dps
static_assert(gcore::gyro_udps_per_lsb(0x42) == 4375);  // 125 dps
static_assert(gcore::gyro_udps_per_lsb(0x4E) == 4375);  // FS_125 wins over FS_G

// --- gc_config validation ---------------------------------------------------------

// The defaults in docs/todo.md: hand, +-4 g / 500 dps, 104 Hz; unit calibration.
constexpr gc_config valid_config() {
    constexpr std::int32_t one_q30 = 1 << 30;
    return gc_config{
        .abi_version = GC_ABI_VERSION,
        .ctrl1_xl = 0x48,
        .ctrl2_g = 0x44,
        .reserved0 = 0,
        .accel_offset_lsb = {0, 0, 0},
        .reserved1 = 0,
        .accel_gain_q30 = {one_q30, one_q30, one_q30},
        .gyro_bias_q8 = {0, 0, 0},
        .still_gyro_std_lsb = 171,
        .still_accel_std_lsb = 123,
        .still_gyro_rate_lsb = 229,
        .hold_n = 20,
        .bias_gyro_std_lsb = 11,
        .bias_accel_std_lsb = 16,
        .clip_lsb = 32700,
        .max_dt_us = 14423,
        .min_stroke_us = 150000,
        .max_stroke_us = 4000000,
        .min_extent_mm = 20,
        .max_v_end_mm_s = 300,
    };
}

template <typename F> constexpr gc_status validate_with(F change) {
    gc_config cfg = valid_config();
    change(cfg);
    return gcore::validate(cfg);
}

static_assert(gcore::validate(valid_config()) == GC_OK);

static_assert(validate_with([](gc_config& c) { c.abi_version = 0; }) == GC_ERR_VERSION);
static_assert(validate_with([](gc_config& c) { c.abi_version = 2; }) == GC_ERR_VERSION);
static_assert(validate_with([](gc_config& c) { c.ctrl1_xl = 0; }) == GC_ERR_SCALE);
static_assert(validate_with([](gc_config& c) { c.ctrl2_g = 0; }) == GC_ERR_SCALE);

static_assert(validate_with([](gc_config& c) { c.reserved0 = 1; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.reserved1 = 1; }) == GC_ERR_CONFIG);
// Gain bounds are inclusive, on every axis.
static_assert(validate_with([](gc_config& c) { c.accel_gain_q30[0] = gcore::gain_min_q30; }) ==
              GC_OK);
static_assert(validate_with([](gc_config& c) { c.accel_gain_q30[1] = gcore::gain_max_q30; }) ==
              GC_OK);
static_assert(validate_with([](gc_config& c) { c.accel_gain_q30[1] = gcore::gain_min_q30 - 1; }) ==
              GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.accel_gain_q30[2] = gcore::gain_max_q30 + 1; }) ==
              GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.still_gyro_std_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.still_accel_std_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.still_gyro_rate_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.bias_gyro_std_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.bias_accel_std_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.clip_lsb = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.hold_n = gcore::guard_n + 1; }) == GC_OK);
static_assert(validate_with([](gc_config& c) { c.hold_n = gcore::guard_n; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.max_dt_us = 0; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.max_dt_us = 1; }) == GC_OK);
static_assert(validate_with([](gc_config& c) { c.max_dt_us = 20000; }) == GC_OK);
static_assert(validate_with([](gc_config& c) { c.max_dt_us = 20001; }) == GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.min_stroke_us = c.max_stroke_us; }) ==
              GC_ERR_CONFIG);
static_assert(validate_with([](gc_config& c) { c.max_stroke_us = 6553500; }) == GC_OK);
static_assert(validate_with([](gc_config& c) { c.max_stroke_us = 6553501; }) == GC_ERR_CONFIG);
// No range: any value passes.
static_assert(validate_with([](gc_config& c) {
                  c.min_extent_mm = 0;
                  c.max_v_end_mm_s = 0;
                  c.accel_offset_lsb[0] = -32768;
                  c.gyro_bias_q8[2] = std::numeric_limits<std::int32_t>::min();
              }) == GC_OK);
// Order: the first failing class in the gc_status table wins.
static_assert(validate_with([](gc_config& c) {
                  c.abi_version = 0;
                  c.ctrl1_xl = 0;
                  c.hold_n = 0;
              }) == GC_ERR_VERSION);
static_assert(validate_with([](gc_config& c) {
                  c.ctrl2_g = 0;
                  c.hold_n = 0;
              }) == GC_ERR_SCALE);

// --- ring ---------------------------------------------------------------------------

constexpr gcore::Raw raw_dt(std::uint16_t dt) {
    return gcore::Raw{.dt = dt, .gyro = {}, .accel = {}};
}

// Pushes 1, 2, ..., n as dt.
constexpr gcore::Ring ring_of(int n) {
    gcore::Ring r{};
    for (int i = 1; i <= n; ++i) {
        gcore::push(r, raw_dt(static_cast<std::uint16_t>(i)));
    }
    return r;
}

static_assert(ring_of(0).count == 0);
static_assert(ring_of(3).count == 3 && gcore::at(ring_of(3), 0).dt == 3 &&
              gcore::at(ring_of(3), 2).dt == 1);
static_assert(ring_of(gcore::ring_n).count == gcore::ring_n &&
              gcore::at(ring_of(gcore::ring_n), gcore::ring_n - 1).dt == 1);
// Past full: the oldest drop out, the count stays.
static_assert(ring_of(100).count == gcore::ring_n && gcore::at(ring_of(100), 0).dt == 100 &&
              gcore::at(ring_of(100), gcore::ring_n - 1).dt == 100 - gcore::ring_n + 1);
static_assert([] {
    gcore::Ring r = ring_of(60);
    gcore::clear(r);
    gcore::push(r, raw_dt(7));
    return r.count == 1 && gcore::at(r, 0).dt == 7;
}());

// --- still window ---------------------------------------------------------------------

// win_n samples alternating between a and b on every gyro and accel axis.
constexpr gcore::Ring alternating(std::int16_t a, std::int16_t b) {
    gcore::Ring r{};
    for (int i = 0; i < gcore::win_n; ++i) {
        const std::int16_t x = i % 2 == 0 ? a : b;
        gcore::push(r, gcore::Raw{.dt = 1, .gyro = {x, x, x}, .accel = {x, x, x}});
    }
    return r;
}

// Alternating 100 +- d: S = 32 * 100, std exactly d.
static_assert(gcore::window_sums(alternating(90, 110)).gyro.s[0] == 3200);
static_assert(gcore::window_sums(alternating(90, 110)).accel.q[2] == 16LL * (8100 + 12100));
// spread: std < th, strictly.
static_assert(gcore::spread_ok(gcore::window_sums(alternating(90, 110)).gyro, 11));
static_assert(!gcore::spread_ok(gcore::window_sums(alternating(90, 110)).gyro, 10));
// Extremes stay in int64: std 32767.5 at full scale.
static_assert(!gcore::spread_ok(gcore::window_sums(alternating(-32768, 32767)).gyro, 32767));
static_assert(gcore::spread_ok(gcore::window_sums(alternating(-32768, 32767)).gyro, 32768));
// rate: |mean - bias| < th, strictly. Mean 100.
constexpr gcore::Sums mean100 = gcore::window_sums(alternating(90, 110)).gyro;
static_assert(gcore::rate_ok(mean100, {0, 0, 0}, 101) && !gcore::rate_ok(mean100, {0, 0, 0}, 100));
static_assert(gcore::rate_ok(mean100, {25600, 25600, 25600}, 1)); // bias 100.0
static_assert(gcore::rate_ok(mean100, {25600 - 255, 25600, 25600}, 1));
static_assert(!gcore::rate_ok(mean100, {25600, 25600 + 256, 25600}, 1));
static_assert(!gcore::rate_ok(mean100, {25600, 25600, -25600}, 200));

// Bias: 1/64 of the way to the window mean, rounded; only under the bias thresholds.
constexpr std::array<std::int32_t, 3> biased(gcore::Ring r, std::array<std::int32_t, 3> b,
                                             std::uint16_t th) {
    gc_config cfg{};
    cfg.bias_gyro_std_lsb = th;
    cfg.bias_accel_std_lsb = th;
    gcore::update_bias(b, gcore::window_sums(r), cfg);
    return b;
}
static_assert(biased(alternating(90, 110), {0, 2560, 25600}, 11) ==
              std::array<std::int32_t, 3>{400, 2560 + 360, 25600});
static_assert(biased(alternating(90, 110), {0, 0, -1}, 10) ==
              std::array<std::int32_t, 3>{0, 0, -1});
// rdiv on both: mean 100.5 * 256 = 25728; (25728 - 25600) / 64 = 2; -(32 / 64) rounds to -1.
static_assert(biased(alternating(100, 101), {25600, 25760, 0}, 1) ==
              std::array<std::int32_t, 3>{25602, 25759, 402});

// --- stroke start: the gravity window -------------------------------------------------

// A full ring: ages 0 .. guard_n and past the window hold +-30000 (must not count); the
// gravity window (ages guard_n + 1 .. guard_n + grav_n) alternates z between z0 and z0 + 1
// and x between x0 and x0 - 1.
constexpr gcore::Ring gravity_ring(std::int16_t x0, std::int16_t z0) {
    gcore::Ring r{};
    for (int age = gcore::ring_n - 1; age >= 0; --age) {
        const bool window = age > gcore::guard_n && age <= gcore::guard_n + gcore::grav_n;
        const std::int16_t odd = age % 2 == 0 ? 0 : 1;
        const std::int16_t far = age <= gcore::guard_n ? 30000 : -30000;
        gcore::Raw x{};
        x.accel = window ? std::array<std::int16_t, 3>{static_cast<std::int16_t>(x0 - odd), 0,
                                                       static_cast<std::int16_t>(z0 + odd)}
                         : std::array<std::int16_t, 3>{far, far, far};
        gcore::push(r, x);
    }
    return r;
}

struct Cal {
    std::int16_t off;
    std::int32_t gain_q30;
};

// At 4 g full scale (122 ug/LSB).
constexpr std::array<std::int32_t, 3> gravity_with(Cal cal) {
    gc_config cfg = valid_config();
    for (std::size_t i = 0; i < 3; ++i) {
        cfg.accel_offset_lsb[i] = cal.off;
        cfg.accel_gain_q30[i] = cal.gain_q30;
    }
    return gcore::gravity_ug(gravity_ring(-100, 8197), cfg, 122);
}

// Offset 0, gain 1: the mean times sa, rounded once. z: 8197.5 * 122 = 1000095.
static_assert(gravity_with({.off = 0, .gain_q30 = gcore::one_q30}) ==
              std::array<std::int32_t, 3>{-12261, 0, 1000095}); // x: -100.5 * 122 = -12261
// Offset 1, gain 1.5: z (8196.5 * 1.5 * 122 = 1499959.5) and x (-101.5 * 1.5 * 122 =
// -18574.5) are halves, rounded away from zero; y is -1.5 * 122 = -183.
static_assert(gravity_with({.off = 1, .gain_q30 = 3 << 29}) ==
              std::array<std::int32_t, 3>{-18575, -183, 1499960});
// 16 g scale, full-scale readings, gain at its maximum: inside int64.
static_assert([] {
    gc_config cfg = valid_config();
    cfg.accel_offset_lsb[0] = -32768;
    cfg.accel_gain_q30[0] = gcore::gain_max_q30;
    const auto g = gcore::gravity_ug(gravity_ring(32767, 0), cfg, 488);
    // (32766.5 + 32768) * gain_max (1.1 - 4e-10) * 488 = 35178919.588
    return g[0] == 35178920;
}());

// --- integration ---------------------------------------------------------------------

constexpr gcore::Quat identity{gcore::one_q30, 0, 0, 0};
constexpr gcore::Scales scales_4g_500dps = gcore::scales(valid_config()); // 122, 17500

// About 100 dps (5714 LSB at 500 dps) for 10 ms: 999950 udeg, half of it in Q30 rad.
static_assert(gcore::half_angle({0, 0, 5714}, {0, 0, 0}, 17500, 10000) ==
              std::array<std::int64_t, 3>{0, 0, 9369697});
// The bias comes off first (100 / 256 LSB): 959885.75 udeg rounds up.
static_assert(gcore::half_angle({5714, 0, 0}, {100, 0, 0}, 17500, 9600) ==
              std::array<std::int64_t, 3>{8994291, 0, 0});
// Negative rates round the same way.
static_assert(gcore::half_angle({0, -5714, 0}, {0, 0, 0}, 17500, 10000) ==
              std::array<std::int64_t, 3>{0, -9369697, 0});

// h = 2^20 about z from identity: dq = (2^30 - 2^40 / 2^31, 0, 0, 2^20), exactly; |q|^2 =
// 2^60 + 2^18 rounds to 2^30, so the renormalization leaves it.
constexpr gcore::Quat small_turn{gcore::one_q30 - 512, 0, 0, 1 << 20};
static_assert(gcore::turn(identity, {0, 0, 1 << 20}) == small_turn);
static_assert(gcore::renormalize(small_turn) == small_turn);
static_assert(gcore::renormalize(identity) == identity);
// Off the unit sphere by 1e-4: one Newton step leaves 1 - 1e-8 (2^30 * 1e-8 = 11 LSB,
// plus the rounding of |q|^2).
static_assert(gcore::renormalize({gcore::one_q30 + 107374, 0, 0, 0}) ==
              gcore::Quat{gcore::one_q30 - 16, 0, 0, 0});

// R(identity) is the unit matrix; a quarter turn about z sends board x to start y.
static_assert(gcore::rotation(identity) ==
              gcore::Mat3{{{gcore::one_q30, 0, 0}, {0, gcore::one_q30, 0}, {0, 0, gcore::one_q30}}});
static_assert([] {
    constexpr std::int32_t c = 759250125; // 2^30 / sqrt(2)
    const gcore::Mat3 r = gcore::rotation({c, 0, 0, c});
    const auto near = [](std::int64_t x, std::int64_t want) {
        return x - want <= 2 && want - x <= 2;
    };
    return near(r[1][0], gcore::one_q30) && near(r[0][1], -gcore::one_q30) &&
           near(r[0][0], 0) && near(r[1][1], 0) && near(r[2][2], gcore::one_q30) &&
           r[2][0] == 0 && r[0][2] == 0;
}());

// The calibration: (raw - off) * gain * sa, one rounding.
static_assert([] {
    gc_config cfg = valid_config();
    cfg.accel_offset_lsb[0] = 10;
    cfg.accel_offset_lsb[2] = -10;
    cfg.accel_gain_q30[1] = 3 << 29;
    // x: 8187 * 122; y: 1.5 * 3 * 122 = 549; z: -32758 * 122.
    return gcore::accel_ug({8197, 3, -32768}, cfg, 122) ==
           std::array<std::int64_t, 3>{998814, 549, -3996476};
}());

// A stroke from rest at identity, with this gravity (ug, start frame).
constexpr gcore::Stroke fresh(std::array<std::int32_t, 3> gravity_ug) {
    gcore::Stroke s{};
    s.gravity_ug = gravity_ug;
    s.q = identity;
    s.step = 1;
    s.n = 1;
    return s;
}

constexpr gcore::Raw raw(std::array<std::int16_t, 3> gyro, std::array<std::int16_t, 3> accel,
                         std::uint16_t dt = 9600) {
    return gcore::Raw{.dt = dt, .gyro = gyro, .accel = accel};
}

// n samples of x, t counted from t_s0 = 0.
constexpr gcore::Stroke run(gcore::Stroke s, const gcore::Raw& x, int n,
                            const gc_config& cfg = valid_config()) {
    for (int i = 0; i < n; ++i) {
        gcore::integrate(s, x, cfg, scales_4g_500dps, s.t + x.dt);
    }
    return s;
}

// At rest, exactly at gravity and bias: nothing moves, nothing turns, ever.
static_assert([] {
    const gcore::Stroke s = run(fresh({0, 0, 8197 * 122}), raw({0, 0, 0}, {0, 0, 8197}), 300);
    return s.q == identity && s.v == std::array<std::int64_t, 3>{} &&
           s.p == std::array<std::int64_t, 3>{} && !s.clipped && s.c == 300;
}());

// 1 g along x for 10 x 10 ms, no gravity in the frame: v grows by the same step each
// sample, p by the new v times dt.
static_assert([] {
    const gcore::Stroke s = run(fresh({0, 0, 0}), raw({0, 0, 0}, {8197, 0, 0}, 10000), 10);
    // f = 8197 * 122 = 1000034 ug; dv = 1000034 * 10^4 * 980665 / 10^8 = 98069834.2 nm/s.
    constexpr std::int64_t dv = 98069834;
    // dp_k = k * dv * 10^4 / 10^6 = k * 980698.34 nm, rounded per step.
    std::int64_t p = 0;
    for (std::int64_t k = 1; k <= 10; ++k) {
        p += gcore::rdiv(k * dv * 10000, 1000000);
    }
    return s.v == std::array<std::int64_t, 3>{10 * dv, 0, 0} &&
           s.p == std::array<std::int64_t, 3>{p, 0, 0};
}());

// Turn ~90 deg about z (90 x 10 ms at 99.995 dps), then 1 g along board x: it must show
// up along start +y (a transposed R would give -y).
static_assert([] {
    gcore::Stroke s = run(fresh({0, 0, 0}), raw({0, 0, 5714}, {0, 0, 0}, 10000), 90);
    // q ~ (cos 45, 0, 0, sin 45): 759279940 each for exactly 89.9955 deg.
    if (s.v != std::array<std::int64_t, 3>{} || s.q[1] != 0 || s.q[2] != 0 ||
        s.q[0] < 759200000 || s.q[0] > 759300000 || s.q[3] < 759200000 ||
        s.q[3] > 759300000) {
        return false;
    }
    s = run(s, raw({0, 0, 0}, {8197, 0, 0}, 10000), 1);
    // All of one step (98069834) on y; on x, cos(89.996 deg) of it, under 10^4.
    return s.v[1] == 98069834 && s.v[0] > 0 && s.v[0] < 10000 && s.v[2] == 0;
}());

// The clip flag: |reading| >= clip_lsb on any axis, -32768 included.
static_assert(!run(fresh({}), raw({0, 0, 0}, {32699, 0, 0}), 1).clipped);
static_assert(run(fresh({}), raw({0, 0, 0}, {0, 0, 32700}), 1).clipped);
static_assert(run(fresh({}), raw({0, -32768, 0}, {0, 0, 0}), 1).clipped);
static_assert([] {
    gcore::Stroke s = run(fresh({}), raw({0, 0, 0}, {-32700, 0, 0}), 1);
    s = run(s, raw({0, 0, 0}, {0, 0, 0}), 5); // and it stays set
    return s.clipped;
}());

// The point buffer: point i at c = i * step. Full at 64 points (c = 0 .. 63); the store at
// c = 64 first keeps 0, 2, .. 62 (32 points), doubles step, then stores c = 64.
static_assert([] {
    const gcore::Raw x = raw({0, 0, 0}, {0, 0, 0}); // 96 x 100 us per sample
    gcore::Stroke s = run(fresh({}), x, 63);
    if (s.n != 64 || s.step != 1 || s.points[63].t_100us != 63 * 96) {
        return false;
    }
    s = run(s, x, 1);
    if (s.n != 33 || s.step != 2 || s.points[1].t_100us != 2 * 96 ||
        s.points[31].t_100us != 62 * 96 || s.points[32].t_100us != 64 * 96) {
        return false;
    }
    s = run(s, x, 1); // c = 65: not a multiple of 2
    if (s.n != 33) {
        return false;
    }
    s = run(s, x, 63); // c = 128: full at c = 126, step 4, keeps 0, 4, .. 124, stores 128
    return s.n == 33 && s.step == 4 && s.points[1].t_100us == 4 * 96 &&
           s.points[32].t_100us == 128 * 96;
}());

// Points round p (nm) to mm and t to 100 us, halves away from zero, and saturate.
static_assert([] {
    const gc_point a = gcore::point({1500000, -1500000, 1499999}, 149);
    const gc_point b = gcore::point({40000000000, -40000000000, 0}, 6553550);
    return a.x_mm == 2 && a.y_mm == -2 && a.z_mm == 1 && a.t_100us == 1 && b.x_mm == 32767 &&
           b.y_mm == -32768 && b.t_100us == 65535;
}());

// --- stroke end ------------------------------------------------------------------------

// Points at 0, 0.5, 1 and 1.5 s (x 0, 100, 200, 300 mm); the snapshot at 1 s with
// p = 200 mm and a leftover 100 mm/s on x.
constexpr gcore::Stroke ended(std::uint32_t t_s0 = 1000) {
    gcore::Stroke s = fresh({0, 0, 1000000});
    s.t_s0 = t_s0;
    s.points[1] = {.x_mm = 100, .y_mm = 0, .z_mm = 0, .t_100us = 5000};
    s.points[2] = {.x_mm = 200, .y_mm = 0, .z_mm = 0, .t_100us = 10000};
    s.points[3] = {.x_mm = 300, .y_mm = 0, .z_mm = 0, .t_100us = 15000};
    s.n = 4;
    s.snap = {.v = {100000000, 0, -400000}, .p = {200000000, 0, 0}, .t = t_s0 + 1000000,
              .clipped = false};
    s.snapped = true;
    return s;
}

// c = v t^2 / (2 T): 12.5 mm at 0.5 s (rounds to 13), 50 mm at 1 s. The 1.5 s point goes;
// the snapshot point is appended even though a point already sits at t = T.
static_assert([] {
    gcore::Stroke s = ended(0xFFFFFFF0U); // the clock wraps inside the stroke
    const std::uint8_t reason = gcore::finish(s, valid_config());
    const auto at = [&](std::size_t i, std::int16_t x, std::uint16_t t_100us) {
        return s.points[i].x_mm == x && s.points[i].y_mm == 0 && s.points[i].z_mm == 0 &&
               s.points[i].t_100us == t_100us;
    };
    return reason == 0 && s.done && s.n == 4 && at(0, 0, 0) && at(1, 87, 5000) &&
           at(2, 150, 10000) && at(3, 150, 10000) && s.duration_us == 1000000 &&
           s.v_end_mm_s == 100 && s.extent_mm == 150;
}());

// The checks, in order: CLIPPED, SHORT (duration), DRIFT, SHORT (extent).
template <typename F> constexpr std::uint8_t end_with(F change) {
    gcore::Stroke s = ended();
    gc_config cfg = valid_config();
    change(s, cfg);
    const std::uint8_t reason = gcore::finish(s, cfg);
    return s.done == (reason == 0) ? reason : 0xFF;
}

static_assert(end_with([](gcore::Stroke&, gc_config&) {}) == 0);
static_assert(end_with([](gcore::Stroke& s, gc_config& c) {
                  s.snap.clipped = true;
                  c.min_stroke_us = 2000000;
              }) == GC_REJ_CLIPPED);
static_assert(end_with([](gcore::Stroke&, gc_config& c) {
                  c.min_stroke_us = 1000001;
                  c.max_v_end_mm_s = 99;
              }) == GC_REJ_SHORT);
static_assert(end_with([](gcore::Stroke&, gc_config& c) { c.min_stroke_us = 1000000; }) == 0);
static_assert(end_with([](gcore::Stroke&, gc_config& c) {
                  c.max_v_end_mm_s = 99;
                  c.min_extent_mm = 151;
              }) == GC_REJ_DRIFT);
static_assert(end_with([](gcore::Stroke&, gc_config& c) { c.max_v_end_mm_s = 100; }) == 0);
static_assert(end_with([](gcore::Stroke&, gc_config& c) { c.min_extent_mm = 151; }) ==
              GC_REJ_SHORT);
static_assert(end_with([](gcore::Stroke&, gc_config& c) { c.min_extent_mm = 150; }) == 0);

// DRIFT compares before saturation: 70 m/s is rejected even at max_v_end_mm_s 65535, and
// before the correction (whose products it bounds). The event still gets T and v_end.
static_assert([] {
    gcore::Stroke s = ended();
    s.snap.v = {70000000000, 0, 0};
    gc_config cfg = valid_config();
    cfg.max_v_end_mm_s = 65535;
    const std::uint8_t reason = gcore::finish(s, cfg);
    return reason == GC_REJ_DRIFT && !s.done && s.v_end_mm_s == 65535 &&
           s.duration_us == 1000000 && s.n == 4;
}());

// A full buffer at the end: compacted first, then the snapshot point appended.
static_assert([] {
    gcore::Stroke s = run(fresh({}), raw({0, 0, 0}, {0, 0, 0}), 63); // 64 points
    s.snap = {.v = {}, .p = {}, .t = s.t, .clipped = false};
    const std::uint8_t reason = gcore::finish(s, valid_config());
    return reason == GC_REJ_SHORT && s.n == 33 && s.step == 2 &&
           s.points[31].t_100us == 62 * 96 && s.points[32].t_100us == 63 * 96;
}());

// --- gc_init through the C ABI ------------------------------------------------------

void test_init(gesture_test::Tests& t) {
    gc_state s{};
    gc_config cfg = valid_config();
    check(t, gc_init(&s, &cfg) == GC_OK);
    cfg.abi_version = 0;
    check(t, gc_init(&s, &cfg) == GC_ERR_VERSION);
    cfg = valid_config();
    cfg.max_dt_us = 0;
    check(t, gc_init(&s, &cfg) == GC_ERR_CONFIG);
    cfg = valid_config();
    check(t, gc_init(&s, &cfg) == GC_OK); // usable again after an error
}

// --- gc_feed: the state machine -------------------------------------------------------

constexpr std::uint32_t period_us = 9600;

struct Feeder {
    gc_state s{};
    std::uint32_t t = 1000;

    explicit Feeder(const gc_config& cfg) {
        gc_init(&s, &cfg);
    }

    gc_event feed(std::array<std::int16_t, 3> gyro, std::array<std::int16_t, 3> accel,
                  std::uint32_t dt = period_us) {
        t += dt;
        const gc_sample in{.t_us = t,
                           .gyro = {gyro[0], gyro[1], gyro[2]},
                           .accel = {accel[0], accel[1], accel[2]}};
        return gc_feed(&s, &in);
    }

    // Board at rest: flat, 1 g on z at 4 g full scale, gyro at the bias.
    gc_event rest(std::uint32_t dt = period_us) {
        return feed({0, 0, 0}, {0, 0, 8197}, dt);
    }

    // Feeds rest samples until an event; returns its 1-based sample count, 0 if none in n.
    int rest_until_event(int n, gc_event& ev) {
        for (int i = 1; i <= n; ++i) {
            ev = rest();
            if (ev.kind != GC_EV_NONE) {
                return i;
            }
        }
        return 0;
    }

    [[nodiscard]] const gcore::State& st() const {
        return gcore::state(&s);
    }
};

bool is(const gc_event& ev, std::uint8_t kind, std::uint8_t state, std::uint8_t reason = 0) {
    return ev.kind == kind && ev.state == state && ev.reason == reason && ev.reserved == 0 &&
           ev.points == 0 && ev.v_end_mm_s == 0 && ev.duration_us == 0;
}

// The first window is full at sample win_n; hold_n quiet windows later it is READY.
constexpr int ready_at = gcore::win_n + 20 - 1;

void test_warmup(gesture_test::Tests& t) {
    Feeder f(valid_config());
    gc_event ev{};
    check(t, f.rest_until_event(200, ev) == ready_at);
    check(t, is(ev, GC_EV_READY, GC_ST_STILL));
    check(t, is(f.rest(), GC_EV_NONE, GC_ST_STILL));

    // A tremor above the gyro std threshold (171) never confirms a hold.
    Feeder g(valid_config());
    bool none = true;
    for (int i = 0; i < 200; ++i) {
        const auto x = static_cast<std::int16_t>(i % 2 == 0 ? 172 : -172);
        none = none && is(g.feed({x, 0, 0}, {0, 0, 8197}), GC_EV_NONE, GC_ST_WARMUP);
    }
    check(t, none);
    // So does a steady turn: gyro z at the rate threshold (229) above the bias.
    Feeder h(valid_config());
    none = true;
    for (int i = 0; i < 200; ++i) {
        none = none && is(h.feed({0, 0, 229}, {0, 0, 8197}), GC_EV_NONE, GC_ST_WARMUP);
    }
    check(t, none);
    // One LSB below it is quiet.
    Feeder k(valid_config());
    int n = 0;
    while (n < 200 && k.feed({0, 0, 228}, {0, 0, 8197}).kind == GC_EV_NONE) {
        ++n;
    }
    check(t, n + 1 == ready_at);
}

void test_moving(gesture_test::Tests& t) {
    Feeder f(valid_config());
    gc_event ev{};
    f.rest_until_event(200, ev);
    // A jolt that pushes the window's accel x std over 123 LSB starts a move at once.
    check(t, is(f.feed({0, 0, 0}, {1000, 0, 8197}), GC_EV_MOVING, GC_ST_MOVING));
    check(t, is(f.rest(), GC_EV_NONE, GC_ST_MOVING));
    check(t, f.st().streak == 0); // the jolt is still in the window
}

void test_start(gesture_test::Tests& t) {
    // Gyro x at 10 LSB: the bias moves during STILL and must freeze at the start.
    Feeder f(valid_config());
    gc_event ev{};
    for (int i = 0; i < ready_at; ++i) {
        f.feed({10, 0, 0}, {0, 0, 8197});
    }
    // Uneven steps before the move: t_s0 must sum the right ones.
    for (std::uint32_t i = 0; i < 30; ++i) {
        ev = f.feed({10, 0, 0}, {0, 0, 8197}, 9000 + i);
    }
    check(t, ev.state == GC_ST_STILL);
    const auto bias = f.st().bias_q8;
    check(t, bias[0] > 0);
    check(t, is(f.feed({10, 0, 0}, {1000, 0, 8197}, 9500), GC_EV_MOVING, GC_ST_MOVING));
    const gcore::Stroke& s = f.st().stroke;
    std::uint32_t replay_us = 9500;
    for (std::uint32_t i = 20; i < 30; ++i) {
        replay_us += 9000 + i;
    }
    check(t, s.t_s0 == f.t - replay_us);
    check(t, s.t == f.t && s.c == gcore::guard_n + 1); // replayed up to sample k
    check(t, s.bias_q8 == bias);
    check(t, s.gravity_ug == std::array<std::int32_t, 3>{0, 0, 8197 * 122});
    // The bias has not caught up with the 10 LSB yet: the replay turns a little about x.
    check(t, s.q[0] < gcore::one_q30 && s.q[1] > 0 && s.q[2] == 0 && s.q[3] == 0);
    check(t, s.n == gcore::guard_n + 2 && s.step == 1 && !s.clipped); // point 0 + replay
    check(t, s.v[0] > 0); // the jolt, on x
    // MOVING: one sample integrated per feed; the bias stays.
    f.feed({10, 0, 0}, {0, 0, 8197});
    check(t, s.t == f.t && s.c == gcore::guard_n + 2);
    check(t, f.st().bias_q8 == bias);

    // The clock wraps inside the replay: t_s0 is still guard_n + 1 steps back.
    Feeder w(valid_config());
    w.t = 0xFFFFFFFFU - ((ready_at + 5) * period_us);
    w.rest_until_event(200, ev);
    for (int i = 0; i < 8; ++i) {
        w.rest();
    }
    w.feed({0, 0, 0}, {1000, 0, 8197});
    const gcore::Stroke& ws = w.st().stroke;
    check(t, w.st().mode == GC_ST_MOVING && w.t < 0x10000U);
    check(t, ws.t_s0 == w.t - ((gcore::guard_n + 1) * period_us) && ws.t_s0 > w.t);
}

void test_gap(gesture_test::Tests& t) {
    gc_event ev{};
    // dt 0, dt > max_dt: a gap restarts the warmup (silently outside MOVING).
    for (const std::uint32_t dt : {0U, 14424U}) {
        Feeder f(valid_config());
        f.rest_until_event(200, ev);
        check(t, is(f.rest(dt), GC_EV_NONE, GC_ST_WARMUP));
        check(t, f.st().ring.count == 1 && f.st().streak == 0);
        check(t, f.rest_until_event(200, ev) == ready_at - 1);
    }
    // max_dt itself is a step, not a gap.
    Feeder f(valid_config());
    f.rest_until_event(200, ev);
    check(t, is(f.rest(14423), GC_EV_NONE, GC_ST_STILL));
    // In MOVING a gap rejects the move.
    f.feed({0, 0, 0}, {1000, 0, 8197});
    check(t, is(f.rest(30000), GC_EV_REJECTED, GC_ST_WARMUP, GC_REJ_TIME_GAP));
    check(t, is(f.rest(), GC_EV_NONE, GC_ST_WARMUP));
    // The MCU clock wraps: no gap across 2^32.
    Feeder w(valid_config());
    w.t = 0xFFFFFFFFU - (10 * period_us);
    check(t, w.rest_until_event(200, ev) == ready_at);
}

void test_bias(gesture_test::Tests& t) {
    // Gyro x steady at 10 LSB: within every threshold. No update in WARMUP, then 1/64
    // of the remaining distance per STILL sample.
    gc_config cfg = valid_config();
    Feeder f(cfg);
    for (int i = 1; i < ready_at; ++i) {
        f.feed({10, 0, 0}, {0, 0, 8197});
    }
    check(t, f.st().bias_q8[0] == 0);
    check(t, is(f.feed({10, 0, 0}, {0, 0, 8197}), GC_EV_READY, GC_ST_STILL));
    check(t, f.st().bias_q8[0] == 0); // the READY sample was still WARMUP at step 4
    f.feed({10, 0, 0}, {0, 0, 8197});
    check(t, f.st().bias_q8[0] == 40); // rdiv(2560, 64)
    f.feed({10, 0, 0}, {0, 0, 8197});
    check(t, f.st().bias_q8[0] == 40 + 39); // rdiv(2520, 64) = 39.375
    for (int i = 0; i < 1000; ++i) {
        f.feed({10, 0, 0}, {0, 0, 8197});
    }
    // Converges to within the rounding dead band: |d| / 64 rounds to 0 for |d| < 32.
    check(t, f.st().bias_q8[0] > 2560 - 32 && f.st().bias_q8[0] <= 2560);
    check(t, f.st().bias_q8[1] == 0 && f.st().bias_q8[2] == 0);

    // Above the bias thresholds (still quiet): no update.
    Feeder g(cfg);
    for (int i = 0; i < ready_at + 50; ++i) {
        const auto x = static_cast<std::int16_t>(i % 2 == 0 ? 12 : -12); // std 12 > 11
        g.feed({x, 0, 0}, {0, 0, 8197});
    }
    check(t, g.st().mode == GC_ST_STILL && g.st().bias_q8[0] == 0);
}

void test_live(gesture_test::Tests& t) {
    Feeder f(valid_config());
    gc_point p{.x_mm = 1, .y_mm = 1, .z_mm = 1, .t_100us = 1};
    check(t, gc_live(&f.s, &p) == 0 && p.x_mm == 0 && p.y_mm == 0 && p.z_mm == 0 &&
                 p.t_100us == 0);
    gc_event ev{};
    f.rest_until_event(200, ev);
    check(t, gc_live(&f.s, &p) == 0);
    f.feed({0, 0, 0}, {1000, 0, 8197}); // MOVING: the replay already moved
    for (int i = 0; i < 5; ++i) {
        f.rest();
        const gcore::Stroke& s = f.st().stroke;
        check(t, gc_live(&f.s, &p) == 1);
        // As a stored point would be, here every sample (step 1).
        const gc_point& last = s.points[s.n - 1];
        check(t, p.x_mm == last.x_mm && p.y_mm == last.y_mm && p.z_mm == last.z_mm &&
                     p.t_100us == last.t_100us);
        check(t, p.t_100us == static_cast<std::uint32_t>(gcore::guard_n + 2 + i) * period_us / 100);
    }
    check(t, f.st().stroke.v[0] > 0 && f.st().stroke.p[0] > 0);
    // A gap ends the move: nothing live.
    f.rest(30000);
    check(t, gc_live(&f.s, &p) == 0 && p.t_100us == 0);
}

// A hold, then a move along x: +a for n samples, -a for n (back to rest), then rest.
// 1000 LSB at 4 g is 1.2 m/s^2; 30 + 30 samples of 9.6 ms move it ~100 mm.
struct Move {
    std::int16_t a = 1000;
    int n = 30;
    int brake = 30;          // samples of -a
    std::int16_t clip_at = 0; // if not 0: the 6th sample reads this instead of a
};

constexpr std::array<std::int16_t, 3> flat(std::int16_t ax) {
    return {ax, 0, 8197};
}

// Returns the first event that is not NONE, from the MOVING one on, within 200 rest
// samples after the move.
gc_event stroke(Feeder& f, Move m) {
    gc_event ev{};
    f.rest_until_event(200, ev); // READY
    ev = f.feed({0, 0, 0}, flat(m.a));
    if (ev.kind != GC_EV_MOVING) {
        return ev;
    }
    for (int i = 1; i < m.n; ++i) {
        f.feed({0, 0, 0}, flat(i == 5 && m.clip_at != 0 ? m.clip_at : m.a));
    }
    for (int i = 0; i < m.brake; ++i) {
        f.feed({0, 0, 0}, flat(static_cast<std::int16_t>(-m.a)));
    }
    for (int i = 0; i < 200; ++i) {
        ev = f.rest();
        if (ev.kind != GC_EV_NONE) {
            return ev;
        }
    }
    return ev;
}

void test_stroke(gesture_test::Tests& t) {
    Feeder f(valid_config());
    gc_stroke_info info{};
    info.duration_us = 1;
    std::array<gc_point, gcore::points_n> pts{};
    check(t, gc_stroke(&f.s, &info, pts.data(), gcore::points_n) == 0 && info.duration_us == 0);

    const gc_event ev = stroke(f, {});
    // Snapshot at the first quiet window: win_n samples after the last braking one; the
    // end hold_n - 1 samples later.
    const std::uint32_t big_t = (gcore::guard_n + 1 + 59 + gcore::win_n) * period_us;
    check(t, ev.kind == GC_EV_STROKE && ev.state == GC_ST_STILL && ev.reason == 0);
    check(t, ev.duration_us == big_t && ev.v_end_mm_s <= 1);
    check(t, f.st().streak == 20 && gc_live(&f.s, pts.data()) == 0);

    check(t, gc_stroke(&f.s, &info, pts.data(), gcore::points_n) == ev.points);
    check(t, info.duration_us == big_t && info.points == ev.points &&
                 info.v_end_mm_s == ev.v_end_mm_s && info.step == 2);
    check(t, info.gravity_ug[0] == 0 && info.gravity_ug[1] == 0 &&
                 info.gravity_ug[2] == 8197 * 122);
    // ~1.2 m/s^2 * (0.288 s)^2 = 100 mm along x, and nothing else.
    check(t, info.extent_mm >= 95 && info.extent_mm <= 110);
    const gc_point& last = pts[ev.points - 1];
    check(t, last.t_100us == big_t / 100 && std::cmp_equal(last.x_mm, info.extent_mm) && last.y_mm == 0 &&
                 last.z_mm == 0);
    // The snapshot (c = 102) falls on a stored point (step 2): docs/core.md appends it
    // anyway, so the last two points are the same.
    check(t, pts[ev.points - 2].t_100us == last.t_100us && pts[ev.points - 2].x_mm == last.x_mm);
    bool rising = pts[0].t_100us == 0 && pts[0].x_mm == 0;
    for (std::size_t i = 1; i + 1 < ev.points; ++i) {
        rising = rising && pts[i].t_100us > pts[i - 1].t_100us;
    }
    check(t, rising);
    // cap limits the copy, not info.
    check(t, gc_stroke(&f.s, &info, pts.data(), 3) == 3 && info.points == ev.points);
    // Readable until the next MOVING, through a gap too.
    f.rest(30000);
    check(t, gc_stroke(&f.s, &info, pts.data(), gcore::points_n) == ev.points);
    gc_event next{};
    f.rest_until_event(200, next);
    f.feed({0, 0, 0}, flat(1000));
    check(t, gc_stroke(&f.s, &info, pts.data(), gcore::points_n) == 0 && info.points == 0);
}

void test_stroke_rejects(gesture_test::Tests& t) {
    const auto end_hold = [](const gc_event& ev, std::uint8_t reason) {
        return ev.kind == GC_EV_REJECTED && ev.state == GC_ST_STILL && ev.reason == reason &&
               ev.points == 0 && ev.duration_us > 0;
    };
    gc_stroke_info info{};
    gc_point pt{};
    {
        Feeder f(valid_config());
        check(t, end_hold(stroke(f, {.clip_at = 32700}), GC_REJ_CLIPPED));
        check(t, gc_stroke(&f.s, &info, &pt, 1) == 0 && info.duration_us == 0);
    }
    {
        gc_config cfg = valid_config();
        cfg.min_stroke_us = 2000000;
        Feeder f(cfg);
        check(t, end_hold(stroke(f, {}), GC_REJ_SHORT));
    }
    {
        gc_config cfg = valid_config();
        cfg.min_extent_mm = 200;
        Feeder f(cfg);
        check(t, end_hold(stroke(f, {}), GC_REJ_SHORT));
    }
    {
        // 10 braking samples short: 1.2 m/s^2 * 96 ms = 117 mm/s left over.
        gc_config cfg = valid_config();
        cfg.max_v_end_mm_s = 100;
        Feeder f(cfg);
        const gc_event ev = stroke(f, {.brake = 20});
        check(t, end_hold(ev, GC_REJ_DRIFT) && ev.v_end_mm_s > 105 && ev.v_end_mm_s < 125);
    }
}

void test_long(gesture_test::Tests& t) {
    Feeder f(valid_config());
    gc_event ev{};
    f.rest_until_event(200, ev);
    check(t, f.feed({0, 0, 0}, flat(1000)).kind == GC_EV_MOVING);
    // At MOVING t - t_s0 is (guard_n + 1) steps; LONG once it passes max_stroke_us.
    constexpr int j_long = ((4000000 - ((gcore::guard_n + 1) * 9600)) / 9600) + 1; // 406
    int j = 1;
    for (; j < 1000; ++j) {
        ev = f.feed({0, 0, 0}, flat(j % 2 == 0 ? 1000 : -1000));
        if (ev.kind != GC_EV_NONE) {
            break;
        }
    }
    check(t, j == j_long && is(ev, GC_EV_REJECTED, GC_ST_WARMUP, GC_REJ_LONG));
    gc_point pt{};
    gc_stroke_info info{};
    check(t, gc_live(&f.s, &pt) == 0 && gc_stroke(&f.s, &info, &pt, 1) == 0);
    // Then the usual warmup: no gap, so the ring stays and READY comes on the hold.
    check(t, f.rest_until_event(200, ev) > 0 && is(ev, GC_EV_READY, GC_ST_STILL));
}

void test_pause(gesture_test::Tests& t) {
    // A pause shorter than the hold: the snapshot taken in it is dropped when the move
    // resumes, and the stroke ends at the later one.
    Feeder f(valid_config());
    gc_event ev{};
    f.rest_until_event(200, ev);
    f.feed({0, 0, 0}, flat(1000));
    for (int i = 0; i < 40; ++i) { // a quiet window: streak 9, snapshot at streak 1
        f.rest();
    }
    check(t, f.st().mode == GC_ST_MOVING && f.st().streak == 9 && f.st().stroke.snapped);
    f.feed({0, 0, 0}, flat(1000));
    check(t, f.st().streak == 0 && !f.st().stroke.snapped);
    f.feed({0, 0, 0}, flat(0));
    while ((ev = f.rest()).kind == GC_EV_NONE) {
    }
    // Snapshot win_n samples after the second jolt's -> 0 step, 2 + win_n after s0+11+40.
    check(t, ev.duration_us == (gcore::guard_n + 1 + 40 + 2 + gcore::win_n - 1) * period_us);
}

} // namespace

int main() {
    gesture_test::Tests t;
    test_init(t);
    test_warmup(t);
    test_moving(t);
    test_start(t);
    test_gap(t);
    test_bias(t);
    test_live(t);
    test_stroke(t);
    test_stroke_rejects(t);
    test_long(t);
    test_pause(t);
    return gesture_test::finish(t, "core_test");
}
