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
    check(t, s.q == std::array<std::int32_t, 4>{gcore::one_q30, 0, 0, 0});
    check(t, s.n == 1 && s.step == 1 && !s.clipped);
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

} // namespace

int main() {
    gesture_test::Tests t;
    test_init(t);
    test_warmup(t);
    test_moving(t);
    test_start(t);
    test_gap(t);
    test_bias(t);
    return gesture_test::finish(t, "core_test");
}
