#pragma once

// docs/core.md "Stroke start" and "Integration": one stroke, from the start sample s0 to
// the newest integrated sample.

#include "arith.hpp"
#include "config.hpp"
#include "gesture/core.h"
#include "ring.hpp"

#include <array>
#include <cstdint>

namespace gcore {

inline constexpr std::int32_t one_q30 = 1 << 30;

struct Stroke {
    std::array<std::int32_t, 3> bias_q8;    // gyro bias, frozen at the start
    std::array<std::int32_t, 3> gravity_ug; // gravity window mean, calibrated, start frame
    std::array<std::int32_t, 4> q;          // w, x, y, z, Q30: start frame <- board frame
    std::array<std::int64_t, 3> v;          // nm/s, start frame
    std::array<std::int64_t, 3> p;          // nm, start frame
    std::uint32_t t_s0;                     // t_us of the start sample
    std::uint32_t t;                        // t_us of the newest integrated sample
    std::uint16_t c;                        // samples integrated since s0
    std::uint16_t step;                     // samples per stored point
    std::uint8_t n;                         // points stored
    bool clipped;                           // a raw reading at clip_lsb or beyond
    std::array<gc_point, points_n> points;
};

// Gravity, ug per axis: the mean raw accel of ages guard_n + 1 .. guard_n + grav_n (the
// window that ends at s0), calibrated. One rdiv over the sum, so the mean is not rounded
// first. |S - N * off| < 2^22, * gain < 2^53, * sa < 2^62.
[[nodiscard]] constexpr std::array<std::int32_t, 3> gravity_ug(const Ring& r, const gc_config& cfg,
                                                               std::int32_t sa) {
    std::array<std::int64_t, 3> s{};
    for (int age = guard_n + 1; age <= guard_n + grav_n; ++age) {
        for (std::size_t i = 0; i < 3; ++i) {
            s[i] += at(r, age).accel[i];
        }
    }
    std::array<std::int32_t, 3> g{};
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int64_t d = s[i] - (std::int64_t{grav_n} * cfg.accel_offset_lsb[i]);
        g[i] = static_cast<std::int32_t>(
            rdiv(d * cfg.accel_gain_q30[i] * sa, std::int64_t{grav_n} * one_q30));
    }
    return g;
}

// One sample at time t (its dt already in the ring entry).
// TODO(3c.1 step 4): rotation, velocity, position, clipped flag, points.
constexpr void integrate(Stroke& s, const Raw& /*x*/, std::uint32_t t) {
    s.t = t;
    s.c = satu16(s.c + 1);
}

// At sample k, the first non-quiet window after a hold (streak >= hold_n >= guard_n + 1,
// so the ring holds ages 0 .. guard_n + grav_n). t_k: t_us of sample k.
constexpr void start(Stroke& s, const Ring& r, const gc_config& cfg, std::int32_t sa,
                     const std::array<std::int32_t, 3>& bias_q8, std::uint32_t t_k) {
    s = Stroke{};
    s.bias_q8 = bias_q8;
    s.gravity_ug = gravity_ug(r, cfg, sa);
    s.q = {one_q30, 0, 0, 0};
    s.step = 1;
    s.n = 1; // point 0: (0, 0, 0) at t 0
    // s0 is guard_n + 1 steps back; unsigned, so the clock may wrap in between.
    std::uint32_t t = t_k;
    for (int age = 0; age <= guard_n; ++age) {
        t -= at(r, age).dt;
    }
    s.t_s0 = t;
    s.t = t;
    // The replay: ages guard_n .. 0, oldest first.
    for (int age = guard_n; age >= 0; --age) {
        t += at(r, age).dt;
        integrate(s, at(r, age), t);
    }
}

} // namespace gcore
