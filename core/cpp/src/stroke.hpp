#pragma once

// docs/core.md "Stroke start", "Integration" and "Stroke end": one stroke, from the start
// sample s0 to the newest integrated sample, then cut back to the snapshot.

#include "arith.hpp"
#include "config.hpp"
#include "gesture/core.h"
#include "ring.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace gcore {

inline constexpr std::int32_t one_q30 = 1 << 30;

// The stroke as of the first quiet window after the move (streak 1).
struct Snap {
    std::array<std::int64_t, 3> v; // nm/s
    std::array<std::int64_t, 3> p; // nm
    std::uint32_t t;               // t_us
    bool clipped;
};

struct Stroke {
    std::array<std::int32_t, 3> bias_q8;    // gyro bias, frozen at the start
    std::array<std::int32_t, 3> gravity_ug; // gravity window mean, calibrated, start frame
    std::array<std::int32_t, 4> q;          // w, x, y, z, Q30: start frame <- board frame
    std::array<std::int64_t, 3> v;          // nm/s, start frame
    std::array<std::int64_t, 3> p;          // nm, start frame
    std::uint32_t t_s0;                     // t_us of the start sample
    std::uint32_t t;                        // t_us of the newest integrated sample
    std::uint32_t c;                        // samples integrated since s0
    std::uint32_t step;                     // samples per stored point (point i: c = i * step)
    std::uint8_t n;                         // points stored
    bool clipped;                           // a raw reading at clip_lsb or beyond
    bool snapped;                           // snap is valid
    bool done;                              // ended as GC_EV_STROKE: gc_stroke reads it
    Snap snap;
    std::uint32_t duration_us; // after the end: t_snap - t_s0
    std::uint16_t v_end_mm_s;  // after the end
    std::uint16_t extent_mm;   // after the end
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

using Quat = std::array<std::int32_t, 4>; // w, x, y, z, Q30
using Mat3 = std::array<std::array<std::int64_t, 3>, 3>;

// Half the turn of one sample, Q30 rad, per axis: w = 256 * gyro - bias_q8 (LSB x 256),
// ang_udeg = w * sg * dt / (256 * 10^6), h = ang_udeg * pi / 360 * 2^30 with pi = 355/113.
[[nodiscard]] constexpr std::array<std::int64_t, 3>
half_angle(const std::array<std::int16_t, 3>& gyro, const std::array<std::int32_t, 3>& bias_q8,
           std::int32_t sg, std::uint16_t dt) {
    std::array<std::int64_t, 3> h{};
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int64_t w = (std::int64_t{256} * gyro[i]) - bias_q8[i];
        const std::int64_t ang_udeg = rdiv(w * sg * dt, std::int64_t{256} * 1000000);
        h[i] = rdiv(ang_udeg * 355 * (std::int64_t{1} << 23), std::int64_t{113} * 2812500);
    }
    return h;
}

// q * dq, dq = (2^30 - |h|^2 / 2^31, h): the Hamilton product, one rdiv per component.
[[nodiscard]] constexpr Quat turn(const Quat& q, const std::array<std::int64_t, 3>& h) {
    const std::array<std::int64_t, 4> b{
        one_q30 - rdiv((h[0] * h[0]) + (h[1] * h[1]) + (h[2] * h[2]), std::int64_t{1} << 31), h[0],
        h[1], h[2]};
    const std::array<std::int64_t, 4> a{q[0], q[1], q[2], q[3]};
    const std::array<std::int64_t, 4> r{
        (a[0] * b[0]) - (a[1] * b[1]) - (a[2] * b[2]) - (a[3] * b[3]),
        (a[0] * b[1]) + (a[1] * b[0]) + (a[2] * b[3]) - (a[3] * b[2]),
        (a[0] * b[2]) - (a[1] * b[3]) + (a[2] * b[0]) + (a[3] * b[1]),
        (a[0] * b[3]) + (a[1] * b[2]) - (a[2] * b[1]) + (a[3] * b[0]),
    };
    Quat out{};
    for (std::size_t i = 0; i < 4; ++i) {
        out[i] = static_cast<std::int32_t>(rdiv(r[i], one_q30));
    }
    return out;
}

// One Newton step towards |q| = 1: q * (3 - |q|^2) / 2.
[[nodiscard]] constexpr Quat renormalize(const Quat& q) {
    std::int64_t sq = 0;
    for (const std::int64_t x : q) {
        sq += x * x;
    }
    const std::int64_t k = (3 * std::int64_t{one_q30}) - rdiv(sq, one_q30);
    Quat out{};
    for (std::size_t i = 0; i < 4; ++i) {
        out[i] = static_cast<std::int32_t>(rdiv(q[i] * k, std::int64_t{1} << 31));
    }
    return out;
}

// R(q), Q30: board frame -> start frame (v' = q v q*).
[[nodiscard]] constexpr Mat3 rotation(const Quat& q) {
    const std::int64_t w = q[0];
    const std::int64_t x = q[1];
    const std::int64_t y = q[2];
    const std::int64_t z = q[3];
    const auto d = [](std::int64_t sum) { return rdiv(sum, std::int64_t{1} << 29); };
    return Mat3{{
        {one_q30 - d((y * y) + (z * z)), d((x * y) - (w * z)), d((x * z) + (w * y))},
        {d((x * y) + (w * z)), one_q30 - d((x * x) + (z * z)), d((y * z) - (w * x))},
        {d((x * z) - (w * y)), d((y * z) + (w * x)), one_q30 - d((x * x) + (y * y))},
    }};
}

// Calibrated accel, ug: (raw - off) * gain * sa.
[[nodiscard]] constexpr std::array<std::int64_t, 3>
accel_ug(const std::array<std::int16_t, 3>& accel, const gc_config& cfg, std::int32_t sa) {
    std::array<std::int64_t, 3> a{};
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int64_t d = std::int64_t{accel[i]} - cfg.accel_offset_lsb[i];
        a[i] = rdiv(d * cfg.accel_gain_q30[i] * sa, one_q30);
    }
    return a;
}

// p (nm) at t (us since s0) as a point; stored points and gc_live share it.
[[nodiscard]] constexpr gc_point point(const std::array<std::int64_t, 3>& p, std::uint32_t t) {
    return gc_point{.x_mm = sat16(rdiv(p[0], 1000000)),
                    .y_mm = sat16(rdiv(p[1], 1000000)),
                    .z_mm = sat16(rdiv(p[2], 1000000)),
                    .t_100us = satu16(rdiv(t, 100))};
}

// A full buffer keeps points 0, 2, 4, ... and doubles step.
constexpr void compact(Stroke& s) {
    for (std::size_t i = 0; i < points_n / 2; ++i) {
        s.points[i] = s.points[2 * i];
    }
    s.n = points_n / 2;
    s.step *= 2;
}

// Stores the current point if c is a multiple of step, compacting first when full.
constexpr void store(Stroke& s) {
    if (s.c % s.step != 0) {
        return;
    }
    if (s.n == points_n) {
        compact(s);
        if (s.c % s.step != 0) {
            return;
        }
    }
    s.points[s.n] = point(s.p, s.t - s.t_s0);
    ++s.n;
}

// One sample x at time t: turn, rotate the accel into the start frame, remove gravity,
// integrate twice, store a point every step samples.
constexpr void integrate(Stroke& s, const Raw& x, const gc_config& cfg, const Scales& sc,
                         std::uint32_t t) {
    s.q = renormalize(turn(s.q, half_angle(x.gyro, s.bias_q8, sc.sg, x.dt)));
    const Mat3 r = rotation(s.q);
    const std::array<std::int64_t, 3> a = accel_ug(x.accel, cfg, sc.sa);
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int64_t f =
            rdiv((r[i][0] * a[0]) + (r[i][1] * a[1]) + (r[i][2] * a[2]), one_q30);
        s.v[i] += rdiv((f - s.gravity_ug[i]) * x.dt * 980665, 100000000);
        s.p[i] += rdiv(s.v[i] * x.dt, 1000000);
    }
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int32_t g = x.gyro[i];
        const std::int32_t ac = x.accel[i];
        if ((g < 0 ? -g : g) >= cfg.clip_lsb || (ac < 0 ? -ac : ac) >= cfg.clip_lsb) {
            s.clipped = true;
        }
    }
    s.t = t;
    ++s.c;
    store(s);
}

// End at the snapshot: the checks that need no points (CLIPPED, SHORT by duration, DRIFT),
// then cut the points back to the snapshot, append it, remove the leftover velocity
// (zero-velocity correction) and check the extent. Returns 0 (a stroke) or GC_REJ_*.
// DRIFT before the correction bounds v_end there (docs/core.md "Ranges").
[[nodiscard]] constexpr std::uint8_t finish(Stroke& s, const gc_config& cfg) {
    const std::uint32_t big_t = s.snap.t - s.t_s0;
    std::int64_t v_max = 0;
    for (const std::int64_t v : s.snap.v) {
        v_max = std::max(v_max, v < 0 ? -v : v);
    }
    const std::int64_t v_end = rdiv(v_max, 1000000); // mm/s, before saturation
    s.duration_us = big_t;
    s.v_end_mm_s = satu16(v_end);

    if (s.snap.clipped) {
        return GC_REJ_CLIPPED;
    }
    if (big_t < cfg.min_stroke_us) {
        return GC_REJ_SHORT;
    }
    if (std::cmp_greater(v_end, cfg.max_v_end_mm_s)) {
        return GC_REJ_DRIFT;
    }

    const std::int64_t t_end = rdiv(big_t, 100);
    while (s.n > 0 && std::cmp_greater(s.points[s.n - 1].t_100us, t_end)) {
        --s.n;
    }
    if (s.n == points_n) {
        compact(s);
    }
    s.points[s.n] = point(s.snap.p, big_t);
    ++s.n;

    // c = v_end * t^2 / (2 T): the drift a constant leftover acceleration leaves.
    std::uint16_t extent = 0;
    for (std::size_t k = 0; k < s.n; ++k) {
        gc_point& pt = s.points[k];
        const std::int64_t t = std::int64_t{100} * pt.t_100us;
        std::array<std::int16_t*, 3> xs{&pt.x_mm, &pt.y_mm, &pt.z_mm};
        for (std::size_t i = 0; i < 3; ++i) {
            const std::int64_t c =
                rdiv(rdiv(s.snap.v[i] * t, 1000000) * t, std::int64_t{2} * big_t);
            *xs[i] = sat16(*xs[i] - rdiv(c, 1000000));
            const std::int32_t x = *xs[i];
            extent = std::max(extent, static_cast<std::uint16_t>(x < 0 ? -x : x));
        }
    }
    s.extent_mm = extent;
    if (s.extent_mm < cfg.min_extent_mm) {
        return GC_REJ_SHORT;
    }
    s.done = true;
    return 0;
}

// At sample k, the first non-quiet window after a hold (streak >= hold_n >= guard_n + 1,
// so the ring holds ages 0 .. guard_n + grav_n). t_k: t_us of sample k.
constexpr void start(Stroke& s, const Ring& r, const gc_config& cfg, const Scales& sc,
                     const std::array<std::int32_t, 3>& bias_q8, std::uint32_t t_k) {
    s = Stroke{};
    s.bias_q8 = bias_q8;
    s.gravity_ug = gravity_ug(r, cfg, sc.sa);
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
        integrate(s, at(r, age), cfg, sc, t);
    }
}

} // namespace gcore
