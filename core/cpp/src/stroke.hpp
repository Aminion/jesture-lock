#pragma once

// docs/core.md "Stroke start", "Integration" and "Stroke end": one stroke, from the start
// sample s0 to the newest integrated sample, then cut back to the snapshot.

#include "arith.hpp"
#include "config.hpp"
#include "gesture/core.h"
#include "quat.hpp"
#include "ring.hpp"
#include "units.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace gcore {

// The stroke as of the first quiet window after the move (streak 1).
struct Snap {
    Vec3<NanometersPerS> v;
    Vec3<Nanometers> p;
    Micros t; // stamp
    bool clipped;
};

struct Stroke {
    std::array<std::int32_t, 3> bias_q8; // gyro bias, frozen at the start
    Vec3<MicroG32> gravity;              // gravity window mean, calibrated, start frame
    Quat q;                              // start frame <- board frame
    Vec3<NanometersPerS> v;              // start frame
    Vec3<Nanometers> p;                  // start frame
    Micros t_s0;                         // stamp of the start sample
    Micros t;                            // stamp of the newest integrated sample
    std::uint32_t c;                     // samples integrated since s0
    std::uint32_t step;                  // samples per stored point (point i: c = i * step)
    std::uint8_t n;                      // points stored
    bool clipped;                        // a raw reading at clip_lsb or beyond
    bool snapped;                        // snap is valid
    bool done;                           // ended as GC_EV_STROKE: gc_stroke reads it
    Snap snap;
    Micros duration;          // after the end: t_snap - t_s0
    std::uint16_t v_end_mm_s; // after the end
    std::uint16_t extent_mm;  // after the end
    std::array<gc_point, points_n> points;
};

// Gravity per axis: the mean raw accel of ages guard_n + 1 .. guard_n + grav_n (the
// window that ends at s0), calibrated. One rdiv over the sum, so the mean is not rounded
// first. |S - N * off| < 2^22, * gain < 2^53, * sa < 2^62.
[[nodiscard]] constexpr Vec3<MicroG32> gravity_of(const Ring& r, const gc_config& cfg,
                                                  std::int32_t sa) {
    std::array<std::int64_t, 3> s{};
    for (int age = guard_n + 1; age <= guard_n + grav_n; ++age) {
        for (std::size_t i = 0; i < 3; ++i) {
            s[i] += at(r, age).accel[i];
        }
    }
    Vec3<MicroG32> g{};
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int64_t d = s[i] - (std::int64_t{grav_n} * cfg.accel_offset_lsb[i]);
        g[i] = MicroG32{static_cast<std::int32_t>(
            rdiv(d * cfg.accel_gain_q30[i] * sa, std::int64_t{grav_n} * one_q30))};
    }
    return g;
}

// Half the turn of one sample, Q30 rad, per axis: w = 256 * gyro - bias_q8 (LSB x 256),
// ang_udeg = w * sg * dt / (256 * 10^6), h = ang_udeg * pi / 360 * 2^30 with pi = 355/113.
[[nodiscard]] constexpr std::array<std::int64_t, 3>
half_angle(const std::array<std::int16_t, 3>& gyro, const std::array<std::int32_t, 3>& bias_q8,
           std::int32_t sg, MicrosStep dt) {
    std::array<std::int64_t, 3> h{};
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int64_t w = (std::int64_t{256} * gyro[i]) - bias_q8[i];
        const std::int64_t ang_udeg = rdiv(w * sg * us(dt), std::int64_t{256} * 1000000);
        h[i] = rdiv(ang_udeg * 355 * (std::int64_t{1} << 23), std::int64_t{113} * 2812500);
    }
    return h;
}

// Calibrated accel: (raw - off) * gain * sa.
[[nodiscard]] constexpr Vec3<MicroG> calibrated(const std::array<std::int16_t, 3>& accel,
                                                const gc_config& cfg, std::int32_t sa) {
    Vec3<MicroG> a{};
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int64_t d = std::int64_t{accel[i]} - cfg.accel_offset_lsb[i];
        a[i] = MicroG{rdiv(d * cfg.accel_gain_q30[i] * sa, one_q30)};
    }
    return a;
}

// A coordinate of a point: mm, saturated to int16.
[[nodiscard]] constexpr std::int16_t point_mm(Nanometers x) {
    return sat16(round_to<Millimeters>(x).count());
}

// p at t (since s0) as a point; stored points and gc_live share it.
[[nodiscard]] constexpr gc_point point(const Vec3<Nanometers>& p, Micros t) {
    return gc_point{.x_mm = point_mm(p[0]),
                    .y_mm = point_mm(p[1]),
                    .z_mm = point_mm(p[2]),
                    .t_100us = point_ticks(t).count()};
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
                         Micros t) {
    s.q = renormalize(turn(s.q, half_angle(x.gyro, s.bias_q8, sc.sg, x.dt)));
    const Mat3 r = rotation(s.q);
    const Vec3<MicroG> a = calibrated(x.accel, cfg, sc.sa);
    for (std::size_t i = 0; i < 3; ++i) {
        const MicroG f{
            rdiv((r[i][0] * a[0].count()) + (r[i][1] * a[1].count()) + (r[i][2] * a[2].count()),
                 one_q30)};
        // ug * us * 9.80665 m/s^2 = nm/s * 980665 / 10^8; nm/s * us = nm * 10^6.
        s.v[i] +=
            NanometersPerS{rdiv((f.count() - s.gravity[i].count()) * us(x.dt) * 980665, 100000000)};
        s.p[i] += Nanometers{rdiv(s.v[i].count() * us(x.dt), 1000000)};
    }
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int32_t g = x.gyro[i];
        const std::int32_t ac = x.accel[i];
        if (iabs(g) >= cfg.clip_lsb || iabs(ac) >= cfg.clip_lsb) {
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
    const Micros big_t = s.snap.t - s.t_s0;
    NanometersPerS v_max{};
    for (const NanometersPerS v : s.snap.v) {
        v_max = std::max(v_max, NanometersPerS{iabs(v.count())});
    }
    const std::int64_t v_end = round_to<MillimetersPerS>(v_max).count(); // before saturation
    s.duration = big_t;
    s.v_end_mm_s = satu16(v_end);

    if (s.snap.clipped) {
        return GC_REJ_CLIPPED;
    }
    if (big_t < Micros{cfg.min_stroke_us}) {
        return GC_REJ_SHORT;
    }
    if (std::cmp_greater(v_end, cfg.max_v_end_mm_s)) {
        return GC_REJ_DRIFT;
    }

    const std::int64_t t_end = rdiv(us(big_t), us(PointTicks{1})); // in point ticks
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
        const std::int64_t t = us(PointTicks{pt.t_100us});
        std::array<std::int16_t*, 3> xs{&pt.x_mm, &pt.y_mm, &pt.z_mm};
        for (std::size_t i = 0; i < 3; ++i) {
            const Nanometers c{rdiv(rdiv(s.snap.v[i].count() * t, 1000000) * t, 2 * us(big_t))};
            *xs[i] = sat16(*xs[i] - round_to<Millimeters>(c).count());
            const std::int32_t x = *xs[i];
            extent = std::max(extent, static_cast<std::uint16_t>(iabs(x)));
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
// so the ring holds ages 0 .. guard_n + grav_n). t_k: the stamp of sample k.
constexpr void start(Stroke& s, const Ring& r, const gc_config& cfg, const Scales& sc,
                     const std::array<std::int32_t, 3>& bias_q8, Micros t_k) {
    s = Stroke{};
    s.bias_q8 = bias_q8;
    s.gravity = gravity_of(r, cfg, sc.sa);
    s.q = {one_q30, 0, 0, 0};
    s.step = 1;
    s.n = 1; // point 0: (0, 0, 0) at t 0
    // s0 is guard_n + 1 steps back; unsigned, so the clock may wrap in between.
    Micros t = t_k;
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
