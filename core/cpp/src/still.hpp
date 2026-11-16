#pragma once

// docs/core.md "Per sample", steps 3 and 4: the still window and the gyro bias update.

#include "arith.hpp"
#include "config.hpp"
#include "gesture/core.h"
#include "ring.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace gcore {

// Per-axis S = sum x and Q = sum x^2 (raw LSB) over a window. |S| < 2^20, Q < 2^35.
struct Sums {
    std::array<std::int64_t, 3> s;
    std::array<std::int64_t, 3> q;
};

struct WindowSums {
    Sums gyro;
    Sums accel;
};

constexpr void add(Sums& sums, const std::array<std::int16_t, 3>& x) {
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int32_t v = x[i];
        sums.s[i] += v;
        const std::int32_t sq = v * v; // <= 2^30: a 32-bit multiply, cheap on M0+
        sums.q[i] += sq;
    }
}

// Over the newest win_n samples; the ring holds at least win_n.
[[nodiscard]] constexpr WindowSums window_sums(const Ring& r) {
    WindowSums w{};
    for (int age = 0; age < win_n; ++age) {
        add(w.gyro, at(r, age).gyro);
        add(w.accel, at(r, age).accel);
    }
    return w;
}

// N * Q - S^2 < N^2 * th^2 on every axis: the std is below th. N*Q, S^2, N^2*th^2 < 2^43.
[[nodiscard]] constexpr bool spread_ok(const Sums& w, std::uint16_t th) {
    constexpr std::int64_t n = win_n;
    const std::int64_t limit = n * n * th * th;
    for (std::size_t i = 0; i < 3; ++i) {
        if ((n * w.q[i]) - (w.s[i] * w.s[i]) >= limit) {
            return false;
        }
    }
    return true;
}

// |one_q8 * S - N * bias_q8| < one_q8 * N * th on every axis: the mean is within th of the bias.
[[nodiscard]] constexpr bool rate_ok(const Sums& g, const std::array<std::int32_t, 3>& bias_q8,
                                     std::uint16_t th) {
    constexpr std::int64_t n = win_n;
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int64_t d = (one_q8 * g.s[i]) - (n * bias_q8[i]);
        if (iabs(d) >= one_q8 * n * th) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] constexpr bool quiet(const WindowSums& w, const gc_config& cfg,
                                   const std::array<std::int32_t, 3>& bias_q8) {
    return spread_ok(w.gyro, cfg.still_gyro_std_lsb) &&
           spread_ok(w.accel, cfg.still_accel_std_lsb) &&
           rate_ok(w.gyro, bias_q8, cfg.still_gyro_rate_lsb);
}

inline constexpr std::int64_t bias_rate = 64; // docs/core.md step 4: a slow follower

// Step 4, in STILL with the window quiet: if it is also still enough for the bias
// thresholds, bias_q8 moves 1 / bias_rate of the way to the window mean.
constexpr void update_bias(std::array<std::int32_t, 3>& bias_q8, const WindowSums& w,
                           const gc_config& cfg) {
    if (!spread_ok(w.gyro, cfg.bias_gyro_std_lsb) || !spread_ok(w.accel, cfg.bias_accel_std_lsb)) {
        return;
    }
    for (std::size_t i = 0; i < 3; ++i) {
        const std::int64_t mean_q8 = rdiv(one_q8 * w.gyro.s[i], win_n);
        bias_q8[i] += static_cast<std::int32_t>(rdiv(mean_q8 - bias_q8[i], bias_rate));
    }
}

} // namespace gcore
