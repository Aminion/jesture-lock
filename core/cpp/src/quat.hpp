#pragma once

// docs/core.md "Integration": the orientation quaternion, Q30, and its rotation matrix.

#include "arith.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace gcore {

using Quat = std::array<std::int32_t, 4>; // w, x, y, z, Q30
using Mat3 = std::array<std::array<std::int64_t, 3>, 3>;

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

} // namespace gcore
