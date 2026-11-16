#pragma once

// docs/core.md "Arithmetic": the only rounding and narrowing the algorithm uses.

#include <cstdint>
#include <numeric>

namespace gcore {

// 1.0 in Q30: x stands for x / 2^30 (quaternions, rotation matrices, accel gains).
inline constexpr std::int32_t one_q30 = 1 << 30;
// 1.0 in Q8: the gyro bias and rates in LSB x 256.
inline constexpr std::int32_t one_q8 = 1 << 8;

// C++26 renamed saturate_cast to saturating_cast (__cpp_lib_saturation_arithmetic 202603):
// GCC 15 (firmware) has the old name, GCC 16 (host) the new one.
template <typename T> [[nodiscard]] constexpr T saturating(std::int64_t x) {
#if __cpp_lib_saturation_arithmetic >= 202603L
    return std::saturating_cast<T>(x);
#else
    return std::saturate_cast<T>(x);
#endif
}

// a / b rounded to nearest, halves away from zero: sign(a) * floor((|a| + floor(b/2)) / b).
// b > 0; |a| + b/2 fits in int64 by "Ranges" in docs/core.md.
[[nodiscard]] constexpr std::int64_t rdiv(std::int64_t a, std::int64_t b) {
    const std::int64_t half = b / 2;
    return a >= 0 ? (a + half) / b : -((-a + half) / b);
}

// |x|. Not std::abs: libstdc++ does not declare its integer overloads constexpr (GCC
// folds them as builtins, clang rejects them in constant expressions).
template <typename T> [[nodiscard]] constexpr T iabs(T x) {
    return x < 0 ? -x : x;
}

[[nodiscard]] constexpr std::int16_t sat16(std::int64_t x) {
    return saturating<std::int16_t>(x);
}

[[nodiscard]] constexpr std::uint16_t satu16(std::int64_t x) {
    return saturating<std::uint16_t>(x);
}

} // namespace gcore
