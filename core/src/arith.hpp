#pragma once

// docs/core.md "Arithmetic": the only rounding and narrowing the algorithm uses.

#include <cstdint>
#include <numeric>

namespace gcore {

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

[[nodiscard]] constexpr std::int16_t sat16(std::int64_t x) {
    return saturating<std::int16_t>(x);
}

[[nodiscard]] constexpr std::uint16_t satu16(std::int64_t x) {
    return saturating<std::uint16_t>(x);
}

} // namespace gcore
