#pragma once

// Units of the core's time values, as types: docs/core.md "Per sample" and "Ranges".
// MCU time stamps (gc_sample.t_us) wrap; they share the unsigned 32-bit rep of the spans
// so that a difference of two stamps still wraps. A change of unit rounds with rdiv
// (halves away from zero), never with duration_cast (truncates) or chrono::round (ties to
// even); duration_cast only narrows the rep within one unit.

#include "arith.hpp"

#include <chrono>
#include <cstdint>
#include <ratio>

namespace gcore {

using Micros = std::chrono::duration<std::uint32_t, std::micro>;     // stamps, stroke spans
using MicrosStep = std::chrono::duration<std::uint16_t, std::micro>; // one sample's dt
using PointTicks = std::chrono::duration<std::uint16_t, std::ratio<1, 10000>>; // t_100us

// The count of d in microseconds, widened for the integration math. Exact: every period
// here is a whole number of microseconds.
template <typename Rep, typename Period>
[[nodiscard]] constexpr std::int64_t us(std::chrono::duration<Rep, Period> d) {
    static_assert(std::ratio_divide<Period, std::micro>::den == 1);
    return std::chrono::duration_cast<std::chrono::duration<std::int64_t, std::micro>>(d).count();
}

// A span as a point time: rounded to the tick, saturated.
[[nodiscard]] constexpr PointTicks point_ticks(Micros t) {
    constexpr std::int64_t us_per_tick = us(PointTicks{1});
    return PointTicks{satu16(rdiv(t.count(), us_per_tick))};
}

} // namespace gcore
