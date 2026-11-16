#pragma once

// Units of the core's values, as types: docs/core.md "Per sample" and "Ranges".
// MCU time stamps (gc_sample.t_us) wrap; they share the unsigned 32-bit rep of the spans
// so that a difference of two stamps still wraps. A change of unit rounds with rdiv
// (halves away from zero), never with duration_cast (truncates) or chrono::round (ties to
// even); duration_cast only narrows the rep within one unit.

#include "arith.hpp"

#include <array>
#include <chrono>
#include <compare>
#include <cstdint>
#include <limits>
#include <ratio>
#include <type_traits>
#include <utility>

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

// Other quantities, in the manner of chrono: a number in one unit. Only the same unit
// adds, subtracts and compares; formulas across dimensions read the number with
// in<Unit>(), which names the unit they expect, and stay the integer expressions of
// docs/core.md.
template <typename Dim, typename Ratio> struct Unit {
    using dim = Dim;
    using ratio = Ratio; // the unit's size in the dimension's base unit
};

// To has unit U and a rep that holds every Rep: reading a Quantity<Rep, U> as To is exact.
template <typename To, typename Rep, typename U>
concept exactly_as = std::is_same_v<typename To::unit, U> &&
                     std::in_range<typename To::rep>(std::numeric_limits<Rep>::min()) &&
                     std::in_range<typename To::rep>(std::numeric_limits<Rep>::max());

template <typename Rep, typename U> class Quantity {
  public:
    using rep = Rep;
    using unit = U;

    constexpr Quantity() = default;
    constexpr explicit Quantity(Rep n) : n_{n} {}

    // The number in To's unit: only this quantity's own unit, never narrowed. A change of
    // unit goes through round_to.
    template <typename To>
        requires exactly_as<To, Rep, U>
    [[nodiscard]] constexpr typename To::rep in() const {
        return n_;
    }

    constexpr Quantity& operator+=(Quantity q) {
        n_ = static_cast<Rep>(n_ + q.n_);
        return *this;
    }
    constexpr Quantity& operator-=(Quantity q) {
        n_ = static_cast<Rep>(n_ - q.n_);
        return *this;
    }
    [[nodiscard]] friend constexpr Quantity operator+(Quantity a, Quantity b) {
        return a += b;
    }
    [[nodiscard]] friend constexpr Quantity operator-(Quantity a, Quantity b) {
        return a -= b;
    }
    friend constexpr auto operator<=>(const Quantity&, const Quantity&) = default;

  private:
    Rep n_; // no initializer: Quantity stays trivial, value-initialized to 0 like an int
};

// Dimensions.
struct Length {};
struct Speed {};
struct Accel {}; // in standard gravity, g

using Nanometers = Quantity<std::int64_t, Unit<Length, std::nano>>;
using Millimeters = Quantity<std::int64_t, Unit<Length, std::milli>>;
using NanometersPerS = Quantity<std::int64_t, Unit<Speed, std::nano>>;
using MillimetersPerS = Quantity<std::int64_t, Unit<Speed, std::milli>>;
using MicroG = Quantity<std::int64_t, Unit<Accel, std::micro>>;
using MicroG32 = Quantity<std::int32_t, Unit<Accel, std::micro>>; // stored

template <typename Q> using Vec3 = std::array<Q, 3>;

// A factor num / den for the formulas that cross units or dimensions (docs/core.md
// "Integration"): scaled(x, s) is x * num / den with one rdiv. Composing multiplies num and
// den without reducing, so the integers stay those of the spec.
struct Scale {
    std::int64_t num;
    std::int64_t den;
};

[[nodiscard]] constexpr Scale operator*(Scale a, Scale b) {
    return {.num = a.num * b.num, .den = a.den * b.den};
}

[[nodiscard]] constexpr std::int64_t scaled(std::int64_t x, Scale s) {
    return rdiv(x * s.num, s.den);
}

template <typename R> [[nodiscard]] constexpr bool same_value(Scale s) {
    return s.num * R::den == R::num * s.den; // the same fraction, reduced or not
}

using StandardGravity = std::ratio<980665, 100000>; // 1 g in m/s^2 (CGPM 1901)
using Pi = std::ratio<355, 113>;                    // relative error 8.5e-8

// Q8 to its integer: LSB x 256 -> LSB.
inline constexpr Scale from_q8{.num = 1, .den = one_q8};

// ug * us -> nm/s: 10^-6 g * 10^-6 s * 9.80665 m/s^2 per g.
inline constexpr Scale ug_us_to_nm_s{.num = 980665, .den = 100000000};
static_assert(
    same_value<std::ratio_divide<
        std::ratio_multiply<std::ratio_multiply<MicroG::unit::ratio, std::micro>, StandardGravity>,
        NanometersPerS::unit::ratio>>(ug_us_to_nm_s));

// nm/s * us -> nm.
inline constexpr Scale nm_s_us_to_nm{.num = 1, .den = 1000000};
static_assert(
    same_value<std::ratio_divide<std::ratio_multiply<NanometersPerS::unit::ratio, std::micro>,
                                 Nanometers::unit::ratio>>(nm_s_us_to_nm));

// us -> s, as a factor inside a longer product (the tilt term of the correction).
inline constexpr Scale us_to_s{.num = 1, .den = 1000000};
static_assert(same_value<std::micro>(us_to_s));

// Halves and sixths: the t^2 / 2 and t^3 / 6 of a constant and a linearly growing error.
inline constexpr Scale half{.num = 1, .den = 2};
inline constexpr Scale sixth{.num = 1, .den = 6};

// udps * us -> udeg.
inline constexpr Scale udps_us_to_udeg{.num = 1, .den = 1000000};
static_assert(
    same_value<std::ratio_divide<std::ratio_multiply<std::micro, std::micro>, std::micro>>(
        udps_us_to_udeg));

// udeg -> half the angle in rad, Q30: pi / 180 / 2 * 2^30 / 10^6 = pi * 2^23 / 2812500.
inline constexpr Scale udeg_to_half_rad_q30{.num = Pi::num * (std::int64_t{1} << 23),
                                            .den = Pi::den * 2812500};
static_assert(same_value<std::ratio_multiply<Pi, std::ratio<std::int64_t{1} << 30, 360000000>>>(
    udeg_to_half_rad_q30));

// q in To's unit (the same dimension, a whole factor coarser), rounded with rdiv and
// saturated to To's rep.
template <typename To, typename Rep, typename U>
[[nodiscard]] constexpr To round_to(Quantity<Rep, U> q) {
    static_assert(std::is_same_v<typename To::unit::dim, typename U::dim>);
    using factor = std::ratio_divide<typename To::unit::ratio, typename U::ratio>;
    static_assert(factor::den == 1);
    return To{saturating<typename To::rep>(rdiv(q.template in<Quantity<Rep, U>>(), factor::num))};
}

} // namespace gcore
