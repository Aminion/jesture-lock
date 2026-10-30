#pragma once

// I2C_TIMINGR fields and a build-time check against the I2C-bus mode limits.
// Register-free, so it is unit-tested on the host (firmware/board/tests).

#include <cstdint>

namespace board {

// RM0444 I2C_TIMINGR. t_PRESC = (PRESC + 1) x t_I2CCLK.
struct I2cTiming {
    std::uint8_t presc;  // 4 bits, like SCLDEL and SDADEL
    std::uint8_t scldel; // 4 bits, data setup: t_SCLDEL = (SCLDEL + 1) x t_PRESC
    std::uint8_t sdadel; // 4 bits, data hold: t_SDADEL = SDADEL x t_PRESC
    std::uint8_t sclh;   // t_SCLH = (SCLH + 1) x t_PRESC
    std::uint8_t scll;   // t_SCLL = (SCLL + 1) x t_PRESC
};

// Largest value of a 4-bit TIMINGR field.
inline constexpr std::uint8_t i2c_timing_field4_max = 0xF;

// Bus limits of one I2C-bus mode, RM0444 Tables 169 and 171, in ps so a 16 MHz clock
// period (62.5 ns) stays an integer. tHD;DAT(min) is 0 in every mode.
struct I2cBusLimits {
    std::int64_t tr_max;
    std::int64_t tf_max;
    std::int64_t tsu_dat_min;
    std::int64_t thd_dat_max; // RM0444 note under the SDADEL conditions
    std::int64_t tlow_min;
    std::int64_t thigh_min;
};

inline constexpr I2cBusLimits i2c_standard_mode{.tr_max = 1'000'000,
                                                .tf_max = 300'000,
                                                .tsu_dat_min = 250'000,
                                                .thd_dat_max = 3'450'000,
                                                .tlow_min = 4'700'000,
                                                .thigh_min = 4'000'000};
inline constexpr I2cBusLimits i2c_fast_mode{.tr_max = 300'000,
                                            .tf_max = 300'000,
                                            .tsu_dat_min = 100'000,
                                            .thd_dat_max = 900'000,
                                            .tlow_min = 1'300'000,
                                            .thigh_min = 600'000};

// Controller mode with the analog filter on and no digital filter (DNF = 0).
// SCLDEL and SDADEL conditions from RM0444 "I2C timings", tAF from DS12992. The SDADEL
// upper bound is the tHD;DAT(max) form: RM0444 lets the stricter tVD;DAT form be violated
// with NOSTRETCH = 0, which a controller always is.
// tLOW = tSYNC1 + tSCLL and tHIGH = tSYNC2 + tSCLH; each tSYNC is credited with its
// minimum, 2 x t_I2CCLK (RM0444 Tables 172/173 notes: tSYNC1 + tSYNC2 >= 4 x t_I2CCLK).
// ST's own fast-mode rows need that credit.
[[nodiscard]] constexpr bool i2c_timing_ok(I2cTiming t, std::uint32_t i2cclk_hz,
                                           const I2cBusLimits& bus) {
    constexpr auto f4 = i2c_timing_field4_max;
    if (t.presc > f4 || t.scldel > f4 || t.sdadel > f4 || i2cclk_hz == 0) {
        return false;
    }
    constexpr std::int64_t ps_per_s = 1'000'000'000'000;
    constexpr std::int64_t taf_min = 50'000;
    constexpr std::int64_t taf_max = 260'000;

    const std::int64_t tclk = ps_per_s / std::int64_t{i2cclk_hz};
    const std::int64_t tpresc = (t.presc + 1) * tclk;
    const std::int64_t tsync_min = 2 * tclk;
    return (t.scldel + 1) * tpresc >= bus.tr_max + bus.tsu_dat_min &&
           t.sdadel * tpresc >= bus.tf_max - taf_min - (3 * tclk) &&
           t.sdadel * tpresc <= bus.thd_dat_max - taf_max - (4 * tclk) &&
           tsync_min + ((t.scll + 1) * tpresc) >= bus.tlow_min &&
           tsync_min + ((t.sclh + 1) * tpresc) >= bus.thigh_min;
}

// Fast mode (400 kHz), RM0444 Rev 6 Table 173 at f_I2CCLK = 16 MHz.
inline constexpr I2cTiming i2c_timing_400khz_16mhz{
    .presc = 1, .scldel = 3, .sdadel = 2, .sclh = 0x03, .scll = 0x09};

} // namespace board
