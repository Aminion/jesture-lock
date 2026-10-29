#pragma once

// I2C_TIMINGR fields and a build-time check against the I2C-bus Standard-mode limits.
// Register-free, so it is unit-tested on the host (fw/hal/tests).

#include <cstdint>

namespace hal {

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

// Standard mode (100 kHz) with the analog filter on and no digital filter (DNF = 0).
// Conditions from RM0444 "I2C timings" (SCLDEL, SDADEL), bus limits from RM0444 Table 169
// and the tLOW/tHIGH table, tAF from DS12992. tLOW/tHIGH are checked on tSCLL/tSCLH alone,
// without the SCL sync delays that only lengthen them.
// Times are in ps so a 16 MHz clock period (62.5 ns) stays an integer.
constexpr bool i2c_timing_standard_mode_ok(I2cTiming t, std::uint32_t i2cclk_hz) {
    constexpr auto f4 = i2c_timing_field4_max;
    if (t.presc > f4 || t.scldel > f4 || t.sdadel > f4 || i2cclk_hz == 0) {
        return false;
    }
    constexpr std::int64_t ps_per_s = 1'000'000'000'000;
    constexpr std::int64_t tr_max = 1'000'000;
    constexpr std::int64_t tf_max = 300'000;
    constexpr std::int64_t tsu_dat_min = 250'000;
    constexpr std::int64_t thd_dat_min = 0;
    constexpr std::int64_t tvd_dat_max = 3'450'000;
    constexpr std::int64_t tlow_min = 4'700'000;
    constexpr std::int64_t thigh_min = 4'000'000;
    constexpr std::int64_t taf_min = 50'000;
    constexpr std::int64_t taf_max = 260'000;

    const std::int64_t tclk = ps_per_s / std::int64_t{i2cclk_hz};
    const std::int64_t tpresc = (t.presc + 1) * tclk;
    return (t.scldel + 1) * tpresc >= tr_max + tsu_dat_min &&
           t.sdadel * tpresc >= tf_max + thd_dat_min - taf_min - (3 * tclk) &&
           t.sdadel * tpresc <= tvd_dat_max - tr_max - taf_max - (4 * tclk) &&
           (t.scll + 1) * tpresc >= tlow_min && (t.sclh + 1) * tpresc >= thigh_min;
}

// Standard mode SCL frequency, and RM0444 Rev 6 Table 173 timing for it at f_I2CCLK = 16 MHz.
inline constexpr std::uint32_t i2c_scl_hz = 100'000;
inline constexpr I2cTiming i2c_timing_100khz_16mhz{
    .presc = 3, .scldel = 4, .sdadel = 2, .sclh = 0x0F, .scll = 0x13};

} // namespace hal
