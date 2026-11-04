#pragma once

// docs/core.md: "Scales", "Fixed constants" and gc_init's validation of gc_config.

#include "gesture/core.h"

#include <array>
#include <cstdint>

namespace gcore {

// Fixed constants.
inline constexpr int win_n = 32;    // still window
inline constexpr int guard_n = 10;  // gravity window end -> stroke start
inline constexpr int grav_n = 32;   // gravity window
inline constexpr int ring_n = 48;   // raw samples kept
inline constexpr int points_n = 64; // stroke buffer
static_assert(ring_n >= grav_n + guard_n + 1);

// Accel sensitivity, ug/LSB, from FS_XL = CTRL1_XL[3:2]. Every code is in the table.
[[nodiscard]] constexpr std::int32_t accel_ug_per_lsb(std::uint8_t ctrl1_xl) {
    constexpr std::array<std::int32_t, 4> sa{61, 488, 122, 244}; // 2, 16, 4, 8 g
    return sa[(ctrl1_xl >> 2U) & 3U];
}

// Gyro sensitivity, udps/LSB, from FS_G = CTRL2_G[3:2] and FS_125 = CTRL2_G[1].
[[nodiscard]] constexpr std::int32_t gyro_udps_per_lsb(std::uint8_t ctrl2_g) {
    if (((ctrl2_g >> 1U) & 1U) != 0) {
        return 4375; // 125 dps
    }
    constexpr std::array<std::int32_t, 4> sg{8750, 17500, 35000, 70000}; // 250 .. 2000 dps
    return sg[(ctrl2_g >> 2U) & 3U];
}

// Both sensitivities of a config: sa ug/LSB, sg udps/LSB.
struct Scales {
    std::int32_t sa;
    std::int32_t sg;
};

[[nodiscard]] constexpr Scales scales(const gc_config& cfg) {
    return {.sa = accel_ug_per_lsb(cfg.ctrl1_xl), .sg = gyro_udps_per_lsb(cfg.ctrl2_g)};
}

// accel_gain_q30 range: 0.9 * 2^30 rounded up, 1.1 * 2^30 rounded down.
inline constexpr std::int32_t gain_min_q30 = 966367642;
inline constexpr std::int32_t gain_max_q30 = 1181116006;
static_assert(10LL * gain_min_q30 >= 9LL << 30 && 10LL * (gain_min_q30 - 1) < 9LL << 30);
static_assert(10LL * gain_max_q30 <= 11LL << 30 && 10LL * (gain_max_q30 + 1) > 11LL << 30);

inline constexpr std::uint16_t max_dt_limit_us = 20000;       // "Ranges" assume dt <= 20 ms
inline constexpr std::uint32_t max_stroke_limit_us = 6553500; // t_100us is uint16

// gc_status for cfg, checked in the order of the gc_status table (docs/core.md).
[[nodiscard]] constexpr gc_status validate(const gc_config& cfg) {
    if (cfg.abi_version != GC_ABI_VERSION) {
        return GC_ERR_VERSION;
    }
    // Every FS code is in the tables, so only a sensor that is off fails here.
    if (cfg.ctrl1_xl == 0 || cfg.ctrl2_g == 0) {
        return GC_ERR_SCALE;
    }
    if (cfg.reserved0 != 0 || cfg.reserved1 != 0) {
        return GC_ERR_CONFIG;
    }
    for (const std::int32_t gain : cfg.accel_gain_q30) {
        if (gain < gain_min_q30 || gain > gain_max_q30) {
            return GC_ERR_CONFIG;
        }
    }
    const bool ok = cfg.still_gyro_std_lsb >= 1 && cfg.still_accel_std_lsb >= 1 &&
                    cfg.still_gyro_rate_lsb >= 1 && cfg.hold_n >= guard_n + 1 &&
                    cfg.bias_gyro_std_lsb >= 1 && cfg.bias_accel_std_lsb >= 1 &&
                    cfg.clip_lsb >= 1 && cfg.max_dt_us >= 1 && cfg.max_dt_us <= max_dt_limit_us &&
                    cfg.min_stroke_us < cfg.max_stroke_us &&
                    cfg.max_stroke_us <= max_stroke_limit_us;
    return ok ? GC_OK : GC_ERR_CONFIG;
}

} // namespace gcore
