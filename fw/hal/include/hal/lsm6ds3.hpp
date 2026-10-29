#pragma once

// LSM6DS3-family driver: WHO_AM_I detection (accept every known variant), config, sample read.

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

#include "hal/i2c.hpp"

namespace hal {

// GY-LSM6DS3 modules ship with different chips; their register maps differ in details.
enum class Lsm6ds3Chip : std::uint8_t {
    lsm6ds3,    // WHO_AM_I 0x69 (also LSM6DS33, LSM6DS3H)
    lsm6ds3trc, // 0x6A (also LSM6DSL, LSM6DSM)
    lsm6dso,    // 0x6C (also LSM6DSOX)
};

struct Lsm6ds3Probe {
    std::uint8_t addr; // 7-bit I2C address that answered
    std::uint8_t who_am_i;
};

// Read WHO_AM_I at 0x6B (SA0 high), then 0x6A (SA0 low).
// If neither answers, returns the most telling bus error (anything over a plain NACK).
std::expected<Lsm6ds3Probe, I2cError> lsm6ds3_probe();

// nullopt for any WHO_AM_I value outside the known variants.
constexpr std::optional<Lsm6ds3Chip> lsm6ds3_chip(std::uint8_t who_am_i) {
    switch (who_am_i) {
    case 0x69:
        return Lsm6ds3Chip::lsm6ds3;
    case 0x6A:
        return Lsm6ds3Chip::lsm6ds3trc;
    case 0x6C:
        return Lsm6ds3Chip::lsm6dso;
    default:
        return std::nullopt;
    }
}

// One accelerometer + gyroscope reading, raw LSB, axes as printed on the chip.
// 104 Hz, +-2 g (0.061 mg/LSB, 1 g ~ 16393), +-250 dps (8.75 mdps/LSB).
struct Lsm6ds3Sample {
    std::array<std::int16_t, 3> gyro;  // x, y, z
    std::array<std::int16_t, 3> accel; // x, y, z
};

// OUTX_L_G (22h) .. OUTZ_H_XL (2Dh): gyro then accel, each axis little endian (BLE = 0).
// Same layout on all three known chips.
inline constexpr std::size_t lsm6ds3_sample_bytes = 12;

constexpr Lsm6ds3Sample lsm6ds3_decode(std::span<const std::uint8_t, lsm6ds3_sample_bytes> raw) {
    const auto axis = [raw](std::size_t i) {
        return static_cast<std::int16_t>(raw[2 * i] | (raw[(2 * i) + 1] << 8U));
    };
    return {.gyro = {axis(0), axis(1), axis(2)}, .accel = {axis(3), axis(4), axis(5)}};
}

// Both sensors at 104 Hz, +-2 g, +-250 dps, block data update on. Registers outside
// CTRL1_XL..CTRL3_C keep their values across an MCU reset (only a power cycle clears them).
std::expected<void, I2cError> lsm6ds3_init(std::uint8_t addr);

// Latest sample if both sensors have new data since the last read (STATUS_REG), else nullopt.
std::expected<std::optional<Lsm6ds3Sample>, I2cError> lsm6ds3_read(std::uint8_t addr);

} // namespace hal
