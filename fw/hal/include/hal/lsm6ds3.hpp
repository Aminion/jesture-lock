#pragma once

// LSM6DS3-family driver: WHO_AM_I detection (accept every known variant), config, FIFO/sample read.

#include <cstdint>
#include <expected>
#include <optional>

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

} // namespace hal
