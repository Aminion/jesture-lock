#pragma once

// LSM6DS3-family driver: WHO_AM_I detection (accept every known variant), config, sample read.

#include "gesture/board/i2c.hpp"
#include "gesture/proto/le.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

namespace board {

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

// 7-bit I2C addresses, chosen by the SA0 pin (the GY module pulls it high).
inline constexpr std::uint8_t lsm6ds3_addr_sa0_high = 0x6B;
inline constexpr std::uint8_t lsm6ds3_addr_sa0_low = 0x6A;

// Read WHO_AM_I at the SA0-high address, then at SA0-low.
// If neither answers, returns the most telling bus error (anything over a plain NACK).
[[nodiscard]] std::expected<Lsm6ds3Probe, I2cError> lsm6ds3_probe();

// nullopt for any WHO_AM_I value outside the known variants.
[[nodiscard]] constexpr std::optional<Lsm6ds3Chip> lsm6ds3_chip(std::uint8_t who_am_i) {
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
using Lsm6ds3Axes = std::array<std::int16_t, 3>; // x, y, z

struct Lsm6ds3Sample {
    Lsm6ds3Axes gyro;
    Lsm6ds3Axes accel;
};

// OUTX_L_G (22h) .. OUTZ_H_XL (2Dh): gyro then accel, each axis little endian (BLE = 0).
// Same layout on all three known chips.
inline constexpr std::size_t lsm6ds3_sample_bytes =
    2 * std::tuple_size_v<Lsm6ds3Axes> * sizeof(std::int16_t);

[[nodiscard]] constexpr Lsm6ds3Sample
lsm6ds3_decode(std::span<const std::uint8_t, lsm6ds3_sample_bytes> raw) {
    const auto axis = [raw](std::size_t i) {
        return proto::load_le<std::int16_t>(
            raw.subspan(i * sizeof(std::int16_t)).first<sizeof(std::int16_t)>());
    };
    return {.gyro = {axis(0), axis(1), axis(2)}, .accel = {axis(3), axis(4), axis(5)}};
}

// What lsm6ds3_init() configures; Hello reports the two registers (docs/protocol.md).
inline constexpr std::uint32_t lsm6ds3_odr_hz = 104;
inline constexpr std::uint8_t lsm6ds3_ctrl1_xl = 0x40; // ODR_XL 104 Hz, FS_XL +-2 g
inline constexpr std::uint8_t lsm6ds3_ctrl2_g = 0x40;  // ODR_G 104 Hz, FS_G +-250 dps

// Both sensors at 104 Hz, +-2 g, +-250 dps, block data update on, gyro data ready on INT1.
// INT1 is level, not a pulse (AN4650 3.3): it goes high with a new sample and low when the
// sample is read; an unread sample keeps it high and later samples give no new edge.
// Registers outside INT1_CTRL and CTRL1_XL..CTRL3_C keep their values across an MCU reset
// (only a power cycle clears them).
[[nodiscard]] std::expected<void, I2cError> lsm6ds3_init(std::uint8_t addr);

// Latest sample if both sensors have new data since the last read (STATUS_REG), else nullopt.
[[nodiscard]] std::expected<std::optional<Lsm6ds3Sample>, I2cError> lsm6ds3_read(std::uint8_t addr);

} // namespace board
