#include "hal/lsm6ds3.hpp"

#include <array>
#include <span>

namespace {

constexpr std::uint8_t reg_who_am_i = 0x0F;

constexpr std::array<std::uint8_t, 2> addresses{0x6B, 0x6A};

} // namespace

namespace hal {

std::expected<Lsm6ds3Probe, I2cError> lsm6ds3_probe() {
    I2cError error = I2cError::nack;
    for (const std::uint8_t addr : addresses) {
        std::uint8_t who_am_i = 0;
        const auto r = i2c_read_regs(addr, reg_who_am_i, std::span(&who_am_i, 1));
        if (r) {
            return Lsm6ds3Probe{.addr = addr, .who_am_i = who_am_i};
        }
        if (error == I2cError::nack) {
            error = r.error();
        }
    }
    return std::unexpected(error);
}

std::optional<Lsm6ds3Chip> lsm6ds3_chip(std::uint8_t who_am_i) {
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
