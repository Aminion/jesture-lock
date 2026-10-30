#pragma once

// I2C master on LL, blocking register read/write.
// I2C1: SCL PA9 (D5), SDA PA10 (D4), AF6; 100 kHz from PCLK = 16 MHz.
// Pull-ups are on the IMU module. Needs clock_init() first (timeouts use millis()).

#include <cstdint>
#include <expected>
#include <span>

namespace board {

// From 1, so 0 can stand for "no error" where the code is stored as a number.
enum class I2cError : std::uint8_t {
    nack = 1,  // address or byte not acknowledged: no device, wrong address
    bus_error, // misplaced START/STOP (BERR) or arbitration lost (ARLO)
    timeout,   // flag never came: SCL held low, missing pull-ups
    too_long,  // over the 255-byte NBYTES limit (RELOAD is not supported)
};

void i2c_init();

// Write `data` to consecutive registers starting at `reg` (7-bit `addr`).
std::expected<void, I2cError> i2c_write_regs(std::uint8_t addr, std::uint8_t reg,
                                             std::span<const std::uint8_t> data);

// Read `out.size()` consecutive registers starting at `reg`: write reg, repeated START, read.
std::expected<void, I2cError> i2c_read_regs(std::uint8_t addr, std::uint8_t reg,
                                            std::span<std::uint8_t> out);

// One register. (Writing one is i2c_write_regs(addr, reg, {value}).)
std::expected<std::uint8_t, I2cError> i2c_read_reg(std::uint8_t addr, std::uint8_t reg);

} // namespace board
