#include "gesture/board/i2c.hpp"

#include "gesture/board/clock.hpp"
#include "gesture/board/i2c_timing.hpp"
#include "pins.hpp"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_i2c.h"

namespace {

using board::I2cError;
using Result = std::expected<void, I2cError>;

I2C_TypeDef* const bus = I2C1;

GPIO_TypeDef* const port = board::pins::i2c_port;
constexpr std::uint32_t scl_pin = board::pins::i2c_scl;
constexpr std::uint32_t sda_pin = board::pins::i2c_sda;

// Kernel clock: RCC_CCIPR.I2C1SEL reset value = PCLK.
constexpr std::uint32_t i2cclk_hz = board::pclk_hz;

constexpr board::I2cTiming timing = board::i2c_timing_400khz_16mhz;
static_assert(i2cclk_hz == 16'000'000, "timing is from the 16 MHz table");
static_assert(board::i2c_timing_ok(timing, i2cclk_hz, board::i2c_fast_mode));
constexpr std::uint32_t timingr =
    __LL_I2C_CONVERT_TIMINGS(timing.presc, timing.scldel, timing.sdadel, timing.sclh, timing.scll);
static_assert(timingr == 0x1032'0309, "RM0444 Rev 6 Table 173");

// Per flag wait; one byte takes ~25 us at 400 kHz.
constexpr std::uint32_t timeout_ms = 5;

constexpr std::size_t max_nbytes = 255;

// Bus clear clocks at standard-mode speed, slow enough for any slave and any pull-up.
constexpr std::uint32_t bus_clear_hz = 100'000;
constexpr std::uint32_t half_bit_us = board::micros_hz / (2 * bus_clear_hz);
static_assert(half_bit_us * 2 * bus_clear_hz == board::micros_hz);
void half_bit_delay() {
    board::delay_us(half_bit_us);
}

// A slave holding SDA lets go within one byte plus its ACK.
constexpr int max_clear_pulses = 9;

// Open-drain, no internal pull-ups (they are on the IMU module).
LL_GPIO_InitTypeDef pin_config(std::uint32_t mode) {
    return {.Pin = scl_pin | sda_pin,
            .Mode = mode,
            .Speed = LL_GPIO_SPEED_FREQ_LOW,
            .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
            .Pull = LL_GPIO_PULL_NO,
            .Alternate = board::pins::i2c_af};
}

// A reset in the middle of a read can leave the slave holding SDA low, waiting for
// clocks. Clock SCL until it lets go, then send a STOP.
void bus_clear() {
    LL_GPIO_SetOutputPin(port, scl_pin | sda_pin);
    auto cfg = pin_config(LL_GPIO_MODE_OUTPUT);
    LL_GPIO_Init(port, &cfg);
    half_bit_delay();
    for (int i = 0; i < max_clear_pulses && LL_GPIO_IsInputPinSet(port, sda_pin) == 0U; ++i) {
        LL_GPIO_ResetOutputPin(port, scl_pin);
        half_bit_delay();
        LL_GPIO_SetOutputPin(port, scl_pin);
        half_bit_delay();
    }
    // STOP: SDA rises while SCL is high.
    LL_GPIO_ResetOutputPin(port, scl_pin);
    half_bit_delay();
    LL_GPIO_ResetOutputPin(port, sda_pin);
    half_bit_delay();
    LL_GPIO_SetOutputPin(port, scl_pin);
    half_bit_delay();
    LL_GPIO_SetOutputPin(port, sda_pin);
    half_bit_delay();
}

Result wait_for(std::uint32_t isr_flag) {
    const std::uint32_t start = board::millis();
    for (;;) {
        const std::uint32_t isr = bus->ISR;
        if ((isr & I2C_ISR_NACKF) != 0) {
            return std::unexpected(I2cError::nack);
        }
        if ((isr & (I2C_ISR_BERR | I2C_ISR_ARLO)) != 0) {
            return std::unexpected(I2cError::bus_error);
        }
        if ((isr & isr_flag) != 0) {
            return {};
        }
        if (board::millis() - start > timeout_ms) {
            return std::unexpected(I2cError::timeout);
        }
    }
}

// After a NACK the peripheral sends STOP by itself; let it finish, then do the
// software reset (RM0444 I2C software reset): PE low for >= 3 APB cycles, ensured
// by reading it back. It clears every ISR state flag.
void recover() {
    const std::uint32_t start = board::millis();
    while (LL_I2C_IsActiveFlag_STOP(bus) == 0U && board::millis() - start <= timeout_ms) {
    }
    LL_I2C_Disable(bus);
    while (LL_I2C_IsEnabled(bus) != 0U) {
    }
    LL_I2C_Enable(bus);
}

// Passes `r` through; after a failure, resets the peripheral for the next transfer.
Result recover_on_error(Result r) {
    if (!r) {
        recover();
    }
    return r;
}

// Only called by the public wrappers, which take the 7-bit address and shift it.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Result write_regs(std::uint32_t sadd, std::uint8_t reg, std::span<const std::uint8_t> data) {
    LL_I2C_HandleTransfer(bus, sadd, LL_I2C_ADDRSLAVE_7BIT, 1 + data.size(), LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_WRITE);
    if (auto r = wait_for(I2C_ISR_TXIS); !r) {
        return r;
    }
    LL_I2C_TransmitData8(bus, reg);
    for (const std::uint8_t byte : data) {
        if (auto r = wait_for(I2C_ISR_TXIS); !r) {
            return r;
        }
        LL_I2C_TransmitData8(bus, byte);
    }
    if (auto r = wait_for(I2C_ISR_STOPF); !r) {
        return r;
    }
    LL_I2C_ClearFlag_STOP(bus);
    return {};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Result read_regs(std::uint32_t sadd, std::uint8_t reg, std::span<std::uint8_t> out) {
    LL_I2C_HandleTransfer(bus, sadd, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_WRITE);
    if (auto r = wait_for(I2C_ISR_TXIS); !r) {
        return r;
    }
    LL_I2C_TransmitData8(bus, reg);
    if (auto r = wait_for(I2C_ISR_TC); !r) {
        return r;
    }
    // Repeated START into the read phase; AUTOEND sends NACK + STOP after the last byte.
    LL_I2C_HandleTransfer(bus, sadd, LL_I2C_ADDRSLAVE_7BIT, out.size(), LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_READ);
    for (std::uint8_t& byte : out) {
        if (auto r = wait_for(I2C_ISR_RXNE); !r) {
            return r;
        }
        byte = LL_I2C_ReceiveData8(bus);
    }
    if (auto r = wait_for(I2C_ISR_STOPF); !r) {
        return r;
    }
    LL_I2C_ClearFlag_STOP(bus);
    return {};
}

} // namespace

namespace board {

void i2c_init() {
    LL_IOP_GRP1_EnableClock(board::pins::i2c_port_clock);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    bus_clear();
    auto cfg = pin_config(LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_Init(port, &cfg);

    // TIMINGR and the filters are writable only with PE = 0.
    LL_I2C_Disable(bus);
    LL_I2C_SetTiming(bus, timingr);
    LL_I2C_EnableAnalogFilter(bus);
    LL_I2C_Enable(bus);
}

std::expected<void, I2cError> i2c_write_regs(std::uint8_t addr, std::uint8_t reg,
                                             std::span<const std::uint8_t> data) {
    if (data.size() + 1 > max_nbytes) {
        return std::unexpected(I2cError::too_long);
    }
    return recover_on_error(write_regs(std::uint32_t{addr} << 1, reg, data));
}

std::expected<void, I2cError> i2c_read_regs(std::uint8_t addr, std::uint8_t reg,
                                            std::span<std::uint8_t> out) {
    if (out.size() > max_nbytes) {
        return std::unexpected(I2cError::too_long);
    }
    if (out.empty()) {
        return {};
    }
    return recover_on_error(read_regs(std::uint32_t{addr} << 1, reg, out));
}

std::expected<std::uint8_t, I2cError> i2c_read_reg(std::uint8_t addr, std::uint8_t reg) {
    std::uint8_t value = 0;
    return i2c_read_regs(addr, reg, std::span{&value, 1}).transform([&value] { return value; });
}

} // namespace board
