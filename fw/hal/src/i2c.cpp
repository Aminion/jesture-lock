#include "hal/i2c.hpp"

#include <initializer_list>

#include "hal/clock.hpp"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_i2c.h"

namespace {

using hal::I2cError;
using Result = std::expected<void, I2cError>;

I2C_TypeDef* const bus = I2C1;

// PA9/PA10 are their own pins on LQFP32 (DS12992). Keep SYSCFG_CFGR1.PA11_RMP/PA12_RMP
// at reset (0): setting them makes the PA11/PA12 pins act as PA9/PA10 and forces the real
// PA9/PA10 to analog (RM0444 SYSCFG_CFGR1), which would cut the IMU off.
GPIO_TypeDef* const port = GPIOA;
constexpr std::uint32_t scl_pin = LL_GPIO_PIN_9;
constexpr std::uint32_t sda_pin = LL_GPIO_PIN_10;

// RM0444 Rev 6 Table 173 (f_I2CCLK = 16 MHz), standard mode 100 kHz:
// PRESC=3, SCLDEL=4, SDADEL=2, SCLH=0x0F, SCLL=0x13.
constexpr std::uint32_t timing_100khz = 0x3042'0F13;

// Per flag wait; one byte takes ~0.1 ms at 100 kHz.
constexpr std::uint32_t timeout_ms = 5;

constexpr std::size_t max_nbytes = 255;

// ~8 us at 16 MHz: at least half an SCL period at 100 kHz.
void half_bit_delay() {
    for (int i = 0; i < 32; ++i) {
        __NOP();
    }
}

// A reset in the middle of a read can leave the slave holding SDA low, waiting for
// clocks. Clock SCL until it lets go (at most 9 pulses), then send a STOP.
void bus_clear() {
    for (const auto pin : {scl_pin, sda_pin}) {
        LL_GPIO_SetOutputPin(port, pin);
        LL_GPIO_SetPinOutputType(port, pin, LL_GPIO_OUTPUT_OPENDRAIN);
        LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_NO);
        LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_OUTPUT);
    }
    half_bit_delay();
    for (int i = 0; i < 9 && LL_GPIO_IsInputPinSet(port, sda_pin) == 0U; ++i) {
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
    const std::uint32_t start = hal::millis();
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
        if (hal::millis() - start > timeout_ms) {
            return std::unexpected(I2cError::timeout);
        }
    }
}

// After a NACK the peripheral sends STOP by itself; let it finish, then do the
// software reset (RM0444 I2C software reset): PE low for >= 3 APB cycles, ensured
// by reading it back. It clears every ISR state flag.
void recover() {
    const std::uint32_t start = hal::millis();
    while (LL_I2C_IsActiveFlag_STOP(bus) == 0U && hal::millis() - start <= timeout_ms) {
    }
    LL_I2C_Disable(bus);
    while (LL_I2C_IsEnabled(bus) != 0U) {
    }
    LL_I2C_Enable(bus);
}

// Only called by the public wrappers, which take the 7-bit address and shift it.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Result write_regs(std::uint32_t sadd, std::uint8_t reg, std::span<const std::uint8_t> data) {
    LL_I2C_HandleTransfer(bus, sadd, LL_I2C_ADDRSLAVE_7BIT, 1 + data.size(),
                          LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
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

namespace hal {

void i2c_init() {
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    // Kernel clock: RCC_CCIPR.I2C1SEL reset value = PCLK.
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    bus_clear();
    for (const auto pin : {scl_pin, sda_pin}) {
        LL_GPIO_SetAFPin_8_15(port, pin, LL_GPIO_AF_6);
        LL_GPIO_SetPinOutputType(port, pin, LL_GPIO_OUTPUT_OPENDRAIN);
        LL_GPIO_SetPinSpeed(port, pin, LL_GPIO_SPEED_FREQ_LOW);
        LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_NO);
        LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ALTERNATE);
    }

    // TIMINGR and the filters are writable only with PE = 0.
    LL_I2C_Disable(bus);
    LL_I2C_SetTiming(bus, timing_100khz);
    LL_I2C_EnableAnalogFilter(bus);
    LL_I2C_Enable(bus);
}

std::expected<void, I2cError> i2c_write_regs(std::uint8_t addr, std::uint8_t reg,
                                             std::span<const std::uint8_t> data) {
    if (data.size() + 1 > max_nbytes) {
        return std::unexpected(I2cError::too_long);
    }
    auto r = write_regs(std::uint32_t{addr} << 1, reg, data);
    if (!r) {
        recover();
    }
    return r;
}

std::expected<void, I2cError> i2c_read_regs(std::uint8_t addr, std::uint8_t reg,
                                            std::span<std::uint8_t> out) {
    if (out.size() > max_nbytes) {
        return std::unexpected(I2cError::too_long);
    }
    if (out.empty()) {
        return {};
    }
    auto r = read_regs(std::uint32_t{addr} << 1, reg, out);
    if (!r) {
        recover();
    }
    return r;
}

} // namespace hal
