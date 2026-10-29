#pragma once

// Every MCU pin fw/hal drives, in one place. Mirrors docs/pinout.md: change both together.
// Private to fw/hal/src; the drivers' public headers describe what they do, not where.

#include <cstdint>

#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_gpio.h"

namespace hal::pins {

// User LED LD3 (green), UM2591.
inline GPIO_TypeDef* const led_port = GPIOC;
inline constexpr std::uint32_t led_port_clock = LL_IOP_GRP1_PERIPH_GPIOC;
inline constexpr std::uint32_t led = LL_GPIO_PIN_6;

// USART2 to the ST-LINK virtual COM port, AF1 (DS12992 Table 13).
inline GPIO_TypeDef* const uart_port = GPIOA;
inline constexpr std::uint32_t uart_port_clock = LL_IOP_GRP1_PERIPH_GPIOA;
inline constexpr std::uint32_t uart_tx = LL_GPIO_PIN_2;
inline constexpr std::uint32_t uart_rx = LL_GPIO_PIN_3;
inline constexpr std::uint32_t uart_af = LL_GPIO_AF_1;

// I2C1 to the IMU, AF6: SCL PA9 (D5), SDA PA10 (D4). These are their own pins on LQFP32
// (DS12992). Keep SYSCFG_CFGR1.PA11_RMP/PA12_RMP at reset (0): setting them makes the
// PA11/PA12 pins act as PA9/PA10 and forces the real PA9/PA10 to analog (RM0444
// SYSCFG_CFGR1), which would cut the IMU off.
inline GPIO_TypeDef* const i2c_port = GPIOA;
inline constexpr std::uint32_t i2c_port_clock = LL_IOP_GRP1_PERIPH_GPIOA;
inline constexpr std::uint32_t i2c_scl = LL_GPIO_PIN_9;
inline constexpr std::uint32_t i2c_sda = LL_GPIO_PIN_10;
inline constexpr std::uint32_t i2c_af = LL_GPIO_AF_6;

// IMU INT1: PA15 (D2), EXTI15. EXTI_EXTICR4.EXTI15 = 0x00 selects port A (RM0444).
inline GPIO_TypeDef* const imu_int1_port = GPIOA;
inline constexpr std::uint32_t imu_int1_port_clock = LL_IOP_GRP1_PERIPH_GPIOA;
inline constexpr std::uint32_t imu_int1 = LL_GPIO_PIN_15;
inline constexpr std::uint32_t imu_int1_exti_port = LL_EXTI_CONFIG_PORTA;

} // namespace hal::pins
