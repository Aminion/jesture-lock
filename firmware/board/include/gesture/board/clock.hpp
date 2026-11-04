#pragma once

// System clock (RCC), SysTick and the MCU timestamp source.

#include <cstdint>

namespace board {

// SYSCLK = HCLK = PLLRCLK from HSI16: 16 MHz / M 1 * N 8 / R 2 = 64 MHz (VCO 128 MHz),
// the core's speed (gc_feed). APB prescaler 4 keeps PCLK at 16 MHz, so the I2C and UART
// kernel clocks stay where their timing was verified. TIMPCLK = 2 PCLK (RM0444 RCC, timer
// clock: APB prescaler not 1). Derive clock-dependent constants from these, not from a
// literal.
inline constexpr std::uint32_t hsi16_hz = 16'000'000;
inline constexpr std::uint32_t pll_n = 8;
inline constexpr std::uint32_t pll_r = 2;
inline constexpr std::uint32_t sysclk_hz = hsi16_hz * pll_n / pll_r;
inline constexpr std::uint32_t apb_div = 4;
inline constexpr std::uint32_t pclk_hz = sysclk_hz / apb_div;
inline constexpr std::uint32_t timpclk_hz = apb_div == 1 ? pclk_hz : 2 * pclk_hz;
static_assert(sysclk_hz == 64'000'000 && pclk_hz == 16'000'000);

// Tick rate of micros(): one tick per microsecond.
inline constexpr std::uint32_t micros_hz = 1'000'000;

// SYSCLK = PLL at 64 MHz (above), SysTick at 1 kHz, TIM2 free-running at 1 MHz.
void clock_init();

// Milliseconds since clock_init(); wraps after ~49 days.
[[nodiscard]] std::uint32_t millis();

// Microseconds since clock_init() (TIM2 counter, 32 bits); wraps after ~71.6 min.
// Safe to call from interrupts. This is the sample timestamp.
[[nodiscard]] std::uint32_t micros();

void delay_ms(std::uint32_t ms);

// Busy-waits at least `us` microseconds (TIM2). For short waits, e.g. bit-banged bus timing.
void delay_us(std::uint32_t us);

} // namespace board
