#pragma once

// System clock (RCC), SysTick and the MCU timestamp source.

#include <cstdint>

namespace hal {

// HSI16 (reset default), AHB and APB prescalers 1: every peripheral clock below is 16 MHz.
// Derive clock-dependent constants from these, not from a literal.
inline constexpr std::uint32_t sysclk_hz = 16'000'000;
inline constexpr std::uint32_t pclk_hz = sysclk_hz;

// SYSCLK = HSI16 (reset default), SysTick at 1 kHz, TIM2 free-running at 1 MHz.
void clock_init();

// Milliseconds since clock_init(); wraps after ~49 days.
std::uint32_t millis();

// Microseconds since clock_init() (TIM2 counter, 32 bits); wraps after ~71.6 min.
// Safe to call from interrupts. This is the sample timestamp.
std::uint32_t micros();

void delay_ms(std::uint32_t ms);

// Busy-waits at least `us` microseconds (TIM2). For short waits, e.g. bit-banged bus timing.
void delay_us(std::uint32_t us);

} // namespace hal
