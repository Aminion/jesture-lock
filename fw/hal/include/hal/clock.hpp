#pragma once

// System clock (RCC), SysTick and the MCU timestamp source.

#include <cstdint>

namespace hal {

// SYSCLK = HSI16 (reset default), SysTick at 1 kHz, TIM2 free-running at 1 MHz.
void clock_init();

// Milliseconds since clock_init(); wraps after ~49 days.
std::uint32_t millis();

// Microseconds since clock_init() (TIM2 counter, 32 bits); wraps after ~71.6 min.
// Safe to call from interrupts. This is the sample timestamp.
std::uint32_t micros();

void delay_ms(std::uint32_t ms);

} // namespace hal
