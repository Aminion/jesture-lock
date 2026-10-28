#pragma once

// System clock (RCC), SysTick and the MCU timestamp source.

#include <cstdint>

namespace hal {

// SYSCLK = HSI16 (reset default), SysTick at 1 kHz.
void clock_init();

// Milliseconds since clock_init(); wraps after ~49 days.
std::uint32_t millis();

void delay_ms(std::uint32_t ms);

} // namespace hal
