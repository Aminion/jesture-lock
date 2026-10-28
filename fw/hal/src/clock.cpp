#include "hal/clock.hpp"

#include "stm32g0xx.h"

namespace {

constexpr std::uint32_t sysclk_hz = 16'000'000;

volatile std::uint32_t g_ms = 0;

} // namespace

extern "C" void SysTick_Handler() { g_ms = g_ms + 1; }

namespace hal {

void clock_init() {
    SystemCoreClock = sysclk_hz;
    SysTick_Config(sysclk_hz / 1000);
}

std::uint32_t millis() { return g_ms; }

void delay_ms(std::uint32_t ms) {
    const std::uint32_t start = millis();
    while (millis() - start < ms) {
        __WFI();
    }
}

} // namespace hal
