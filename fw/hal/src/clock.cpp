#include "hal/clock.hpp"

#include "stm32g0xx.h"

namespace {

constexpr std::uint32_t sysclk_hz = 16'000'000;
constexpr std::uint32_t systick_hz = 1000;
static_assert(sysclk_hz % systick_hz == 0, "SysTick period is not a whole number of cycles");
static_assert((sysclk_hz / systick_hz) - 1 <= SysTick_LOAD_RELOAD_Msk, "SysTick LOAD is 24 bits");

volatile std::uint32_t g_ms = 0;

} // namespace

extern "C" void SysTick_Handler() { g_ms = g_ms + 1; }

namespace hal {

void clock_init() {
    SystemCoreClock = sysclk_hz;
    SysTick_Config(sysclk_hz / systick_hz);
}

std::uint32_t millis() { return g_ms; }

void delay_ms(std::uint32_t ms) {
    const std::uint32_t start = millis();
    while (millis() - start < ms) {
        __WFI();
    }
}

} // namespace hal
