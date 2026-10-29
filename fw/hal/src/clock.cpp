#include "hal/clock.hpp"

#include <limits>

#include "stm32g0xx.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_tim.h"

namespace {

using hal::micros_hz;
using hal::sysclk_hz;
constexpr std::uint32_t systick_hz = 1000;
static_assert(sysclk_hz % systick_hz == 0, "SysTick period is not a whole number of cycles");
static_assert((sysclk_hz / systick_hz) - 1 <= SysTick_LOAD_RELOAD_Msk, "SysTick LOAD is 24 bits");

// TIM2 is the only 32-bit timer on G031 (DS12992). APB prescaler 1, so TIMPCLK = PCLK
// (RM0444 RCC, timer clock); CK_CNT = TIMPCLK / (PSC + 1) (RM0444 TIMx_PSC).
constexpr std::uint32_t timpclk_hz = hal::pclk_hz;
static_assert(timpclk_hz % micros_hz == 0 &&
                  (timpclk_hz / micros_hz) - 1 <= std::numeric_limits<std::uint16_t>::max(),
              "PSC is 16 bits");

volatile std::uint32_t g_ms = 0;

} // namespace

extern "C" void SysTick_Handler() { g_ms = g_ms + 1; }

namespace hal {

void clock_init() {
    SystemCoreClock = sysclk_hz;
    SysTick_Config(sysclk_hz / systick_hz);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetPrescaler(TIM2, (timpclk_hz / micros_hz) - 1);
    LL_TIM_SetAutoReload(TIM2, std::numeric_limits<std::uint32_t>::max());
    LL_TIM_GenerateEvent_UPDATE(TIM2); // PSC is preloaded: load it now
    LL_TIM_EnableCounter(TIM2);
}

std::uint32_t millis() { return g_ms; }

std::uint32_t micros() { return LL_TIM_GetCounter(TIM2); }

void delay_ms(std::uint32_t ms) {
    const std::uint32_t start = millis();
    while (millis() - start < ms) {
        __WFI();
    }
}

void delay_us(std::uint32_t us) {
    // <=: the first tick can come right after `start` was read.
    const std::uint32_t start = micros();
    while (micros() - start <= us) {
    }
}

} // namespace hal
