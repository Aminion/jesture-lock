#include "hal/clock.hpp"

#include "stm32g0xx.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_tim.h"

namespace {

constexpr std::uint32_t sysclk_hz = 16'000'000;
constexpr std::uint32_t systick_hz = 1000;
static_assert(sysclk_hz % systick_hz == 0, "SysTick period is not a whole number of cycles");
static_assert((sysclk_hz / systick_hz) - 1 <= SysTick_LOAD_RELOAD_Msk, "SysTick LOAD is 24 bits");

// TIM2 is the only 32-bit timer on G031 (DS12992). APB prescaler 1, so TIMPCLK = PCLK
// (RM0444 RCC, timer clock); CK_CNT = TIMPCLK / (PSC + 1) (RM0444 TIMx_PSC).
constexpr std::uint32_t timpclk_hz = sysclk_hz;
constexpr std::uint32_t micros_hz = 1'000'000;
static_assert(timpclk_hz % micros_hz == 0 && (timpclk_hz / micros_hz) - 1 <= 0xFFFF,
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
    LL_TIM_SetAutoReload(TIM2, 0xFFFF'FFFF);
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

} // namespace hal
