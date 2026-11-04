#include "gesture/board/clock.hpp"

#include "stm32g0xx.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_tim.h"

#include <limits>

namespace {

using board::micros_hz;
using board::sysclk_hz;
constexpr std::uint32_t systick_hz = 1000;
static_assert(sysclk_hz % systick_hz == 0, "SysTick period is not a whole number of cycles");
static_assert((sysclk_hz / systick_hz) - 1 <= SysTick_LOAD_RELOAD_Msk, "SysTick LOAD is 24 bits");

// PLL limits, voltage range 1 (reset default): input after /M 2.66..16 MHz, VCO 96..344
// MHz, PLLRCLK <= 64 MHz (DS12992 PLL characteristics, RM0444 RCC_PLLCFGR).
constexpr std::uint32_t vco_hz = board::hsi16_hz * board::pll_n;
static_assert(vco_hz >= 96'000'000 && vco_hz <= 344'000'000);
static_assert(sysclk_hz <= 64'000'000);
// HCLK <= 64 MHz in range 1 needs 2 flash wait states (RM0444 Table 13).
static_assert(sysclk_hz > 48'000'000);
static_assert(board::apb_div == 4, "LL_RCC_APB1_DIV_4 below");

// TIM2 is the only 32-bit timer on G031 (DS12992). CK_CNT = TIMPCLK / (PSC + 1)
// (RM0444 TIMx_PSC).
using board::timpclk_hz;
static_assert(timpclk_hz % micros_hz == 0 &&
                  (timpclk_hz / micros_hz) - 1 <= std::numeric_limits<std::uint16_t>::max(),
              "PSC is 16 bits");

volatile std::uint32_t g_ms = 0;

// RM0444 "Increasing the CPU frequency": wait states first, then the clock switch.
void pll_init() {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
    }
    LL_FLASH_EnablePrefetch();

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, board::pll_n,
                                LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_Enable();
    LL_RCC_PLL_EnableDomain_SYS();
    while (LL_RCC_PLL_IsReady() == 0) {
    }
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }
}

} // namespace

extern "C" void SysTick_Handler() {
    g_ms = g_ms + 1;
}

namespace board {

void clock_init() {
    pll_init();
    SystemCoreClock = sysclk_hz;
    SysTick_Config(sysclk_hz / systick_hz);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetPrescaler(TIM2, (timpclk_hz / micros_hz) - 1);
    LL_TIM_SetAutoReload(TIM2, std::numeric_limits<std::uint32_t>::max());
    LL_TIM_GenerateEvent_UPDATE(TIM2); // PSC is preloaded: load it now
    LL_TIM_EnableCounter(TIM2);
}

std::uint32_t millis() {
    return g_ms;
}

std::uint32_t micros() {
    return LL_TIM_GetCounter(TIM2);
}

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

} // namespace board
