#include "hal/imu_int.hpp"

#include <atomic>

#include "hal/clock.hpp"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_gpio.h"

namespace {

GPIO_TypeDef* const port = GPIOA;
constexpr std::uint32_t pin = LL_GPIO_PIN_15;
constexpr std::uint32_t line = LL_EXTI_LINE_15;

// Written only by the interrupt: time first, then count publishes it.
volatile std::uint32_t g_count = 0;
volatile std::uint32_t g_time_us = 0;

} // namespace

extern "C" void EXTI4_15_IRQHandler() {
    if (LL_EXTI_IsActiveRisingFlag_0_31(line) != 0U) {
        LL_EXTI_ClearRisingFlag_0_31(line);
        g_time_us = hal::micros();
        std::atomic_signal_fence(std::memory_order_release);
        g_count = g_count + 1;
    }
}

namespace hal {

void imu_int_init() {
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    // The IMU drives INT1 push-pull; the pull-down only keeps the line low while it does not.
    LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_DOWN);
    LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_INPUT);

    // EXTI_EXTICR4.EXTI15 = 0x00: port A (RM0444 EXTI_EXTICR4).
    LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_CONFIG_LINE15);
    LL_EXTI_EnableRisingTrig_0_31(line);
    LL_EXTI_ClearRisingFlag_0_31(line);
    LL_EXTI_EnableIT_0_31(line);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

ImuEdge imu_int_last() {
    // Retry if an edge lands between the two reads.
    for (;;) {
        const std::uint32_t count = g_count;
        std::atomic_signal_fence(std::memory_order_acquire);
        const std::uint32_t time_us = g_time_us;
        std::atomic_signal_fence(std::memory_order_acquire);
        if (g_count == count) {
            return {.count = count, .time_us = time_us};
        }
    }
}

bool imu_int_high() { return LL_GPIO_IsInputPinSet(port, pin) != 0U; }

} // namespace hal
