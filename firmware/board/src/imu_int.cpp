#include "gesture/board/imu_int.hpp"

#include "gesture/board/clock.hpp"
#include "pins.hpp"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_gpio.h"

#include <atomic>

namespace {

using board::pins::imu_int1;
using board::pins::imu_int1_port;
constexpr std::uint32_t line = board::pins::imu_int1_exti; // served by EXTI4_15_IRQHandler

// Written only by the interrupt: time first, then count publishes it.
volatile std::uint32_t g_count = 0;
volatile std::uint32_t g_time_us = 0;

} // namespace

extern "C" void EXTI4_15_IRQHandler() {
    if (LL_EXTI_IsActiveRisingFlag_0_31(line) != 0U) {
        LL_EXTI_ClearRisingFlag_0_31(line);
        g_time_us = board::micros();
        std::atomic_signal_fence(std::memory_order_release);
        g_count = g_count + 1;
    }
}

namespace board {

void imu_int_init() {
    LL_IOP_GRP1_EnableClock(pins::imu_int1_port_clock);
    // The IMU drives INT1 push-pull; the pull-down only keeps the line low while it does not.
    LL_GPIO_InitTypeDef cfg{.Pin = imu_int1,
                            .Mode = LL_GPIO_MODE_INPUT,
                            .Speed = LL_GPIO_SPEED_FREQ_LOW,
                            .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                            .Pull = LL_GPIO_PULL_DOWN,
                            .Alternate = 0};
    LL_GPIO_Init(imu_int1_port, &cfg);

    LL_EXTI_SetEXTISource(pins::imu_int1_exti_port, pins::imu_int1_exti_source);
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

bool imu_int_high() {
    return LL_GPIO_IsInputPinSet(imu_int1_port, imu_int1) != 0U;
}

} // namespace board
