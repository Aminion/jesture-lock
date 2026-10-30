#include "gesture/board/gpio.hpp"

#include "pins.hpp"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"

namespace {

using board::pins::led;
using board::pins::led_port;

} // namespace

namespace board {

void led_init() {
    LL_IOP_GRP1_EnableClock(pins::led_port_clock);
    LL_GPIO_ResetOutputPin(led_port, led);
    LL_GPIO_InitTypeDef cfg{.Pin = led,
                            .Mode = LL_GPIO_MODE_OUTPUT,
                            .Speed = LL_GPIO_SPEED_FREQ_LOW,
                            .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                            .Pull = LL_GPIO_PULL_NO,
                            .Alternate = 0};
    LL_GPIO_Init(led_port, &cfg);
}

void led_set(bool on) {
    if (on) {
        LL_GPIO_SetOutputPin(led_port, led);
    } else {
        LL_GPIO_ResetOutputPin(led_port, led);
    }
}

void led_toggle() {
    LL_GPIO_TogglePin(led_port, led);
}

} // namespace board
