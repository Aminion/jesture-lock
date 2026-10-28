#include "hal/gpio.hpp"

#include <cstdint>

#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"

namespace {

GPIO_TypeDef* const led_port = GPIOC;
constexpr std::uint32_t led_pin = LL_GPIO_PIN_6;

} // namespace

namespace hal {

void led_init() {
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
    LL_GPIO_ResetOutputPin(led_port, led_pin);
    LL_GPIO_SetPinOutputType(led_port, led_pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(led_port, led_pin, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(led_port, led_pin, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(led_port, led_pin, LL_GPIO_MODE_OUTPUT);
}

void led_set(bool on) {
    if (on) {
        LL_GPIO_SetOutputPin(led_port, led_pin);
    } else {
        LL_GPIO_ResetOutputPin(led_port, led_pin);
    }
}

void led_toggle() { LL_GPIO_TogglePin(led_port, led_pin); }

} // namespace hal
