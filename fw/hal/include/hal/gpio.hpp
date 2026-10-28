#pragma once

// Pin mapping for Nucleo-32 G031K8: USART2 (ST-LINK VCP), I2C, LSM6DS3 INT1/INT2.
// User LED LD3 (green): PC6 (UM2591).

namespace hal {

void led_init();
void led_set(bool on);
void led_toggle();

} // namespace hal
