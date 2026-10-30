#pragma once

// User LED LD3 (green) on the Nucleo-32. Pin assignments: src/pins.hpp, docs/pinout.md.

namespace board {

void led_init();
void led_set(bool on);
void led_toggle();

} // namespace board
