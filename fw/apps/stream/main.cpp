// fw_stream: IMU -> UART -> PC. Raw samples with MCU timestamps, framed per docs/protocol.md.
#include "hal/clock.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/lsm6ds3.hpp"
#include "hal/uart.hpp"
#include "proto/cobs.hpp"
#include "proto/crc16.hpp"

int main() {
    hal::clock_init();
    hal::led_init();
    // TODO: pins, UART, I2C, detect IMU, configure.
    for (;;) {
        // TODO: read sample, timestamp, frame, send. Blink is a board bring-up placeholder.
        hal::led_toggle();
        hal::delay_ms(500);
    }
}
