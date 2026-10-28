// fw_recognize: IMU -> core/ on the MCU -> recognition events over UART.
#include "gesture/core.h"
#include "hal/clock.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/lsm6ds3.hpp"
#include "hal/uart.hpp"
#include "proto/cobs.hpp"
#include "proto/crc16.hpp"

int main() {
    // TODO: clocks, pins, UART, I2C, detect IMU, configure, init core state.
    for (;;) {
        // TODO: read sample, feed core, send events.
    }
}
