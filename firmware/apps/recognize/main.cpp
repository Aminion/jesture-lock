// recognize firmware: IMU -> core/ on the MCU -> recognition events over UART.
#include "gesture/board/clock.hpp"
#include "gesture/board/gpio.hpp"
#include "gesture/board/i2c.hpp"
#include "gesture/board/lsm6ds3.hpp"
#include "gesture/board/uart.hpp"
#include "gesture/core.h"
#include "gesture/proto/cobs.hpp"
#include "gesture/proto/crc16.hpp"

int main() {
    // TODO: clocks, pins, UART, I2C, detect IMU, configure, init core state.
    for (;;) {
        // TODO: read sample, feed core, send events.
    }
}
