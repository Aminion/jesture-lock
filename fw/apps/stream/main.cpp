// fw_stream: IMU -> UART -> PC. Raw samples with MCU timestamps, framed per docs/protocol.md.
#include "hal/clock.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/lsm6ds3.hpp"
#include "hal/uart.hpp"
#include "proto/cobs.hpp"
#include "proto/crc16.hpp"

int main() {
    // TODO: clocks, pins, UART, I2C, detect IMU, configure.
    for (;;) {
        // TODO: read sample, timestamp, frame, send.
    }
}
