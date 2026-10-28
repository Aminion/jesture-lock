// fw_stream: IMU -> UART -> PC. Raw samples with MCU timestamps, framed per docs/protocol.md.
#include <cstdint>

#include "hal/clock.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/lsm6ds3.hpp"
#include "hal/uart.hpp"
#include "proto/cobs.hpp"
#include "proto/crc16.hpp"

namespace {

// Bring-up: IMU probe result, read over SWD until there is UART output.
volatile std::uint8_t g_imu_addr = 0;     // 0: nothing answered
volatile std::uint8_t g_imu_who_am_i = 0;
volatile std::uint8_t g_i2c_error = 0xFF; // hal::I2cError of the failed probe, 0xFF: none

} // namespace

int main() {
    hal::clock_init();
    hal::led_init();
    hal::i2c_init();

    bool imu_ok = false;
    if (const auto probe = hal::lsm6ds3_probe()) {
        g_imu_addr = probe->addr;
        g_imu_who_am_i = probe->who_am_i;
        imu_ok = hal::lsm6ds3_chip(probe->who_am_i).has_value();
    } else {
        g_i2c_error = static_cast<std::uint8_t>(probe.error());
    }

    // TODO: UART, configure IMU.
    for (;;) {
        // TODO: read sample, timestamp, frame, send.
        // Bring-up: slow blink = known IMU found, fast blink = no IMU or unknown WHO_AM_I.
        hal::led_toggle();
        hal::delay_ms(imu_ok ? 500 : 100);
    }
}
