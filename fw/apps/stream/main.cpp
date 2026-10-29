// fw_stream: IMU -> UART -> PC. Raw samples with MCU timestamps, framed per docs/protocol.md.
#include <array>
#include <cstdint>
#include <span>
#include <string_view>

#include "hal/clock.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/lsm6ds3.hpp"
#include "hal/uart.hpp"
#include "proto/cobs.hpp"
#include "proto/crc16.hpp"

namespace {

constexpr std::uint32_t baud = 115'200;

// Bring-up: plain-text status line, replaced by the framed Hello once the protocol is in.
// Not part of docs/protocol.md.
struct Line {
    std::array<std::uint8_t, 64> buf{};
    std::size_t len = 0;
};

void put(Line& line, std::string_view s) {
    for (const char c : s) {
        if (line.len < line.buf.size()) {
            line.buf[line.len++] = static_cast<std::uint8_t>(c);
        }
    }
}

void put_hex8(Line& line, std::uint8_t v) {
    constexpr std::string_view digits = "0123456789ABCDEF";
    const std::array<char, 2> s{digits[v >> 4], digits[v & 0x0F]};
    put(line, {s.data(), s.size()});
}

void put_dec(Line& line, std::uint32_t v) {
    std::array<char, 10> s{};
    std::size_t i = s.size();
    do {
        s[--i] = static_cast<char>('0' + v % 10);
        v /= 10;
    } while (v != 0);
    put(line, {s.data() + i, s.size() - i});
}

} // namespace

int main() {
    hal::clock_init();
    hal::led_init();
    hal::uart_init(baud);
    hal::i2c_init();

    std::uint8_t imu_addr = 0;     // 0: nothing answered
    std::uint8_t imu_who_am_i = 0;
    std::uint8_t i2c_err = 0;      // hal::I2cError of the failed probe
    bool imu_ok = false;
    if (const auto probe = hal::lsm6ds3_probe()) {
        imu_addr = probe->addr;
        imu_who_am_i = probe->who_am_i;
        imu_ok = hal::lsm6ds3_chip(probe->who_am_i).has_value();
    } else {
        i2c_err = static_cast<std::uint8_t>(probe.error());
    }

    // TODO: configure IMU.
    for (std::uint32_t n = 0;; ++n) {
        // TODO: read sample, timestamp, frame, send.
        Line line;
        put(line, "fw_stream ");
        put_dec(line, n);
        put(line, " t=");
        put_dec(line, hal::millis());
        if (imu_addr != 0) {
            put(line, " imu=0x");
            put_hex8(line, imu_addr);
            put(line, " who=0x");
            put_hex8(line, imu_who_am_i);
            put(line, imu_ok ? " ok" : " unknown");
        } else {
            put(line, " imu=none i2c_err=");
            put_dec(line, i2c_err);
        }
        put(line, "\r\n");
        hal::uart_write(std::span{line.buf}.first(line.len));

        // Slow blink = known IMU found, fast blink = no IMU or unknown WHO_AM_I.
        hal::led_toggle();
        hal::delay_ms(imu_ok ? 500 : 100);
    }
}
