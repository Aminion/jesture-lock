// fw_stream: IMU -> UART -> PC. Raw samples with MCU timestamps, framed per docs/protocol.md.
#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <utility>

#include "hal/clock.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/lsm6ds3.hpp"
#include "hal/uart.hpp"
#include "proto/cobs.hpp"
#include "proto/crc16.hpp"

namespace {

constexpr std::uint32_t baud = 115'200;

// Bring-up: plain-text status and echo lines, replaced by framed messages once the
// protocol is in. Not part of docs/protocol.md.
struct Line {
    std::array<std::uint8_t, 72> buf{};
    std::size_t len = 0;
};

// Appends what fits, drops the rest.
void put(Line& line, std::span<const std::uint8_t> bytes) {
    const auto free = std::span{line.buf}.subspan(line.len);
    const auto n = std::min(bytes.size(), free.size());
    std::ranges::copy(bytes.first(n), free.begin());
    line.len += n;
}

void put(Line& line, std::string_view s) {
    const auto free = std::span{line.buf}.subspan(line.len);
    const auto n = std::min(s.size(), free.size());
    std::ranges::transform(s.substr(0, n), free.begin(),
                           [](char c) { return static_cast<std::uint8_t>(c); });
    line.len += n;
}

// Lowercase, no padding for hex: "0x6b".
void put_uint(Line& line, std::uint32_t v, int base = 10) {
    std::array<char, 32> s{}; // base 2 worst case; to_chars cannot fail
    const auto res = std::to_chars(std::to_address(s.begin()), std::to_address(s.end()), v, base);
    put(line, {s.data(), res.ptr});
}

void send(const Line& line) { hal::uart_write(std::span{line.buf}.first(line.len)); }

// Bring-up echo: every received line comes back as "echo <line>". Status lines can
// fall between echoes but never inside one.
struct Echo {
    Line rx;
    bool truncated = false;
};

void echo_input(Echo& echo) {
    while (const auto byte = hal::uart_read()) {
        if (*byte == '\r') {
            continue;
        }
        if (*byte != '\n') {
            if (echo.rx.len == echo.rx.buf.size()) {
                echo.truncated = true;
            } else {
                put(echo.rx, std::span{&*byte, 1});
            }
            continue;
        }
        // "echo! " marks a line cut short: longer than either buffer.
        constexpr std::size_t overhead = std::string_view{"echo! \r\n"}.size();
        Line out;
        const auto len = std::min(echo.rx.len, out.buf.size() - overhead);
        put(out, echo.truncated || len < echo.rx.len ? "echo! " : "echo ");
        put(out, std::span{echo.rx.buf}.first(len));
        put(out, "\r\n");
        send(out);
        echo = {};
    }
}

struct Status {
    std::uint8_t imu_addr = 0; // 0: nothing answered
    std::uint8_t imu_who_am_i = 0;
    std::uint8_t i2c_err = 0;  // hal::I2cError of the failed probe
    bool imu_ok = false;
    std::uint32_t count = 0;
};

void send_status(Status& status, std::uint32_t now) {
    Line line;
    put(line, "fw_stream ");
    put_uint(line, status.count++);
    put(line, " t=");
    put_uint(line, now);
    if (status.imu_addr != 0) {
        put(line, " imu=0x");
        put_uint(line, status.imu_addr, 16);
        put(line, " who=0x");
        put_uint(line, status.imu_who_am_i, 16);
        put(line, status.imu_ok ? " ok" : " unknown");
    } else {
        put(line, " imu=none i2c_err=");
        put_uint(line, status.i2c_err);
    }
    put(line, " rx_err=");
    put_uint(line, hal::uart_rx_errors());
    put(line, "\r\n");
    send(line);
}

} // namespace

int main() {
    hal::clock_init();
    hal::led_init();
    hal::uart_init(baud);
    hal::i2c_init();

    Status status;
    if (const auto probe = hal::lsm6ds3_probe()) {
        status.imu_addr = probe->addr;
        status.imu_who_am_i = probe->who_am_i;
        status.imu_ok = hal::lsm6ds3_chip(probe->who_am_i).has_value();
    } else {
        status.i2c_err = std::to_underlying(probe.error());
    }

    // TODO: configure IMU.
    Echo echo;
    std::uint32_t last_status = hal::millis();
    for (;;) {
        // TODO: read sample, timestamp, frame, send.
        echo_input(echo);

        // Slow blink = known IMU found, fast blink = no IMU or unknown WHO_AM_I.
        const std::uint32_t now = hal::millis();
        if (now - last_status >= (status.imu_ok ? 500U : 100U)) {
            last_status = now;
            hal::led_toggle();
            send_status(status, now);
        }
    }
}
