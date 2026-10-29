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
static_assert(hal::uart_baud_ok(baud));

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

void put_int(Line& line, std::int32_t v) {
    std::array<char, 12> s{}; // "-2147483648"; to_chars cannot fail
    const auto res = std::to_chars(std::to_address(s.begin()), std::to_address(s.end()), v);
    put(line, {s.data(), res.ptr});
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
    std::uint8_t i2c_err = 0;  // hal::I2cError of the failed probe or init
    bool imu_ok = false;       // known chip, configured
    std::uint32_t count = 0;
};

void put_axes(Line& line, const std::array<std::int16_t, 3>& v) {
    put_int(line, v[0]);
    put(line, ",");
    put_int(line, v[1]);
    put(line, ",");
    put_int(line, v[2]);
}

// Bring-up: "imu a=x,y,z g=x,y,z" in raw LSB, "imu stale" if no new sample since the
// last line, "imu i2c_err=N" if the read failed.
void send_sample(std::uint8_t addr) {
    Line line;
    put(line, "imu ");
    const auto sample = hal::lsm6ds3_read(addr);
    if (!sample) {
        put(line, "i2c_err=");
        put_uint(line, std::to_underlying(sample.error()));
    } else if (!sample->has_value()) {
        put(line, "stale");
    } else {
        put(line, "a=");
        put_axes(line, (*sample)->accel);
        put(line, " g=");
        put_axes(line, (*sample)->gyro);
    }
    put(line, "\r\n");
    send(line);
}

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
        if (status.imu_ok) {
            put(line, " ok");
        } else if (status.i2c_err != 0) {
            put(line, " init i2c_err=");
            put_uint(line, status.i2c_err);
        } else {
            put(line, " unknown");
        }
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
        if (hal::lsm6ds3_chip(probe->who_am_i)) {
            const auto init = hal::lsm6ds3_init(probe->addr);
            status.imu_ok = init.has_value();
            if (!init) {
                status.i2c_err = std::to_underlying(init.error());
            }
        }
    } else {
        status.i2c_err = std::to_underlying(probe.error());
    }

    // Bring-up: a sample line every tick (8 Hz), a status line every 4th.
    constexpr std::uint32_t tick_ms = 125;
    constexpr std::uint32_t ticks_per_status = 4;
    Echo echo;
    std::uint32_t last_tick = hal::millis();
    std::uint32_t ticks = 0;
    for (;;) {
        // TODO: read every sample, timestamp, frame, send.
        echo_input(echo);

        const std::uint32_t now = hal::millis();
        if (now - last_tick < tick_ms) {
            continue;
        }
        last_tick += tick_ms; // no drift from late ticks
        ++ticks;
        if (status.imu_ok) {
            send_sample(status.imu_addr);
        }
        // Slow blink (toggle at 2 Hz) = IMU running, fast (8 Hz) = no IMU, unknown or failed.
        if (!status.imu_ok || ticks % ticks_per_status == 0) {
            hal::led_toggle();
        }
        if (ticks % ticks_per_status == 0) {
            send_status(status, now);
        }
    }
}
