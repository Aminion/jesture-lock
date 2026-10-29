// fw_stream: IMU -> UART -> PC. Raw samples with MCU timestamps, framed per docs/protocol.md.
#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <utility>

#include "hal/clock.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/imu_int.hpp"
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

// IMU ODR 104 Hz: nominal time between data-ready edges.
constexpr std::uint32_t sample_period_us = 9'615;
// INT1 high this long with no edge: a sample was not read in time and the latched
// data-ready blocks all later edges (lsm6ds3.hpp).
constexpr std::uint32_t stuck_us = 3 * sample_period_us;

// Bring-up check of the sample timing. n, dt and lat cover one "drdy" line; missed,
// stuck and i2c_err count since reset.
struct Timing {
    std::uint32_t seen = 0; // hal::ImuEdge::count already handled
    std::uint32_t last_us = 0;
    bool have_last = false; // last_us is a real edge, so the next dt is valid
    std::uint32_t n = 0;
    std::uint32_t dt_min = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t dt_max = 0;
    std::uint32_t lat_max = 0; // edge -> sample read
    std::uint32_t missed = 0;  // from dt: samples the IMU made that we never read
    std::uint32_t stuck = 0;
    std::uint32_t i2c_err = 0;
};

struct Imu {
    std::uint8_t addr = 0;
    Timing timing;
    std::optional<hal::Lsm6ds3Sample> latest;
};

void read_sample(Imu& imu) {
    const auto sample = hal::lsm6ds3_read(imu.addr);
    if (!sample) {
        ++imu.timing.i2c_err;
    } else if (*sample) {
        imu.latest = *sample;
    }
}

void poll_imu(Imu& imu) {
    Timing& t = imu.timing;
    const hal::ImuEdge edge = hal::imu_int_last();
    if (edge.count != t.seen) {
        t.seen = edge.count;
        read_sample(imu);
        t.lat_max = std::max(t.lat_max, hal::micros() - edge.time_us);
        if (t.have_last) {
            const std::uint32_t dt = edge.time_us - t.last_us;
            t.dt_min = std::min(t.dt_min, dt);
            t.dt_max = std::max(t.dt_max, dt);
            const std::uint32_t periods = (dt + (sample_period_us / 2)) / sample_period_us;
            t.missed += periods > 1 ? periods - 1 : 0;
        }
        t.last_us = edge.time_us;
        t.have_last = true;
        ++t.n;
    } else if (hal::imu_int_high() && hal::micros() - t.last_us > stuck_us) {
        read_sample(imu); // releases INT1; the gap length is unknown, so no dt
        t.last_us = hal::micros();
        t.have_last = false;
        ++t.stuck;
    }
}

// Bring-up: "imu a=x,y,z g=x,y,z" in raw LSB, the latest sample read.
void send_sample(const Imu& imu) {
    Line line;
    put(line, "imu ");
    if (imu.latest) {
        put(line, "a=");
        put_axes(line, imu.latest->accel);
        put(line, " g=");
        put_axes(line, imu.latest->gyro);
    } else {
        put(line, "none");
    }
    put(line, "\r\n");
    send(line);
}

// Bring-up: "drdy n=52 dt=9600..9630 lat=1900 miss=0 stuck=0 i2c_err=0", times in us.
void send_timing(Timing& t) {
    Line line;
    put(line, "drdy n=");
    put_uint(line, t.n);
    put(line, " dt=");
    put_uint(line, t.n > 1 ? t.dt_min : 0);
    put(line, "..");
    put_uint(line, t.dt_max);
    put(line, " lat=");
    put_uint(line, t.lat_max);
    put(line, " miss=");
    put_uint(line, t.missed);
    put(line, " stuck=");
    put_uint(line, t.stuck);
    put(line, " i2c_err=");
    put_uint(line, t.i2c_err);
    put(line, "\r\n");
    send(line);
    t.n = 0;
    t.dt_min = std::numeric_limits<std::uint32_t>::max();
    t.dt_max = 0;
    t.lat_max = 0;
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

    Imu imu;
    imu.addr = status.imu_addr;
    if (status.imu_ok) {
        hal::imu_int_init();
        // INT1 may still be high from before the MCU reset (the IMU kept running with an
        // unread sample). Read once so the next sample gives an edge.
        read_sample(imu);
    }

    // Bring-up: a sample line every tick (8 Hz); status and timing lines every 4th.
    // At most one line per loop pass: a line takes up to ~8 ms at 115200 baud, two in a
    // row would outlast the 9.6 ms sample period and lose a sample.
    constexpr std::uint32_t tick_ms = 125;
    constexpr std::uint32_t ticks_per_status = 4;
    Echo echo;
    std::uint32_t last_tick = hal::millis();
    std::uint32_t ticks = 0;
    bool sample_due = false;
    bool status_due = false;
    bool timing_due = false;
    for (;;) {
        // TODO: send every sample, framed.
        if (status.imu_ok) {
            poll_imu(imu);
        }

        const std::uint32_t now = hal::millis();
        if (now - last_tick >= tick_ms) {
            last_tick += tick_ms; // no drift from late ticks
            ++ticks;
            sample_due = status.imu_ok;
            // Slow blink (toggle at 2 Hz) = IMU running, fast (8 Hz) = no IMU, unknown or
            // failed.
            if (!status.imu_ok || ticks % ticks_per_status == 0) {
                hal::led_toggle();
            }
            if (ticks % ticks_per_status == 0) {
                status_due = true;
                timing_due = status.imu_ok;
            }
        }

        if (sample_due) {
            sample_due = false;
            send_sample(imu);
        } else if (status_due) {
            status_due = false;
            send_status(status, now);
        } else if (timing_due) {
            timing_due = false;
            send_timing(imu.timing);
        } else {
            echo_input(echo);
        }
    }
}
