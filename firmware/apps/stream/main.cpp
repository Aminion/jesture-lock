// stream firmware: IMU -> UART -> PC. Raw samples with MCU timestamps, framed per docs/protocol.md.
#include "gesture/board/clock.hpp"
#include "gesture/board/gpio.hpp"
#include "gesture/board/i2c.hpp"
#include "gesture/board/imu_int.hpp"
#include "gesture/board/lsm6ds3.hpp"
#include "gesture/board/uart.hpp"
#include "gesture/proto/frame.hpp"
#include "gesture/proto/message.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace {

static_assert(board::uart_baud_ok(proto::baud));

template <std::size_t N> void send(const std::array<std::uint8_t, N>& payload) {
    static_assert(N <= proto::max_payload);
    std::array<std::uint8_t, proto::max_frame> wire{};
    if (const auto n = proto::frame_encode(payload, wire)) {
        board::uart_write(std::span{wire}.first(*n));
    }
}

// Nominal time between data-ready edges: 9615 us at 104 Hz.
constexpr std::uint32_t sample_period_us = board::micros_hz / board::lsm6ds3_odr_hz;
// INT1 high this long with no edge: a sample was not read in time and the latched
// data-ready blocks all later edges (lsm6ds3.hpp).
constexpr std::uint32_t stuck_us = 3 * sample_period_us;

struct Imu {
    std::uint8_t addr = 0;
    std::uint32_t seen = 0; // board::ImuEdge::count already handled
    std::uint32_t last_us = 0;
    bool have_last = false; // last_us is a real edge, so the next step counts missed samples
    std::uint16_t seq = 0;
    // Status counters, docs/protocol.md.
    std::uint32_t missed = 0; // from t_us steps: samples the IMU made that we never read
    std::uint32_t stuck = 0;
    std::uint32_t i2c_err = 0;
};

std::optional<board::Lsm6ds3Sample> read_sample(Imu& imu) {
    const auto sample = board::lsm6ds3_read(imu.addr);
    if (!sample) {
        ++imu.i2c_err;
        return std::nullopt;
    }
    return *sample;
}

// A Sample frame per data-ready edge, timestamped with the edge.
void poll_imu(Imu& imu) {
    const board::ImuEdge edge = board::imu_int_last();
    if (edge.count != imu.seen) {
        imu.seen = edge.count;
        if (imu.have_last) {
            const std::uint32_t periods =
                (edge.time_us - imu.last_us + (sample_period_us / 2)) / sample_period_us;
            imu.missed += periods > 1 ? periods - 1 : 0;
        }
        imu.last_us = edge.time_us;
        imu.have_last = true;
        if (const auto s = read_sample(imu)) {
            send(proto::encode(proto::Sample{
                .seq = imu.seq++, .t_us = edge.time_us, .gyro = s->gyro, .accel = s->accel}));
        }
    } else if (board::imu_int_high() && board::micros() - imu.last_us > stuck_us) {
        // Releases INT1. The sample's time is unknown, so it is not sent, and the gap
        // length is unknown, so the next edge does not count missed samples.
        static_cast<void>(read_sample(imu));
        imu.last_us = board::micros();
        imu.have_last = false;
        ++imu.stuck;
    }
}

proto::Hello detect_imu() {
    proto::Hello hello{.fw = proto::Firmware::stream,
                       .imu = proto::ImuState::none,
                       .imu_addr = 0,
                       .who_am_i = 0,
                       .ctrl1_xl = 0,
                       .ctrl2_g = 0};
    const auto probe = board::lsm6ds3_probe();
    if (!probe) {
        return hello;
    }
    hello.imu_addr = probe->addr;
    hello.who_am_i = probe->who_am_i;
    if (!board::lsm6ds3_chip(probe->who_am_i)) {
        hello.imu = proto::ImuState::unknown_who_am_i;
    } else if (!board::lsm6ds3_init(probe->addr)) {
        hello.imu = proto::ImuState::init_failed;
    } else {
        hello.imu = proto::ImuState::ok;
        hello.ctrl1_xl = board::lsm6ds3_ctrl1_xl;
        hello.ctrl2_g = board::lsm6ds3_ctrl2_g;
    }
    return hello;
}

} // namespace

int main() {
    board::clock_init();
    board::led_init();
    board::uart_init(proto::baud);
    // Ends a frame cut short by the reset, so the receiver drops only that one.
    board::uart_write(std::array<std::uint8_t, 1>{0});
    board::i2c_init();

    const proto::Hello hello = detect_imu();
    const bool imu_ok = hello.imu == proto::ImuState::ok;
    Imu imu{.addr = hello.imu_addr};
    if (imu_ok) {
        board::imu_int_init();
        // INT1 may still be high from before the MCU reset (the IMU kept running with an
        // unread sample). Read once so the next sample gives an edge.
        static_cast<void>(read_sample(imu));
    }

    // Hello and Status once per second, at most one of them per loop pass: with a Sample
    // and the I2C read that is ~6 ms, under the 9.6 ms sample period.
    constexpr std::uint32_t tick_ms = 125;
    constexpr std::uint32_t ticks_per_second = 8;
    std::uint32_t last_tick = board::millis();
    std::uint32_t ticks = 0;
    bool hello_due = true; // at boot
    bool status_due = false;
    for (;;) {
        if (imu_ok) {
            poll_imu(imu);
        }
        // The PC sends nothing yet: drop input so the ring does not fill.
        while (board::uart_read()) {
        }

        if (board::millis() - last_tick >= tick_ms) {
            last_tick += tick_ms; // no drift from late ticks
            ++ticks;
            // Slow blink (toggle every 0.5 s) = IMU running, fast (every tick) = no IMU,
            // unknown or failed.
            if (!imu_ok || ticks % (ticks_per_second / 2) == 0) {
                board::led_toggle();
            }
            if (ticks % ticks_per_second == 0) {
                hello_due = true;
                status_due = true;
            }
        }

        if (hello_due) {
            hello_due = false;
            send(proto::encode(hello));
        } else if (status_due) {
            status_due = false;
            send(proto::encode(proto::Status{.t_us = board::micros(),
                                             .missed = imu.missed,
                                             .stuck = imu.stuck,
                                             .i2c_err = imu.i2c_err,
                                             .rx_err = board::uart_rx_errors()}));
        }
    }
}
