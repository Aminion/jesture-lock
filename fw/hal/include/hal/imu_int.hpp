#pragma once

// LSM6DS3 INT1 -> PA15 (D2), EXTI15 on the rising edge. Each edge is timestamped with
// micros() in the interrupt, so the time is when the sample became ready, not when the
// main loop got to it.

#include <cstdint>

namespace hal {

struct ImuEdge {
    std::uint32_t count;   // rising edges since imu_int_init()
    std::uint32_t time_us; // micros() at the last one
};

// Needs clock_init() first.
void imu_int_init();

// Count and time of the last edge, read consistently.
[[nodiscard]] ImuEdge imu_int_last();

// Current pin level: high means a sample is waiting to be read.
[[nodiscard]] bool imu_int_high();

} // namespace hal
