#pragma once

// docs/core.md "Per sample", step 2: the newest GC_RING_N raw samples with their dt.
// Indexed by age (0 = newest), so the replay and the windows never see the wrap.

#include "config.hpp"

#include <array>
#include <cstdint>

namespace gcore {

struct Raw {
    std::uint16_t dt; // us; <= max_dt_us after the gap check, 0 for the sample after a gap
    std::array<std::int16_t, 3> gyro;
    std::array<std::int16_t, 3> accel;
};
static_assert(sizeof(Raw) == 14);

struct Ring {
    std::array<Raw, ring_n> buf;
    std::uint8_t newest; // index in buf
    std::uint8_t count;  // 0 .. ring_n
};

constexpr void clear(Ring& r) {
    r.count = 0;
}

constexpr void push(Ring& r, const Raw& x) {
    r.newest = static_cast<std::uint8_t>((r.newest + 1) % ring_n);
    r.buf[r.newest] = x;
    if (r.count < ring_n) {
        ++r.count;
    }
}

// The sample `age` steps back from the newest; age < count.
[[nodiscard]] constexpr const Raw& at(const Ring& r, int age) {
    return r.buf[static_cast<std::size_t>((r.newest + ring_n - age) % ring_n)];
}

} // namespace gcore
