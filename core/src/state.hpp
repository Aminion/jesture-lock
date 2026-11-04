#pragma once

// The core's state inside the caller's gc_state buffer. Trivially copyable and free of
// pointers into itself, so the caller may move the buffer between calls (Rust moves
// values); everything refers to the ring and points by index.

#include "gesture/core.h"
#include "ring.hpp"
#include "stroke.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>

namespace gcore {

struct State {
    gc_config cfg;
    std::int32_t sa;                     // accel ug/LSB
    std::int32_t sg;                     // gyro udps/LSB
    std::array<std::int32_t, 3> bias_q8; // gyro bias, LSB x 256
    Ring ring;
    std::uint32_t t_prev; // t_us of the previous sample
    bool fed;             // a sample came since gc_init (else dt is undefined: a gap)
    std::uint16_t streak; // quiet windows in a row, saturating
    std::uint8_t mode;    // GC_ST_*
    Stroke stroke;        // valid in MOVING
};

static_assert(sizeof(State) <= GC_STATE_SIZE);
static_assert(alignof(State) <= GC_STATE_ALIGN);
static_assert(std::is_trivially_copyable_v<State>);

// The State that gc_init placed in the buffer. Cast from gc_state, not from `opaque`: the
// struct type carries GC_STATE_ALIGN, the uint8_t array type (for -Wcast-align) does not.
static_assert(offsetof(gc_state, opaque) == 0);

[[nodiscard]] inline State& state(gc_state* s) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): gc_init created it there
    return *std::launder(reinterpret_cast<State*>(s));
}

[[nodiscard]] inline const State& state(const gc_state* s) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): gc_init created it there
    return *std::launder(reinterpret_cast<const State*>(s));
}

} // namespace gcore
