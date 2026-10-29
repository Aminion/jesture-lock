#pragma once

// Single-producer single-consumer byte ring between an ISR and the main loop (one core).
// Register-free, so it is unit-tested on the host (fw/hal/tests).
//
// The producer writes only `head`, the consumer only `tail`. Byte-wide indices are single
// loads/stores on M0+, so no locking. One slot stays empty to tell full from empty.
// volatile + signal fences, not std::atomic: on ARMv6-M atomics are not always lock-free
// (read-modify-write goes to libatomic, which is not linked).

#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace hal {

template <std::size_t N>
struct Ring {
    static_assert(std::has_single_bit(N), "index wrap is a mask");
    static_assert(N >= 2 && N <= 256, "indices are one byte; one slot stays empty");

    std::array<std::uint8_t, N> buf{};
    volatile std::uint8_t head = 0;
    volatile std::uint8_t tail = 0;
};

// Index after `i`, wrapping at N.
template <std::size_t N>
[[nodiscard]] constexpr std::uint8_t ring_next(std::uint8_t i) {
    return static_cast<std::uint8_t>((i + 1U) & (N - 1));
}

// Producer side. false (byte dropped) if the ring holds N - 1 bytes.
template <std::size_t N>
[[nodiscard]] bool ring_push(Ring<N>& ring, std::uint8_t byte) {
    const std::uint8_t head = ring.head;
    const std::uint8_t next = ring_next<N>(head);
    if (next == ring.tail) {
        return false;
    }
    ring.buf[head] = byte;
    // Compiler-only fence (single core): the byte is stored before `head` publishes it.
    std::atomic_signal_fence(std::memory_order_release);
    ring.head = next;
    return true;
}

// Consumer side. nullopt if empty; never blocks.
template <std::size_t N>
[[nodiscard]] std::optional<std::uint8_t> ring_pop(Ring<N>& ring) {
    const std::uint8_t tail = ring.tail;
    if (tail == ring.head) {
        return std::nullopt;
    }
    // Read the slot after seeing `head` move, and finish reading it before `tail` hands
    // it back to the producer.
    std::atomic_signal_fence(std::memory_order_acquire);
    const std::uint8_t byte = ring.buf[tail];
    std::atomic_signal_fence(std::memory_order_release);
    ring.tail = ring_next<N>(tail);
    return byte;
}

} // namespace hal
