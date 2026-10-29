#pragma once

// Little-endian integer fields, as in every payload (docs/protocol.md) and the LSM6DS3
// output registers. Byte by byte, so the result does not depend on the host's byte order.

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace proto {

template <std::integral T>
[[nodiscard]] constexpr T load_le(std::span<const std::uint8_t, sizeof(T)> in) {
    using U = std::make_unsigned_t<T>;
    U v = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        v = static_cast<U>(v | (U{in[i]} << (8U * i)));
    }
    return static_cast<T>(v); // modular for signed T
}

template <std::integral T>
constexpr void store_le(std::span<std::uint8_t, sizeof(T)> out, T value) {
    auto v = static_cast<std::make_unsigned_t<T>>(value);
    for (std::uint8_t& byte : out) {
        byte = static_cast<std::uint8_t>(v & 0xFFU);
        v = static_cast<decltype(v)>(v >> 8U);
    }
}

} // namespace proto
