#pragma once

// CRC16 over frame payloads: CRC-16/IBM-3740 (a.k.a. CRC-16/CCITT-FALSE), docs/protocol.md.
// Bitwise, no table: ~10-20 us per frame on the M0+, next to ~2 ms on the wire.

#include <cstdint>
#include <span>

namespace proto {

inline constexpr std::uint16_t crc16_poly = 0x1021;
inline constexpr std::uint16_t crc16_init = 0xFFFF;

// MSB first, no reflection, xorout 0. Pass a previous result as `crc` to continue over
// more data.
[[nodiscard]] constexpr std::uint16_t crc16(std::span<const std::uint8_t> data,
                                            std::uint16_t crc = crc16_init) {
    for (const std::uint8_t byte : data) {
        crc = static_cast<std::uint16_t>(crc ^ (byte << 8U));
        for (int bit = 0; bit < 8; ++bit) {
            const bool msb = (crc & 0x8000U) != 0;
            crc = static_cast<std::uint16_t>(crc << 1U);
            if (msb) {
                crc ^= crc16_poly;
            }
        }
    }
    return crc;
}

} // namespace proto
