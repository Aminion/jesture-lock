#pragma once

// Frames on the wire: COBS(payload || crc16) 0x00, docs/protocol.md "Framing".

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

#include "proto/cobs.hpp"
#include "proto/crc16.hpp"
#include "proto/le.hpp"

namespace proto {

inline constexpr std::uint8_t version = 1;
inline constexpr std::uint32_t baud = 115'200; // 8N1

inline constexpr std::size_t header_bytes = 2; // type, version
inline constexpr std::size_t max_payload = 32;
inline constexpr std::size_t crc_bytes = 2;
// A frame after COBS decoding: payload + CRC.
inline constexpr std::size_t max_decoded = max_payload + crc_bytes;
// Encoded payload + CRC + the 0x00 delimiter: 36 bytes.
inline constexpr std::size_t max_frame = cobs_max_encoded(max_decoded) + 1;

// Writes the whole frame, delimiter included. Bytes written, or nullopt if the payload is
// longer than max_payload or `out` is shorter than max_frame.
[[nodiscard]] constexpr std::optional<std::size_t> frame_encode(std::span<const std::uint8_t> payload,
                                                                std::span<std::uint8_t> out) {
    if (payload.size() > max_payload || out.size() < max_frame) {
        return std::nullopt;
    }
    std::array<std::uint8_t, max_decoded> raw{};
    std::ranges::copy(payload, raw.begin());
    store_le(std::span{raw}.subspan(payload.size()).first<crc_bytes>(), crc16(payload));
    const auto encoded = cobs_encode(std::span{raw}.first(payload.size() + crc_bytes), out);
    if (!encoded) {
        return std::nullopt; // cannot happen: out holds max_frame
    }
    out[*encoded] = 0;
    return *encoded + 1;
}

enum class FrameError : std::uint8_t {
    cobs = 1,  // COBS decoding failed
    too_short, // fewer than header + CRC bytes
    too_long,  // more than max_payload + CRC bytes
    crc,       // CRC mismatch
};

// Decodes one frame, given without its 0x00 delimiter, into `out`. Returns the payload
// length; the payload starts at out[0], its CRC follows it. On error `out` holds garbage.
[[nodiscard]] constexpr std::expected<std::size_t, FrameError>
frame_decode(std::span<const std::uint8_t> encoded, std::span<std::uint8_t, max_decoded> out) {
    const auto decoded = cobs_decode(encoded, out);
    if (!decoded) {
        return std::unexpected(decoded.error() == CobsError::overflow ? FrameError::too_long
                                                                      : FrameError::cobs);
    }
    if (*decoded < header_bytes + crc_bytes) {
        return std::unexpected(FrameError::too_short);
    }
    const std::size_t len = *decoded - crc_bytes;
    if (crc16(out.first(len)) != load_le<std::uint16_t>(out.subspan(len).first<crc_bytes>())) {
        return std::unexpected(FrameError::crc);
    }
    return len;
}

} // namespace proto
