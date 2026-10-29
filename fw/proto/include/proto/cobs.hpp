#pragma once

// COBS encode/decode for UART frames (docs/protocol.md). The encoded bytes contain no 0x00;
// the 0x00 delimiter is not part of them.
//
// Canonical encoding: a run of 254 non-zero bytes that ends the input gets no extra empty
// block, so the output is at most n + ceil(n / 254) bytes (1 for empty input).

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

namespace proto {

// Data bytes in a full block; its code byte is cobs_max_block + 1 (0xFF).
inline constexpr std::size_t cobs_max_block = 254;
inline constexpr std::uint8_t cobs_full_code = cobs_max_block + 1;

// Upper bound on the encoded size of `n` input bytes; size encode buffers with it.
[[nodiscard]] constexpr std::size_t cobs_max_encoded(std::size_t n) {
    return n + (n / cobs_max_block) + 1;
}

// Bytes written, or nullopt if `out` is smaller than cobs_max_encoded(in.size()).
[[nodiscard]] constexpr std::optional<std::size_t> cobs_encode(std::span<const std::uint8_t> in,
                                                               std::span<std::uint8_t> out) {
    if (out.size() < cobs_max_encoded(in.size())) {
        return std::nullopt;
    }
    std::size_t code_at = 0; // slot of the current block's code byte
    std::size_t o = 1;
    std::uint8_t code = 1; // 1 + data bytes in the current block
    for (std::size_t i = 0; i < in.size(); ++i) {
        if (in[i] != 0) {
            out[o++] = in[i];
            ++code;
            if (code != cobs_full_code || i + 1 == in.size()) {
                continue;
            }
        }
        // A zero, or a full block with more input to come: close the block.
        out[code_at] = code;
        code_at = o++;
        code = 1;
    }
    out[code_at] = code;
    return o;
}

enum class CobsError : std::uint8_t {
    zero_byte = 1, // 0x00 inside the encoded bytes
    truncated,     // a code byte promises more bytes than are left
    overflow,      // decoded data does not fit `out`
};

// Bytes written to `out`. Empty input decodes to nothing.
[[nodiscard]] constexpr std::expected<std::size_t, CobsError>
cobs_decode(std::span<const std::uint8_t> in, std::span<std::uint8_t> out) {
    std::size_t i = 0;
    std::size_t o = 0;
    while (i < in.size()) {
        const std::uint8_t code = in[i++];
        if (code == 0) {
            return std::unexpected(CobsError::zero_byte);
        }
        const std::size_t len = code - 1U;
        if (len > in.size() - i) {
            return std::unexpected(CobsError::truncated);
        }
        if (len > out.size() - o) {
            return std::unexpected(CobsError::overflow);
        }
        for (std::size_t k = 0; k < len; ++k) {
            const std::uint8_t byte = in[i++];
            if (byte == 0) {
                return std::unexpected(CobsError::zero_byte);
            }
            out[o++] = byte;
        }
        // Every block but a full one stands for a zero, except at the very end.
        if (code != cobs_full_code && i < in.size()) {
            if (o == out.size()) {
                return std::unexpected(CobsError::overflow);
            }
            out[o++] = 0;
        }
    }
    return o;
}

} // namespace proto
