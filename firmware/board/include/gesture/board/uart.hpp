#pragma once

// USART2 to the ST-LINK virtual COM port: TX PA2, RX PA3, AF1, 8N1.
// TX by DMA from two 64-byte buffers; RX by interrupt into a 64-byte ring, drained with
// uart_read().
// G031 USART2 is the basic variant (RM0444 Table 183): no FIFO, no prescaler,
// kernel clock is always PCLK = 16 MHz.

#include "gesture/board/clock.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <span>

namespace board {

// RCC_CCIPR.USART2SEL is reserved on G031 (RM0444 5.4.21 note 1): the kernel clock is PCLK.
inline constexpr std::uint32_t uart_clock_hz = pclk_hz;

// Oversampling by 16: BRR = USARTDIV = f_ck / baud, rounded to nearest (RM0444 33.5.7).
constexpr std::uint32_t uart_brr(std::uint32_t baud) {
    return (uart_clock_hz + baud / 2) / baud;
}

// How far the real baud rate (f_ck / BRR) is from `baud`, in ppm.
constexpr std::uint32_t uart_baud_error_ppm(std::uint32_t baud) {
    const std::uint64_t real_x_brr = uart_clock_hz;
    const std::uint64_t ideal_x_brr = std::uint64_t{baud} * uart_brr(baud);
    const std::uint64_t diff =
        real_x_brr > ideal_x_brr ? real_x_brr - ideal_x_brr : ideal_x_brr - real_x_brr;
    return static_cast<std::uint32_t>(diff * 1'000'000 / ideal_x_brr);
}

// The receiver tolerates 3.33% in total for 8N1, oversampling by 16, 3-sample majority
// (RM0444 Table 189). Give BRR rounding at most 1% of it; the rest is for the PC side
// and the HSI16 deviation.
inline constexpr std::uint32_t uart_max_baud_error_ppm = 10'000;

// BRR must be >= 16 (RM0444 33.5.7) and fit in 16 bits.
constexpr bool uart_baud_ok(std::uint32_t baud) {
    if (baud == 0) {
        return false;
    }
    const std::uint32_t brr = uart_brr(baud);
    return brr >= 16 && brr <= std::numeric_limits<std::uint16_t>::max() &&
           uart_baud_error_ppm(baud) <= uart_max_baud_error_ppm;
}

// Uses uart_brr(); check `baud` with uart_baud_ok() at compile time.
void uart_init(std::uint32_t baud);

// Copies `data` into a TX buffer and returns; DMA sends it in the background. Blocks only
// while both buffers are full, until the one on the wire is sent (~5.6 ms for 64 B).
void uart_write(std::span<const std::uint8_t> data);

// Next received byte, nullopt if the ring is empty. Never blocks.
[[nodiscard]] std::optional<std::uint8_t> uart_read();

// Bytes lost since uart_init(): ring full, hardware overrun, framing or noise error.
[[nodiscard]] std::uint32_t uart_rx_errors();

} // namespace board
