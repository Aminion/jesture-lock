#pragma once

// USART2 to the ST-LINK virtual COM port: TX PA2, RX PA3, AF1, 8N1.
// Blocking TX; RX by interrupt into a 64-byte ring, drained with uart_read().
// G031 USART2 is the basic variant (RM0444 Table 183): no FIFO, no prescaler,
// kernel clock is always PCLK = 16 MHz.

#include <cstdint>
#include <optional>
#include <span>

namespace hal {

// Picks the closest BRR for `baud` with oversampling by 16.
void uart_init(std::uint32_t baud);

// Blocks until the last byte is handed to the transmitter (it may still be on the wire).
void uart_write(std::span<const std::uint8_t> data);

// Next received byte, nullopt if the ring is empty. Never blocks.
[[nodiscard]] std::optional<std::uint8_t> uart_read();

// Bytes lost since uart_init(): ring full, hardware overrun, framing or noise error.
[[nodiscard]] std::uint32_t uart_rx_errors();

} // namespace hal
