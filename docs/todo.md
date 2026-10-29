# Later

Deferred adjustments, with the numbers that motivated them.

## Firmware timing

Measured 2026-10-29 (`fw_stream` bring-up, `drdy` line): sample period 9600 us, latency from
INT1 edge to sample read up to 7.5 ms, so about 2 ms left before a sample is lost.
None of this uses DMA yet (`stm32g0xx_ll_dma.c` is built but unused).

- **UART TX by DMA.** Today `uart_write` blocks ~87 us per byte (115200 baud), ~5 ms per
  text line. Fill a buffer, start DMA1 (route through DMAMUX), return. Needs the buffer to stay
  untouched until the transfer ends: two frame-sized buffers (2 x 32-64 B), count the RAM.
  Do it after step 2b, once frames are binary; `lat` and `miss` show the gain.
- **I2C at 400 kHz.** One sample read (STATUS + 12 bytes) takes ~1.9 ms at 100 kHz,
  ~0.5 ms at 400 kHz. New TIMINGR value (RM0444 Table 173, fast mode) plus a fast-mode
  variant of `i2c_timing_standard_mode_ok`, with its host tests.
- **I2C RX by DMA.** Only if the CPU must be free during the read; low value after 400 kHz.
- UART RX stays interrupt-driven: the PC sends little.
