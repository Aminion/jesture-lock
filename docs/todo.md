# To do

## Step 2b: full-rate binary stream

Every sample goes to the PC as a binary frame (`docs/protocol.md`), replacing the bring-up
text lines.

1. [x] `docs/protocol.md`: framing, messages, test vector.
2. [x] `proto`: constexpr COBS, CRC16, frame codec, host tests.
3. [x] Rust `protocol` crate: CRC via the `crc` crate (`CRC_16_IBM_3740`), COBS by hand
   (same canonical encoding as `proto`), frame encode/decode with the same limits and
   errors, a stream deframer that resyncs on 0x00, Hello/Status/Sample types. Tests use
   the `docs/protocol.md` vector and the same COBS cases as `proto_test.cpp`.
4. [x] `stream` firmware: a Sample frame per INT1 edge, Hello/Status every 1 s; drop the text
   lines and the echo.
5. [x] `gesture-cli check`: read the stream, count frames, CRC errors and sample gaps.

# Later

Deferred adjustments, with the numbers that motivated them.

## Firmware timing

Measured 2026-10-29 (`stream` firmware bring-up, `drdy` line): sample period 9600 us, latency from
INT1 edge to sample read up to 7.5 ms, so about 2 ms left before a sample is lost.

Done 2026-10-30: I2C at 400 kHz (RM0444 Table 173 fast-mode timing, `i2c_timing_ok` checks
Sm and Fm) and UART TX by DMA (DMA1 channel 1, two 64 B buffers, +136 B RAM). Worst case over
~20 s with temporary SWD-read maxima in `poll_imu`:

| build                      | INT1 edge -> sample read | edge -> Sample frame queued |
|----------------------------|--------------------------|-----------------------------|
| 100 kHz, blocking TX       | 3577 us                  | 5718 us                     |
| 400 kHz, blocking TX       | 2583 us                  | 4725 us                     |
| 400 kHz, DMA TX            | 751 us                   | 1006 us                     |

`gesture-cli check --seconds 60`: 0 errors, `missed` 0 before and after (no loss at 104 Hz
either way; the gain is headroom). The pull-ups on the GY module set the SCL/SDA rise time,
which may exceed the 300 ns fast-mode limit; no I2C errors seen so far.

- **I2C RX by DMA.** Only if the CPU must be free during the read; low value after 400 kHz.
- UART RX stays interrupt-driven: the PC sends little.
