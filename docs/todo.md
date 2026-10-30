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

## Step 3: stroke path

See the 3D path of one stroke. No recognition and no templates yet. Later: a gesture is a
sequence of strokes with holds between them.

Stroke: still -> move -> still. Still ~0.3 s, detected automatically from gyro and accel
(no button on the Nucleo-32). The holds give zero velocity at both ends of a stroke.

Path (details to be discussed first): accel and gyro fused. At rest: gyro bias and gravity
direction. During the stroke: gyro updates the tilt, accel minus gravity is integrated
twice, then the zero-velocity correction at the end. All of it in `core/`: integer
fixed point, bit-identical on the PC (FFI) and the MCU (`recognize`). The viewer only
draws and converts to physical units.

### 3a: recording

1. [ ] `gesture-cli record`: raw stream to a file in `data/`, replayable, so fusion is
   tuned on the same data every run.

### 3b: core contract

`gesture/core.h` is a contract between languages, like the wire format: the C++ code is its
first implementation, another language may replace it later without changes to the Rust
host or the firmware.

1. [ ] `docs/core.md`, the ABI: opaque caller-owned state (size and alignment fixed in the
   header, so the RAM budget is part of the contract), init, per-sample feed with `t_us`,
   events (still / moving / stroke done), reading out the stroke path into a caller
   buffer. Only C types cross it: fixed-width integers, fixed-layout structs, pointer +
   length; no callbacks, no allocation.
2. [ ] `docs/core.md`, the algorithm spec: fixed-point formats, rounding, saturation and
   order of operations, exact enough for a second implementation to match bit for bit.
3. [ ] RAM budget for the stroke buffer: 104 Hz x 2 s x 3 x int32 is ~2.5 KB of 8 KB.
   Options: fixed point count per stroke (downsampled), int16 positions, a stroke length
   limit.

### 3c: C++ implementation

1. [ ] `core/`: still detection, rest calibration, tilt from the gyro, gravity removal,
   double integration, zero-velocity correction. Integer only; no divide on the
   per-sample path (M0+ has no hardware divide).
2. [ ] Conformance vectors as data files (raw recording in -> events and path out),
   checked through `core.h` only, so any implementation runs the same suite. Synthetic
   strokes (known path -> ideal IMU signals) plus real recordings.
3. [ ] Build: the `gesture-core` build.rs and the firmware link "a static lib
   `gesture_core` providing `core.h`"; the implementation is one build option.

### 3d: host tools

1. [ ] `gesture-core` FFI: safe wrapper over `core.h`.
2. [ ] `gesture-cli strokes <file>`: per stroke duration, net displacement, peak speed,
   residual velocity before the correction. Ruler test: 30 cm slide -> ~0.30 m.

### 3e: viewer

Live stream -> `core/` through FFI -> drawing.

UI (`viewer`, egui):
1. [ ] Live input: open the serial port, decode with the `protocol` crate.
2. [ ] State indicator: still (calibrating / ready) or moving.
3. [ ] Stroke capture from moving to the next still; the last stroke stays until the next.
4. [ ] 3D view on the 2D egui `Painter`, our own projection (no 3D library):
   - orbit with mouse drag, zoom with scroll, front/top/side presets;
   - axes: up from gravity, forward = board heading at the stroke start (no magnetometer);
   - floor grid, axis marker, start/end markers;
   - far segments drawn first (sort by depth, no depth buffer).
5. [ ] Path drawing: a dot per sample, colored by speed (perceptual gradient, e.g. blue ->
   yellow, fixed range ~0..1.5 m/s with a legend); optional thin connecting line.
6. [ ] Raw signal plot (gyro and accel, 6 axes) for debugging, with clipping marked
   (+-250 dps and +-2 g may clip on fast moves).

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
