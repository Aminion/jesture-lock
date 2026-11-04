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

Stroke: still -> move -> still. Still >= ~1 s in the hand (measured below), detected
automatically from gyro and accel (no button on the Nucleo-32). The holds give zero velocity at both ends of a stroke.

Path (details to be discussed first): accel and gyro fused. At rest: gyro bias and gravity
direction. During the stroke: gyro updates the tilt, accel minus gravity is integrated
twice, then the zero-velocity correction at the end. All of it in `core/`: integer
fixed point, bit-identical on the PC (FFI) and the MCU (`recognize`). The viewer only
draws and converts to physical units.

### 3a: recording

1. [x] `gesture-cli record`: raw stream to a file in `data/`, replayable, so fusion is
   tuned on the same data every run.
2. [x] Analysis on recordings (`gesture-cli`, f64, for reading only): `stats` (bias, noise,
   drift on still data), `timeline` (spread per window), `calib` (accel offset and gain per
   axis from a tumble session), `path` (float reference of the stroke path, to try the
   method before the integer spec; `core/` replaces it).

#### Measurements (2026-10-30, `data/sessions/`, +-2 g / 250 dps, 104 Hz)

| session | what | result |
|---|---|---|
| `rest-table` | 60 s flat on the table | gyro bias +1.84 -5.71 -0.40 dps (6 deg/s total), moves < 0.01 dps over 60 s; noise 0.05 dps, 0.5-0.8 mg; \|accel\| 1032.6 mg; drift on noise alone with ZUPT: 0.3 mm per 1 s, 0.9 mm per 2 s |
| `pose-edge`, `pose-end` | on the long edge, on the short end | y reads -999 mg, x +977 mg; gyro bias the same within 0.06 dps in every pose |
| `tumble` | 14 hand-held poses | offset x -11.2 y +3.7 z +27.6 mg, gain x 1.0089 y 0.9985 z 0.9965; \|a\| error 18.2 mg rms raw -> 1.1 mg fitted |
| `hand-hold` | 4 holds in hand, small moves between | still in hand: gyro std 0.5-2.3 dps, accel std 3-13 mg, rate 0.1-2.8 dps; moving: >= 5 dps, >= 15 mg, >= 4.5 dps; ~0.5 s to settle after a move; soft starts are seen ~0.25 s late |
| `ruler-30` | 5 table slides of 30 cm, 2.7-4.8 s each | 307-359 mm (mean 334, +11%); gyro std 0.1-0.3 dps and accel std 9-30 mg while sliding, so hand thresholds miss the slow end: needs table thresholds |
| `ruler-15` | 5 table slides of 15 cm, 2.1-3.3 s each | 143-156 mm (mean 149, -0.5%), v_end 10-36 mm/s, tilt error 0.05-0.12 deg: no scale error in the method; the +11% at 30 cm is unexplained (longer strokes, 0.5 deg turn, or overshoot) |
| `air-lines` | ~20 cm lines in the air, hand holds between | per-hold gyro bias: tilt error 1.3-3.8 deg, v_end 140-570 mm/s; session bias: tilt error 0.4-1.0 deg, v_end 33-170 mm/s, strokes along clean directions (up/down within 25-51 mm of vertical) |
| `peaks` | 20 s brisk gestures, 10 s max effort | gyro peak 182 dps (73% of 250), accel under 1.8 g (one 1.998 g sample when picking the board up); accel std per second up to 237 mg (brisk), 594 mg (max). Thin margin, so the firmware moved to +-4 g / 500 dps (quantization 0.122 mg, 17.5 mdps: still below the noise) |
| `rest-table-4g` | 20 s on the table after the switch to +-4 g / 500 dps | gyro bias +1.82 -5.67 -0.24 dps; median accel noise per 1 s 0.9 mg (was 0.8) |
| `tumble-4g` | 15 hand-held poses at +-4 g | offset x -4.3 y +5.5 z +25.1 mg (LSB -35 +45 +205), gain x 1.0093 y 0.9990 z 0.9947; \|a\| error 17.6 mg rms raw -> 1.4 mg fitted. Gains and the z offset agree with the +-2 g fit; x offset differs by 7 mg, x seen only from -0.73 to +0.74 (less constrained) |

Design inputs for `docs/core.md`:
- Accel calibration per board (3 offsets, 3 gains) from a tumble session, passed in at
  init: without it rotation leaks up to ~28 mg (~14 cm per 1 s stroke).
- Gyro bias: one long-lived estimate (power-up on the table, updated only when table-quiet),
  never from a short hand hold (a still hand turns 1-3 dps).
- Still detection: gyro std, accel std and gyro rate together; table and hand need
  different thresholds (adaptive to the noise in the hold, or two levels). A still run
  under 0.5 s is a pause inside the move.
- Gravity: mean of 0.3 s ending 0.1 s before the detected move.
- Zero-velocity correction p - v_end t^2 / (2T), linear velocity error.
- Drop strokes below a minimum size or speed (table bumps gave 0.1-0.5 mm "strokes").
- A hold between strokes is >= ~1 s in the hand (0.5 s settle + window).

### 3b: core contract

`gesture/core.h` is a contract between languages, like the wire format: the C++ code is its
first implementation, another language may replace it later without changes to the Rust
host or the firmware.

1. [x] `docs/core.md`, the ABI: opaque caller-owned state (size and alignment fixed in the
   header, so the RAM budget is part of the contract), init, per-sample feed with `t_us`,
   events (ready / moving / stroke / rejected, with the state), reading out the stroke
   path into a caller buffer. Only C types cross it: fixed-width integers, fixed-layout structs, pointer +
   length; no callbacks, no allocation.
2. [x] `docs/core.md`, the algorithm spec (frozen 2026-10-30): fixed-point formats, rounding, saturation and
   order of operations, exact enough for a second implementation to match bit for bit.
   Streaming: look-back ring at the start, rewind to a snapshot at the end. Frozen after:
   - [x] `gesture-cli path` rewritten as a streaming float reference that follows the
     text (config bias, causal still windows, ring replay, snapshot). With the +-2 g
     tumble and rest-table bias: ruler-15 142-155 mm (mean 148), tilt error 0.02-0.06
     deg; air-lines 8 strokes in the drawn directions, tilt error 0.10-1.20 deg (6 of 8
     under 0.25), v_end 64-249 mm/s; ruler-30 slides over 4 s are rejected LONG
     (max_stroke_us), the other 3 give 302-363 mm;
   - [x] a tumble session at +-4 g (`tumble-4g`).
3. [x] RAM budget: state 2048 B fixed in the header (ring 48 x 14 B, 64 points x 8 B as
   int16 mm, halving the point rate when full).
4. [x] `core.h`: ABI v1 declarations, valid C11 and C++ (compiled as C11 by
   `core/contract/tests/core_c_check.c`). Sizes asserted in the header; alignment and every field
   offset asserted in `core_c_check.c` and in `gesture-core/src/ffi.rs`, so a layout
   drift on either side fails the build.

### 3c: C++ implementation

1. [x] `core/`: still detection, rest calibration, tilt from the gyro, gravity removal,
   double integration, zero-velocity correction, as `docs/core.md`. Integer only; M0+ has
   no hardware divide, so measure `gc_feed` on the board (worst case: the stroke-start
   replay) against the 9.6 ms sample period (16 MHz; 64-bit multiplies and `rdiv` are
   library calls). If it does not fit: shifts for power-of-two divisors, spreading the
   replay, or the PLL (64 MHz). State: ring 48 x 14 B, points 64 x 8 B, the rest ~300 B,
   ~1.5 KB of the 2048 B `GC_STATE_SIZE`.
   - [x] step 4, integration: turn, renormalize, R(q) (now spelled out in full in
     `core.md`, same formula), gravity removal, v and p, clip flag, point buffer,
     `gc_live`. Checked against the float reference on `hand-hold`: the first stroke's
     `gc_live` path (406 samples, 197 mm) is within 0.53 mm of it, i.e. the mm rounding
     (`events.rs`).
   - [x] step 5: LONG, snapshot, stroke end, `gc_stroke`. Every event of every session
     against `path` (`events.rs`): same sample, kind, reason and duration; v_end within
     0.5 % (the first-order quaternion step leaks a little gravity over big turns);
     one decision differs, on the extent threshold (ruler-15, 19.25 vs 20 mm: integer
     points are rounded twice). `path` now rounds its config to the gc_config formats,
     so both decide on the same thresholds. State 1464 B of 2048.
   - [x] Board self-test (`recognize`, `core/contract/vectors/v1/README.md`): 2400 samples of
     air-lines (19 events, 4 strokes) replayed through `gc_feed` at boot. 2026-11-04,
     16 MHz HSI: digest matches the host (events, strokes, points and every `gc_live`
     point bit for bit); stack peak 1088 B (`_Min_Stack_Size` raised from 1 KB to 1.5 KB); flash
     47.9 KB with the 38.5 KB vector. `gc_feed` does not fit the 9600 us period:
     mean 1420 us, worst 22.9 ms at a stroke end (sample 1129), 11.9 ms at MOVING
     (the ring replay), 17 calls of 2400 over the period.
     64 MHz (PLL from HSI16, 2 flash wait states, prefetch; APB / 4 keeps PCLK at
     16 MHz for I2C and UART): same digest; mean 392 us, worst 6.6 ms (stroke end),
     3.4 ms (replay), none over the period. 3.5x, not 4x (wait states). `stream` at
     64 MHz: `check --seconds 20` clean (0 missed, 0 errors).
   - [x] Spec gaps closed in `core.md` (2026-11-04), outputs unchanged on every defined
     input (board digest still 5f314142): the snapshot point is appended even on a point
     at t = T (two equal points, as v1 means it); CLIPPED, SHORT (duration) and DRIFT run
     before the cut and the correction, DRIFT on the unsaturated v_end, which bounds the
     correction products for any config; c and step are uint32, `info.step` saturates;
     "Ranges" derived from the gates instead of a hand's range.
   - The v_end tolerance is the rotation step: with `path` switched to the same
     first-order step, every session agrees to within the rounding (checked once).
   - Open: "Ranges" needs |gyro_bias_q8| <= 2^23 (int16 range), which `gc_init` does not
     check; a v2 could make it a "Valid" rule.
2. [x] Conformance vectors as data files (`core/contract/vectors/v1/README.md`): 22 GCV1 files, the
   board vector, 4 whole sessions (ruler-15 near the extent threshold, ruler-30, hand-hold,
   air-lines) and 17 synthetic cases (`gesture-cli vectors`, `synth.rs`: min-jerk moves
   and turns through the inverse calibration; every scale, gaps, LONG, CLIPPED in the
   replay, SHORT both ways, DRIFT at 65535, clock wraps, compaction, and a buffer still
   full at the end, so the end step compacts: checked to reach that branch). Run by `ctest` (`core_vectors`, `core.h` only) and `cargo test`, which
   also checks each synthetic case's events and true end point apart from the digest.
   Synthetic lines end within 2 mm of the truth.
   Config defaults (hand, +-4 g / 500 dps; from the measurements above, used by the
   synthetic cases at each scale): still gyro std 171 LSB (3 dps), accel std 123 (15 mg),
   gyro rate 229 (4 dps); table preset 29 / 25 / 57 (0.5 dps, 3 mg, 1 dps); hold_n 20;
   bias update 11 (0.2 dps), 16 (2 mg); clip 32700; max_dt 14423 us; stroke
   150000..4000000 us; min extent 20 mm; max v_end 300 mm/s; calibration from
   `tumble-4g`, bias from `rest-table-4g`.
3. [x] Build option `GESTURE_CORE_IMPL` (`core/CMakeLists.txt`, default `cpp`): a directory
   under `core/` that defines the static library `gesture_core` linking `gesture_core_abi`
   (built by CMake, or IMPORTED GLOBAL from another toolchain), checked at configure.
   The firmware and `core_vectors` link `gesture_core`; the implementation's own unit
   tests, if any, live in `<dir>/tests`. `gesture-core` build.rs passes the environment
   variable of the same name. Checked with a copy of `cpp` changed by one LSB in v_end:
   `core_vectors` and the cargo vector tests fail on it.

### 3d: host tools

1. [x] `gesture-core` FFI: safe wrapper over `core.h`. `ffi.rs` is the raw mirror (layout
   checked); `lib.rs` has `Core` (boxed state), `Error` for `gc_init`, typed `Event`
   (`Kind`, `State`, `Reject`) and an owned `Stroke`. Every output is checked against the
   "else 0" rules of `docs/core.md` (a violation panics), so `GcEvent::from(event)` gives
   the implementation's exact bytes and the cargo-side digests stay honest.
2. [x] `gesture-cli strokes <file>`: per stroke duration, net displacement, peak speed,
   residual velocity before the correction. Ruler test: 30 cm slide -> ~0.30 m.
   The integer core through the wrapper (`strokes.rs`), configured as `path`; rejected
   moves listed with their reason. Peak speed is between the decimated points (step 4 or
   8 here), so it averages short peaks. ruler-30 (tumble, rest-table bias, hand): three
   long strokes of 309, 361 and 249 mm along x (mean 306), v_end 16..34 mm/s, peak
   0.20..0.29 m/s, within 1 mm of the float reference; checked by a test.

### 3e: viewer

Live stream -> `core/` through FFI -> drawing.

Layout: a top bar (input source, connection, counters), a side panel (state, last event,
stroke numbers, buttons, event log), the 3D view as the main area, a collapsible raw plot
below it. Built in this order, each step usable on its own; replay makes everything except
the serial port testable without the board.

1. [x] Shared host crate (`session`): move out of `gesture-cli` what the viewer needs too,
   so there is one copy: recording load (`stats::Recording`), the serial port `open`,
   frame -> sample decoding, `reference_config` (calibration from `tumble`, bias from
   `rest-table`, hold thresholds -> `GcConfig`). A move only: `cargo test` and the vectors
   unchanged.
   Modules: `recording`, `calib` (the fit; its report stays in the CLI), `config` (was
   `path::Config`; the float algorithm stays in `path`), `check` (stream health, for the
   top bar), `serial` (`open`, `read_some`, `settle`), `reference` (`ReferenceArgs`,
   `Hold`, `reference_config`, clap args the viewer reuses). The outputs of `stats`,
   `calib`, `timeline`, `path`, `events` and `strokes` are byte for byte the same.
2. [x] Input behind one interface, a worker thread that feeds `Core` and sends samples,
   events, live points and strokes to the UI over a channel:
   - replay of a `.bin` session (real time or faster, pause);
   - live serial: port list with refresh, connect / disconnect, decode with the `protocol`
     crate; rate, frame and error counters in the top bar.
   Configured by the same `--calib`, `--bias`, `--hold` arguments as `strokes` (file
   pickers later). Check: replay of ruler-30 gives the strokes `gesture-cli strokes` lists.
   Done: `viewer/src/feed.rs` (frames -> `check::Stats` + `Core`, restarted on a new or
   changed `Hello` and on an MCU reset), `source.rs` (replay paced by MCU time at 1x / 2x /
   5x / max with pause; serial: `open`, `settle`, read loop; stop by dropping the handle).
   `session` gained `CoreArgs` / `Setup` (config from a `Hello`, so live needs no recording),
   `strokes` (`Summary`, `run`, moved from the CLI) and `serial::ports`. Live without
   `--bias` starts from zero bias (the core updates it in holds). The test replays ruler-30
   through the worker path and matches a direct core run sample by sample (events, live
   points, strokes) and `strokes::run`. Run live on the board with the `stream` firmware.
3. [x] State indicator: WARMUP / READY / MOVING, and the last event with its reject reason
   (`REJ SHORT`), so a missing stroke shows why. Event log in the side panel.
   Done: right side panel. The state follows every sample (also `NONE` events); the last
   event and each log line give a reject's reason against the core's limits, which
   `Update::Started` now carries (`REJ SHORT: <dur> s < <min> s`, `REJ DRIFT: v_end <v> > <max>
   mm/s`; SHORT with a passing duration means the extent). STROKE lines get |d| and extent
   from the summary. The log keeps the last 1000 lines; its clock (MCU time) goes on across
   an MCU reset. Test: ruler-30 replayed into the view logs 4 STROKE, 3 REJ SHORT, 1 REJ
   DRIFT and ends READY.
4. [ ] Stroke capture in a 3D view (no 2D step first: points are in the board's axes at
   the stroke start, `docs/core.md`, so a fixed top view is only right for a flat board).
   Drawn on the 2D egui `Painter` with our own projection, no 3D library:
   - capture: while moving, the live path (`gc_live`, uncorrected); at `STROKE` the
     corrected points, with the live path kept faint beside them, so the gap shows the
     drift; a rejected move greys out. The last stroke stays until the next. Beside it
     the `strokes` numbers (|d|, duration, peak speed, v_end, extent).
   - level: rotate by the stroke's `gravity_ug` so up is up; forward = the board's x at
     the stroke start (no magnetometer). Gravity comes only with `gc_stroke`, so the live
     path and a rejected move stay in the board frame until then.
   - camera: orbit with mouse drag, zoom with scroll; presets front / top / side, fit,
     and plane (look along the normal of the stroke's best-fit plane: the flat view of
     a gesture drawn in any direction).
   - scene: floor grid, axis marker, start/end markers; far segments drawn first (sort by
     depth, no depth buffer).
   - Clear: wipe the view. Keep all: overlay several strokes (does the same gesture
     repeat?).
   Check: projection and levelling unit-tested (known points -> known screen positions,
   `gravity_ug` along +z, which is up -> no rotation); ruler-30 strokes look straight and ~30 cm in fit view.
5. [ ] Record session: write the incoming stream to `data/sessions/*.bin` in the `record`
   format, so anything seen live goes through the CLI tools. Saving one stroke as a
   template waits for `docs/template_format.md`.
6. [ ] Path drawing: a dot per point, colored by speed (perceptual gradient, e.g. blue ->
   yellow, fixed range ~0..1.5 m/s with a legend); optional thin connecting line.
7. [ ] Raw signal plot (`egui_plot`, collapsible, off by default): gyro and accel, 6 axes,
   last few seconds, with clipping marked (near the range ends: the `peaks` run at
   +-2 g / 250 dps reached ~1.8 g and 182 dps).

## Step 4: fusion algorithms compared

After Step 3 works end to end. Standard filters, each where it fits, chosen by numbers on
the recordings. A winner becomes a new `core.h` version (another implementation of the
contract, run against the same conformance vectors).

1. [ ] Bake-off in `gesture-cli` (f64): Mahony, Madgwick, ESKF with zero-velocity updates
   and an RTS smoother, and the v1 method, on ruler-15, ruler-30, air-lines and peaks.
   Compare distance error, tilt error at the end hold, v_end, and how the path looks.
2. [ ] Continuous attitude (Mahony or Madgwick, by the bake-off): board orientation at all
   times, also between strokes, for a live 3D board model in the viewer. Accel as "down"
   is right while the board is near still; core v2 adds `gc_attitude()` (fixed point, on
   the MCU too).
3. [ ] ESKF + RTS smoother as the stroke path (if it beats v1): estimates position,
   velocity, attitude and gyro/accel biases together; the holds are zero-velocity
   measurements and the smoother replaces the linear drift correction. Fixed-point
   matrices and numerical stability on the M0+; state size may grow (`GC_STATE_SIZE`).

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
