# Calibration

How one board's IMU calibration is measured (`gesture-cli calibrate`), stored (the
calibration file) and used (linked into the `recognize` firmware and the desktop tools).
Change this document first, then the tool and both readers.

Status: implemented. The first `board.cal` is from the run of 2026-11-14
(`data/sessions/20261114-221042_calibrate`: 12 holds, 17.7 mg raw -> 2.6 mg fitted). Numbers
marked *initial* come from the two tumble sessions and the `hand-hold` session of
2026-10-30 (`docs/todo.md`); that one run passed them, which is not yet a reason to change
them.

File format version: **1**.

## Decisions

1. **Gyro bias is in the file**, taken from the table rest step. It is only the core's
   initial value (`gyro_bias_q8`); the core keeps updating it in quiet holds. Without it the
   firmware starts from zero bias.
2. **Free holds with a coverage rule** instead of target poses: at least 12 holds, each
   axis seen near both +1 g and -1 g (see Gates).
3. **Still thresholds are not in the file.** They are tuning, not a property of the
   board. Both sides take them from the table below; see "From the file to
   `gc_config`" for how the two sides are kept equal.
4. **One board, one committed file**: `data/calibration/board.cal`.

## Scope

- Calibrated: accelerometer offset and gain per axis, initial gyro bias. These are the
  calibration fields of `gc_config` (`docs/core.md`), and nothing else.
- Not calibrated: gyro scale, cross-axis terms, temperature dependence. `gc_config` has no
  field for them. The tool saves the raw session, so a later gyro scale fit (from the
  rotations between holds) needs no new capture.
- No change to `docs/protocol.md` or `core.h`. The `stream` firmware keeps sending raw
  samples; "the file is used for `stream`" means the desktop applies it.
- A ruler slide is not part of calibration: gravity is the scale reference, and a ruler
  result fills no field. It stays a separate check of the whole chain (`strokes` on a
  recorded slide).

## Calibration file

`data/calibration/board.cal`, 44 bytes, little-endian. Values are the final integers of
`gc_config`: no reader rounds or converts.

| Offset | Type     | Field            | Notes |
|--------|----------|------------------|-------|
| 0      | uint8[4] | magic            | ASCII `GCAL` |
| 4      | uint8    | version          | 1 |
| 5      | uint8    | ctrl1_xl         | CTRL1_XL from the `Hello` of the calibration run |
| 6      | uint8    | ctrl2_g          | CTRL2_G from the same `Hello` |
| 7      | uint8    | reserved0        | 0 |
| 8      | int16[3] | accel_offset_lsb | as `gc_config` |
| 14     | uint16   | reserved1        | 0 |
| 16     | int32[3] | accel_gain_q30   | as `gc_config`, 966367642..1181116006 |
| 28     | int32[3] | gyro_bias_q8     | as `gc_config` |
| 40     | uint16   | reserved2        | 0 |
| 42     | uint16   | crc16            | CRC-16/IBM-3740 (`docs/protocol.md`) over bytes 0..41 |

Bytes 8..39 have the layout of `gc_config` bytes 8..39 and are copied as they are.

A reader rejects the file when the size is not 44, the magic or version is wrong, a
reserved field is not 0, the CRC does not match, a gain is outside its range, or a scale
code is not in the tables of `docs/core.md`.

**Scale match.** The values are in LSB of the scales the run used. A file applies to a
stream only when the full-scale fields are equal: `ctrl1_xl & 0x0C` (FS_XL) and
`ctrl2_g & 0x0E` (FS_G, FS_125). The ODR bits may differ. On a mismatch the reader fails;
it does not rescale.

The file holds no date and no quality numbers. They are in the session saved by the same
run (below) and in git history.

## From the file to `gc_config`

`gc_config` = calibration file + still thresholds + fixed limits.

- **Calibration fields** (`ctrl1_xl`, `ctrl2_g`, offsets, gains, bias): from the file.
  The ctrl bytes come from the stream's `Hello` on the desktop, and from the values the
  driver writes on the firmware; both are checked against the file with the scale match.
- **Still thresholds and limits**: the table below. The desktop computes them from the
  physical values and the stream's scale (`session::config`, `session::calib`). The
  firmware carries the LSB column as constants for the one scale pair it configures.

| Field | Value | LSB at ±4 g, 500 dps |
|-------|-------|----------------------|
| still_gyro_std_lsb  | hand 3 dps (table 0.5)  | 171 (29) |
| still_accel_std_lsb | hand 15 mg (table 3)    | 123 (25) |
| still_gyro_rate_lsb | hand 4 dps (table 1)    | 229 (57) |
| hold_n              | 20 | |
| bias_gyro_std_lsb   | 0.2 dps | 11 |
| bias_accel_std_lsb  | 2 mg    | 16 |
| clip_lsb            | 32700 | |
| max_dt_us           | 14423 | |
| min_stroke_us, max_stroke_us | 150000, 4000000 | |
| min_extent_mm, max_v_end_mm_s | 20, 300 | |

The firmware uses the hand preset.

Both sides must produce the same 68 bytes, or the two paths stop matching bit for bit.
Kept equal by:
- a host test that builds the config from the linked `board.cal` with the hand preset and
  compares the threshold and limit fields with this table;
- the firmware constants (`live_config` in `recognize`) being the same table, with a
  `static_assert` that the driver's scales are the pair the LSB column is for.

The conformance vectors do not change. Each `.gcv` stores its own `gc_config`, made from
the 2026-10-30 tumble and rest sessions, and `gesture-cli vectors` keeps using those
sessions. `cargo test` still regenerates the vectors byte for byte.

## The tool

```
gesture-cli calibrate [--port /dev/ttyACM0] [--out ../data/calibration/board.cal]
gesture-cli calibrate --replay <session.bin> [--out <file>]
```

Runs against the `stream` firmware. `--replay` runs the same steps on a recorded session
(one saved by this tool) without prompts and without retries: a step that does not
complete before the file ends fails the run. It writes a file only when `--out` is given,
and replaces it without asking.

The steps are a state machine in `session::calibrate`, fed with decoded frames; the live
run and the replay share it. The tests run the rest step on `rest-table-4g` and the holds
on `tumble-4g` separately: neither session is a whole run, and `tumble-4g` must fail the
coverage gate.

Common to all steps:
- Time is MCU time from the sample timestamps. Step timing, hold length and progress bars
  count samples' time, not the PC clock. The exception is the three timeouts: 5 s without a
  `Hello`, 30 s without a rest window, and 2 s without a sample after connect. They must
  fire when nothing arrives, so they are PC time.
- When samples stop for 2 s after connect (the IMU stalled: the firmware does not start it
  again), the run ends with the MCU counters in the message. The session is saved.
- Every step shows its name, elapsed time and a progress bar with the time left, redrawn
  in place. Plain terminal output, no TUI crate.
- A step that fails says why and what to change, then offers: retry the step, or quit.
  Quitting writes no calibration file. Connect and rest can fail; the holds step has no
  failure of its own: single holds are dropped (`u`, `d N`) or all of them (`r`).
- A lost frame, CRC error, time gap (`max_dt_us`) or clipped reading inside a
  measurement window discards that window. The reason is shown.
- All received bytes are recorded, as `record` does.

### Step 1: connect

Waits for a `Hello` (sent once per second) and then for samples. Passes when `fw` =
`stream`, `imu` = ok, and both scales are in the tables of `docs/core.md`. Shows the
scales and the sample rate measured over 1 s. Fails after 5 s without a `Hello`, or with a
`Hello` but no samples. A changed `Hello` or an MCU reset later in the run ends the run.

### Step 2: rest on the table

"Put the board on the table and do not touch it."

- Waits for stillness by the table thresholds (0.5 dps gyro std, 3 mg accel std), then
  measures for 10 s.
- Result: gyro bias = mean gyro over the 10 s. Also shown: gyro and accel std per axis,
  against the 0.05 dps and 0.5 to 0.8 mg measured on 2026-10-30.
- Fails and restarts the 10 s when the board stops being still. Fails the step after 30 s
  without a complete window.

This is the only source of gyro bias. A hand hold is not used for it: a still hand turns
1 to 3 dps.

### Step 3: holds

"Turn the board to a new orientation and keep it still. Lean it against something if you
like. Exact angles do not matter."

The method is the gravity-norm (multi-position) fit: a still accelerometer reads 1 g in
any orientation, so the orientations need to be different, not known.

- The board may stay on the table after step 2: that position is the first hold.
- A hold is detected automatically with the hand thresholds (3 dps, 15 mg, 4 dps): 0.5 s
  of settling, then 2.0 s measured (*initial*). The bar shows the 2.0 s filling; movement
  resets it.
- A completed hold is accepted when its gravity direction is at least 20° (*initial*) from
  every accepted hold. Otherwise: "too close to hold N, turn it further".
- For each accepted hold the tool prints its number, the mean accel in g, and the gyro and
  accel std, so the thresholds can be judged from real runs.
- The coverage display has six entries, +x -x +y -y +z -z, each with the largest reading
  seen so far in that direction and a mark once it passes the limit. The hint line names
  what is missing: "x has not pointed down yet".
- From 9 holds on, the fit runs after every hold and its residual is shown.
- Commands (a letter and Enter): `u` drops the last hold, `d N` drops hold N, `r` drops
  all holds, `f` finishes, `q` quits.
- After a completed hold (accepted or not) the board must move before the next hold
  starts.

The step finishes by itself when all gates below pass. `f` before that reports which gate
fails and continues the step.

### Step 4: fit and gates

The fit is the existing one (`session::calib`): axis-aligned ellipsoid, offset and gain per
axis, least squares on the hold means.

| Gate | Value | Why |
|------|-------|-----|
| Holds | >= 12 | Six unknowns; 12 leaves a meaningful residual. The literature finds 12 enough. |
| Coverage | each axis reads >= +0.85 g in one hold and <= -0.85 g in another | Offset and gain separate only with both signs. `tumble-4g` saw x only to ±0.73 g and its x offset moved 7 mg. |
| Residual rms | <= 3 mg | The hand-held tumbles reached 1.1 and 1.4 mg. |
| Worst hold | residual <= 6 mg | One disturbed hold; the tool names it and offers to drop it. |
| Gains | 0.9..1.1 | The range `gc_init` accepts. |

All values are *initial*. 0.85 g means within about 30° of the axis: each of the six
faces roughly up once, propped if the board does not stand on that side. The other holds
can be anything in between.

The report shows offsets in mg and LSB, gains, the residual before and after the fit, and
the difference from the current `board.cal` if one exists. A large difference is shown,
not gated: the tool cannot tell which run is right.

### Step 5: write

Asks before replacing an existing file. Writes:

- `--out` (default `data/calibration/board.cal`): the 44 bytes above, then reads the file
  back and checks it.
- `data/sessions/<stamp>_calibrate.bin` and `.toml`: the raw stream of the whole run, as
  `record` writes. The note holds the accepted holds, the gate results and the fitted
  values. A run that was quit still saves the session.

The firmware and the tools pick up the new file at the next build. The tool says so.

## Use on the desktop

- `path`, `events`, `strokes`, the viewer: `--calib <file>` takes either a `.cal` file or a
  tumble session, told apart by the magic. A `.cal` file also gives the initial gyro bias;
  `--bias <rest session>` overrides it. A `.cal` file for other scales than the stream's
  is an error.
- The `session` crate links `board.cal` with `include_bytes!` and checks it in a test.
- The linked file is the default calibration when its scales match the stream's. This
  changes the default: today a tool without `--calib` uses no accelerometer calibration.
- When the scales do not match (every session of 2026-10-30 before `rest-table-4g` is
  +-2 g / 250 dps), the default stays as today, raw, and the config line says so.
- `calib` (the report on a tumble session) and `vectors` are unchanged.

## Use in the firmware

- `recognize` embeds `board.cal` with `#embed`, like the board vector, named in
  `firmware/apps/CMakeLists.txt` so a new file triggers a rebuild.
- Checked at build time with `static_assert`: size, magic, version, CRC (`crc16` in
  `proto/` is `constexpr`), gain range, and the scale match against the driver's ctrl
  values. A bad file does not build.
- Cost once used: the 68 bytes of `gc_config` in flash, no RAM beyond the state.
- The checked file and the table above make `live_config`, a `constexpr gc_config`. Nothing
  uses it yet, so it costs no flash today: the boot self-test keeps the config stored in
  the board vector, and `recognize` does not feed the live IMU into the core yet.
- `stream` does not embed the file: it sends raw samples.

## Order of work

1. This document agreed. Done.
2. File reader and writer in `session`, with tests (round trip, each rejection). Done.
3. `calibrate` with `--replay`. Done; tested on a synthetic whole run, on `rest-table-4g`
   (rest step) and on `tumble-4g` (holds and fit).
4. A real run produces the first `board.cal`. Done, 2026-11-14.
5. Linking: desktop default, then `recognize`. Done.

Open: `recognize` feeding the live IMU into the core with `live_config`.
