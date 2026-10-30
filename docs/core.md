# Core contract (`gesture/core.h`)

The contract between the recognition core and its callers: the `recognize` firmware (MCU)
and `host/crates/gesture-core` (Rust FFI, used by `gesture-cli` and the viewer).
Change this document first, then the implementation and the FFI.

`core.h` is a contract between languages, like the wire format. The C++ library in
`core/` is its first implementation; another one (another language) may replace it at
link time without changes to the firmware or the host. An implementation conforms when it
gives the same events and the same stroke points, bit for bit, on the conformance vectors
(Step 3c), run through `core.h` only.

ABI version: **1**. Scope of v1: still detection, tilt from the gyro,
gravity removal, double integration and the zero-velocity correction, giving the 3D path of
one stroke. Recognition comes later, in a new ABI version.

Status: frozen (2026-10-30). The streaming float reference (`gesture-cli path`) follows the
algorithm section and reproduces the measurements in `docs/todo.md` (ruler-15 142-155 mm,
air-lines tilt error 0.10-1.20 deg). Any change to a layout, a constant or a formula is a
new ABI version; the config values (calibration, thresholds) are not part of the spec.

## Conventions
- Only C types cross the ABI: fixed-width integers and structs of them with the layout
  given here (no padding the layout does not show), pointers with a length. No callbacks,
  no allocation, no global state, no I/O, no clock. Enumerations are integer `#define`s,
  never C `enum` (its size is up to the compiler).
- The caller owns all memory: the state buffer, the config, the output arrays.
- All functions are deterministic: the same calls with the same inputs give the same
  outputs on any conforming implementation and target.
- Preconditions (violating them is a caller bug, not an error code): pointers are non-null
  and point to objects of the given type; `gc_state` is aligned to `GC_STATE_ALIGN`;
  `gc_feed` and `gc_stroke` only after a successful `gc_init` on the same state.
- Byte order is the platform's; the structs are not a wire format (that is
  `docs/protocol.md`).

## API

```c
#define GC_ABI_VERSION   1u
#define GC_STATE_SIZE    2048u   /* bytes; a change is an ABI version change */
#define GC_STATE_ALIGN   8u

typedef struct { _Alignas(8) uint8_t opaque[GC_STATE_SIZE]; } gc_state;

gc_status gc_init(gc_state *s, const gc_config *cfg);
gc_event  gc_feed(gc_state *s, const gc_sample *in);
uint16_t  gc_stroke(const gc_state *s, gc_stroke_info *info, gc_point *out, uint16_t cap);
uint16_t  gc_live(const gc_state *s, gc_point *out);
```

- `gc_init` validates `cfg`, copies it into the state and resets everything. On an error
  the state is left unusable (call `gc_init` again).
- `gc_feed` takes one sample, in the order the sensor produced them, and returns at most
  one event. It never fails: bad input (a time gap, a clipped reading) is reported through
  the event.
- `gc_stroke` reads the last stroke after a `GC_EV_STROKE` event, until the next
  `GC_EV_MOVING`. It fills `info`, copies `min(points, cap)` points to `out` and returns
  that number; it returns 0 (and fills `info` with zeros) when no stroke is readable.
- `gc_live` reads where the stroke is now, for drawing it while it is drawn: in state
  `MOVING` it writes the position after the latest integrated sample to `out` and returns
  1; otherwise it writes zeros and returns 0. The position is uncorrected (the
  zero-velocity correction needs the end of the stroke), so it drifts, and the points
  from `gc_stroke` differ from it by up to v_end x T / 2; a viewer replaces the live
  path with them at `GC_EV_STROKE`. It changes nothing in the state.

### `gc_sample` (16 B)

| Offset | Type      | Field   | Notes |
|--------|-----------|---------|-------|
| 0      | uint32    | t_us    | MCU time of the sample, 1 us per tick, wraps (as `Sample.t_us`, `docs/protocol.md`) |
| 4      | int16[3]  | gyro    | raw LSB, x y z as the LSM6DS3 reports them |
| 10     | int16[3]  | accel   | raw LSB, x y z |

The same fields as the wire `Sample`, so the host feeds recordings unchanged.

### `gc_config` (68 B)

Everything that depends on the board or is tuned on recordings. The spec fixes how each
value is used, not its value; `docs/todo.md` has the measurements behind the defaults.

| Offset | Type     | Field                | Default (hand, +-4 g / 500 dps) | Notes |
|--------|----------|----------------------|------|-------|
| 0      | uint32   | abi_version          | 1    | must be `GC_ABI_VERSION` |
| 4      | uint8    | ctrl1_xl             | 0x48 | the sensor's CTRL1_XL as written (Hello); selects the accel scale |
| 5      | uint8    | ctrl2_g              | 0x44 | the sensor's CTRL2_G; selects the gyro scale |
| 6      | uint16   | reserved0            | 0    | must be 0 |
| 8      | int16[3] | accel_offset_lsb     | from `calib` | per axis, LSB at this scale |
| 14     | uint16   | reserved1            | 0    | must be 0 |
| 16     | int32[3] | accel_gain_q30       | from `calib` | per axis, 1.0 = 2^30; valid 0.9..1.1 |
| 28     | int32[3] | gyro_bias_q8         | from a rest session | per axis, LSB x 256; the initial bias |
| 40     | uint16   | still_gyro_std_lsb   | 171 (3 dps)  | still window: gyro std per axis below this |
| 42     | uint16   | still_accel_std_lsb  | 123 (15 mg)  | still window: accel std per axis below this |
| 44     | uint16   | still_gyro_rate_lsb  | 229 (4 dps)  | still window: gyro mean minus bias, per axis, below this |
| 46     | uint16   | hold_n               | 20           | quiet windows in a row that confirm a hold; >= `GC_GUARD_N` + 1 |
| 48     | uint16   | bias_gyro_std_lsb    | 11 (0.2 dps) | bias update: gyro std below this (table-quiet) |
| 50     | uint16   | bias_accel_std_lsb   | 16 (2 mg)    | bias update: accel std below this |
| 52     | uint16   | clip_lsb             | 32700        | a reading with abs >= this is clipped |
| 54     | uint16   | max_dt_us            | 14423 (1.5 periods) | larger step (or 0) is a time gap; <= 20000 |
| 56     | uint32   | min_stroke_us        | 150000       | shorter moves are rejected |
| 60     | uint32   | max_stroke_us        | 4000000      | longer moves are rejected; <= 6553500 |
| 64     | uint16   | min_extent_mm        | 20           | smaller paths are rejected (table bumps, twitches) |
| 66     | uint16   | max_v_end_mm_s       | 300          | larger leftover velocity is rejected |

Table preset (board resting on a table between strokes): still_gyro_std_lsb 29 (0.5 dps),
still_accel_std_lsb 25 (3 mg), still_gyro_rate_lsb 57 (1 dps). The hand thresholds miss
the slow end of a table slide (ruler-30 run).

The calibration and the thresholds are in LSB of the scale given by `ctrl1_xl`/`ctrl2_g`.
Core cannot see the sensor's registers: the caller passes the codes the sensor runs at,
and a calibration made at another scale must be converted or redone (the tumble session
was recorded at +-2 g; redo it at +-4 g).

### `gc_status` (int32)

| Value | Name             | Meaning |
|-------|------------------|---------|
| 0     | `GC_OK`          | |
| 1     | `GC_ERR_VERSION` | `abi_version` is not `GC_ABI_VERSION` |
| 2     | `GC_ERR_SCALE`   | `ctrl1_xl` or `ctrl2_g` has an FS code not in the scale tables, or is 0 |
| 3     | `GC_ERR_CONFIG`  | a value outside its valid range (see the table), or a reserved field not 0 |

Only `gc_init` returns a status.

### `gc_event` (12 B)

Events report changes; most samples give `GC_EV_NONE`. `state` is the state after this
sample, in every event, so nothing is lost when one sample changes two things.

| Offset | Type   | Field       | Notes |
|--------|--------|-------------|-------|
| 0      | uint8  | kind        | `GC_EV_*` |
| 1      | uint8  | state       | `GC_ST_*` after this sample |
| 2      | uint8  | reason      | `GC_EV_REJECTED`: `GC_REJ_*`; else 0 |
| 3      | uint8  | reserved    | 0 |
| 4      | uint16 | points      | `GC_EV_STROKE`: points readable with `gc_stroke`; else 0 |
| 6      | uint16 | v_end_mm_s  | `GC_EV_STROKE`/`REJECTED`: leftover velocity, largest axis, saturated |
| 8      | uint32 | duration_us | `GC_EV_STROKE`/`REJECTED`: move length; else 0 |

| kind | Name              | When |
|------|-------------------|------|
| 0    | `GC_EV_NONE`      | nothing changed |
| 1    | `GC_EV_READY`     | a hold confirmed while warming up: gravity and bias known, a move now starts a stroke |
| 2    | `GC_EV_MOVING`    | a move started after a hold |
| 3    | `GC_EV_STROKE`    | the hold after a move confirmed; the stroke is readable |
| 4    | `GC_EV_REJECTED`  | the move gave no stroke; `reason` says why |

| state | Name             | Meaning |
|-------|------------------|---------|
| 0     | `GC_ST_WARMUP`   | no confirmed hold yet (after init or a time gap, or a rejected long move) |
| 1     | `GC_ST_STILL`    | in a hold; a move starts a stroke |
| 2     | `GC_ST_MOVING`   | a stroke is being integrated |

| reason | Name               | Reported | Cause |
|--------|--------------------|----------|-------|
| 1      | `GC_REJ_SHORT`     | at the end hold | duration < `min_stroke_us` or extent < `min_extent_mm` |
| 2      | `GC_REJ_LONG`      | when it happens | the move passed `max_stroke_us`; state becomes `WARMUP` |
| 3      | `GC_REJ_CLIPPED`   | at the end hold | a reading at the range end during the stroke |
| 4      | `GC_REJ_TIME_GAP`  | when it happens | `t_us` step 0 or > `max_dt_us` during the move; state becomes `WARMUP` |
| 5      | `GC_REJ_DRIFT`     | at the end hold | `v_end_mm_s` > `max_v_end_mm_s` |

At the end hold the checks run in the order CLIPPED, SHORT (duration), DRIFT, SHORT
(extent); the first that fails is the reason.

Typical sequence: `READY` -> `MOVING` -> `STROKE` -> `MOVING` -> `REJECTED(SHORT)` -> ...

### `gc_point` (8 B) and `gc_stroke_info` (24 B)

| Offset | Type   | Field    | Notes |
|--------|--------|----------|-------|
| 0      | int16  | x_mm     | position in the start frame, after the zero-velocity correction |
| 2      | int16  | y_mm     | |
| 4      | int16  | z_mm     | |
| 6      | uint16 | t_100us  | time since the stroke start, 100 us units |

The start frame is the board's own axes at the stroke start (the first point is 0, 0, 0 at
t 0). `gc_stroke_info.gravity_ug` gives "up" in that frame, so the caller can level the
path; leveling and a heading convention move into core with recognition.

| Offset | Type     | Field        | Notes |
|--------|----------|--------------|-------|
| 0      | uint32   | duration_us  | as in the event |
| 4      | uint16   | points       | as in the event |
| 6      | uint16   | v_end_mm_s   | as in the event |
| 8      | int32[3] | gravity_ug   | the hold's accel before the stroke, calibrated, start frame (points up) |
| 20     | uint16   | extent_mm    | largest abs coordinate over the points |
| 22     | uint16   | step         | samples per point at the end (1, 2, 4, ...) |

## Algorithm

### Arithmetic
- All values are integers. Intermediate products use 64-bit signed integers; the ranges
  below keep every product inside them.
- `rdiv(a, b)`, b > 0: a / b rounded to nearest, halves away from zero:
  `sign(a) * floor((|a| + floor(b / 2)) / b)`. Every rescaling in this section is an
  `rdiv`; an implementation may use shifts for power-of-two divisors only if the result is
  the same for every input.
- `sat16(x)`: x clamped to [-32768, 32767]; `satu16(x)`: x clamped to [0, 65535].
- Time steps: `dt = (t_us - t_prev) mod 2^32`, as unsigned 32-bit subtraction.
- pi enters only as 355/113 (relative error 8.5e-8).

### Scales

Accel sensitivity `sa`, ug/LSB, from FS_XL = (ctrl1_xl >> 2) & 3:

| FS_XL | 00 (2 g) | 01 (16 g) | 10 (4 g) | 11 (8 g) |
|-------|----------|-----------|----------|----------|
| sa    | 61       | 488       | 122      | 244      |

Gyro sensitivity `sg`, udps/LSB, from FS_G = (ctrl2_g >> 2) & 3 and FS_125 = (ctrl2_g >> 1) & 1:

| FS_125 = 1 | FS_G 00 (250) | 01 (500) | 10 (1000) | 11 (2000) |
|------------|---------------|----------|-----------|-----------|
| 4375       | 8750          | 17500    | 35000     | 70000     |

(LSM6DS3 and LSM6DS3TR-C datasheets, mechanical characteristics; the TR-C's 245 dps range has the same code
and sensitivity.)

### Fixed constants
- `GC_WIN_N` = 32: samples in the still window (0.31 s at 104 Hz).
- `GC_GUARD_N` = 10: samples between the end of the gravity window and the stroke start
  (0.1 s; a soft start is seen late).
- `GC_GRAV_N` = 32: samples in the gravity window.
- `GC_RING_N` = 48 (>= GC_GRAV_N + GC_GUARD_N + 1): raw samples kept, with their dt.
- `GC_POINTS_N` = 64: stroke buffer.

### Per sample

1. **Time.** On the first sample after init `dt` is undefined: treat it as a gap. A gap
   (dt = 0 or dt > max_dt_us): clear the ring, quiet streak 0; in `MOVING` report
   `REJECTED(TIME_GAP)`; state `WARMUP`. The sample itself then enters the empty ring.
2. **Ring.** Push (dt, gyro, accel) into the ring.
3. **Still window** (only when the ring holds >= `GC_WIN_N` samples): over the newest
   `GC_WIN_N` samples, per axis `S = sum x`, `Q = sum x^2` (raw LSB). The spread test
   `N * Q - S^2 < N^2 * th^2` for each gyro axis (th = still_gyro_std_lsb) and accel axis
   (still_accel_std_lsb); the rate test `|256 * S - N * bias_q8| < 256 * N * th` for each
   gyro axis (still_gyro_rate_lsb). Quiet when all pass; quiet streak + 1, else 0.
4. **Bias update** (state `STILL`, window quiet, and the spread test passing with the bias
   thresholds): `mean_q8 = rdiv(256 * S, N)`, `bias_q8 += rdiv(mean_q8 - bias_q8, 64)`
   per gyro axis.
5. **State.**
   - `WARMUP`: streak >= hold_n -> `STILL`, event `READY`.
   - `STILL`: window not quiet -> start the stroke (below), `MOVING`, event `MOVING`.
   - `MOVING`: integrate this sample (below). If the move is now longer than
     max_stroke_us: `REJECTED(LONG)`, `WARMUP`. If the streak became 1: snapshot (v, p,
     t, clipped flag). If it became 0: drop the snapshot. If it reached hold_n: end the
     stroke at the snapshot (below), `STILL`, event `STROKE` or `REJECTED`.

### Stroke start
At sample k (first non-quiet window), with ring indices relative to k:
- start sample `s0 = k - GC_GUARD_N - 1`; gravity window `k - GC_GUARD_N - GC_GRAV_N ..
  k - GC_GUARD_N - 1` (ends at s0). The hold that came before (streak >= hold_n >=
  GC_GUARD_N + 1) makes these samples quiet.
- gravity, ug, per axis: `g = rdiv((S_a - GC_GRAV_N * off) * gain_q30 * sa, GC_GRAV_N * 2^30)`
  with `S_a` the raw accel sum over the gravity window.
- the stroke's gyro bias is `bias_q8` as of now (frozen for the stroke).
- q = (2^30, 0, 0, 0); v = p = 0; points: one point (0, 0, 0, t 0); step 1.
- integrate the ring samples `k - GC_GUARD_N .. k` in order (the replay), as below.

### Integration (one sample, dt from the ring or the input)
- rate, LSB x 256: `w = 256 * gyro - bias_q8`.
- half angle, Q30 rad: `ang_udeg = rdiv(w * sg * dt, 256 * 10^6)`,
  `h = rdiv(ang_udeg * 355 * 2^23, 113 * 2812500)` (2^29 / (180 * 10^6) = 2^23 / 2812500).
- dq = (`2^30 - rdiv(hx^2 + hy^2 + hz^2, 2^31)`, hx, hy, hz); q = q * dq (Hamilton
  product, each component `rdiv(sum of four products, 2^30)`).
- renormalize: `n = rdiv(qw^2 + qx^2 + qy^2 + qz^2, 2^30)`, each `q_i = rdiv(q_i * (3 * 2^30 - n), 2^31)`.
- accel, ug: `a_i = rdiv((accel_i - off_i) * gain_q30_i * sa, 2^30)`.
- rotate into the start frame with R(q) (Q30; diagonal `2^30 - rdiv(y^2 + z^2, 2^29)` etc.,
  off-diagonal `rdiv(xy - wz, 2^29)` etc.): `f_i = rdiv(sum_j R_ij * a_j, 2^30)`.
- velocity, nm/s: `v_i += rdiv((f_i - g_i) * dt * 980665, 10^8)` (9.80665 m/s^2 per g).
- position, nm: `p_i += rdiv(v_i * dt, 10^6)`, with the new v.
- clipped flag: set if any raw reading of this sample has abs >= clip_lsb.
- sample counter c + 1; if c is a multiple of step, store a point: `x_mm = sat16(rdiv(p, 10^6))`,
  `t_100us = satu16(rdiv(t - t_s0, 100))`. A full buffer first keeps points 0, 2, 4, ...
  (32 left) and doubles step; then the multiple-of-step test is repeated.
- `gc_live` gives the same rounding of the current p and t as a stored point, whether or
  not this sample was stored.

### Stroke end (at the snapshot)
- T = t_snap - t_s0; drop points with t_100us > rdiv(T, 100); append the snapshot point
  (compacting first if the buffer is full).
- zero-velocity correction per point, with t = 100 * t_100us:
  `c = rdiv(rdiv(v_end * t, 10^6) * t, 2 * T)` nm, `x_mm = sat16(x_mm - rdiv(c, 10^6))`.
- `v_end_mm_s = satu16(rdiv(max_i |v_end_i|, 10^6))`; extent = largest abs coordinate.
- checks in the order given under `gc_event`.

### Ranges
Inputs within int16, dt <= 20000 us (config check), gyro <= 2000 dps: `w * sg * dt` <
2^57, `ang_udeg * 355 * 2^23` < 2^58, accel products < 2^57, gravity < 2^62, velocity
products < 2^60 for |f - g| < 32 g, position products < 2^48 for |v| < 10 m/s.

### Budget
- State: ring 48 x 14 B, points 64 x 8 B, the rest about 300 B: ~1.5 KB of the 2048 B in
  `GC_STATE_SIZE`.
- Time: 16 MHz Cortex-M0+ without hardware divide; 64-bit multiplies and `rdiv` are
  library calls. The stroke start replays GC_GUARD_N + 1 samples in one `gc_feed`, and the
  whole call must fit in one sample period (9.6 ms) next to the firmware's own work.
  To measure on the board in Step 3c; if it does not fit: shifts for power-of-two
  divisors, spreading the replay, or the PLL (64 MHz).

## Later (a new ABI version)
- Adaptive still thresholds (measure the noise in the hold) instead of the two presets.
- Leveled output frame and heading, with recognition.
