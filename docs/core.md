# Core contract (`gesture/core.h`)

The contract between the core and its callers (the `recognize` firmware,
`host/crates/gesture-core`). Change this document first, then the implementation and the
FFI. An implementation conforms when it gives the same events and points, bit for bit, on
the conformance vectors, run through `core.h` only.

ABI version: **1**, frozen 2026-10-30: the 3D path of one stroke (still detection, gyro
tilt, gravity removal, double integration, zero-velocity correction). Any change to a
layout, a constant or a formula is a new version; config values are not part of the spec.

## Conventions
- Only C types cross the ABI: fixed-width integers, structs with exactly the layout given
  here, pointers. No callbacks, allocation, global state, I/O or clock. Enumerations are
  integer `#define`s, never C `enum`.
- The caller owns all memory. Byte order is the platform's.
- Deterministic: the same calls give the same outputs on any implementation and target.
- Preconditions (a caller bug, not an error code): valid non-null pointers, `gc_state`
  aligned to `GC_STATE_ALIGN`, `gc_init` succeeded before the other calls.

## API

```c
#define GC_ABI_VERSION   1u
#define GC_STATE_SIZE    2048u
#define GC_STATE_ALIGN   8u

typedef struct { _Alignas(8) uint8_t opaque[GC_STATE_SIZE]; } gc_state;

gc_status gc_init(gc_state *s, const gc_config *cfg);
gc_event  gc_feed(gc_state *s, const gc_sample *in);
uint16_t  gc_stroke(const gc_state *s, gc_stroke_info *info, gc_point *out, uint16_t cap);
uint16_t  gc_live(const gc_state *s, gc_point *out);
```

- `gc_init`: validates `cfg`, copies it, resets everything. After an error the state is
  unusable until the next successful `gc_init`.
- `gc_feed`: one sample, in sensor order; returns one event; never fails.
- `gc_stroke`: after `GC_EV_STROKE`, until the next `GC_EV_MOVING`: fills `info`, copies
  `min(points, cap)` points, returns that number. Otherwise returns 0 and zeroes `info`.
- `gc_live`: in `MOVING`, writes the current position (uncorrected) and returns 1;
  otherwise writes zeros and returns 0. Read-only.

### `gc_sample` (16 B)

The fields of the wire `Sample` (`docs/protocol.md`).

| Offset | Type      | Field   | Notes |
|--------|-----------|---------|-------|
| 0      | uint32    | t_us    | MCU time, 1 us per tick, wraps |
| 4      | int16[3]  | gyro    | raw LSB, x y z |
| 10     | int16[3]  | accel   | raw LSB, x y z |

### `gc_config` (68 B)

Calibration and thresholds are in LSB of the scale selected by `ctrl1_xl`/`ctrl2_g`.

| Offset | Type     | Field                | Meaning | Valid |
|--------|----------|----------------------|---------|-------|
| 0      | uint32   | abi_version          | | `GC_ABI_VERSION` |
| 4      | uint8    | ctrl1_xl             | the sensor's CTRL1_XL (accel scale) | FS code in the table |
| 5      | uint8    | ctrl2_g              | the sensor's CTRL2_G (gyro scale) | FS code in the table |
| 6      | uint16   | reserved0            | | 0 |
| 8      | int16[3] | accel_offset_lsb     | per axis | |
| 14     | uint16   | reserved1            | | 0 |
| 16     | int32[3] | accel_gain_q30       | per axis, 1.0 = 2^30 | 0.9..1.1 |
| 28     | int32[3] | gyro_bias_q8         | initial bias, LSB x 256 | |
| 40     | uint16   | still_gyro_std_lsb   | still: gyro std per axis below | >= 1 |
| 42     | uint16   | still_accel_std_lsb  | still: accel std per axis below | >= 1 |
| 44     | uint16   | still_gyro_rate_lsb  | still: gyro mean minus bias below | >= 1 |
| 46     | uint16   | hold_n               | quiet windows in a row that confirm a hold | >= `GC_GUARD_N` + 1 |
| 48     | uint16   | bias_gyro_std_lsb    | bias update: gyro std below | >= 1 |
| 50     | uint16   | bias_accel_std_lsb   | bias update: accel std below | >= 1 |
| 52     | uint16   | clip_lsb             | abs reading >= this is clipped | >= 1 |
| 54     | uint16   | max_dt_us            | larger step, or 0, is a time gap | 1..20000 |
| 56     | uint32   | min_stroke_us        | shorter moves are rejected | < max_stroke_us |
| 60     | uint32   | max_stroke_us        | longer moves are rejected | <= 6553500 |
| 64     | uint16   | min_extent_mm        | smaller paths are rejected | |
| 66     | uint16   | max_v_end_mm_s       | larger leftover velocity is rejected | |

### `gc_status` (int32, `gc_init` only)

| Value | Name             | Meaning |
|-------|------------------|---------|
| 0     | `GC_OK`          | |
| 1     | `GC_ERR_VERSION` | `abi_version` wrong |
| 2     | `GC_ERR_SCALE`   | an FS code not in the scale tables, or a ctrl value 0 |
| 3     | `GC_ERR_CONFIG`  | another value outside "Valid" |

### `gc_event` (12 B)

Events report changes; `state` is the state after the sample.

| Offset | Type   | Field       | Notes |
|--------|--------|-------------|-------|
| 0      | uint8  | kind        | `GC_EV_*` |
| 1      | uint8  | state       | `GC_ST_*` |
| 2      | uint8  | reason      | `GC_REJ_*` for `REJECTED`, else 0 |
| 3      | uint8  | reserved    | 0 |
| 4      | uint16 | points      | `STROKE`: readable points, else 0 |
| 6      | uint16 | v_end_mm_s  | `STROKE`, `REJECTED` at the end hold: leftover velocity, else 0 |
| 8      | uint32 | duration_us | `STROKE`, `REJECTED` at the end hold: move length, else 0 |

| kind | Name              | When |
|------|-------------------|------|
| 0    | `GC_EV_NONE`      | nothing changed |
| 1    | `GC_EV_READY`     | a hold confirmed in `WARMUP` |
| 2    | `GC_EV_MOVING`    | a move started in `STILL` |
| 3    | `GC_EV_STROKE`    | the end hold confirmed, the stroke passed the checks |
| 4    | `GC_EV_REJECTED`  | the move gave no stroke |

| state | Name             | Meaning |
|-------|------------------|---------|
| 0     | `GC_ST_WARMUP`   | no confirmed hold (after init, a time gap, or a long move) |
| 1     | `GC_ST_STILL`    | in a hold |
| 2     | `GC_ST_MOVING`   | in a stroke |

| reason | Name               | Reported | Cause |
|--------|--------------------|----------|-------|
| 1      | `GC_REJ_SHORT`     | end hold | duration < min_stroke_us, or extent < min_extent_mm |
| 2      | `GC_REJ_LONG`      | at once  | the move passed max_stroke_us |
| 3      | `GC_REJ_CLIPPED`   | end hold | a clipped reading up to the snapshot |
| 4      | `GC_REJ_TIME_GAP`  | at once  | a time gap during the move |
| 5      | `GC_REJ_DRIFT`     | end hold | v_end_mm_s > max_v_end_mm_s |

End-hold checks in order: CLIPPED, SHORT (duration), DRIFT, SHORT (extent).

### `gc_point` (8 B) and `gc_stroke_info` (24 B)

Points are in the start frame: the board's axes at the stroke start (first point 0, 0, 0
at t 0); `gravity_ug` gives up in that frame.

| Offset | Type   | Field    | Notes |
|--------|--------|----------|-------|
| 0      | int16  | x_mm     | after the zero-velocity correction (`gc_live`: before) |
| 2      | int16  | y_mm     | |
| 4      | int16  | z_mm     | |
| 6      | uint16 | t_100us  | since the stroke start, 100 us units |

| Offset | Type     | Field        | Notes |
|--------|----------|--------------|-------|
| 0      | uint32   | duration_us  | as in the event |
| 4      | uint16   | points       | as in the event |
| 6      | uint16   | v_end_mm_s   | as in the event |
| 8      | int32[3] | gravity_ug   | gravity window mean, calibrated |
| 20     | uint16   | extent_mm    | largest abs coordinate over the points |
| 22     | uint16   | step         | samples per point at the end |

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

(LSM6DS3 and LSM6DS3TR-C datasheets; the TR-C's 245 dps range has the same code and
sensitivity.)

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
Inputs within int16, dt <= 20000 us, gyro <= 2000 dps: `w * sg * dt` < 2^57,
`ang_udeg * 355 * 2^23` < 2^58, accel products < 2^57, gravity < 2^62, velocity products
< 2^60 for |f - g| < 32 g, position products < 2^48 for |v| < 10 m/s.
