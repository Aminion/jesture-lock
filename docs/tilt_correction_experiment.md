# Tilt term in the zero-velocity correction (experiment, dropped)

An attempt to correct, at the end of a stroke, the part of the drift that comes from the
orientation error of the gyro integration. It was specified as ABI version 2, implemented in
the C++ core and the float reference, and evaluated on the recorded sessions (2026-11-16).

**Decision: dropped.** It helps on table slides with truly still holds, but the product is
gestures in the hand, where the measurement it relies on is dominated by other errors.
`master` stays on ABI version 1 (`docs/core.md`).

The work is kept on the local branch `core-abi-v2-tilt`: `a63bc41` (spec), `8e5f1fd`
(implementation, vectors, tool changes). Everything needed to understand or redo it is in
this document.

## Contents

1. Why it was tried
2. The idea
3. The specification (ABI 2 draft)
4. Implementation and cost
5. Evaluation: three rounds
6. Why it does not suit gestures in the hand
7. What the literature does
8. What to do instead
9. Lessons

## 1. Why it was tried

ABI version 1 ends a stroke with a zero-velocity correction (`correct_drift()` in
`core/cpp/src/stroke.hpp`, "Stroke end" in `docs/core.md`). At the end hold the board is
still, so the velocity left in the integration, `v_end`, is error. The correction assumes
that error came from a **constant acceleration error** over the whole stroke and moves each
stored point by

```
c(t) = v_end · t² / (2T)
```

That is exact for a constant error (a wrong gravity estimate, an accel offset while the board
does not turn). It is only partly right for an error that **grows** during the stroke. The
main one: a residual gyro bias tilts `q` a little more every moment, so a growing part of
gravity leaks into the rotated readings. Its velocity error grows like `t²` and its position
error like `t³`, not `t²`. Version 1 cannot tell the two apart because it sees only one
number per axis at the end (`v_end`).

## 2. The idea

At the end hold the accelerometer reads gravity again, as in the gravity window before the
stroke. Rotated into the start frame with the final `q`, it should equal the start gravity
`g`. The difference is a second, independent measurement:

```
e = calibrated mean accel of the end-hold window (board frame)
u = R(q) · e                     (start frame)
m = u − g
```

If `q` has a tilt error `δ` (a small rotation, start frame), then `u = g + δ × g`, so
`m = δ × g`: exactly the gravity the error of `q` leaks into every rotated reading at that
moment. No rotation vector or cross product is needed; `m` already is the acceleration error,
in µg. Two refinements:

- A tilt can only produce a component **across** gravity (`δ × g ⊥ g`), so the part of `m`
  along `g` is removed: it has another cause (accel gain, magnitude errors).
- A constant gyro bias tilts `q` linearly in time, so the leak grows linearly from 0 at the
  start to `m` at the end-hold sample (time `T_e` after the start, `T_e ≥ T`).

With the leak growing linearly, its effect on a point at time `t` is `L·t³/(6T)`, where `L`
is the leak at the snapshot. It also explains part of `v_end` (`L·T/2`); the rest is the
constant part `vc`. The correction becomes

```
c(t) = vc · t² / (2T)      constant error, as in version 1, but with what the tilt leaves
d(t) = L  · t³ / (6T)      the growing tilt leak
point(t) -= c(t) + d(t)
```

At `t = T` the two together still remove exactly `v_end`, so the corrected path ends at rest.
With `m = 0` the result equals version 1.

## 3. The specification (ABI 2 draft)

Written into `docs/core.md` first, as the repository requires for a `core.h` change
(`a63bc41` on the branch). Any change to a formula is a new ABI version, so this was
version 2: `GC_ABI_VERSION 2u`, layouts and config unchanged, one new fixed constant.

Stroke end, after the existing checks and the cut back to the snapshot, per axis `i`, in
integers:

| Step | Formula |
|---|---|
| end-hold mean, µg, board frame | `e_i = rdiv((S_i − N·off_i) · gain_q30_i · sa, N · 2^30)`, `S_i` the raw accel sum of the still window of the end-hold sample, `N = GC_WIN_N` |
| into the start frame | `u_i = rdiv(Σ_j R_ij · e_j, 2^30)`, `R = R(q)` after that sample |
| mismatch | `m_i = u_i − g_i` |
| across gravity only | `G = rdiv(g·g, 2^20)`; `G = 0` gives `m = 0`; else `k = rdiv(m·g, G)` (Q20), `m_i −= rdiv(k · g_i, 2^20)` |
| gate | any `|m_i| > GC_TILT_MAX_UG = 65536` (≈ 3.8°) gives `m = 0` |
| leak at T, µg | `L_i = rdiv(m_i · T, T_e)` |
| velocity it left, nm/s | `vl_i = rdiv(L_i · T · 980665, 2 · 10^8)` |
| constant part | `vc_i = v_end_i − vl_i` |
| correction per point, nm | `c = rdiv(rdiv(vc · t, 10^6) · t, 2T)`, `d = rdiv(rdiv(rdiv(rdiv(L · t, T) · t, 10^6) · t · 980665, 6 · 10^8)` |
| point, mm | `x_mm = sat16(x_mm − rdiv(c + d, 10^6))` |

Every product was bounded in "Ranges". The tightest is `d` at about 2^62.4, which holds only
because of the gate; that is the second reason the gate exists. The gate value was a guess at
"more than gyro bias can explain in a few seconds".

## 4. Implementation and cost

On the branch (`8e5f1fd`):

- **C++ core**: `calibrated_mean()` shared by the start gravity and the end hold;
  `tilt_leak()` (rotate, subtract, project, gate); `correct_drift()` with both terms;
  `finish()` takes the end-hold window's raw accel sum, passed from step 3 of `gc_feed`
  through `moving()`. New constants: `one_q20`, `tilt_max`, the `us_to_s`, `half` and
  `sixth` scales.
- **Unit tests**: `tilt_leak` (projection, both gate edges, zero gravity), the tilt term of
  `correct_drift`, a full `finish` with a tilted end. Expected values worked out by hand.
- **Float reference** (`host/crates/gesture-cli/src/path.rs`): the same term in `f64`; the
  end-hold mean is no longer rounded to whole LSB before calibration.
- **Conformance vectors**: new `core/contract/vectors/v2/` (22 files); the runner, the cargo
  checks and the firmware board vector switched to it; `v1/` kept as the frozen record.

| | Version 1 | Version 2 |
|---|---|---|
| `recognize` flash | 47,864 B | 49,088 B (+1,224 B) |
| RAM | 4,200 B | 4,200 B |
| CPU | | once per stroke: ≈ 200 extra 64-bit divisions, well under 1 ms at 64 MHz |

All checks passed: host ctest 4/4 (clang-tidy, vectors), clippy, all Rust tests. Event counts
in all 22 vectors were identical to version 1; digests changed only in the vectors that have
accepted strokes (`drift`, `drift-max`, `short`, `turn-in-place` were byte-identical), as the
spec predicts. As a side effect `every_session_matches_the_reference`, which fails on
`master` because two strokes sit at the 20 mm extent edge, passed: those strokes moved off the
edge. That is not evidence for the method.

## 5. Evaluation: three rounds

Ground truth: the ruler sessions are slides of a known length along the board's x axis on a
table (`data/sessions/*ruler*`, `docs/todo.md` "Ruler test"). `gesture-cli strokes` on
`master` (v1) and on the branch (v2), the same config otherwise. "Tilt" is the end mismatch as
an angle and "turned" the rotation the gyro saw, both from `gesture-cli path`.

### Round 1: hand thresholds (misleading)

The first comparison used the default **hand** still thresholds. Stroke lengths, mm:

| Session (true) | v1 | v2 |
|---|---|---|
| ruler-30 (300) | 306, 359, 248 | 265, 355, 225 |
| ruler-30-4g (300) | 253, 219, 198, 292, 236, 302 | 286, 202, 154, 295, 171, 301 |

Mean error got worse (39 → 55 mm and 51 → 66 mm), and several short strokes flipped between
STROKE and SHORT. The measured tilt explained why:

| ruler-30-4g stroke at | 4.6 s | 11.0 s | 17.7 s | 25.4 s | 45.3 s | 52.8 s | 61.4 s |
|---|---|---|---|---|---|---|---|
| duration, s | 3.44 | 2.49 | 1.64 | 2.47 | 2.58 | 2.60 | 1.11 |
| turned | 1.8° | 1.2° | 0.2° | 1.5° | 0.4° | 0.3° | 0.4° |
| tilt | 0.33° | 0.20° | **1.26°** | 0.06° | **0.73°** | 0.02° | **1.86°** |

A gyro-bias tilt of 1.3° in 1.6 s needs a bias error near 1 dps, against about 0.05 dps from
bias tracking, while the gyro saw almost no rotation. The two strokes that got much worse
(198 → 154, 236 → 171 mm) are the ones with the largest "tilt". The mismatch lay along x, the
slide direction: motion inside the windows that were supposed to be still. The hand thresholds
(3 dps, 15 mg) let the start of a slow push into the gravity window and the tail of the
slide into the end window.

`docs/todo.md` already says table slides must be evaluated with `--hold table`: with the hand
thresholds they break into fragments. This round ignored that.

### Round 2: table thresholds (it helps)

`--hold table` (0.5 dps, 3 mg), as documented. Stroke lengths, mm; for ruler-30-4g the first
stroke of the session (the board being placed) is left out, as in `docs/todo.md`:

| Session (true) | strokes | v1 mean error | v2 mean error | worst v1 → v2 | mean length v1 → v2 |
|---|---|---|---|---|---|
| ruler-30-4g (300) | 10 | 8.8 | **4.3** | 28.7 → 10.7 | 293.7 → 297.8 |
| ruler-15-4g (150) | 13 | 2.1 | 2.6 | 8.0 → 5.0 | 149.7 → 150.3 |
| ruler-30 (300) | 3 | 31.2 | 30.9 | 60.8 → 60.4 | |
| ruler-15 (150) | 5 | 4.7 | 4.3 | 8.7 → 7.0 | |

ruler-30-4g, per stroke:

| v1 | 283 | 302 | 293 | 282 | 300 | 294 | 302 | 303 | 305 | 271 |
|---|---|---|---|---|---|---|---|---|---|---|
| v2 | 289 | 301 | 298 | 293 | 304 | 292 | 301 | 300 | 305 | 296 |
| tilt | 0.05° | 0.02° | 0.07° | 0.09° | 0.08° | 0.06° | 0.02° | 0.08° | 0.01° | 0.18° |

With truly still windows the mismatch is 0.01–0.18°, what a residual bias of a few hundredths
of a dps gives over 3 s, and the term halves the error on the longer slides. On the shorter
slides it is neutral within noise. (The +11% of the 2026-10-30 `ruler-30` stays unexplained
either way.)

### Round 3: in the hand

`air-lines`: lines drawn in the air, "hold 1-2 s between" (hand thresholds, which is the right
preset here):

| stroke at | 10.8 s | 15.1 s | 18.8 s | 21.7 s | 25.4 s | 32.6 s | 36.4 s |
|---|---|---|---|---|---|---|---|
| turned | 3.7° | 6.4° | 5.9° | 1.7° | 0.9° | 5.8° | 5.8° |
| tilt | 0.32° | 0.36° | 0.22° | 0.33° | **1.30°** | 0.24° | 0.18° |
| v_end, mm/s | 180 | 108 | 52 | 118 | 153 | 69 | 107 |
| v1, mm | 213 | 313 | 290 | 219 | 361 | 362 | 271 |
| v2, mm | 208 | 295 | 289 | 216 | 294 | 393 | 272 |

The mismatch is several times what the gyro bias can explain (about 0.1° over 2 s), and one
stroke shows 1.30°. The term changes some strokes by 30-70 mm. The session has no reliable
ground truth ("about 20 cm" lines, measured 213-361 mm), so it cannot say which version is
right; given the size of the mismatch, v2 cannot be trusted there.

## 6. Why it does not suit gestures in the hand

A different hold angle at the end is **not** the problem: the gyro tracks the turn, and the
comparison is made after rotating the end reading back into the start frame. The problems are
what else ends up in `m`:

1. **A hand hold is not still.** Sway and tremor leave a mean acceleration of a few mg in a
   0.3 s window. The core cannot tell that from a tilt: both look like gravity pointing
   slightly elsewhere. 1 mg across gravity reads as 0.06°, the same size as the real tilt.
2. **Calibration residuals stop cancelling.** On a table the board has the same orientation at
   both holds, so offset and gain residuals are equal and cancel. In the hand the two holds
   are at different angles, and the residuals (about 2.6 mg after `board.cal`) show up as a
   fake tilt of up to about 0.15°.
3. **Rotation makes tilt errors of another shape.** With turns and flips the real tilt error
   is larger, but most of it comes from gyro scale error during the turn: closer to a step
   at the moment of the turn than to the steady ramp the model assumes.

The natural gate would be the existing bias-update stillness thresholds (`bias_gyro_std_lsb`,
`bias_accel_std_lsb`: 0.2 dps, 2 mg): apply the term only when the end window passes them, or
bound `m` by what a plausible bias residual produces in `T_e` (about 0.1 dps × `T_e`). Either
would have removed all the bogus cases above. But hand holds rarely pass such a gate, so the
term would be off in exactly the case the product is for.

## 7. What the literature does

The problem (still → move → still, integrate, remove drift at the end) is the stride of a
foot-mounted IMU in gait analysis, and the stroke of an inertial pen.

1. **Linear velocity de-drifting** (gait: Sabatini 2005, Mariani 2010; pens: zero velocity
   compensation). Integrate between two zero-velocity instants and remove a velocity error
   that grows linearly from 0 to the leftover velocity. In position that is exactly
   `v_end·t²/(2T)`: **version 1 is the standard lightweight method.**
2. **Orientation from both ends.** Falbriard et al. 2020 integrate the gyro forward from the
   start and backward from the end of a segment, where the orientation is known, and blend
   the two linearly in time. That is the principled form of this experiment's idea, and it
   too needs a trustworthy orientation at the end. It also needs the raw samples of the whole
   segment (about 6 KB for 4 s here): no room on this MCU.
3. **Error-state Kalman filter with zero-velocity updates, plus a smoother** (Foxlin; the
   open-source OpenShoe; Nilsson, Skog and others). The filter estimates the errors of
   position, velocity and attitude, and often sensor biases; during still periods roll,
   pitch and the accel biases become observable, yaw does not. An RTS smoother then spreads
   the corrections over the whole trajectory. It does what this experiment tried, but it
   **separates tilt from bias and other errors with a statistical model** instead of
   attributing the whole mismatch to tilt. Per-sample covariance updates in integer math and
   stored states for the smoother do not fit a bit-exact spec on an 8 KB Cortex-M0+.

Sources:
- Falbriard et al. 2020, foot-worn inertial sensors, orientation drift:
  https://pmc.ncbi.nlm.nih.gov/articles/PMC7031162
- Sabatini et al. 2005, "Assessment of walking features from foot inertial sensing":
  https://iris.sssup.it/bitstream/11382/301746/1/IEEE%20Trans%20BME%202005.pdf
- Colomar, Nilsson, Händel 2012, "Smoothing for ZUPT-aided INSs":
  https://ersearch2.cvtisr.sk/vufind/EdsRecord/edsair,edsair.doi.dedup.....a6b77cf4ac622d93c696bd3a5a74f44d?lng=en
- Cai et al. 2016, online smoothing for ZUPT-aided INSs:
  https://cambridge.org/core/journals/journal-of-navigation/article/an-online-smoothing-method-based-on-reverse-navigation-for-zuptaided-inss/C6422AD6CF7DBA465E6E5CE23F0C40C0
- What zero-velocity updates make observable:
  https://pmc.ncbi.nlm.nih.gov/articles/PMC5948640
- OpenShoe ZUPT-aided INS (Matlab): https://github.com/hcarlsso/ZUPT-aided-INS
- Inertial pen with ZVC against optical tracking:
  https://www.bohrium.com/paper-details/an-optical-tracking-calibration-method-for-mems-based-digital-writing-instrument/811771205366317057-4018

## 8. What to do instead

- **Keep version 1.** It is the established method, and on table slides with the table
  preset it is already within 1-3% on 15 and 30 cm.
- **Improve the start gravity estimate.** Round 1 showed how much a gravity window that
  catches the start of the motion costs. Candidates: require the gravity window itself to
  pass a stricter quiet test, or move the stroke start earlier by searching back for where
  the readings begin to leave gravity. This error is constant in the start frame, so
  version 1's correction already handles what remains of it.
- **Measure in-hand quality differently.** For gestures in the hand, record strokes that end
  where they started (a mark on the table, pick up, draw, put back): the true end
  displacement is 0, so the end error measures drift with no ruler. Recognition compares
  shapes, so consistency between repetitions may matter more than absolute length.
- **Optionally, a host-only reference:** an error-state Kalman filter with a smoother in
  `gesture-cli`, offline on the recordings. It cannot run on the MCU, but it would show how
  much accuracy is left on the table and which error dominates (tilt, bias, start
  detection).

To revive the tilt term (for example for a table-based or very still use case): take the
branch, add a gate on hold quality (the bias-update stillness thresholds, or `m` bounded by
about 0.1 dps × `T_e`), then rerun rounds 2 and 3.

## 9. Lessons

- **Evaluate with the documented preset.** Round 1 ran table slides with the hand thresholds,
  which `docs/todo.md` already warned against. The method looked harmful for a reason that had
  nothing to do with it, and the first conclusion was wrong.
- **Check a measured correction against a physical bound before trusting it.** A 1.3° tilt in
  1.6 s with no rotation is not a gyro error. Comparing the measured mismatch with what the
  modelled cause can produce would have exposed both the bad windows in round 1 and the
  limits in round 3.
- **Test where the product is used.** The term works where holds are truly still (table
  slides). The product is gestures in the hand, where they are not.
