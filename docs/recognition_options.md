# Recognition: options (not decided)

Notes from the 2026-11-16 discussion on how to recognize figures from the stroke path. No
final choice yet; next step is a f64 prototype in `gesture-cli` on recorded figures.
Costs below are estimates, not measurements; measure on the board like `gc_feed`.

## Input and constraints

- The core already gives the stroke path: up to 64 points, int16 mm, 3D, bit-exact on host
  and MCU. Recognition runs once per stroke end, not per sample.
- Budget (`recognize`, 2026-11-16): flash 49.1 KB of 64 (38.5 KB is the self-test vector),
  .bss 4.2 KB of 8 KB; core state 1464 B of 2048 used.
- `gc_feed` worst case is already 6.6 ms of the 9.6 ms sample period (stroke end), so
  matching goes into a separate call (e.g. `gc_match`) run during the hold after the
  STROKE event, not inside `gc_feed`.
- Path accuracy (`docs/todo.md`): direction good (tilt error mostly < 0.25 deg), length
  +-10 %, v_end before the correction 64-249 mm/s in the air. A method should lean on shape
  and direction, with tolerance on exact size.

## Requirements (as agreed so far)

- A set of figures, not one: pentagram, square, triangle, circle, ...
- Only the resulting figure counts: start corner, drawing direction, size, position,
  orientation (upside down, tilted in 3D) must not matter.
- 3D figures (not flat) are wanted; open figures (zig-zag, helix) included.
- No mirrors: a left and a right helix are different figures. Note: for flat figures this
  cannot hold under full rotation invariance (a flat mirror image is the figure turned over),
  which is fine for symmetric figures.
- A lock: open-set. Reject everything that is not a known figure; false accepts matter most.

## Methods considered

### $-family (few-shot template matchers)

| Method | Matching | Order / direction | Rotation | MCU | Notes |
|---|---|---|---|---|---|
| $1 (Wobbrock 2007) | resample, box scale, rotation search, mean point distance | matter | golden-section search | weak: trig in a loop | box scaling breaks lines |
| Protractor (Li 2010) | resample, unit vector, closed-form best rotation, cosine | matter | closed form (2D), or orientation-sensitive | good | successor of $1, faster, equal or better |
| Penny Pincher (Taranta, LaViola 2015) | dot products of segment directions | matter | none (orientation counts) | best | cheapest, same code in 3D |
| 1c (Herold, Stahovich 2012) | distance-to-centroid signature | partly lost | invariant | good | cannot tell CW from CCW |
| $N (Anthony, Wobbrock 2010/12) | $1/Protractor over stroke orders | permutations | as above | fine | multi-stroke only |
| $P (Vatavu 2012), $Q (2018) | point cloud, greedy matching | ignored | none | ok / good | loses the path |
| $3, Protractor3D (Kratz, Rohs 2010/11) | 3D, best 3D rotation (Horn) | matter | closed form | good with QCP, see below | |
| uWave (Liu et al. 2009) | DTW on quantized raw accel | matter | none | good | skips the core's path |
| Jackknife (Taranta et al. 2017) | DTW on direction vectors, threshold from synthetic negatives | matter | none | ok (N^2 per template) | best rejection method |

### Fourier descriptors (closed figures only)

- 2D: project onto the drawing plane (normal from the vector area `sum p_i x p_{i+1}`, sign
  chosen so the figure runs CCW), DFT of `x + iy`, keep |F_k|, k = -8..8, drop F_0,
  divide by the largest. Invariant to start, rotation, size, direction; pentagram is
  dominated by F_2, so it does not collide with a pentagon. ~2K MAC, 32-entry constexpr
  sine table, ~16 numbers per template.
- 3D: elliptic Fourier descriptors: per-axis DFT, each harmonic is a 3x2 matrix M_k; its
  two singular values (from trace and det of M_k^T M_k) are invariant to 3D rotation, start,
  direction and mirror. Loses the relative planes of the harmonics; add a flatness number
  (RMS distance from the best plane / size).
- Dropped as primary: closed figures only, and open 3D figures are wanted.

### Best-rotation matching with QCP (current favourite)

1. Resample by arc length to N = 32 (as a path; as a loop for closed templates, including
   the closing segment), subtract the centroid, scale to unit RMS size, quantize to 12 bits.
2. Per template: minimum RMSD over all proper 3D rotations:
   `RMSD^2 = (|P|^2 + |Q|^2 - 2 lambda_max) / N`, lambda_max the largest eigenvalue of
   Horn's 4x4 matrix built from `H = sum p_i q_i^T`. QCP (Theobald 2005): a few Newton steps
   on the quartic from an upper bound; fixed iteration count, fixed point, bit-exact.
   Only the eigenvalue is needed, not the rotation.
3. Nearest template; reject above a threshold.

- Proper rotations only, so 3D mirrors stay distinct (no mirrors, as required).
- 12-bit coordinates keep `H` in int32 (products < 2^24, sum of 32); int64 only in the
  quartic. ~300 MAC per alignment, ~0.05 ms per alignment at 64 MHz (estimate).
- Template: 32 x 3 x int16 = 192 B, plus a closed flag.
- Pairing direction: open figures need 2 alignments (forward, reversed); closed 3D (not
  flat) figures too.
- Start of closed figures: for symmetric flat figures (triangle, square, pentagram, circle)
  started at a corner, another start corner is a rotation and the other direction is a
  flip, both absorbed by QCP: 1 alignment. Search only if recordings show a need:
  - over the stroke's detected corners (asymmetric closed figures: house, heart): a few
    alignments;
  - full cyclic shift search: N shifts x 2 directions = 64 alignments, ~1.5 ms per
    template; coarse-to-fine (every 4th shift, then +-2) cuts it to ~24.
- Weak point: arc-length resampling assumes each part of the figure takes a similar
  share of the path; if not, DTW after the alignment.

Rejected alternatives for 3D rotation invariance: curvature + torsion (third derivative of
noisy positions), distance-to-centroid (too little information), PCA alignment (sign flips,
unstable for near-equal axes).

### Neural network

- Fits: MLP 96 -> 32 -> 16 -> K on the normalized path, ~3.7K int8 weights (~4 KB flash),
  < 200 B activations, < 1 ms; hand-written int8 inference (TFLite Micro too big),
  deterministic, can be specified in `core.md`. A 1D CNN on raw IMU fits but RAM is tight.
- Data (order of magnitude): raw IMU hundreds per figure from several people; normalized path
  maybe 20-50 per figure with augmentation (random 3D rotations, scale, time warp, noise;
  `synth.rs` could generate synthetic figures). Plus minutes of non-gestures for a "none"
  class.
- Against: rotation invariance must be learned (QCP has it exactly), weak open-set
  rejection, a new figure means a new dataset and retraining, training in Rust (no
  Python: burn, candle), opaque failures.
- For: high-variance figures, many classes, lots of data.
- Use: fallback if template matching is not good enough; the same recordings become the
  training set.

## Open questions

- Final method: QCP (favourite), Penny Pincher / Protractor, $P, Fourier, NN.
- Start handling for closed figures: 1 alignment, corner search, or full shift search.
- N (16 / 32), threshold, templates per figure (3-5 recordings?).
- How repeatable 3D figures are in the air without visual feedback.
- `core.h` call shape (`gc_match`), template layout (`docs/template_format.md`).

## Next steps

1. Record each figure ~10 times in varied orientations (upright, upside down, on the table,
   tilted), a few 3D figures, and a session of non-gestures (normal handling of the box):
   `cargo run -q -p gesture-cli -- record <label> --seconds N --note "..."`.
2. f64 prototype in `gesture-cli`: resample, normalize, QCP against templates from the
   recordings (and Penny Pincher / Protractor / $P for comparison); confusion table and
   the non-gesture scores for the threshold.
3. The winner gets the integer spec in `docs/core.md`, then the C++ implementation,
   conformance vectors and `recognize`.
