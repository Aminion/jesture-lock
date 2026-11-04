# Conformance vectors

Recorded or synthetic IMU samples, each with its `gc_config` and the digest of what
`core.h` gives on them (`docs/core.md`). An implementation conforms when it gives the
stored digest on every file. Checked on the host by `ctest` (`core/tests/core_vectors.cpp`,
through `core.h` only) and by `cargo test` (which also checks that every file regenerates to
the same bytes); on the MCU through the board vector.

Regenerate all of them after a core change, in `host/`: `cargo run -q -p gesture-cli -- vectors`.

## Format (GCV1)

Little-endian; the structs are their `core.h` layouts (no padding in any of them).

| Offset | Size | Field |
|--------|------|-------|
| 0      | 4    | magic `GCV1` |
| 4      | 4    | n, number of samples (u32) |
| 8      | 4    | events: samples whose event is not `GC_EV_NONE` (u32) |
| 12     | 4    | digest (u32) |
| 16     | 68   | `gc_config` |
| 84     | 16 n | `gc_sample` each |

Run: `gc_init` with the config (it must return `GC_OK`), then for each sample k, in
order: `gc_feed`; then add to the digest

1. if the event is not `GC_EV_NONE`: k (u32), then the `gc_event` (12 B);
2. if it is `GC_EV_STROKE`: `gc_stroke` with a 64-point buffer: its return (u16), the
   `gc_stroke_info` (24 B), then the points it returned (8 B each);
3. always: `gc_live`: its return (u16), then its point (8 B; zeros outside `MOVING`).

The digest is FNV-1a 32 (offset basis 0x811c9dc5, prime 0x01000193) over those bytes,
starting from the basis at `gc_init`.

## Files

`air-lines.gcv`, the board vector: the first 2400 samples (23 s) of `air-lines` (19
events: READY, 9 MOVING, 4 SHORT, 1 DRIFT, 4 STROKE). The `recognize` firmware embeds it
and replays it at boot (38.5 KB of flash, so the others stay on the host).

`sessions/`: whole recordings, calibrated with `tumble`, initial gyro bias from
`rest-table`, hand thresholds (`vector::SESSIONS` in `host/crates/gesture-cli/src/vector.rs`):

| File | Events | Why |
|------|--------|-----|
| `air-lines.gcv` | 30: 8 STROKE, 5 SHORT, 1 DRIFT | hand-held strokes in four directions |
| `ruler-15.gcv` | 19: 2 STROKE, 7 SHORT | extent near min_extent_mm (19.25 vs 20 mm) |
| `ruler-30.gcv` | 17: 4 STROKE, 3 SHORT, 1 DRIFT | a longer slide |
| `hand-hold.gcv` | 12: 1 STROKE, 1 SHORT, 2 LONG, 3 READY | LONG, then back through WARMUP |

`synth/`: known motions turned into ideal IMU readings (`host/crates/gesture-cli/src/synth.rs`;
min-jerk moves and turns, inverse calibration to raw LSB, integer noise). Each case's events
and, for strokes, the end point and gravity against the true motion are checked apart from
the digest, so a vector cannot silently stop testing what it was made for.

| File | What it checks |
|------|----------------|
| `lines` | three lines along the world axes, z against gravity: ends within 5 mm |
| `turn-in-place` | a 90 degree tip in place: gravity removal through R(q); SHORT |
| `tilted-turning` | tilted at the start, turning while moving: the start frame |
| `long-path` | a 3 s path: compaction to step 8 |
| `full-at-end` | the buffer full from the snapshot to the end (200 Hz, hold_n 11, step 16): compacted in the end step, before the append |
| `scale-*` | every accel FS code, gyro 250 / 1000 / 2000 dps and FS_125 |
| `time-gap` | dt = 0 in a hold, dt > max_dt_us in a move, dt = max_dt_us (not a gap) |
| `long` | LONG at max_stroke_us, then WARMUP and READY again |
| `clipped` | a clipped reading inside the stroke-start replay |
| `short` | SHORT by duration, then by extent |
| `drift` | DRIFT at 0.5 m/s |
| `drift-max` | 80 m/s with max_v_end_mm_s 65535: DRIFT before saturation |
| `wrap-replay`, `wrap-stroke` | the MCU clock wrapping inside the replay, then inside a stroke |

Not covered here: `gc_init` errors (`core/tests/core_test.cpp`).

## On the board

`cmake --build --preset firmware --target flash_recognize`, wait ~4 s (LED: slow blink =
match, fast = mismatch), then read `bench_result` (14 words, fields in
`firmware/apps/recognize/main.cpp`):

```
addr=$(arm-none-eabi-nm build/firmware/firmware/apps/recognize.elf | awk '/ bench_result$/ {print $1}')
openocd -f interface/stlink.cfg -f target/stm32g0x.cfg -c init -c halt -c "mdw 0x$addr 14" -c resume -c exit
```
