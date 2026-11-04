//! `vector`: a board test vector, a session's first samples with the config and the result
//! the host build of `core/` gives on them. The `recognize` firmware embeds the file,
//! replays it through `gc_feed` at boot and compares its digest with the one here
//! (`firmware/apps/recognize/main.cpp`), so both builds are held to the same bits.
//!
//! File layout, little-endian:
//!
//! | offset | size   | field |
//! |--------|--------|-------|
//! | 0      | 4      | magic `GCV1` |
//! | 4      | 4      | n, number of samples |
//! | 8      | 4      | events: samples whose event is not `GC_EV_NONE` |
//! | 12     | 4      | digest, see [`digest`] |
//! | 16     | 68     | `gc_config` |
//! | 84     | 16 n   | `gc_sample` each |
//!
//! The digest is FNV-1a (32 bit) over, per sample k after `gc_feed`:
//! - if the event is not `GC_EV_NONE`: k (u32), the `gc_event` (12 B);
//! - if it is `GC_EV_STROKE`: the `gc_stroke` return (u16), the `gc_stroke_info` (24 B) and
//!   the points it returned (8 B each; cap 64, the whole buffer);
//! - always: the `gc_live` return (u16) and its point (8 B, zeros outside `MOVING`).
//!
//! Structs are hashed as their C layout (`core.h`, no padding in any of them).

use anyhow::{Context, bail, ensure};
use gesture_core::Core;
use gesture_core::ffi::{
    GC_EV_NONE, GC_EV_STROKE, GcConfig, GcEvent, GcPoint, GcSample, GcStrokeInfo,
};

use crate::stats::Recording;

pub const MAGIC: [u8; 4] = *b"GCV1";
const HEADER: usize = 84;
const CONFIG_SIZE: usize = 68;
const SAMPLE_SIZE: usize = 16;
/// The firmware's `gc_stroke` buffer: the core never holds more points.
const POINTS_CAP: u16 = 64;

/// What the core gives on a vector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Expect {
    pub events: u32,
    pub digest: u32,
}

/// FNV-1a, 32 bit.
#[derive(Debug)]
struct Fnv(u32);

impl Fnv {
    const fn new() -> Self {
        Self(0x811c_9dc5)
    }

    fn bytes(&mut self, b: &[u8]) {
        for &x in b {
            self.0 = (self.0 ^ u32::from(x)).wrapping_mul(0x0100_0193);
        }
    }
}

fn config_bytes(c: &GcConfig) -> Vec<u8> {
    let mut b = Vec::with_capacity(CONFIG_SIZE);
    b.extend(c.abi_version.to_le_bytes());
    b.extend([c.ctrl1_xl, c.ctrl2_g]);
    b.extend(c.reserved0.to_le_bytes());
    b.extend(c.accel_offset_lsb.iter().flat_map(|v| v.to_le_bytes()));
    b.extend(c.reserved1.to_le_bytes());
    b.extend(c.accel_gain_q30.iter().flat_map(|v| v.to_le_bytes()));
    b.extend(c.gyro_bias_q8.iter().flat_map(|v| v.to_le_bytes()));
    for v in [
        c.still_gyro_std_lsb,
        c.still_accel_std_lsb,
        c.still_gyro_rate_lsb,
        c.hold_n,
        c.bias_gyro_std_lsb,
        c.bias_accel_std_lsb,
        c.clip_lsb,
        c.max_dt_us,
    ] {
        b.extend(v.to_le_bytes());
    }
    b.extend(c.min_stroke_us.to_le_bytes());
    b.extend(c.max_stroke_us.to_le_bytes());
    b.extend(c.min_extent_mm.to_le_bytes());
    b.extend(c.max_v_end_mm_s.to_le_bytes());
    debug_assert_eq!(b.len(), CONFIG_SIZE);
    b
}

fn u16_at(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}

fn i16_at(b: &[u8], at: usize) -> i16 {
    i16::from_le_bytes([b[at], b[at + 1]])
}

fn u32_at(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

fn i32_at(b: &[u8], at: usize) -> i32 {
    u32_at(b, at).cast_signed()
}

fn config_from(b: &[u8]) -> GcConfig {
    let i16x3 = |at| [i16_at(b, at), i16_at(b, at + 2), i16_at(b, at + 4)];
    let i32x3 = |at| [i32_at(b, at), i32_at(b, at + 4), i32_at(b, at + 8)];
    GcConfig {
        abi_version: u32_at(b, 0),
        ctrl1_xl: b[4],
        ctrl2_g: b[5],
        reserved0: u16_at(b, 6),
        accel_offset_lsb: i16x3(8),
        reserved1: u16_at(b, 14),
        accel_gain_q30: i32x3(16),
        gyro_bias_q8: i32x3(28),
        still_gyro_std_lsb: u16_at(b, 40),
        still_accel_std_lsb: u16_at(b, 42),
        still_gyro_rate_lsb: u16_at(b, 44),
        hold_n: u16_at(b, 46),
        bias_gyro_std_lsb: u16_at(b, 48),
        bias_accel_std_lsb: u16_at(b, 50),
        clip_lsb: u16_at(b, 52),
        max_dt_us: u16_at(b, 54),
        min_stroke_us: u32_at(b, 56),
        max_stroke_us: u32_at(b, 60),
        min_extent_mm: u16_at(b, 64),
        max_v_end_mm_s: u16_at(b, 66),
    }
}

fn sample_bytes(s: &GcSample) -> impl Iterator<Item = u8> {
    s.t_us
        .to_le_bytes()
        .into_iter()
        .chain(s.gyro.iter().chain(&s.accel).flat_map(|v| v.to_le_bytes()))
}

fn sample_from(b: &[u8; SAMPLE_SIZE]) -> GcSample {
    GcSample {
        t_us: u32_at(b, 0),
        gyro: [i16_at(b, 4), i16_at(b, 6), i16_at(b, 8)],
        accel: [i16_at(b, 10), i16_at(b, 12), i16_at(b, 14)],
    }
}

fn hash_event(h: &mut Fnv, e: &GcEvent) {
    h.bytes(&[e.kind, e.state, e.reason, e.reserved]);
    h.bytes(&e.points.to_le_bytes());
    h.bytes(&e.v_end_mm_s.to_le_bytes());
    h.bytes(&e.duration_us.to_le_bytes());
}

fn hash_point(h: &mut Fnv, p: &GcPoint) {
    for v in [p.x_mm, p.y_mm, p.z_mm] {
        h.bytes(&v.to_le_bytes());
    }
    h.bytes(&p.t_100us.to_le_bytes());
}

fn hash_info(h: &mut Fnv, i: &GcStrokeInfo) {
    h.bytes(&i.duration_us.to_le_bytes());
    h.bytes(&i.points.to_le_bytes());
    h.bytes(&i.v_end_mm_s.to_le_bytes());
    for g in i.gravity_ug {
        h.bytes(&g.to_le_bytes());
    }
    h.bytes(&i.extent_mm.to_le_bytes());
    h.bytes(&i.step.to_le_bytes());
}

/// Runs `samples` through the core from `gc_init` and digests what it gives.
fn digest(cfg: &GcConfig, samples: &[GcSample]) -> anyhow::Result<Expect> {
    let Ok(mut core) = Core::new(cfg) else {
        bail!("gc_init rejected the config: {cfg:?}");
    };
    let mut h = Fnv::new();
    let mut events = 0;
    for (k, s) in samples.iter().enumerate() {
        let ev = core.feed(s);
        if ev.kind != GC_EV_NONE {
            events += 1;
            h.bytes(&u32::try_from(k)?.to_le_bytes());
            hash_event(&mut h, &ev);
        }
        if ev.kind == GC_EV_STROKE {
            let (info, points) = core.stroke().unwrap_or_default();
            ensure!(
                info.points <= POINTS_CAP,
                "stroke at sample {k} has {} points, the board buffer {POINTS_CAP}",
                info.points
            );
            h.bytes(&u16::try_from(points.len())?.to_le_bytes());
            hash_info(&mut h, &info);
            for p in &points {
                hash_point(&mut h, p);
            }
        }
        let live = core.live();
        h.bytes(&u16::from(live.is_some()).to_le_bytes());
        hash_point(&mut h, &live.unwrap_or_default());
    }
    Ok(Expect {
        events,
        digest: h.0,
    })
}

/// The vector of the first `n` samples of a recording.
pub fn build(rec: &Recording, cfg: &GcConfig, n: usize) -> anyhow::Result<Vec<u8>> {
    ensure!(
        n <= rec.samples.len(),
        "the session has {} samples, asked for {n}",
        rec.samples.len()
    );
    let samples: Vec<GcSample> = rec.samples[..n]
        .iter()
        .map(|(_, s)| GcSample {
            t_us: s.t_us,
            gyro: s.gyro,
            accel: s.accel,
        })
        .collect();
    let expect = digest(cfg, &samples)?;
    let mut b = Vec::with_capacity(HEADER + SAMPLE_SIZE * n);
    b.extend(MAGIC);
    b.extend(u32::try_from(n)?.to_le_bytes());
    b.extend(expect.events.to_le_bytes());
    b.extend(expect.digest.to_le_bytes());
    b.extend(config_bytes(cfg));
    b.extend(samples.iter().flat_map(sample_bytes));
    Ok(b)
}

/// Reads a vector: its stored result, and the result the core gives on it now.
pub fn check(b: &[u8]) -> anyhow::Result<(Expect, Expect)> {
    ensure!(b.len() >= HEADER && b[..4] == MAGIC, "not a GCV1 vector");
    let n = usize::try_from(u32_at(b, 4))?;
    ensure!(
        b.len() == HEADER + SAMPLE_SIZE * n,
        "{} bytes for {n} samples",
        b.len()
    );
    let stored = Expect {
        events: u32_at(b, 8),
        digest: u32_at(b, 12),
    };
    let cfg = config_from(&b[16..HEADER]);
    let samples: Vec<GcSample> = b[HEADER..]
        .as_chunks::<SAMPLE_SIZE>()
        .0
        .iter()
        .map(sample_from)
        .collect();
    let now = digest(&cfg, &samples).context("replaying the vector")?;
    Ok((stored, now))
}

#[cfg(test)]
mod tests {
    use std::path::{Path, PathBuf};

    use super::*;

    fn data(rel: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../data")
            .join(rel)
    }

    #[test]
    fn known_hash() {
        // FNV-1a 32 reference values.
        let mut h = Fnv::new();
        h.bytes(b"");
        assert_eq!(h.0, 0x811c_9dc5);
        h.bytes(b"a");
        assert_eq!(h.0, 0xe40c_292c);
    }

    #[test]
    fn config_round_trips() {
        let cfg = GcConfig {
            abi_version: 1,
            ctrl1_xl: 0x48,
            ctrl2_g: 0x44,
            reserved0: 0,
            accel_offset_lsb: [-35, 45, 205],
            reserved1: 0,
            accel_gain_q30: [1, -2, 3],
            gyro_bias_q8: [-4, 5, -6],
            still_gyro_std_lsb: 7,
            still_accel_std_lsb: 8,
            still_gyro_rate_lsb: 9,
            hold_n: 10,
            bias_gyro_std_lsb: 11,
            bias_accel_std_lsb: 12,
            clip_lsb: 13,
            max_dt_us: 14,
            min_stroke_us: 15,
            max_stroke_us: 16,
            min_extent_mm: 17,
            max_v_end_mm_s: 18,
        };
        assert_eq!(config_from(&config_bytes(&cfg)), cfg);
    }

    /// The committed board vector still matches the core, and regenerates to the same
    /// bytes with the command in `data/vectors/README.md`.
    #[test]
    fn board_vector_is_current() {
        let file = data("vectors/air-lines.gcv");
        let bytes = std::fs::read(&file).expect("vector file");
        let (stored, now) = check(&bytes).expect("vector");
        assert_eq!(stored, now, "core changed: regenerate {}", file.display());
        assert!(stored.events >= 15, "{stored:?}");

        let (rec, cfg, _) = crate::reference_config(&crate::ReferenceArgs {
            file: data("sessions/20261030-212525_air-lines.bin"),
            calib: Some(data("sessions/20261030-205757_tumble.bin")),
            hold: crate::Hold::Hand,
            bias: Some(data("sessions/20261030-183544_rest-table.bin")),
        })
        .expect("config");
        let n = usize::try_from(u32_at(&bytes, 4)).expect("n");
        assert!(build(&rec, &cfg.gc(), n).expect("build") == bytes);
    }
}
