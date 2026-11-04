//! `synth`: synthetic conformance vectors (`core/contract/vectors/v1/README.md`). Each case is a known
//! motion turned into ideal IMU readings (f64 physics, then the inverse calibration and
//! rounding to raw LSB, plus integer LCG noise), written as a GCV1 vector (`vector.rs`)
//! with the result the core gives on it.
//!
//! The digest only says that an implementation agrees with the C++ one. What each case is
//! for is checked apart from it, in the tests below: the events it must give, in order,
//! and for the strokes the end point and gravity against the true motion.

use std::path::Path;

use anyhow::Context;
use gesture_core::Core;
use gesture_core::ffi::{
    GC_ABI_VERSION, GC_EV_MOVING, GC_EV_NONE, GC_EV_READY, GC_EV_REJECTED, GC_EV_STROKE,
    GC_REJ_CLIPPED, GC_REJ_DRIFT, GC_REJ_LONG, GC_REJ_SHORT, GC_REJ_TIME_GAP, GcConfig, GcEvent,
    GcPoint, GcSample, GcStrokeInfo,
};
use protocol::{Firmware, Hello, ImuStatus};

use crate::vector;

/// m/s^2 per g, as in docs/core.md.
const G: f64 = 9.80665;
/// The board's measured sample period (104 Hz nominal).
const DT_US: u32 = 9600;
/// `GC_GUARD_N`, docs/core.md "Fixed constants" (not in `core.h`): the stroke starts this
/// many samples plus one before the sample that reports MOVING.
const GUARD_N: usize = 10;

type V3 = [f64; 3];

/// Rotation, board frame -> world frame (v' = q v q*).
#[derive(Debug, Clone, Copy)]
struct Quat([f64; 4]);

impl Quat {
    const ONE: Self = Self([1.0, 0.0, 0.0, 0.0]);

    fn axis_angle(axis: V3, deg: f64) -> Self {
        let n = axis.iter().map(|a| a * a).sum::<f64>().sqrt();
        if n == 0.0 {
            return Self::ONE;
        }
        let h = deg.to_radians() / 2.0;
        let s = h.sin() / n;
        Self([h.cos(), axis[0] * s, axis[1] * s, axis[2] * s])
    }

    fn mul(self, o: Self) -> Self {
        let [a0, a1, a2, a3] = self.0;
        let [b0, b1, b2, b3] = o.0;
        Self([
            a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3,
            a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2,
            a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1,
            a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0,
        ])
    }

    fn conj(self) -> Self {
        let [w, x, y, z] = self.0;
        Self([w, -x, -y, -z])
    }

    fn rotate(self, v: V3) -> V3 {
        let r = self.mul(Quat([0.0, v[0], v[1], v[2]])).mul(self.conj());
        [r.0[1], r.0[2], r.0[3]]
    }
}

/// The simulated sensor: its scales, the calibration the config carries (exact), and the
/// true gyro bias (the config starts from a slightly different one).
#[derive(Debug, Clone, Copy)]
struct Sensor {
    ctrl1_xl: u8,
    ctrl2_g: u8,
    offset_lsb: [i16; 3],
    gain: V3,
    bias_dps: V3,
    initial_bias_dps: V3,
}

impl Sensor {
    /// The board's setting: +-4 g, 500 dps, 104 Hz.
    const HAND: Self = Self::with_scales(0x48, 0x44);

    const fn with_scales(ctrl1_xl: u8, ctrl2_g: u8) -> Self {
        Self {
            ctrl1_xl,
            ctrl2_g,
            offset_lsb: [12, -20, 35],
            gain: [1.02, 0.98, 1.01],
            bias_dps: [0.5, -0.3, 0.2],
            initial_bias_dps: [0.45, -0.25, 0.2],
        }
    }

    fn hello(self) -> Hello {
        Hello {
            fw: Firmware::Stream,
            imu: ImuStatus::Ok,
            imu_addr: 0,
            who_am_i: 0,
            ctrl1_xl: self.ctrl1_xl,
            ctrl2_g: self.ctrl2_g,
        }
    }

    /// ug/LSB.
    fn sa(self) -> f64 {
        f64::from(self.hello().accel_ug_per_lsb().unwrap_or(1))
    }

    /// udps/LSB.
    fn sg(self) -> f64 {
        f64::from(self.hello().gyro_udps_per_lsb().unwrap_or(1))
    }

    fn gain_q30(self) -> [i32; 3] {
        self.gain
            .map(|g| (g * f64::from(1u32 << 30)).round() as i32)
    }

    /// The hand defaults of docs/todo.md, in physical units, at this sensor's scales.
    fn config(self) -> GcConfig {
        let (sa, sg) = (self.sa(), self.sg());
        let dps = |x: f64| ((x * 1e6 / sg).round() as u16).max(1);
        let mg = |x: f64| ((x * 1e3 / sa).round() as u16).max(1);
        GcConfig {
            abi_version: GC_ABI_VERSION,
            ctrl1_xl: self.ctrl1_xl,
            ctrl2_g: self.ctrl2_g,
            accel_offset_lsb: self.offset_lsb,
            accel_gain_q30: self.gain_q30(),
            gyro_bias_q8: self
                .initial_bias_dps
                .map(|b| (b * 1e6 / sg * 256.0).round() as i32),
            still_gyro_std_lsb: dps(3.0),
            still_accel_std_lsb: mg(15.0),
            still_gyro_rate_lsb: dps(4.0),
            hold_n: 20,
            bias_gyro_std_lsb: dps(0.2),
            bias_accel_std_lsb: mg(2.0),
            clip_lsb: 32700,
            max_dt_us: 14423,
            min_stroke_us: 150_000,
            max_stroke_us: 4_000_000,
            min_extent_mm: 20,
            max_v_end_mm_s: 300,
            ..GcConfig::default()
        }
    }
}

/// Minimum jerk, s(tau) for tau in 0..1, and its first two derivatives.
fn min_jerk(tau: f64) -> [f64; 3] {
    let t2 = tau * tau;
    let t3 = t2 * tau;
    [
        t3 * (10.0 - 15.0 * tau + 6.0 * t2),
        30.0 * t2 * (1.0 - tau) * (1.0 - tau),
        60.0 * tau - 180.0 * t2 + 120.0 * t3,
    ]
}

/// Samples from a sequence of motions. The world frame has z up; the board starts level
/// (or as `tilt` sets it), at rest.
#[derive(Debug)]
struct Gen {
    sensor: Sensor,
    t_us: u32,
    /// Time between samples, us.
    dt: u32,
    next_dt: u32,
    q: Quat,
    rng: u32,
    /// Uniform noise amplitude, dps and mg.
    noise: (f64, f64),
    clip_next: Option<i16>,
    samples: Vec<GcSample>,
}

impl Gen {
    fn new(sensor: Sensor) -> Self {
        Self {
            sensor,
            t_us: 1_000_000,
            dt: DT_US,
            next_dt: DT_US,
            q: Quat::ONE,
            rng: 1,
            noise: (0.07, 3.0),
            clip_next: None,
            samples: Vec::new(),
        }
    }

    /// Samples `dt_us` apart (another ODR) from the next one on.
    fn with_dt(mut self, dt_us: u32) -> Self {
        self.dt = dt_us;
        self.next_dt = dt_us;
        self
    }

    fn start_at(mut self, t_us: u32) -> Self {
        self.t_us = t_us;
        self
    }

    fn tilt(mut self, axis: V3, deg: f64) -> Self {
        self.q = Quat::axis_angle(axis, deg);
        self
    }

    /// Uniform in -a..=a LSB (a LCG, so every platform gives the same readings).
    fn noise_lsb(&mut self, a: f64) -> f64 {
        let a = a.round() as u32;
        if a == 0 {
            return 0.0;
        }
        self.rng = self.rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        f64::from((self.rng >> 8) % (2 * a + 1)) - f64::from(a)
    }

    /// One sample: the world acceleration (m/s^2) and the board's rate (dps, board axes).
    fn sample(&mut self, acc: V3, rate: V3) {
        let s = self.sensor;
        let (sa, sg) = (s.sa(), s.sg());
        self.t_us = self.t_us.wrapping_add(self.next_dt);
        self.next_dt = self.dt;
        // Specific force: what an accelerometer reads, in g, board axes.
        let f = self
            .q
            .conj()
            .rotate([acc[0] / G, acc[1] / G, acc[2] / G + 1.0]);
        let (noise_g, noise_a) = (self.noise.0 * 1e6 / sg, self.noise.1 * 1e3 / sa);
        let clamp = |x: f64| x.round().clamp(-32768.0, 32767.0) as i16;
        let mut accel = [0; 3];
        let mut gyro = [0; 3];
        for i in 0..3 {
            let a = f[i] * 1e6 / (sa * s.gain[i]) + f64::from(s.offset_lsb[i]);
            accel[i] = clamp(a + self.noise_lsb(noise_a));
            let w = (rate[i] + s.bias_dps[i]) * 1e6 / sg;
            gyro[i] = clamp(w + self.noise_lsb(noise_g));
        }
        if let Some(x) = self.clip_next.take() {
            accel[0] = x;
        }
        self.samples.push(GcSample {
            t_us: self.t_us,
            gyro,
            accel,
        });
    }

    fn count(&self, secs: f64) -> u32 {
        (secs * 1e6 / f64::from(self.dt)).round() as u32
    }

    fn hold(mut self, secs: f64) -> Self {
        for _ in 0..self.count(secs) {
            self.sample([0.0; 3], [0.0; 3]);
        }
        self
    }

    /// Moves by `d_mm` (world axes) while turning `deg` about `axis` (board axes), both
    /// on a minimum-jerk profile, from rest to rest.
    ///
    /// Each sample reads the mean rate and the mean acceleration over the step before it
    /// (what a filtered sensor gives), the step the core integrates it over; the
    /// instantaneous values would put the core's turn half a step ahead.
    fn moves(mut self, d_mm: V3, axis: V3, deg: f64, secs: f64) -> Self {
        let n = self.count(secs);
        let big_t = f64::from(n * self.dt) / 1e6;
        let dt = big_t / f64::from(n);
        let q0 = self.q;
        let norm = axis.iter().map(|a| a * a).sum::<f64>().sqrt().max(1e-12);
        let mut prev = min_jerk(0.0);
        for j in 1..=n {
            let now = min_jerk(f64::from(j) / f64::from(n));
            self.q = q0.mul(Quat::axis_angle(axis, deg * now[0]));
            // v = d s' / T, so the mean acceleration is d (s'_j - s'_j-1) / (T dt).
            let acc = d_mm.map(|d| d / 1000.0 * (now[1] - prev[1]) / (big_t * dt));
            let rate = axis.map(|a| a / norm * deg * (now[0] - prev[0]) / dt);
            self.sample(acc, rate);
            prev = now;
        }
        self
    }

    /// Changes velocity by `dv` (m/s, world axes) on a minimum-jerk profile; the board
    /// then goes on at that velocity, which the sensor cannot tell from rest.
    fn accelerate(mut self, dv: V3, secs: f64) -> Self {
        let n = self.count(secs);
        let dt = f64::from(self.dt) / 1e6;
        let mut prev = 0.0;
        for j in 1..=n {
            let [s, _, _] = min_jerk(f64::from(j) / f64::from(n));
            let acc = dv.map(|v| v * (s - prev) / dt);
            self.sample(acc, [0.0; 3]);
            prev = s;
        }
        self
    }

    /// The next sample comes `dt_us` after the last one.
    fn gap(mut self, dt_us: u32) -> Self {
        self.next_dt = dt_us;
        self
    }

    /// The next sample reads `x` on accel x.
    fn clip(mut self, x: i16) -> Self {
        self.clip_next = Some(x);
        self
    }

    fn done(self, cfg: GcConfig) -> (GcConfig, Vec<GcSample>) {
        (cfg, self.samples)
    }
}

/// What the core gave on one sample, when it gave something.
#[derive(Debug)]
pub struct Out {
    pub k: usize,
    pub ev: GcEvent,
    pub stroke: Option<(GcStrokeInfo, Vec<GcPoint>)>,
}

/// Runs a vector's samples through the core; the non-NONE events.
pub fn replay(cfg: &GcConfig, samples: &[GcSample]) -> anyhow::Result<Vec<Out>> {
    let mut core = Core::new(cfg).map_err(|e| anyhow::anyhow!("gc_init: status {e}"))?;
    Ok(samples
        .iter()
        .enumerate()
        .filter_map(|(k, s)| {
            let ev = core.feed(s);
            (ev.kind != GC_EV_NONE).then(|| Out {
                k,
                ev,
                stroke: (ev.kind == GC_EV_STROKE).then(|| core.stroke()).flatten(),
            })
        })
        .collect())
}

/// One synthetic vector: how to make it, and what it is for.
#[derive(Debug)]
pub struct Case {
    pub name: &'static str,
    /// The events it must give, (kind, reason), in order.
    pub expect: &'static [(u8, u8)],
    build: fn() -> anyhow::Result<(GcConfig, Vec<GcSample>)>,
}

impl Case {
    pub fn build(&self) -> anyhow::Result<(GcConfig, Vec<GcSample>)> {
        (self.build)()
    }
}

const READY: (u8, u8) = (GC_EV_READY, 0);
const MOVING: (u8, u8) = (GC_EV_MOVING, 0);
const STROKE: (u8, u8) = (GC_EV_STROKE, 0);
const fn rejected(reason: u8) -> (u8, u8) {
    (GC_EV_REJECTED, reason)
}

/// The line used by the scale cases: 120 mm across while turning 40 degrees.
fn scale_case(ctrl1_xl: u8, ctrl2_g: u8) -> (GcConfig, Vec<GcSample>) {
    let s = Sensor::with_scales(ctrl1_xl, ctrl2_g);
    Gen::new(s)
        .hold(1.5)
        .moves([80.0, -60.0, 70.0], [0.3, 1.0, -0.5], 40.0, 0.7)
        .hold(1.0)
        .done(s.config())
}

pub const CASES: &[Case] = &[
    // Three lines along the world axes, z against gravity.
    Case {
        name: "lines",
        expect: &[READY, MOVING, STROKE, MOVING, STROKE, MOVING, STROKE],
        build: || {
            let s = Sensor::HAND;
            Ok(Gen::new(s)
                .hold(1.5)
                .moves([150.0, 0.0, 0.0], [0.0; 3], 0.0, 0.6)
                .hold(1.0)
                .moves([0.0, -120.0, 0.0], [0.0; 3], 0.0, 0.5)
                .hold(1.0)
                .moves([0.0, 0.0, 200.0], [0.0; 3], 0.0, 0.7)
                .hold(1.0)
                .done(s.config()))
        },
    },
    // A quarter turn in place, tipping gravity onto another axis: the path stays near
    // zero, so it is too small for a stroke.
    Case {
        name: "turn-in-place",
        expect: &[READY, MOVING, rejected(GC_REJ_SHORT)],
        build: || {
            let s = Sensor::HAND;
            Ok(Gen::new(s)
                .hold(1.5)
                .moves([0.0; 3], [1.0, 0.0, 0.0], 90.0, 0.8)
                .hold(1.0)
                .done(s.config()))
        },
    },
    // Tilted at the start, turning while it moves: the path is in the start frame.
    Case {
        name: "tilted-turning",
        expect: &[READY, MOVING, STROKE, MOVING, STROKE],
        build: || {
            let s = Sensor::HAND;
            Ok(Gen::new(s)
                .tilt([0.0, 1.0, 0.0], 30.0)
                .hold(1.5)
                .moves([100.0, 80.0, 0.0], [0.0, 0.0, 1.0], 60.0, 0.8)
                .hold(1.0)
                .moves([-50.0, 0.0, 120.0], [1.0, 1.0, 0.0], -45.0, 0.9)
                .hold(1.0)
                .done(s.config()))
        },
    },
    // A slow 3 s path: the point buffer is compacted three times (step 8). A 3 s hold
    // first, so the bias update has converged from the config's initial bias (0.05 dps
    // off): what is left after 1.5 s (~0.01 dps) tilts the frame enough over 3 s to leave
    // ~9 mm, growing as t^3, which the t^2 correction cannot take out.
    Case {
        name: "long-path",
        expect: &[READY, MOVING, STROKE],
        build: || {
            let s = Sensor::HAND;
            Ok(Gen::new(s)
                .hold(3.0)
                .moves([200.0, 0.0, 0.0], [0.0, 0.0, 1.0], 30.0, 1.0)
                .moves([0.0, 150.0, 50.0], [0.0; 3], 0.0, 1.0)
                .moves([-100.0, 0.0, -50.0], [1.0, 0.0, 0.0], 20.0, 1.0)
                .hold(1.0)
                .done(s.config()))
        },
    },
    // The buffer is full at the snapshot and stays so to the end: compacted in `finish`
    // before the snapshot point is appended. That needs a step larger than the hold_n - 1
    // samples after the snapshot: with hold_n 11, step 16 and the snapshot at c in
    // 1008..1014 (63 * 16 .. 64 * 16 - 10), ~5 s at 200 Hz. The last move's length is
    // searched for (the test checks c).
    Case {
        name: "full-at-end",
        expect: &[READY, MOVING, STROKE],
        build: || {
            let s = Sensor::HAND;
            let cfg = GcConfig {
                hold_n: 11,
                max_stroke_us: 6_000_000,
                ..s.config()
            };
            let make = |last: f64| {
                Gen::new(s)
                    .with_dt(5000)
                    .hold(1.5)
                    .moves([150.0, 0.0, 0.0], [0.0, 0.0, 1.0], 40.0, 1.0)
                    .moves([0.0, 150.0, 0.0], [1.0, 0.0, 0.0], 30.0, 1.0)
                    .moves([-150.0, 0.0, 50.0], [0.0, 1.0, 0.0], -30.0, 1.0)
                    .moves([0.0, -150.0, -50.0], [0.0, 0.0, 1.0], -40.0, 1.0)
                    .moves([100.0, 50.0, 0.0], [1.0, 0.0, 0.0], -30.0, last)
                    .hold(1.0)
                    .done(cfg)
            };
            for n in 60..400 {
                let (cfg, samples) = make(f64::from(n) * 0.005);
                let c = snapshot_c(&replay(&cfg, &samples)?, cfg.hold_n);
                if c.is_some_and(|c| FULL_AT_END.contains(&c)) {
                    return Ok((cfg, samples));
                }
            }
            anyhow::bail!("no move length puts the snapshot at c in {FULL_AT_END:?}")
        },
    },
    Case {
        name: "scale-2g-250dps",
        expect: &[READY, MOVING, STROKE],
        build: || Ok(scale_case(0x40, 0x40)),
    },
    Case {
        name: "scale-8g-1000dps",
        expect: &[READY, MOVING, STROKE],
        build: || Ok(scale_case(0x4C, 0x48)),
    },
    Case {
        name: "scale-16g-2000dps",
        expect: &[READY, MOVING, STROKE],
        build: || Ok(scale_case(0x44, 0x4C)),
    },
    Case {
        name: "scale-4g-125dps",
        expect: &[READY, MOVING, STROKE],
        build: || Ok(scale_case(0x48, 0x42)),
    },
    // Time gaps: dt = 0 in a hold (back to WARMUP, silently), dt > max_dt_us in a move,
    // dt = max_dt_us (not a gap) in a hold.
    Case {
        name: "time-gap",
        expect: &[
            READY,
            READY,
            MOVING,
            rejected(GC_REJ_TIME_GAP),
            READY,
            MOVING,
            STROKE,
        ],
        build: || {
            let s = Sensor::HAND;
            let cfg = s.config();
            Ok(Gen::new(s)
                .hold(1.0)
                .gap(0)
                .hold(1.0)
                .moves([100.0, 0.0, 0.0], [0.0; 3], 0.0, 0.6)
                .gap(u32::from(cfg.max_dt_us) + 1)
                .hold(1.0)
                .gap(u32::from(cfg.max_dt_us))
                .hold(0.5)
                .moves([100.0, 0.0, 0.0], [0.0; 3], 0.0, 0.6)
                .hold(1.0)
                .done(cfg))
        },
    },
    // A move longer than max_stroke_us (1.5 s here): rejected at once, then WARMUP.
    Case {
        name: "long",
        expect: &[READY, MOVING, rejected(GC_REJ_LONG), READY, MOVING, STROKE],
        build: || {
            let s = Sensor::HAND;
            let cfg = GcConfig {
                max_stroke_us: 1_500_000,
                ..s.config()
            };
            Ok(Gen::new(s)
                .hold(1.5)
                .moves([150.0, 50.0, 0.0], [0.0, 0.0, 1.0], 90.0, 3.0)
                .hold(1.0)
                .moves([100.0, 0.0, 0.0], [0.0; 3], 0.0, 0.5)
                .hold(1.0)
                .done(cfg))
        },
    },
    // A clipped reading on the first sample of the move, inside the replay.
    Case {
        name: "clipped",
        expect: &[READY, MOVING, rejected(GC_REJ_CLIPPED), MOVING, STROKE],
        build: || {
            let s = Sensor::HAND;
            Ok(Gen::new(s)
                .hold(1.5)
                .clip(32700)
                .moves([100.0, 0.0, 0.0], [0.0; 3], 0.0, 0.6)
                .hold(1.0)
                .moves([100.0, 0.0, 0.0], [0.0; 3], 0.0, 0.6)
                .hold(1.0)
                .done(s.config()))
        },
    },
    // SHORT by duration (min_stroke_us 0.6 s here), then by extent (15 mm out, 10 back).
    Case {
        name: "short",
        expect: &[
            READY,
            MOVING,
            rejected(GC_REJ_SHORT),
            MOVING,
            rejected(GC_REJ_SHORT),
        ],
        build: || {
            let s = Sensor::HAND;
            let cfg = GcConfig {
                min_stroke_us: 600_000,
                ..s.config()
            };
            Ok(Gen::new(s)
                .hold(1.5)
                .moves([60.0, 0.0, 0.0], [0.0; 3], 0.0, 0.15)
                .hold(1.0)
                .moves([15.0, 0.0, 0.0], [0.0; 3], 0.0, 0.3)
                .moves([-10.0, 0.0, 0.0], [0.0; 3], 0.0, 0.3)
                .hold(1.0)
                .done(cfg))
        },
    },
    // The board ends at 0.5 m/s: DRIFT (max 0.3 m/s).
    Case {
        name: "drift",
        expect: &[READY, MOVING, rejected(GC_REJ_DRIFT)],
        build: || {
            let s = Sensor::HAND;
            Ok(Gen::new(s)
                .hold(1.5)
                .accelerate([0.5, 0.0, 0.0], 0.5)
                .hold(1.0)
                .done(s.config()))
        },
    },
    // 80 m/s at the end on the 16 g scale with max_v_end_mm_s 65535: DRIFT compares
    // before saturation, and it bounds the correction.
    Case {
        name: "drift-max",
        expect: &[READY, MOVING, rejected(GC_REJ_DRIFT)],
        build: || {
            let s = Sensor::with_scales(0x44, 0x44);
            let cfg = GcConfig {
                max_v_end_mm_s: u16::MAX,
                ..s.config()
            };
            Ok(Gen::new(s)
                .hold(1.5)
                .accelerate([80.0, 0.0, 0.0], 1.0)
                .hold(1.0)
                .done(cfg))
        },
    },
    // The MCU clock wraps inside the stroke-start replay, then inside a stroke.
    Case {
        name: "wrap-replay",
        expect: &[READY, MOVING, STROKE],
        build: || {
            let s = Sensor::HAND;
            let hold = Gen::new(s).count(1.5);
            Ok(Gen::new(s)
                .start_at(0u32.wrapping_sub((hold + 2) * DT_US) + 100)
                .hold(1.5)
                .moves([100.0, 0.0, 0.0], [0.0; 3], 0.0, 0.6)
                .hold(1.0)
                .done(s.config()))
        },
    },
    Case {
        name: "wrap-stroke",
        expect: &[READY, MOVING, STROKE],
        build: || {
            let s = Sensor::HAND;
            let hold = Gen::new(s).count(1.5);
            Ok(Gen::new(s)
                .start_at(0u32.wrapping_sub((hold + 30) * DT_US) + 100)
                .hold(1.5)
                .moves([100.0, 0.0, 0.0], [0.0; 3], 0.0, 0.6)
                .hold(1.0)
                .done(s.config()))
        },
    },
];

/// Where the snapshot must fall for `full-at-end`, with hold_n 11 (step 16).
const FULL_AT_END: std::ops::Range<usize> = 1008..1014;

/// The stroke's sample counter c at its snapshot: samples from s0 (the MOVING sample
/// less `GC_GUARD_N` + 1) to the sample whose window was the first quiet one (the STROKE
/// sample less hold_n - 1).
fn snapshot_c(outs: &[Out], hold_n: u16) -> Option<usize> {
    let k = outs.iter().find(|o| o.ev.kind == GC_EV_MOVING)?.k;
    let e = outs.iter().find(|o| o.ev.kind == GC_EV_STROKE)?.k;
    Some((e + 1 - usize::from(hold_n)) - (k - GUARD_N - 1))
}

/// Writes every case to `dir/<name>.gcv`; one line per case.
pub fn write_all(dir: &Path) -> anyhow::Result<()> {
    std::fs::create_dir_all(dir)?;
    for case in CASES {
        let (cfg, samples) = case.build().with_context(|| case.name)?;
        let outs = replay(&cfg, &samples)?;
        let kinds: Vec<(u8, u8)> = outs.iter().map(|o| (o.ev.kind, o.ev.reason)).collect();
        anyhow::ensure!(
            kinds == case.expect,
            "{}: events {:?}, meant {:?}",
            case.name,
            outs.iter()
                .map(|o| (o.k, o.ev.kind, o.ev.reason))
                .collect::<Vec<_>>(),
            case.expect
        );
        let bytes = vector::encode(&cfg, &samples)?;
        let file = dir.join(format!("{}.gcv", case.name));
        std::fs::write(&file, &bytes).with_context(|| format!("writing {}", file.display()))?;
        let (expect, _) = vector::check(&bytes)?;
        let ends: Vec<[i16; 3]> = outs
            .iter()
            .filter_map(|o| o.stroke.as_ref()?.1.last())
            .map(|p| [p.x_mm, p.y_mm, p.z_mm])
            .collect();
        println!(
            "{:<20} {:>5} samples {:>3} events  digest {:08x}  stroke ends (mm) {ends:?}",
            case.name,
            samples.len(),
            expect.events,
            expect.digest
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;

    fn dir() -> PathBuf {
        crate::vector::tests::vectors().join("synth")
    }

    fn case(name: &str) -> (GcConfig, Vec<GcSample>, Vec<Out>) {
        let c = CASES.iter().find(|c| c.name == name).expect("case");
        let (cfg, samples) = c.build().expect("build");
        let outs = replay(&cfg, &samples).expect("replay");
        (cfg, samples, outs)
    }

    fn strokes(outs: &[Out]) -> Vec<(GcStrokeInfo, Vec<GcPoint>)> {
        outs.iter().filter_map(|o| o.stroke.clone()).collect()
    }

    /// The last point of a stroke against the true move (mm, start frame).
    fn ends_near(points: &[GcPoint], truth: V3, tol: f64) {
        let p = points.last().expect("points");
        let got = [p.x_mm, p.y_mm, p.z_mm].map(f64::from);
        for i in 0..3 {
            assert!(
                (got[i] - truth[i]).abs() <= tol,
                "end {got:?}, true {truth:?} (+-{tol} mm)"
            );
        }
    }

    /// Gravity (ug, start frame) against the true one for a board turned by `q`.
    fn gravity_near(info: &GcStrokeInfo, q: Quat) {
        let truth = q.conj().rotate([0.0, 0.0, 1e6]);
        for i in 0..3 {
            let d = f64::from(info.gravity_ug[i]) - truth[i];
            assert!(
                d.abs() < 5000.0,
                "gravity {:?}, true {truth:?}",
                info.gravity_ug
            );
        }
    }

    /// Every case gives the events it is for, and the committed file is current.
    #[test]
    fn cases_are_current_and_give_their_events() {
        for c in CASES {
            let (cfg, samples) = c.build().expect(c.name);
            let outs = replay(&cfg, &samples).expect(c.name);
            let kinds: Vec<(u8, u8)> = outs.iter().map(|o| (o.ev.kind, o.ev.reason)).collect();
            assert_eq!(kinds, c.expect, "{}: events", c.name);

            let bytes = vector::encode(&cfg, &samples).expect(c.name);
            let file = dir().join(format!("{}.gcv", c.name));
            let committed = std::fs::read(&file)
                .unwrap_or_else(|e| panic!("{}: {e} (run `gesture-cli synth`)", file.display()));
            assert!(
                committed == bytes,
                "{}: differs from the generator (run `gesture-cli synth`)",
                c.name
            );
            let (stored, now) = vector::check(&committed).expect(c.name);
            assert_eq!(stored, now, "{}: the core changed", c.name);
        }
    }

    #[test]
    fn lines_end_where_they_went() {
        let (_, _, outs) = case("lines");
        let s = strokes(&outs);
        let truth = [[150.0, 0.0, 0.0], [0.0, -120.0, 0.0], [0.0, 0.0, 200.0]];
        for ((info, points), t) in s.iter().zip(truth) {
            ends_near(points, t, 5.0);
            gravity_near(info, Quat::ONE);
            assert!(info.v_end_mm_s < 30, "{info:?}");
        }
    }

    #[test]
    fn turn_in_place_stays_put() {
        let (_, _, outs) = case("turn-in-place");
        let end = outs.last().expect("event").ev;
        assert!(end.v_end_mm_s < 30, "{end:?}");
    }

    #[test]
    fn tilted_turning_is_in_the_start_frame() {
        let (_, _, outs) = case("tilted-turning");
        let s = strokes(&outs);
        let q0 = Quat::axis_angle([0.0, 1.0, 0.0], 30.0);
        let q1 = q0.mul(Quat::axis_angle([0.0, 0.0, 1.0], 60.0));
        for ((info, points), (q, d)) in s
            .iter()
            .zip([(q0, [100.0, 80.0, 0.0]), (q1, [-50.0, 0.0, 120.0])])
        {
            ends_near(points, q.conj().rotate(d), 6.0);
            gravity_near(info, q);
        }
    }

    #[test]
    fn long_path_compacts() {
        let (_, _, outs) = case("long-path");
        let (info, points) = &strokes(&outs)[0];
        assert_eq!(info.step, 8, "{info:?}");
        assert!(points.len() > 32 && points.len() <= 64);
        // The moves are in world axes, and the board starts level: the start frame.
        let truth = [100.0, 150.0, 0.0];
        ends_near(points, truth, 8.0);
    }

    #[test]
    fn full_at_end_compacts_before_the_snapshot_point() {
        let (cfg, _, outs) = case("full-at-end");
        // Full (64 points, step 16) from c = 1008 on; no compaction before the end, as
        // c stays below 1024; so step 32 can only come from `finish`.
        let c = snapshot_c(&outs, cfg.hold_n).expect("c");
        assert!(FULL_AT_END.contains(&c), "c {c}");
        let (info, points) = &strokes(&outs)[0];
        assert_eq!(
            (info.points, info.step, points.len()),
            (33, 32, 33),
            "{info:?}"
        );
    }

    #[test]
    fn scales_end_where_they_went() {
        for name in [
            "scale-2g-250dps",
            "scale-8g-1000dps",
            "scale-16g-2000dps",
            "scale-4g-125dps",
        ] {
            let (_, _, outs) = case(name);
            let (info, points) = &strokes(&outs)[0];
            ends_near(points, [80.0, -60.0, 70.0], 6.0);
            gravity_near(info, Quat::ONE);
        }
    }

    #[test]
    fn short_by_duration_then_by_extent() {
        let (cfg, _, outs) = case("short");
        let ends: Vec<&GcEvent> = outs
            .iter()
            .map(|o| &o.ev)
            .filter(|e| e.kind == GC_EV_REJECTED)
            .collect();
        assert!(ends[0].duration_us < cfg.min_stroke_us, "{:?}", ends[0]);
        assert!(ends[1].duration_us >= cfg.min_stroke_us, "{:?}", ends[1]);
    }

    #[test]
    fn drift_reports_the_leftover_velocity() {
        let (_, _, outs) = case("drift");
        let v = outs.last().expect("event").ev.v_end_mm_s;
        assert!((480..=520).contains(&v), "{v}");
        let (_, _, outs) = case("drift-max");
        assert_eq!(outs.last().expect("event").ev.v_end_mm_s, u16::MAX);
    }

    /// Where the clock wraps: inside the replay (s0 + 1 ..= the MOVING sample), or after.
    #[test]
    fn wraps_fall_where_intended() {
        for (name, in_replay) in [("wrap-replay", true), ("wrap-stroke", false)] {
            let (_, samples, outs) = case(name);
            let w = samples
                .windows(2)
                .position(|p| p[1].t_us < p[0].t_us)
                .expect("a wrap")
                + 1;
            let k = outs[1].k;
            assert_eq!(
                (k - GUARD_N..=k).contains(&w),
                in_replay,
                "{name}: wrap {w}, MOVING {k}"
            );
            if !in_replay {
                assert!(w > k && w < outs[2].k, "{name}: wrap {w}");
            }
            let (_, points) = &strokes(&outs)[0];
            ends_near(points, [100.0, 0.0, 0.0], 5.0);
        }
    }

    #[test]
    fn min_jerk_ends_at_rest() {
        let [s0, v0, _] = min_jerk(0.0);
        let [s1, v1, a1] = min_jerk(1.0);
        assert!(s0 == 0.0 && v0 == 0.0 && (s1 - 1.0).abs() < 1e-12 && v1.abs() < 1e-12);
        assert!(a1.abs() < 1e-9);
        let q = Quat::axis_angle([0.0, 0.0, 1.0], 90.0);
        let r = q.rotate([1.0, 0.0, 0.0]);
        assert!(r[0].abs() < 1e-12 && (r[1] - 1.0).abs() < 1e-12);
    }
}
