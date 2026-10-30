//! `path`: streaming float reference of the algorithm in `docs/core.md`, to check the
//! method on recordings before the integer spec is frozen. Same structure as `core/`
//! (ring, causal still windows, events, look-back replay at the start, snapshot rewind at
//! the end, bias from the config); f64 instead of the fixed-point formats, and every
//! sample kept instead of the decimated point buffer. `core/` replaces it.

use std::collections::VecDeque;
use std::fmt::Write as _;

use anyhow::{Context, bail};
use protocol::Sample;

use crate::calib::{Fit, Still};
use crate::stats::{self, Recording, V3, norm};

// docs/core.md, "Fixed constants".
const WIN_N: usize = 32;
const GUARD_N: usize = 10;
const GRAV_N: usize = 32;
const RING_N: usize = 48;

// docs/core.md, gc_config defaults that are not still thresholds.
const HOLD_N: u32 = 20;
const BIAS_GYRO_STD_DPS: f64 = 0.2;
const BIAS_ACCEL_STD_MG: f64 = 2.0;
const CLIP_LSB: i32 = 32700;
const MAX_DT_US: u64 = 14423;
const MIN_STROKE_US: u64 = 150_000;
const MAX_STROKE_US: u64 = 4_000_000;
const MIN_EXTENT_MM: f64 = 20.0;
const MAX_V_END_MM_S: f64 = 300.0;
/// Bias update rate: `bias += (mean - bias) / 64` per quiet sample.
const BIAS_RATE: f64 = 64.0;

/// `gc_config` in the session's LSB, as the caller would fill it.
pub struct Config {
    /// Sensitivities, ug/LSB and udps/LSB.
    sa: f64,
    sg: f64,
    offset_lsb: V3,
    gain: V3,
    bias_lsb: V3,
    still_gyro_std: f64,
    still_accel_std: f64,
    still_gyro_rate: f64,
    bias_gyro_std: f64,
    bias_accel_std: f64,
}

impl Config {
    /// `bias_dps`: the initial gyro bias; `fit`: accel calibration (none: offset 0, gain 1).
    pub fn new(
        rec: &Recording,
        fit: Option<&Fit>,
        bias_dps: V3,
        still: Still,
    ) -> anyhow::Result<Self> {
        let sa = f64::from(
            rec.hello
                .accel_ug_per_lsb()
                .context("accelerometer not configured")?,
        );
        let sg = f64::from(
            rec.hello
                .gyro_udps_per_lsb()
                .context("gyro not configured")?,
        );
        let gyro_lsb = |dps: f64| dps * 1e6 / sg;
        let accel_lsb = |mg: f64| mg * 1e3 / sa;
        Ok(Self {
            sa,
            sg,
            offset_lsb: fit.map_or([0.0; 3], |f| f.offset.map(|g| g * 1e6 / sa)),
            gain: fit.map_or([1.0; 3], |f| f.gain),
            bias_lsb: bias_dps.map(gyro_lsb),
            still_gyro_std: gyro_lsb(still.gyro_std_dps),
            still_accel_std: accel_lsb(still.accel_std_mg),
            still_gyro_rate: gyro_lsb(still.gyro_mean_dps),
            bias_gyro_std: gyro_lsb(BIAS_GYRO_STD_DPS),
            bias_accel_std: accel_lsb(BIAS_ACCEL_STD_MG),
        })
    }

    /// Calibrated accel, ug.
    fn accel_ug(&self, raw: [i16; 3]) -> V3 {
        std::array::from_fn(|i| (f64::from(raw[i]) - self.offset_lsb[i]) * self.gain[i] * self.sa)
    }
}

type Quat = [f64; 4];

fn quat_mul(a: Quat, b: Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// Body vector into the start frame.
fn rotate(q: Quat, v: V3) -> V3 {
    let r = quat_mul(
        quat_mul(q, [0.0, v[0], v[1], v[2]]),
        [q[0], -q[1], -q[2], -q[3]],
    );
    [r[1], r[2], r[3]]
}

/// Turns `q` by body rates `w` (rad/s) for `dt` (s).
fn turn(q: Quat, w: V3, dt: f64) -> Quat {
    let angle = norm(w) * dt;
    if angle == 0.0 {
        return q;
    }
    let (s, c) = (angle / 2.0).sin_cos();
    let k = s / norm(w);
    quat_mul(q, [c, w[0] * k, w[1] * k, w[2] * k])
}

fn dot(a: V3, b: V3) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}

#[derive(Clone, Copy)]
struct Entry {
    dt_us: u64,
    gyro: [i16; 3],
    accel: [i16; 3],
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum State {
    Warmup,
    Still,
    Moving,
}

#[derive(Clone, Copy, Debug)]
pub enum Reject {
    Short,
    Long,
    Clipped,
    TimeGap,
    Drift,
}

/// What a finished stroke looks like (`gc_stroke_info` plus diagnostics).
pub struct Done {
    duration_us: u64,
    /// End position after the zero-velocity correction, start frame, mm.
    end_mm: V3,
    extent_mm: f64,
    v_end_mm_s: f64,
    gravity_ug: V3,
    turned_deg: f64,
    /// Angle between the start hold's gravity and the end hold's, turned back into the
    /// start frame: the tilt error the stroke ended with.
    tilt_error_deg: f64,
}

pub enum Event {
    Ready,
    Moving,
    Stroke(Done),
    Rejected(Reject, Option<Done>),
}

#[derive(Clone, Copy)]
struct Snap {
    t_us: u64,
    q: Quat,
    v: V3,
    p: V3,
    clipped: bool,
}

struct Stroke {
    bias_lsb: V3,
    gravity_ug: V3,
    q: Quat,
    /// m/s and m.
    v: V3,
    p: V3,
    /// Time since s0, us.
    t_us: u64,
    clipped: bool,
    /// (t since s0, uncorrected position m), one per sample.
    points: Vec<(u64, V3)>,
    snap: Option<Snap>,
}

impl Stroke {
    fn integrate(&mut self, cfg: &Config, e: &Entry) {
        let dt = e.dt_us as f64 / 1e6;
        let w: V3 = std::array::from_fn(|i| {
            ((f64::from(e.gyro[i]) - self.bias_lsb[i]) * cfg.sg / 1e6).to_radians()
        });
        self.q = turn(self.q, w, dt);
        let f = rotate(self.q, cfg.accel_ug(e.accel));
        for (i, fi) in f.iter().enumerate() {
            self.v[i] += (fi - self.gravity_ug[i]) * 1e-6 * stats::G * dt;
            self.p[i] += self.v[i] * dt;
        }
        self.t_us += e.dt_us;
        self.clipped |= e
            .gyro
            .iter()
            .chain(&e.accel)
            .any(|&x| i32::from(x).abs() >= CLIP_LSB);
        self.points.push((self.t_us, self.p));
    }

    /// Ends at the snapshot. `end_gravity_ug`: the end hold's accel, body frame.
    fn finish(&self, end_gravity_ug: V3) -> Result<Done, (Reject, Done)> {
        let snap = self.snap.expect("a confirmed hold has a snapshot");
        let t_end = snap.t_us as f64;
        let correct = |t: u64, p: V3| -> V3 {
            let t = t as f64;
            std::array::from_fn(|i| p[i] - snap.v[i] * (t / 1e6) * (t / 1e6) / (2.0 * t_end / 1e6))
        };
        let extent_mm = self
            .points
            .iter()
            .filter(|(t, _)| *t <= snap.t_us)
            .map(|&(t, p)| correct(t, p).iter().fold(0.0, |m: f64, x| m.max(x.abs())))
            .fold(0.0, f64::max)
            * 1000.0;
        let end = correct(snap.t_us, snap.p);
        let up = rotate(snap.q, end_gravity_ug);
        let cos = dot(up, self.gravity_ug) / (norm(up) * norm(self.gravity_ug));
        let done = Done {
            duration_us: snap.t_us,
            end_mm: end.map(|x| x * 1000.0),
            extent_mm,
            v_end_mm_s: snap.v.iter().fold(0.0, |m: f64, x| m.max(x.abs())) * 1000.0,
            gravity_ug: self.gravity_ug,
            turned_deg: 2.0 * snap.q[0].clamp(-1.0, 1.0).acos().to_degrees(),
            tilt_error_deg: cos.clamp(-1.0, 1.0).acos().to_degrees(),
        };
        let reject = if snap.clipped {
            Some(Reject::Clipped)
        } else if done.duration_us < MIN_STROKE_US {
            Some(Reject::Short)
        } else if done.v_end_mm_s > MAX_V_END_MM_S {
            Some(Reject::Drift)
        } else if done.extent_mm < MIN_EXTENT_MM {
            Some(Reject::Short)
        } else {
            None
        };
        match reject {
            Some(r) => Err((r, done)),
            None => Ok(done),
        }
    }
}

pub struct Core<'a> {
    cfg: &'a Config,
    state: State,
    ring: VecDeque<Entry>,
    last_t: Option<u64>,
    streak: u32,
    bias_lsb: V3,
    stroke: Option<Stroke>,
}

impl<'a> Core<'a> {
    pub fn new(cfg: &'a Config) -> Self {
        Self {
            cfg,
            state: State::Warmup,
            ring: VecDeque::with_capacity(RING_N),
            last_t: None,
            streak: 0,
            bias_lsb: cfg.bias_lsb,
            stroke: None,
        }
    }

    /// Per-axis sums over the newest `n` ring entries ending `skip` entries back.
    fn sums(&self, n: usize, skip: usize, f: fn(&Entry) -> [i16; 3]) -> (V3, V3) {
        let end = self.ring.len() - skip;
        let mut s = [0.0; 3];
        let mut q = [0.0; 3];
        for e in self.ring.range(end - n..end) {
            for i in 0..3 {
                let x = f64::from(f(e)[i]);
                s[i] += x;
                q[i] += x * x;
            }
        }
        (s, q)
    }

    /// `N * Q - S^2 < N^2 * th^2` on every axis: the window's std is below `th`.
    fn spread_ok((s, q): (V3, V3), th: f64) -> bool {
        let n = WIN_N as f64;
        (0..3).all(|i| n * q[i] - s[i] * s[i] < n * n * th * th)
    }

    pub fn feed(&mut self, t_us: u64, s: &Sample) -> Option<Event> {
        let mut event = None;
        // 1. Time.
        let dt_us = self.last_t.map(|last| t_us.wrapping_sub(last));
        self.last_t = Some(t_us);
        if dt_us.is_none_or(|dt| dt == 0 || dt > MAX_DT_US) {
            self.ring.clear();
            self.streak = 0;
            if self.state == State::Moving {
                event = Some(Event::Rejected(Reject::TimeGap, None));
                self.stroke = None;
            }
            self.state = State::Warmup;
        }
        // 2. Ring.
        if self.ring.len() == RING_N {
            self.ring.pop_front();
        }
        let entry = Entry {
            dt_us: dt_us.unwrap_or(0),
            gyro: s.gyro,
            accel: s.accel,
        };
        self.ring.push_back(entry);
        // 3. Still window.
        let quiet = self.ring.len() >= WIN_N && {
            let (g, a) = (
                self.sums(WIN_N, 0, |e| e.gyro),
                self.sums(WIN_N, 0, |e| e.accel),
            );
            let n = WIN_N as f64;
            let rate_ok = (0..3)
                .all(|i| (g.0[i] - n * self.bias_lsb[i]).abs() < n * self.cfg.still_gyro_rate);
            let quiet = Self::spread_ok(g, self.cfg.still_gyro_std)
                && Self::spread_ok(a, self.cfg.still_accel_std)
                && rate_ok;
            // 4. Bias update.
            if quiet
                && self.state == State::Still
                && Self::spread_ok(g, self.cfg.bias_gyro_std)
                && Self::spread_ok(a, self.cfg.bias_accel_std)
            {
                for i in 0..3 {
                    self.bias_lsb[i] += (g.0[i] / n - self.bias_lsb[i]) / BIAS_RATE;
                }
            }
            quiet
        };
        self.streak = if quiet { self.streak + 1 } else { 0 };
        // 5. State.
        match self.state {
            State::Warmup if self.streak >= HOLD_N => {
                self.state = State::Still;
                event = Some(Event::Ready);
            }
            State::Still if !quiet => {
                self.start();
                self.state = State::Moving;
                event = Some(Event::Moving);
            }
            State::Moving => event = self.moving(&entry),
            _ => {}
        }
        event
    }

    fn start(&mut self) {
        let (sa, _) = self.sums(GRAV_N, GUARD_N + 1, |e| e.accel);
        // Mean raw accel of the gravity window, then calibrated (affine, so the same).
        let gravity_ug = std::array::from_fn(|i| {
            (sa[i] / GRAV_N as f64 - self.cfg.offset_lsb[i]) * self.cfg.gain[i] * self.cfg.sa
        });
        let mut stroke = Stroke {
            bias_lsb: self.bias_lsb,
            gravity_ug,
            q: [1.0, 0.0, 0.0, 0.0],
            v: [0.0; 3],
            p: [0.0; 3],
            t_us: 0,
            clipped: false,
            points: vec![(0, [0.0; 3])],
            snap: None,
        };
        let replay: Vec<Entry> = self
            .ring
            .range(self.ring.len() - GUARD_N - 1..)
            .copied()
            .collect();
        for e in &replay {
            stroke.integrate(self.cfg, e);
        }
        self.stroke = Some(stroke);
    }

    fn moving(&mut self, entry: &Entry) -> Option<Event> {
        let stroke = self.stroke.as_mut().expect("moving has a stroke");
        stroke.integrate(self.cfg, entry);
        if stroke.t_us > MAX_STROKE_US {
            self.stroke = None;
            self.state = State::Warmup;
            return Some(Event::Rejected(Reject::Long, None));
        }
        match self.streak {
            0 => stroke.snap = None,
            1 => {
                stroke.snap = Some(Snap {
                    t_us: stroke.t_us,
                    q: stroke.q,
                    v: stroke.v,
                    p: stroke.p,
                    clipped: stroke.clipped,
                });
            }
            n if n >= HOLD_N => {
                let (sa, _) = self.sums(WIN_N, 0, |e| e.accel);
                let end_gravity = self.cfg.accel_ug(
                    // Mean raw accel of the window, rounded only for the calibration call.
                    sa.map(|x| (x / WIN_N as f64).round() as i16),
                );
                let result = self
                    .stroke
                    .take()
                    .expect("moving has a stroke")
                    .finish(end_gravity);
                self.state = State::Still;
                return Some(match result {
                    Ok(done) => Event::Stroke(done),
                    Err((r, done)) => Event::Rejected(r, Some(done)),
                });
            }
            _ => {}
        }
        None
    }
}

pub fn report(rec: &Recording, cfg: &Config) -> anyhow::Result<String> {
    if rec.samples.is_empty() {
        bail!("no samples");
    }
    let t0 = rec.samples[0].0;
    let mut core = Core::new(cfg);
    let mut out = String::new();
    writeln!(
        out,
        "  t, s  event        dur, s   end x/y/z, mm (start frame)   |d|, mm  up, mm  horiz  extent  v_end mm/s  turned  tilt err"
    )?;
    let (mut strokes, mut rejected) = (0, 0);
    for (t, s) in &rec.samples {
        let Some(event) = core.feed(*t, s) else {
            continue;
        };
        let at = (t - t0) as f64 / 1e6;
        let (name, done) = match event {
            Event::Ready => ("READY".to_owned(), None),
            Event::Moving => ("MOVING".to_owned(), None),
            Event::Stroke(d) => {
                strokes += 1;
                ("STROKE".to_owned(), Some(d))
            }
            Event::Rejected(r, d) => {
                rejected += 1;
                (format!("REJ {r:?}").to_uppercase(), d)
            }
        };
        match done {
            None => writeln!(out, "{at:>6.2}  {name}")?,
            Some(d) => {
                let up_dir = d.gravity_ug.map(|x| x / norm(d.gravity_ug));
                let up = dot(d.end_mm, up_dir);
                let horiz = norm(std::array::from_fn(|i| d.end_mm[i] - up * up_dir[i]));
                writeln!(
                    out,
                    "{at:>6.2}  {name:<12} {:>5.2}  {:>7.1} {:>7.1} {:>7.1}        {:>7.1} {:>7.1} {:>6.1} {:>7.1}  {:>9.1}  {:>5.1}  {:>5.2} deg",
                    d.duration_us as f64 / 1e6,
                    d.end_mm[0],
                    d.end_mm[1],
                    d.end_mm[2],
                    norm(d.end_mm),
                    up,
                    horiz,
                    d.extent_mm,
                    d.v_end_mm_s,
                    d.turned_deg,
                    d.tilt_error_deg
                )?;
            }
        }
    }
    writeln!(out, "\n{strokes} strokes, {rejected} rejected")?;
    Ok(out)
}
