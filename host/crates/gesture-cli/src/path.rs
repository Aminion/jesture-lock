//! `path`: float reference of the stroke path, for designing the integer spec in
//! `docs/core.md`. `core/` replaces it; it only exists to try the method on recordings.
//!
//! Per stroke (hold -> move -> hold): gyro bias and gravity from the hold before, the gyro
//! turns the board from there, accel minus gravity in the start frame integrated twice, and
//! the end hold's zero velocity removes a linearly growing velocity error.

use std::fmt::Write as _;

use anyhow::bail;

use crate::calib::{self, Fit, Still, WINDOW_S};
use crate::stats::{self, Recording, V3, mean, norm};

/// Hold used for bias and gravity, s, ending `GUARD_S` before the detected move (the
/// detector notices a soft start late; seen in the hand-hold run).
const HOLD_S: f64 = 0.3;
const GUARD_S: f64 = 0.1;
/// A still run shorter than this is a pause inside the move, not a hold: slow table slides
/// (ruler-30 run) dip below the still thresholds for a window or two mid-slide.
const MIN_HOLD_S: f64 = 0.5;

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

/// Turns `q` by body rates `w` (rad/s) for `dt`.
fn turn(q: Quat, w: V3, dt: f64) -> Quat {
    let angle = norm(w) * dt;
    if angle == 0.0 {
        return q;
    }
    let (s, c) = (angle / 2.0).sin_cos();
    let k = s / norm(w);
    quat_mul(q, [c, w[0] * k, w[1] * k, w[2] * k])
}

struct Stroke {
    t0: f64,
    duration: f64,
    /// Position at the end before and after the zero-velocity correction, mm.
    raw: V3,
    corrected: V3,
    /// Velocity left at the end hold, mm/s.
    v_end: V3,
    /// Peak corrected speed, m/s.
    peak: f64,
    /// Board rotation from start to end, deg.
    turned: f64,
    /// Up (along the start hold's gravity) and horizontal parts of `corrected`, mm.
    up: f64,
    horizontal: f64,
    /// Angle between the start hold's gravity and the end hold's, turned back into the
    /// start frame by the gyro: the tilt error the stroke ended with, deg.
    tilt_error: f64,
}

/// `hold`: the hold before (bias, gravity); `end`: first sample of the next hold, whose
/// first `end_n` samples check the tilt.
fn stroke(
    s: &stats::Scaled,
    accel: &[V3],
    hold: (usize, usize),
    end: usize,
    end_n: usize,
    fixed_bias: Option<V3>,
) -> Stroke {
    let bias = fixed_bias.unwrap_or_else(|| mean(&s.gyro[hold.0..hold.1]));
    let gravity = mean(&accel[hold.0..hold.1]);
    let start = hold.1 - 1;
    let mut q: Quat = [1.0, 0.0, 0.0, 0.0];
    let (mut v, mut p) = ([0.0; 3], [0.0; 3]);
    let mut trace = Vec::new();
    let steps = s.t_s[start..=end]
        .windows(2)
        .zip(&s.gyro[start + 1..=end])
        .zip(&accel[start + 1..=end]);
    for ((t, g), a) in steps {
        let dt = t[1] - t[0];
        let w: V3 = std::array::from_fn(|i| (g[i] - bias[i]).to_radians());
        q = turn(q, w, dt);
        let f = rotate(q, *a);
        for i in 0..3 {
            v[i] += (f[i] - gravity[i]) / 1000.0 * stats::G * dt;
            p[i] += v[i] * dt;
        }
        trace.push((t[1] - s.t_s[start], v));
    }
    let end_gravity = rotate(q, mean(&accel[end..(end + end_n).min(accel.len())]));
    let cos = (0..3).map(|i| end_gravity[i] * gravity[i]).sum::<f64>()
        / (norm(end_gravity) * norm(gravity));
    let duration = s.t_s[end] - s.t_s[start];
    let corrected: V3 = std::array::from_fn(|i| (p[i] - v[i] * duration / 2.0) * 1000.0);
    let up_dir = gravity.map(|x| x / norm(gravity));
    let up = (0..3).map(|i| corrected[i] * up_dir[i]).sum::<f64>();
    let peak = trace
        .iter()
        .map(|(t, vk)| norm(std::array::from_fn(|i| vk[i] - v[i] * t / duration)))
        .fold(0.0, f64::max);
    Stroke {
        t0: s.t_s[start],
        duration,
        raw: p.map(|x| x * 1000.0),
        corrected,
        v_end: v.map(|x| x * 1000.0),
        peak,
        turned: 2.0 * q[0].clamp(-1.0, 1.0).acos().to_degrees(),
        up,
        horizontal: norm(std::array::from_fn(|i| corrected[i] - up * up_dir[i])),
        tilt_error: cos.clamp(-1.0, 1.0).acos().to_degrees(),
    }
}

/// `session_bias`: integrate with the session's median gyro instead of each hold's mean
/// (a hand hold drifts by a few dps, which the hold mean takes for bias).
pub fn report(
    rec: &Recording,
    fit: Option<&Fit>,
    still: Still,
    session_bias: bool,
) -> anyhow::Result<String> {
    let s = stats::scaled(rec)?;
    let period_s = f64::from(s.period_us) / 1e6;
    let window = ((WINDOW_S / period_s).round() as usize).max(2);
    let accel: Vec<V3> = s
        .accel
        .iter()
        .map(|a| fit.map_or(*a, |f| f.apply(a.map(|x| x / 1000.0)).map(|x| x * 1000.0)))
        .collect();
    // Detection uses the session's median gyro (mostly still); each stroke then takes its
    // own bias from its hold.
    let bias: V3 = std::array::from_fn(|i| {
        let mut v: Vec<f64> = s.gyro.iter().map(|g| g[i]).collect();
        v.sort_by(f64::total_cmp);
        v[v.len() / 2]
    });
    let min_hold = (MIN_HOLD_S / period_s).round() as usize;
    let still: Vec<_> = calib::runs(&calib::still_mask(
        &s.gyro,
        &accel,
        window,
        Some(bias),
        still,
    ))
    .into_iter()
    .filter(|r| r.1 - r.0 >= min_hold)
    .collect();
    let (hold_n, guard_n) = (
        (HOLD_S / period_s).round() as usize,
        (GUARD_S / period_s).round() as usize,
    );
    let strokes: Vec<Stroke> = still
        .windows(2)
        .filter(|w| w[0].1 - w[0].0 >= hold_n + guard_n)
        .map(|w| {
            let hold_end = w[0].1 - guard_n;
            stroke(
                &s,
                &accel,
                (hold_end - hold_n, hold_end),
                w[1].0,
                hold_n,
                session_bias.then_some(bias),
            )
        })
        .collect();
    if strokes.is_empty() {
        bail!("no stroke found (hold -> move -> hold)");
    }

    let mut out = String::new();
    writeln!(
        out,
        "{} strokes, accel {}, gyro bias {}, hold {HOLD_S} s ending {GUARD_S} s before the move\n",
        strokes.len(),
        if fit.is_some() {
            "calibrated"
        } else {
            "raw (no --calib)"
        },
        if session_bias {
            "of the session"
        } else {
            "of each hold"
        },
    )?;
    writeln!(
        out,
        "  start   dur   corrected x/y/z, mm       |d|, mm  up, mm  horiz  raw |d|  v_end, mm/s  peak m/s  turned  tilt err"
    )?;
    for k in &strokes {
        writeln!(
            out,
            "{:>6.2} {:>5.2}  {:>7.1} {:>7.1} {:>7.1}  {:>7.1} {:>7.1} {:>6.1}  {:>7.1}  {:>9.1}  {:>8.2}  {:>5.1}  {:>5.2} deg",
            k.t0,
            k.duration,
            k.corrected[0],
            k.corrected[1],
            k.corrected[2],
            norm(k.corrected),
            k.up,
            k.horizontal,
            norm(k.raw),
            norm(k.v_end),
            k.peak,
            k.turned,
            k.tilt_error
        )?;
    }
    Ok(out)
}
