//! `stats`: numbers from a recorded session, for the rest runs: bias, noise, drift, and how
//! far a stroke would drift on noise alone. Physical units here are for reading only;
//! `core/` works in raw LSB.

use std::fmt::Write as _;

use protocol::Sample;
use session::recording::{G, Recording, Scaled, V3, mean, median, norm, scaled, std_dev};

/// Picks the gyro or accel triple of a sample.
type Axes = fn(&Sample) -> [i16; 3];

/// Rest calibration window before a simulated stroke, s (the still hold).
const HOLD_S: f64 = 0.3;
/// Simulated stroke lengths, s.
const STROKES_S: [f64; 3] = [0.5, 1.0, 2.0];

/// Drift of one simulated stroke on a board that never moved: bias taken from the hold
/// before it, accel integrated twice in the body frame (no rotation at rest), and the
/// zero-velocity correction applied at the end. Returns (position before, after), m, and
/// gyro angle error, deg.
fn simulate(
    t_s: &[f64],
    gyro: &[V3],
    accel_ms2: &[V3],
    start: usize,
    len: usize,
) -> (f64, f64, f64) {
    let hold = |v: &[V3]| {
        let first = t_s.partition_point(|&t| t < t_s[start] - HOLD_S);
        mean(&v[first..start])
    };
    let (gyro_bias, accel_bias) = (hold(gyro), hold(accel_ms2));
    let mut vel = [0.0; 3];
    let mut pos = [0.0; 3];
    let mut angle = [0.0; 3];
    for k in start + 1..start + len {
        let dt = t_s[k] - t_s[k - 1];
        for i in 0..3 {
            vel[i] += (accel_ms2[k][i] - accel_bias[i]) * dt;
            pos[i] += vel[i] * dt;
            angle[i] += (gyro[k][i] - gyro_bias[i]) * dt;
        }
    }
    let duration = t_s[start + len - 1] - t_s[start];
    // Velocity drift taken as growing linearly: its integral is v_end * T / 2.
    let corrected: V3 = std::array::from_fn(|i| pos[i] - vel[i] * duration / 2.0);
    (norm(pos), norm(corrected), norm(angle))
}

pub fn report(rec: &Recording) -> anyhow::Result<String> {
    let Scaled {
        mg_per_lsb,
        dps_per_lsb,
        period_us,
        t_s,
        gyro,
        accel,
    } = scaled(rec)?;
    let raw_range = |f: Axes, i: usize| {
        let it = rec.samples.iter().map(|(_, s)| f(s)[i]);
        (it.clone().min().unwrap_or(0), it.max().unwrap_or(0))
    };
    let duration = *t_s.last().unwrap_or(&0.0);

    let mut out = String::new();
    writeln!(
        out,
        "{:.1} s, {} samples, accel {mg_per_lsb} mg/LSB, gyro {} mdps/LSB",
        duration,
        rec.samples.len(),
        dps_per_lsb * 1000.0
    )?;

    // First and last 10 s (or a third of the session): does the bias wander?
    let edge = ((10.0 * 1e6 / f64::from(period_us)) as usize)
        .min(rec.samples.len() / 3)
        .max(1);
    let n = rec.samples.len();
    writeln!(
        out,
        "\n            mean      std    min..max LSB    first {:.0} s  last {:.0} s",
        edge as f64 * f64::from(period_us) / 1e6,
        edge as f64 * f64::from(period_us) / 1e6
    )?;
    let rows: [(&str, &str, &[V3], Axes); 2] = [
        ("gyro", "dps", &gyro, |s| s.gyro),
        ("accel", "mg", &accel, |s| s.accel),
    ];
    for (name, unit, v, f) in rows {
        let (m, sd) = (mean(v), std_dev(v));
        let (head, tail) = (mean(&v[..edge]), mean(&v[n - edge..]));
        for (i, axis) in ["x", "y", "z"].iter().enumerate() {
            let (lo, hi) = raw_range(f, i);
            writeln!(
                out,
                "{name:>5} {axis} {:>9.3} {:>8.3} {:>7}..{:<7} {:>9.3} {:>9.3}  {unit}",
                m[i], sd[i], lo, hi, head[i], tail[i]
            )?;
        }
    }
    let a = mean(&accel);
    let tilt = (a[2] / norm(a)).clamp(-1.0, 1.0).acos().to_degrees();
    writeln!(
        out,
        "\n|accel| {:.2} mg, z axis {tilt:.2} deg from gravity",
        norm(a)
    )?;
    // Near clipping: a reading within 10% of the end of the int16 range (the chip may
    // saturate a few LSB short of it: peaks run reached 32764); listed per second.
    const NEAR_LIMIT: i32 = i16::MAX as i32 * 9 / 10;
    let rows: [(&str, Axes); 2] = [("gyro", |s| s.gyro), ("accel", |s| s.accel)];
    for (name, f) in rows {
        let clipped: Vec<f64> = rec
            .samples
            .iter()
            .zip(&t_s)
            .filter(|((_, s), _)| f(s).iter().any(|&x| i32::from(x).abs() >= NEAR_LIMIT))
            .map(|(_, &t)| t)
            .collect();
        let mut seconds: Vec<u64> = clipped.iter().map(|&t| t as u64).collect();
        seconds.dedup();
        writeln!(
            out,
            "{name} >= 90% of range: {} samples{}",
            clipped.len(),
            if seconds.is_empty() {
                String::new()
            } else {
                format!(", in seconds {seconds:?}")
            }
        )?;
    }
    let gm = mean(&gyro);
    writeln!(
        out,
        "gyro bias if not removed: {:.2} deg after 1 s",
        norm(gm)
    )?;

    // Simulated strokes on this still data.
    let accel_ms2: Vec<V3> = accel.iter().map(|v| v.map(|x| x / 1000.0 * G)).collect();
    let hold = (HOLD_S * 1e6 / f64::from(period_us)).ceil() as usize + 1;
    writeln!(
        out,
        "\nstroke drift on noise alone (bias from the {HOLD_S} s hold before, every 0.5 s):"
    )?;
    writeln!(
        out,
        "  length   |p| raw mean/max     |p| corrected mean/max   angle mean/max"
    )?;
    for len_s in STROKES_S {
        let len = (len_s * 1e6 / f64::from(period_us)).round() as usize + 1;
        let step = (0.5 * 1e6 / f64::from(period_us)) as usize;
        let runs: Vec<_> = (hold..n.saturating_sub(len))
            .step_by(step.max(1))
            .map(|start| simulate(&t_s, &gyro, &accel_ms2, start, len))
            .collect();
        if runs.is_empty() {
            continue;
        }
        let stat = |f: fn(&(f64, f64, f64)) -> f64| {
            let vals: Vec<f64> = runs.iter().map(f).collect();
            let mean = vals.iter().sum::<f64>() / vals.len() as f64;
            (mean, vals.iter().copied().fold(0.0, f64::max))
        };
        let (raw, cor, ang) = (stat(|r| r.0), stat(|r| r.1), stat(|r| r.2));
        writeln!(
            out,
            "  {len_s:.1} s   {:>6.1} / {:>6.1} mm      {:>6.2} / {:>6.2} mm        {:.3} / {:.3} deg  ({} runs)",
            raw.0 * 1000.0,
            raw.1 * 1000.0,
            cor.0 * 1000.0,
            cor.1 * 1000.0,
            ang.0,
            ang.1,
            runs.len()
        )?;
    }
    Ok(out)
}

/// Per window: spread of gyro and accel (largest axis std), gyro rate after removing the
/// session bias (median of the window means, so the still majority sets it), and |accel|.
/// Shows where the board was still and how still.
pub fn timeline(rec: &Recording, window_s: f64) -> anyhow::Result<String> {
    let s = scaled(rec)?;
    let window = ((window_s * 1e6 / f64::from(s.period_us)).round() as usize).max(2);
    let starts: Vec<usize> = (0..s.gyro.len().saturating_sub(window - 1))
        .step_by(window)
        .collect();
    let gyro_means: Vec<V3> = starts
        .iter()
        .map(|&k| mean(&s.gyro[k..k + window]))
        .collect();
    let bias = median(&gyro_means);
    let max = |v: V3| v.into_iter().fold(0.0, f64::max);

    let mut out = String::new();
    writeln!(
        out,
        "window {window_s} s, gyro bias (median) {:.2} {:.2} {:.2} dps\n",
        bias[0], bias[1], bias[2]
    )?;
    writeln!(out, "   t, s   gyro std  accel std   |gyro-bias|   |accel|")?;
    for (&k, m) in starts.iter().zip(&gyro_means) {
        let rate = norm(std::array::from_fn(|i| m[i] - bias[i]));
        writeln!(
            out,
            "{:>7.2} {:>8.2}   {:>7.1}   {:>9.2}   {:>8.1}",
            s.t_s[k],
            max(std_dev(&s.gyro[k..k + window])),
            max(std_dev(&s.accel[k..k + window])),
            rate,
            norm(mean(&s.accel[k..k + window]))
        )?;
    }
    writeln!(out, "\n(dps, mg)")?;
    Ok(out)
}
