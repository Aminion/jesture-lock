//! `calib`: accelerometer offset and scale per axis from a tumble session (the board held
//! still in many orientations). Every still segment must read 1 g, so the segment means lie
//! on an axis-aligned ellipsoid; its center is the offset and its radii the scales.

use std::fmt::Write as _;

use anyhow::bail;

use crate::stats::{self, Recording, V3, mean, norm, std_dev};

/// Still-detection window, s.
pub const WINDOW_S: f64 = 0.25;
/// Still thresholds: per-axis std over the window, and gyro mean distance from the bias
/// (rules out a slow steady turn).
#[derive(Clone, Copy, Debug)]
pub struct Still {
    pub gyro_std_dps: f64,
    pub accel_std_mg: f64,
    pub gyro_mean_dps: f64,
}

/// Hand-held: holds show 1-2.3 dps and 3-13 mg (tumble and hand-hold runs). Tremor
/// averages out over a segment.
pub const HAND: Still = Still {
    gyro_std_dps: 3.0,
    accel_std_mg: 15.0,
    gyro_mean_dps: 4.0,
};

/// On a table: rest is 0.05 dps and 0.5-0.8 mg, while a slow slide is 9-30 mg with the
/// gyro barely moving (ruler-30 run), so the hand thresholds miss its slow end.
pub const TABLE: Still = Still {
    gyro_std_dps: 0.5,
    accel_std_mg: 3.0,
    gyro_mean_dps: 1.0,
};
/// Shortest still segment used, s; trimmed by `TRIM_S` at both ends.
const MIN_SEGMENT_S: f64 = 1.0;
const TRIM_S: f64 = 0.1;
/// Unknowns of the fit: three squares and three linear terms.
const UNKNOWNS: usize = 6;

struct Segment {
    t: (f64, f64),
    /// Mean accel, g.
    accel: V3,
    /// Mean gyro, dps.
    gyro: V3,
}

/// Offset (g) and gain per axis: true = (raw - offset) * gain.
pub struct Fit {
    offset: V3,
    gain: V3,
}

impl Fit {
    pub fn apply(&self, a: V3) -> V3 {
        std::array::from_fn(|i| (a[i] - self.offset[i]) * self.gain[i])
    }
}

/// Flags each window start where the board is still: low spread, and gyro near `bias`
/// if given.
pub fn still_mask(
    gyro: &[V3],
    accel: &[V3],
    window: usize,
    bias: Option<V3>,
    th: Still,
) -> Vec<bool> {
    let n = gyro.len();
    let mut still = vec![false; n];
    for start in 0..n.saturating_sub(window) {
        let g = &gyro[start..start + window];
        let a = &accel[start..start + window];
        let quiet = std_dev(g).iter().all(|&s| s < th.gyro_std_dps)
            && std_dev(a).iter().all(|&s| s < th.accel_std_mg);
        let near_bias = bias.is_none_or(|b| {
            let m = mean(g);
            (0..3).all(|i| (m[i] - b[i]).abs() < th.gyro_mean_dps)
        });
        if quiet && near_bias {
            still[start..start + window].fill(true);
        }
    }
    still
}

/// Runs of `true` as index ranges.
pub fn runs(mask: &[bool]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut start = None;
    for (i, &m) in mask.iter().chain([&false]).enumerate() {
        match (m, start) {
            (true, None) => start = Some(i),
            (false, Some(s)) => {
                out.push((s, i));
                start = None;
            }
            _ => {}
        }
    }
    out
}

/// Solves `m x = b` by Gaussian elimination with partial pivoting; None if singular.
fn solve<const N: usize>(mut m: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
    for col in 0..N {
        let pivot = (col..N).max_by(|&i, &j| m[i][col].abs().total_cmp(&m[j][col].abs()))?;
        if m[pivot][col].abs() < 1e-12 {
            return None;
        }
        m.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..N {
            let f = m[row][col] / m[col][col];
            let (top, bottom) = m.split_at_mut(row);
            for (a, p) in bottom[0][col..].iter_mut().zip(&top[col][col..]) {
                *a -= f * p;
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0; N];
    for row in (0..N).rev() {
        let s: f64 = (row + 1..N).map(|k| m[row][k] * x[k]).sum();
        x[row] = (b[row] - s) / m[row][row];
    }
    Some(x)
}

/// Least squares for A x^2 + B y^2 + C z^2 + D x + E y + F z = 1 over the segment means
/// (g), then offset and gain so that the corrected means have length 1.
fn fit(points: &[V3]) -> Option<Fit> {
    let mut m = [[0.0; UNKNOWNS]; UNKNOWNS];
    let mut b = [0.0; UNKNOWNS];
    for p in points {
        let row = [p[0] * p[0], p[1] * p[1], p[2] * p[2], p[0], p[1], p[2]];
        for i in 0..UNKNOWNS {
            for j in 0..UNKNOWNS {
                m[i][j] += row[i] * row[j];
            }
            b[i] += row[i];
        }
    }
    let x = solve(m, b)?;
    let sq: V3 = [x[0], x[1], x[2]];
    if sq.iter().any(|&a| a <= 0.0) {
        return None;
    }
    // A (x - o)^2 + ... = 1 + A o^2 + ...; the right side R scales A..C to gain^2.
    let offset: V3 = std::array::from_fn(|i| -x[i + 3] / (2.0 * sq[i]));
    let r = 1.0 + (0..3).map(|i| sq[i] * offset[i] * offset[i]).sum::<f64>();
    let gain = std::array::from_fn(|i| (sq[i] / r).sqrt());
    Some(Fit { offset, gain })
}

fn rms(v: impl Iterator<Item = f64>) -> f64 {
    let (sum, n) = v.fold((0.0, 0usize), |(s, n), x| (s + x * x, n + 1));
    (sum / n.max(1) as f64).sqrt()
}

struct Calibration {
    bias: V3,
    segments: Vec<Segment>,
    fit: Fit,
}

fn calibrate(rec: &Recording) -> anyhow::Result<(stats::Scaled, Calibration)> {
    let s = stats::scaled(rec)?;
    let period_s = f64::from(s.period_us) / 1e6;
    let window = ((WINDOW_S / period_s).round() as usize).max(2);

    // Pass 1 finds the quiet parts to estimate the gyro bias; pass 2 also requires the
    // gyro near that bias.
    let quiet: Vec<V3> = still_mask(&s.gyro, &s.accel, window, None, HAND)
        .iter()
        .zip(&s.gyro)
        .filter_map(|(&m, g)| m.then_some(*g))
        .collect();
    if quiet.is_empty() {
        bail!("no still part found");
    }
    let bias = mean(&quiet);
    let trim = (TRIM_S / period_s).round() as usize;
    let min_len = (MIN_SEGMENT_S / period_s).round() as usize;
    let segments: Vec<Segment> = runs(&still_mask(&s.gyro, &s.accel, window, Some(bias), HAND))
        .into_iter()
        .filter(|&(a, b)| b - a >= min_len)
        .map(|(a, b)| {
            let (a, b) = (a + trim, b - trim);
            Segment {
                t: (s.t_s[a], s.t_s[b - 1]),
                accel: mean(&s.accel[a..b]).map(|x| x / 1000.0),
                gyro: mean(&s.gyro[a..b]),
            }
        })
        .collect();
    if segments.len() < UNKNOWNS {
        bail!(
            "{} still segments of >= {MIN_SEGMENT_S} s found, need at least {UNKNOWNS}",
            segments.len()
        );
    }
    let points: Vec<V3> = segments.iter().map(|g| g.accel).collect();
    let Some(fit) = fit(&points) else {
        bail!("fit failed: the poses do not cover enough directions");
    };
    Ok((
        s,
        Calibration {
            bias,
            segments,
            fit,
        },
    ))
}

/// The accelerometer fit of a tumble session.
pub fn fit_session(rec: &Recording) -> anyhow::Result<Fit> {
    Ok(calibrate(rec)?.1.fit)
}

pub fn report(rec: &Recording) -> anyhow::Result<String> {
    let (
        s,
        Calibration {
            bias,
            segments,
            fit,
        },
    ) = calibrate(rec)?;
    let points: Vec<V3> = segments.iter().map(|g| g.accel).collect();

    let mut out = String::new();
    writeln!(
        out,
        "{} still segments (>= {MIN_SEGMENT_S} s), gyro bias {:.3} {:.3} {:.3} dps\n",
        segments.len(),
        bias[0],
        bias[1],
        bias[2]
    )?;
    writeln!(
        out,
        "   t, s          accel x/y/z, mg            |a| raw  |a| fit, mg   gyro x/y/z, dps"
    )?;
    for g in &segments {
        let a = g.accel.map(|x| x * 1000.0);
        writeln!(
            out,
            "{:>5.1}..{:<5.1} {:>8.1} {:>8.1} {:>8.1}   {:>8.1} {:>8.1}   {:>6.2} {:>6.2} {:>6.2}",
            g.t.0,
            g.t.1,
            a[0],
            a[1],
            a[2],
            norm(g.accel) * 1000.0,
            norm(fit.apply(g.accel)) * 1000.0,
            g.gyro[0],
            g.gyro[1],
            g.gyro[2]
        )?;
    }
    // How well each axis was seen pointing both ways: the fit separates offset from gain
    // only with both signs or many tilts.
    writeln!(
        out,
        "\naxis  seen (unit dir)    offset, mg   gain     LSB offset"
    )?;
    for (i, axis) in ["x", "y", "z"].iter().enumerate() {
        let dirs = points.iter().map(|p| p[i] / norm(*p));
        let lo = dirs.clone().fold(f64::INFINITY, f64::min);
        let hi = dirs.fold(f64::NEG_INFINITY, f64::max);
        writeln!(
            out,
            "  {axis}   {lo:>+5.2}..{hi:<+5.2}      {:>+7.1}   {:.4}   {:>+6.0}",
            fit.offset[i] * 1000.0,
            fit.gain[i],
            fit.offset[i] * 1000.0 / s.mg_per_lsb
        )?;
    }
    let before = rms(points.iter().map(|p| (norm(*p) - 1.0) * 1000.0));
    let after = rms(points.iter().map(|p| (norm(fit.apply(*p)) - 1.0) * 1000.0));
    writeln!(
        out,
        "\n|a| error, rms over segments: {before:.1} mg raw -> {after:.1} mg fitted"
    )?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fit_recovers_offset_and_gain() {
        let offset = [0.02, -0.01, 0.03];
        let gain = [1.02, 0.99, 0.97];
        // Unit directions, then raw = true / gain + offset.
        let dirs: [V3; 8] = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [-0.6, 0.0, 0.8],
            [0.0, -0.6, 0.8],
            [0.6, 0.6, -0.529_150_262_212_918_1],
            [-0.48, 0.6, 0.64],
            [0.0, 0.8, -0.6],
        ];
        let points: Vec<V3> = dirs
            .iter()
            .map(|d| std::array::from_fn(|i| d[i] / gain[i] + offset[i]))
            .collect();
        let f = fit(&points).expect("fit");
        for i in 0..3 {
            assert!((f.offset[i] - offset[i]).abs() < 1e-9, "offset {i}");
            assert!((f.gain[i] - gain[i]).abs() < 1e-9, "gain {i}");
        }
    }

    #[test]
    fn runs_of_true() {
        assert_eq!(
            runs(&[false, true, true, false, true]),
            vec![(1, 3), (4, 5)]
        );
    }
}
