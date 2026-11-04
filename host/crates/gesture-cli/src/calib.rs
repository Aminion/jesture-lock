//! `calib`: report of the accelerometer fit from a tumble session (`session::calib`).

use std::fmt::Write as _;

use session::calib::{Calibration, MIN_SEGMENT_S, calibrate, rms};
use session::recording::{Recording, V3, norm};

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
