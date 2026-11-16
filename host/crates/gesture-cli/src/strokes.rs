//! `strokes`: what the integer core gives per stroke on a recording (`session::strokes`).
//! One line per stroke end: duration, net displacement (the last corrected point), peak
//! speed between points, and the residual velocity before the zero-velocity correction
//! (`v_end_mm_s`). Rejected moves are listed too, with their reason, so a missing stroke
//! shows why.
//!
//! The float reference of the same algorithm, with more per-stroke detail, is `path`.

use std::fmt::Write as _;

use anyhow::bail;
use gesture_core::GcConfig;

use session::recording::Recording;
use session::strokes::run;

pub fn report(rec: &Recording, cfg: &GcConfig) -> anyhow::Result<String> {
    let Some(&(t0, _)) = rec.samples.first() else {
        bail!("no samples");
    };
    let mut out = String::new();
    writeln!(
        out,
        "  t, s  event          dur, s   end x/y/z, mm (start frame)  |d|, mm  extent  peak m/s  v_end mm/s  points  step"
    )?;
    let (mut strokes, mut rejected) = (0, 0);
    for (k, ev, summary) in run(rec, cfg)? {
        let at = (rec.samples[k].0 - t0) as f64 / 1e6;
        let name = ev.kind.to_string();
        match summary {
            Some(s) => {
                strokes += 1;
                writeln!(
                    out,
                    "{at:>6.2}  {name:<13} {:>6.2}  {:>6} {:>6} {:>6}            {:>7.1}  {:>6}  {:>8.2}  {:>10}  {:>6}  {:>4}",
                    s.duration.as_secs_f64(),
                    s.end[0],
                    s.end[1],
                    s.end[2],
                    s.distance_mm,
                    s.extent,
                    s.peak_mm_s / 1e3,
                    s.v_end,
                    s.points,
                    s.step
                )?;
            }
            // LONG and TIME_GAP end at once, without a duration or v_end.
            None if ev.duration.is_zero() => {
                rejected += 1;
                writeln!(out, "{at:>6.2}  {name}")?;
            }
            None => {
                rejected += 1;
                // The stroke row's format, its columns blank up to v_end.
                writeln!(
                    out,
                    "{at:>6.2}  {name:<13} {:>6.2}  {:>6} {:>6} {:>6}            {:>7}  {:>6}  {:>8}  {:>10}",
                    ev.duration.as_secs_f64(),
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    ev.v_end
                )?;
            }
        }
    }
    writeln!(out, "\n{strokes} strokes, {rejected} rejected")?;
    Ok(out)
}
