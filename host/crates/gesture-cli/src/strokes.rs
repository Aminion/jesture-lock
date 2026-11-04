//! `strokes`: what the integer core (`core/` via `gesture-core`) gives per stroke on a
//! recording, the numbers the board would see. One line per stroke end: duration, net
//! displacement (the last corrected point), peak speed between points, and the residual
//! velocity before the zero-velocity correction (`v_end_mm_s`). Rejected moves are listed
//! too, with their reason, so a missing stroke shows why.
//!
//! The float reference of the same algorithm, with more per-stroke detail, is `path`.

use std::fmt::Write as _;

use anyhow::{Context, bail};
use gesture_core::{Core, Event, GcConfig, GcPoint, GcSample, Kind, Stroke};

use session::recording::Recording;

/// One accepted stroke, summed up from `gc_stroke`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Summary {
    pub duration_us: u32,
    /// The last point: net displacement in the start frame, mm.
    pub end_mm: [i16; 3],
    /// |end|, mm.
    pub distance_mm: f64,
    /// Largest speed between consecutive points, mm/s. The points are decimated (a step of
    /// `info.step` samples), so short peaks are averaged out.
    pub peak_mm_s: f64,
    /// Leftover velocity before the correction, mm/s.
    pub v_end_mm_s: u16,
    pub extent_mm: u16,
    pub points: u16,
    pub step: u16,
}

fn mm(p: &GcPoint) -> [f64; 3] {
    [p.x_mm, p.y_mm, p.z_mm].map(f64::from)
}

fn norm(v: [f64; 3]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

impl Summary {
    #[must_use]
    pub fn of(s: &Stroke) -> Self {
        let end = s.points.last().copied().unwrap_or_default();
        let peak_mm_s = s
            .points
            .windows(2)
            .filter_map(|w| {
                let dt_100us = w[1].t_100us.checked_sub(w[0].t_100us)?;
                (dt_100us > 0).then(|| {
                    let (a, b) = (mm(&w[0]), mm(&w[1]));
                    norm([b[0] - a[0], b[1] - a[1], b[2] - a[2]]) * 1e4 / f64::from(dt_100us)
                })
            })
            .fold(0.0, f64::max);
        Self {
            duration_us: s.info.duration_us,
            end_mm: [end.x_mm, end.y_mm, end.z_mm],
            distance_mm: norm(mm(&end)),
            peak_mm_s,
            v_end_mm_s: s.info.v_end_mm_s,
            extent_mm: s.info.extent_mm,
            points: s.info.points,
            step: s.info.step,
        }
    }
}

/// The move ends of a recording, accepted and rejected, in order: (sample index, event,
/// for a stroke its summary).
pub fn run(
    rec: &Recording,
    cfg: &GcConfig,
) -> anyhow::Result<Vec<(usize, Event, Option<Summary>)>> {
    let mut core = Core::new(cfg).with_context(|| format!("config {cfg:?}"))?;
    let mut out = Vec::new();
    for (k, (_, s)) in rec.samples.iter().enumerate() {
        let ev = core.feed(&GcSample {
            t_us: s.t_us,
            gyro: s.gyro,
            accel: s.accel,
        });
        match ev.kind {
            Kind::Stroke => {
                let stroke = core.stroke().context("gc_stroke after STROKE")?;
                out.push((k, ev, Some(Summary::of(&stroke))));
            }
            Kind::Rejected(_) => out.push((k, ev, None)),
            _ => {}
        }
    }
    Ok(out)
}

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
                    f64::from(s.duration_us) / 1e6,
                    s.end_mm[0],
                    s.end_mm[1],
                    s.end_mm[2],
                    s.distance_mm,
                    s.extent_mm,
                    s.peak_mm_s / 1e3,
                    s.v_end_mm_s,
                    s.points,
                    s.step
                )?;
            }
            // LONG and TIME_GAP end at once, without a duration or v_end.
            None if ev.duration_us == 0 => {
                rejected += 1;
                writeln!(out, "{at:>6.2}  {name}")?;
            }
            None => {
                rejected += 1;
                // The stroke row's format, its columns blank up to v_end.
                writeln!(
                    out,
                    "{at:>6.2}  {name:<13} {:>6.2}  {:>6} {:>6} {:>6}            {:>7}  {:>6}  {:>8}  {:>10}",
                    f64::from(ev.duration_us) / 1e6,
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    ev.v_end_mm_s
                )?;
            }
        }
    }
    writeln!(out, "\n{strokes} strokes, {rejected} rejected")?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use gesture_core::GcStrokeInfo;

    use super::*;

    fn point(x_mm: i16, t_100us: u16) -> GcPoint {
        GcPoint {
            x_mm,
            y_mm: 0,
            z_mm: 0,
            t_100us,
        }
    }

    #[test]
    fn summary_takes_the_last_point_and_the_fastest_step() {
        let stroke = Stroke {
            info: GcStrokeInfo {
                points: 4,
                ..GcStrokeInfo::default()
            },
            // 10 mm in 0.1 s, 30 mm in 0.1 s, 0 in 0.1 s.
            points: vec![
                point(0, 0),
                point(10, 1000),
                point(40, 2000),
                point(40, 3000),
            ],
        };
        let s = Summary::of(&stroke);
        assert_eq!(s.end_mm, [40, 0, 0]);
        assert!((s.distance_mm - 40.0).abs() < 1e-9);
        assert!((s.peak_mm_s - 300.0).abs() < 1e-9, "{}", s.peak_mm_s);
    }

    /// The ruler test (`docs/todo.md` 3d.2): ruler-30 slides the board 30 cm along its x
    /// axis. Configured as the vectors are (tumble calibration, rest-table bias, hand
    /// thresholds), the long strokes come out near 0.30 m and along x.
    #[test]
    fn ruler_30_strokes_are_about_30_cm_along_x() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../data/sessions");
        let (rec, cfg, _) =
            session::reference::reference_config(&session::reference::ReferenceArgs {
                file: dir.join("20261030-211435_ruler-30.bin"),
                calib: Some(dir.join("20261030-205757_tumble.bin")),
                hold: session::reference::Hold::Hand,
                bias: Some(dir.join("20261030-183544_rest-table.bin")),
            })
            .expect("config");
        let long: Vec<Summary> = run(&rec, &cfg.gc())
            .expect("run")
            .into_iter()
            .filter_map(|(_, _, s)| s)
            .filter(|s| s.distance_mm > 100.0)
            .collect();
        // One slide drifts out (DRIFT) and one is a short twitch; three remain.
        assert_eq!(long.len(), 3, "{long:#?}");
        for s in &long {
            // Hand-held along a ruler: 0.25..0.36 m here, the error from tilt and drift.
            assert!((200.0..=400.0).contains(&s.distance_mm), "{s:?}");
            assert!(
                f64::from(s.end_mm[0].unsigned_abs()) > 0.95 * s.distance_mm,
                "{s:?}"
            );
            assert!(s.v_end_mm_s < 50, "{s:?}");
            assert!((100.0..=600.0).contains(&s.peak_mm_s), "{s:?}");
        }
        let mean = long.iter().map(|s| s.distance_mm).sum::<f64>() / 3.0;
        assert!((mean - 300.0).abs() < 30.0, "mean {mean:.1} mm");
    }
}
