//! `events`: a recording through the integer core (`core/`, via `gesture-core`), one line
//! per event. The float reference in `path` runs the same algorithm; the tests hold the
//! two to the same events.

use std::fmt::Write as _;

use anyhow::bail;
use gesture_core::Core;
use gesture_core::ffi::{
    GC_EV_MOVING, GC_EV_NONE, GC_EV_READY, GC_EV_REJECTED, GC_EV_STROKE, GC_REJ_CLIPPED,
    GC_REJ_DRIFT, GC_REJ_LONG, GC_REJ_SHORT, GC_REJ_TIME_GAP, GcConfig, GcEvent, GcSample,
};

use crate::stats::Recording;

/// The events of a recording: (sample index, event), `GC_EV_NONE` left out.
pub fn run(rec: &Recording, cfg: &GcConfig) -> anyhow::Result<Vec<(usize, GcEvent)>> {
    let Ok(mut core) = Core::new(cfg) else {
        bail!("gc_init rejected the config: {cfg:?}");
    };
    Ok(rec
        .samples
        .iter()
        .enumerate()
        .filter_map(|(k, (_, s))| {
            let ev = core.feed(&GcSample {
                t_us: s.t_us,
                gyro: s.gyro,
                accel: s.accel,
            });
            (ev.kind != GC_EV_NONE).then_some((k, ev))
        })
        .collect())
}

fn name(ev: &GcEvent) -> String {
    let reason = match ev.reason {
        GC_REJ_SHORT => "SHORT",
        GC_REJ_LONG => "LONG",
        GC_REJ_CLIPPED => "CLIPPED",
        GC_REJ_TIME_GAP => "TIME_GAP",
        GC_REJ_DRIFT => "DRIFT",
        _ => "?",
    };
    match ev.kind {
        GC_EV_READY => "READY".to_owned(),
        GC_EV_MOVING => "MOVING".to_owned(),
        GC_EV_STROKE => "STROKE".to_owned(),
        GC_EV_REJECTED => format!("REJ {reason}"),
        k => format!("kind {k}"),
    }
}

pub fn report(rec: &Recording, cfg: &GcConfig) -> anyhow::Result<String> {
    let Some(&(t0, _)) = rec.samples.first() else {
        bail!("no samples");
    };
    let mut out = String::new();
    writeln!(out, "  t, s  sample  event")?;
    for (k, ev) in run(rec, cfg)? {
        let at = (rec.samples[k].0 - t0) as f64 / 1e6;
        writeln!(out, "{at:>6.2}  {k:>6}  {}", name(&ev))?;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use gesture_core::ffi::GcPoint;

    use super::*;
    use crate::calib::HAND;
    use crate::path::{self, Event, MIN_EXTENT_MM, MIN_STROKE_US, Reject};
    use crate::stats;

    fn load(name: &str) -> Recording {
        let file = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../data/sessions")
            .join(name);
        stats::load(&std::fs::read(file).expect("session file")).expect("session")
    }

    /// (sample index, `GC_EV_*`).
    type Kinds = Vec<(usize, u8)>;

    /// Events of the float reference.
    fn reference(rec: &Recording, cfg: &path::Config) -> Kinds {
        let mut core = path::Core::new(cfg);
        rec.samples
            .iter()
            .enumerate()
            .filter_map(|(k, (t, s))| {
                core.feed(*t, s).map(|e| {
                    let kind = match e {
                        Event::Ready => GC_EV_READY,
                        Event::Moving => GC_EV_MOVING,
                        Event::Stroke(_) => GC_EV_STROKE,
                        Event::Rejected(..) => GC_EV_REJECTED,
                    };
                    (k, kind)
                })
            })
            .collect()
    }

    /// Integer and float events, as the CLI configures `path` (hand thresholds, this
    /// session's median bias, no accel calibration).
    fn both(name: &str) -> (Kinds, Kinds) {
        let rec = load(name);
        let bias = stats::median(&stats::scaled(&rec).expect("scales").gyro);
        let cfg = path::Config::new(&rec, None, bias, HAND).expect("config");
        let ints = run(&rec, &cfg.gc())
            .expect("gc_init")
            .into_iter()
            .map(|(k, ev)| (k, ev.kind))
            .collect();
        (ints, reference(&rec, &cfg))
    }

    #[test]
    fn rest_table_is_one_hold() {
        let (ints, floats) = both("20261030-183544_rest-table.bin");
        // READY once the first hold_n windows are quiet, then nothing for 60 s.
        assert_eq!(ints, [(32 + 20 - 2, GC_EV_READY)]);
        assert_eq!(ints, floats);
    }

    /// Integer and float paths of the first stroke, sample by sample while the float
    /// reference is in it: (t_100us, mm) of `gc_live` against the float position.
    fn first_path(name: &str) -> Vec<(GcPoint, (u64, [f64; 3]))> {
        let rec = load(name);
        let bias = stats::median(&stats::scaled(&rec).expect("scales").gyro);
        let cfg = path::Config::new(&rec, None, bias, HAND).expect("config");
        let gc = cfg.gc();
        let mut ints = Core::new(&gc).expect("gc_init");
        let mut floats = path::Core::new(&cfg);
        let mut out = Vec::new();
        let mut started = false;
        for (t, s) in &rec.samples {
            ints.feed(&GcSample {
                t_us: *t as u32,
                gyro: s.gyro,
                accel: s.accel,
            });
            floats.feed(*t, s);
            match (ints.live(), floats.live()) {
                (Some(i), Some(f)) => {
                    started = true;
                    out.push((i, f));
                }
                (None, None) if !started => {}
                _ => break,
            }
        }
        out
    }

    #[test]
    fn hand_hold_path_matches_the_reference() {
        let path = first_path("20261030-210755_hand-hold.bin");
        assert!(path.len() > 50, "a stroke of {} samples", path.len());
        let mut worst = 0.0_f64;
        for (i, (t_us, p)) in &path {
            assert_eq!(u64::from(i.t_100us), (t_us + 50) / 100);
            let mm = [i.x_mm, i.y_mm, i.z_mm];
            for k in 0..3 {
                worst = worst.max((f64::from(mm[k]) - p[k] * 1000.0).abs());
            }
        }
        let (_, (_, end)) = path.last().expect("points");
        let far = end.iter().fold(0.0_f64, |m, x| m.max(x.abs())) * 1000.0;
        // Rounding to whole mm is 0.5; the integer formats add far less.
        eprintln!(
            "{} samples, worst {worst:.3} mm over a {far:.0} mm path",
            path.len()
        );
        assert!(worst <= 0.6, "worst {worst:.3} mm over a {far:.0} mm path");
        assert!(far > 20.0, "the stroke moved: {far:.1} mm");
    }

    /// One event, as both cores report it: (sample index, kind, reason, duration_us,
    /// v_end mm/s, extent mm). The last three only at a stroke end; the integer core gives
    /// extent only for `STROKE` (through `gc_stroke`), the float one at every end.
    type Full = (usize, u8, u8, u32, f64, Option<f64>);

    fn full_events(rec: &Recording, cfg: &path::Config) -> (Vec<Full>, Vec<Full>) {
        let gc = cfg.gc();
        let mut ints = Core::new(&gc).expect("gc_init");
        let mut floats = path::Core::new(cfg);
        let (mut a, mut b) = (Vec::new(), Vec::new());
        for (k, (t, s)) in rec.samples.iter().enumerate() {
            let ev = ints.feed(&GcSample {
                t_us: *t as u32,
                gyro: s.gyro,
                accel: s.accel,
            });
            if ev.kind != GC_EV_NONE {
                let extent = (ev.kind == GC_EV_STROKE).then(|| {
                    let (info, points) = ints.stroke().expect("gc_stroke after STROKE");
                    assert_eq!(usize::from(info.points), points.len());
                    assert_eq!(info.points, ev.points);
                    f64::from(info.extent_mm)
                });
                a.push((
                    k,
                    ev.kind,
                    ev.reason,
                    ev.duration_us,
                    f64::from(ev.v_end_mm_s),
                    extent,
                ));
            }
            if let Some(e) = floats.feed(*t, s) {
                let reason = |r: &Reject| match r {
                    Reject::Short => GC_REJ_SHORT,
                    Reject::Long => GC_REJ_LONG,
                    Reject::Clipped => GC_REJ_CLIPPED,
                    Reject::TimeGap => GC_REJ_TIME_GAP,
                    Reject::Drift => GC_REJ_DRIFT,
                };
                let end = |d: &path::Done| (d.duration_us as u32, d.v_end_mm_s);
                b.push(match &e {
                    Event::Ready => (k, GC_EV_READY, 0, 0, 0.0, None),
                    Event::Moving => (k, GC_EV_MOVING, 0, 0, 0.0, None),
                    Event::Stroke(d) => {
                        let (dur, v) = end(d);
                        (k, GC_EV_STROKE, 0, dur, v, Some(d.extent_mm))
                    }
                    Event::Rejected(r, d) => {
                        let (dur, v) = d.as_ref().map_or((0, 0.0), end);
                        let extent = d.as_ref().map(|d| d.extent_mm);
                        (k, GC_EV_REJECTED, reason(r), dur, v, extent)
                    }
                });
            }
        }
        (a, b)
    }

    /// Whether an integer event matches the float one, and if so whether only because the
    /// float value sat within rounding of a threshold (an edge).
    fn matches(i: &Full, f: &Full) -> Option<bool> {
        if i.0 != f.0 || i.3 != f.3 {
            return None;
        }
        // The quaternion step is first order (docs/core.md): over big turns the tilt
        // drifts a little from the exact one, and gravity leaks into v by ~0.5 %.
        if (i.4 - f.4).abs() > 0.5 + 0.005 * f.4 {
            return None;
        }
        if let (Some(a), Some(b)) = (i.5, f.5) {
            // Rounding to mm, plus a peak the decimated points can miss.
            if (a - b).abs() > 2.0 {
                return None;
            }
        }
        if (i.1, i.2) == (f.1, f.2) {
            return Some(false);
        }
        // A stroke on one side, SHORT (extent) on the other: the extent is on the
        // threshold to within the rounding. Integer points are rounded twice (the stored
        // mm, then the correction in mm), so up to 1 mm off the exact value.
        let short = |e: &Full| e.1 == GC_EV_REJECTED && e.2 == GC_REJ_SHORT;
        let near_min = f.5.is_some_and(|x| (x - MIN_EXTENT_MM).abs() <= 1.0);
        let edge = ((i.1 == GC_EV_STROKE && short(f)) || (short(i) && f.1 == GC_EV_STROKE))
            && near_min
            && u64::from(f.3) >= MIN_STROKE_US;
        edge.then_some(true)
    }

    /// Every session, every event: the same sample, kind, reason and duration; v_end and
    /// extent within their rounding. A decision may differ only on a threshold edge.
    #[test]
    fn every_session_matches_the_reference() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../data/sessions");
        let mut names: Vec<_> = std::fs::read_dir(&dir)
            .expect("sessions")
            .filter_map(|e| e.ok()?.file_name().into_string().ok())
            .filter(|n| n.ends_with(".bin"))
            .collect();
        names.sort();
        let (mut bad, mut edges) = (Vec::new(), Vec::new());
        let mut ends = 0;
        for name in &names {
            let rec = load(name);
            let bias = stats::median(&stats::scaled(&rec).expect("scales").gyro);
            let cfg = path::Config::new(&rec, None, bias, HAND).expect("config");
            let (ints, floats) = full_events(&rec, &cfg);
            if ints.len() != floats.len() {
                bad.push(format!("{name}: {} events vs {}", ints.len(), floats.len()));
            }
            for (i, f) in ints.iter().zip(&floats) {
                match matches(i, f) {
                    None => bad.push(format!("{name}: int {i:?} vs float {f:?}")),
                    Some(true) => edges.push(format!("{name}: int {i:?} vs float {f:?}")),
                    Some(false) => {}
                }
                ends += usize::from(i.3 > 0);
            }
        }
        assert!(bad.is_empty(), "{}", bad.join("\n"));
        // Known: ruler-15 at 14.5 s, float extent 19.25 mm, integer 20 mm.
        assert!(edges.len() <= 1, "{}", edges.join("\n"));
        assert!(ends > 40, "{ends} stroke ends compared");
    }
}
