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

    use super::*;
    use crate::calib::HAND;
    use crate::path::{self, Event};
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

    // TODO(3c.1 step 5): compare every event once the core ends strokes.
    #[test]
    fn hand_hold_starts_like_the_reference() {
        let (ints, floats) = both("20261030-210755_hand-hold.bin");
        let first_move = |ev: &[(usize, u8)]| {
            ev.iter()
                .position(|&(_, kind)| kind == GC_EV_MOVING)
                .expect("a move")
        };
        let n = first_move(&ints) + 1;
        assert_eq!(ints, floats[..n]);
        // The core stays in MOVING until step 5: no events after it.
        assert_eq!(ints.len(), n);
    }
}
