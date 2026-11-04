//! The stream -> core step, without threads or UI: frames in, updates out. Both sources
//! (replay, serial) push their frames through it, so a replay tests what the live path does.

use gesture_core::{Core, Event, GcPoint, Kind};
use protocol::time::Unwrapper;
use protocol::{FrameError, Hello, Message};
use session::check::{self, Stats};
use session::recording::V3;
use session::reference::Setup;
use session::strokes::Summary;

/// What the UI hears from a source.
#[derive(Debug, Clone)]
pub enum Update {
    /// The stream (re)started: first `Hello`, a changed one, or an MCU reset (`check`). The
    /// core starts over; `Err` if it cannot be configured for this `Hello` (no samples
    /// reach it until the next start).
    Started {
        why: check::Event,
        hello: Hello,
        /// The config line and the stroke checks, or why there is no core.
        config: Result<(String, Limits), String>,
    },
    /// A sample fed to the core: unwrapped MCU time (us), the event, `gc_live`.
    Sample {
        t_us: u64,
        event: Event,
        live: Option<GcPoint>,
    },
    /// After a `Sample` with `Kind::Stroke`: the corrected points and their numbers.
    Stroke {
        stroke: gesture_core::Stroke,
        summary: Summary,
    },
    /// Stream health so far (`check`): counters for the top bar.
    Health(check::Totals),
    /// The source stopped: the end of a replay (`None`) or an error.
    Ended(Option<String>),
}

/// The stroke checks of the running core (`GcConfig`), to say why a move was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Limits {
    pub min_stroke_us: u32,
    pub max_stroke_us: u32,
    pub min_extent_mm: u16,
    pub max_v_end_mm_s: u16,
}

/// Stream health plus one core, restarted with the stream.
#[derive(Debug)]
pub struct Feed {
    setup: Setup,
    /// Initial gyro bias when the setup has no rest session, and where it came from.
    fallback_bias: (V3, &'static str),
    stats: Stats,
    clock: Unwrapper,
    /// The `Hello` the core was started for.
    hello: Option<Hello>,
    core: Option<Core>,
}

impl Feed {
    pub fn new(setup: Setup, fallback_bias: (V3, &'static str)) -> Self {
        Self {
            setup,
            fallback_bias,
            stats: Stats::new(),
            clock: Unwrapper::new(),
            hello: None,
            core: None,
        }
    }

    pub fn totals(&self) -> &check::Totals {
        self.stats.totals()
    }

    /// One frame off the wire, as the deframer gives it.
    pub fn frame(&mut self, frame: Result<Vec<u8>, FrameError>, out: &mut Vec<Update>) {
        let payload = match frame {
            Ok(p) => p,
            Err(e) => return self.stats.frame_error(e),
        };
        match Message::decode(&payload) {
            Err(e) => self.stats.message_error(e),
            Ok(None) => self.stats.skipped(),
            Ok(Some(m)) => self.message(&m, out),
        }
    }

    fn message(&mut self, m: &Message, out: &mut Vec<Update>) {
        // `Stats` starts at the first Hello and sees resets on the sample that shows them,
        // so that sample already goes to the new core.
        match self.stats.message(m) {
            Some(why @ (check::Event::Started(hello) | check::Event::HelloChanged(hello))) => {
                self.start(why, hello, out);
            }
            Some(why @ check::Event::Reset) => {
                if let Some(hello) = self.hello {
                    self.start(why, hello, out);
                }
            }
            None => {}
        }
        let (Message::Sample(s), Some(core)) = (m, self.core.as_mut()) else {
            return;
        };
        let t_us = self.clock.extend(s.t_us);
        let event = core.feed(&gesture_core::GcSample {
            t_us: s.t_us,
            gyro: s.gyro,
            accel: s.accel,
        });
        out.push(Update::Sample {
            t_us,
            event,
            live: core.live(),
        });
        if event.kind == Kind::Stroke
            && let Some(stroke) = core.stroke()
        {
            let summary = Summary::of(&stroke);
            out.push(Update::Stroke { stroke, summary });
        }
    }

    fn start(&mut self, why: check::Event, hello: Hello, out: &mut Vec<Update>) {
        self.clock.reset();
        self.hello = Some(hello);
        let fallback = self.fallback_bias;
        let config = self
            .setup
            .config(&hello, || Ok(fallback))
            .and_then(|(cfg, about)| {
                let gc = cfg.gc();
                let limits = Limits {
                    min_stroke_us: gc.min_stroke_us,
                    max_stroke_us: gc.max_stroke_us,
                    min_extent_mm: gc.min_extent_mm,
                    max_v_end_mm_s: gc.max_v_end_mm_s,
                };
                Ok((Core::new(&gc)?, about, limits))
            });
        let config = match config {
            Ok((core, about, limits)) => {
                self.core = Some(core);
                Ok((about.trim_end().to_owned(), limits))
            }
            Err(e) => {
                self.core = None;
                Err(format!("{e:#}"))
            }
        };
        out.push(Update::Started { why, hello, config });
    }
}
