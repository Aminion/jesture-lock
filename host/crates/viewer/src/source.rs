//! Input sources on a worker thread: replay of a session file, or the live serial port.
//! Either one pushes its frames through `Feed` and sends the updates to the UI.

use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, TryRecvError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::Context;

use protocol::Deframer;
use session::recording;
use session::reference::Setup;
use session::serial::{self, PortArgs, READ_BUF};

use crate::feed::{Feed, Update};

#[derive(Debug, Clone)]
pub enum Source {
    Replay(PathBuf),
    Serial(PortArgs),
}

/// From the UI to the worker. Dropping the worker's handle stops it.
#[derive(Debug, Clone, Copy)]
pub enum Control {
    /// Replay only.
    Pause(bool),
    /// Replay only: times real time; `f64::INFINITY` as fast as it goes.
    Speed(f64),
}

/// How often the top bar counters are sent.
const HEALTH_EVERY: Duration = Duration::from_millis(500);

/// A running source. Dropping it stops the thread and waits for it (at most a port read
/// timeout or the settle time), so the port is free again for the next connect.
#[derive(Debug)]
pub struct Worker {
    control: Option<Sender<Control>>,
    updates: Receiver<Update>,
    thread: Option<JoinHandle<()>>,
}

impl Worker {
    /// `repaint` runs after each batch of updates (the UI's `request_repaint`).
    pub fn start(
        source: Source,
        setup: Setup,
        speed: f64,
        repaint: impl Fn() + Send + 'static,
    ) -> Self {
        let (control, control_rx) = mpsc::channel();
        let (tx, updates) = mpsc::channel();
        let thread = thread::spawn(move || {
            let out = Out { tx, repaint };
            let result = match source {
                Source::Replay(file) => replay(&file, setup, speed, &control_rx, &out),
                Source::Serial(port) => live(&port, setup, &control_rx, &out),
            };
            match result {
                Ok(Stop::End) => out.send(vec![Update::Ended(None)]),
                Ok(Stop::Dropped) => {}
                Err(e) => out.send(vec![Update::Ended(Some(format!("{e:#}")))]),
            }
        });
        Self {
            control: Some(control),
            updates,
            thread: Some(thread),
        }
    }

    pub fn send(&self, c: Control) {
        if let Some(control) = &self.control {
            // The worker may have ended already; nothing to tell it then.
            let _ = control.send(c);
        }
    }

    /// Updates received so far.
    pub fn updates(&self) -> impl Iterator<Item = Update> + '_ {
        self.updates.try_iter()
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        self.control = None;
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

enum Stop {
    /// The replay reached the end of the file.
    End,
    /// The UI dropped the worker.
    Dropped,
}

struct Out<F> {
    tx: Sender<Update>,
    repaint: F,
}

impl<F: Fn()> Out<F> {
    fn send(&self, updates: Vec<Update>) {
        if updates.is_empty() {
            return;
        }
        for u in updates {
            // The UI is gone: the control channel says so on the next check.
            let _ = self.tx.send(u);
        }
        (self.repaint)();
    }
}

/// Paces a replay by MCU time: sample k is due at `wall0 + (t_k - t0) / speed`. Re-anchored
/// on a speed change, a resume, and a clock that went backwards (MCU reset).
struct Pace {
    speed: f64,
    paused: bool,
    anchor: Option<(Instant, u64)>,
}

impl Pace {
    fn apply(&mut self, c: Control) {
        match c {
            Control::Pause(p) => self.paused = p,
            Control::Speed(s) => self.speed = s,
        }
        self.anchor = None;
    }

    /// Waits until the sample at `t_us` is due, handling controls meanwhile.
    fn wait(&mut self, t_us: u64, control: &Receiver<Control>) -> Option<Stop> {
        loop {
            if self.paused {
                match control.recv() {
                    Ok(c) => self.apply(c),
                    Err(_) => return Some(Stop::Dropped),
                }
                continue;
            }
            let (wall0, t0) = match self.anchor {
                Some((w, t)) if t <= t_us => (w, t),
                _ => *self.anchor.insert((Instant::now(), t_us)),
            };
            if !self.speed.is_finite() {
                return match control.try_recv() {
                    Ok(c) => {
                        self.apply(c);
                        continue;
                    }
                    Err(TryRecvError::Empty) => None,
                    Err(TryRecvError::Disconnected) => Some(Stop::Dropped),
                };
            }
            let due = wall0 + Duration::from_secs_f64((t_us - t0) as f64 / 1e6 / self.speed);
            match control.recv_timeout(due.saturating_duration_since(Instant::now())) {
                Ok(c) => self.apply(c),
                Err(RecvTimeoutError::Timeout) => return None,
                Err(RecvTimeoutError::Disconnected) => return Some(Stop::Dropped),
            }
        }
    }
}

/// A feed for a session file. Without a rest session the initial bias is, as in
/// `reference_config`, the session's own median.
fn replay_feed(bytes: &[u8], setup: Setup) -> anyhow::Result<Feed> {
    let fallback = match setup.bias_dps {
        Some(_) => ([0.0; 3], "unused"),
        None => {
            let rec = recording::load(bytes)?;
            (
                recording::median(&recording::scaled(&rec)?.gyro),
                "this session's median",
            )
        }
    };
    Ok(Feed::new(setup, fallback))
}

fn replay(
    file: &Path,
    setup: Setup,
    speed: f64,
    control: &Receiver<Control>,
    out: &Out<impl Fn()>,
) -> anyhow::Result<Stop> {
    let bytes = std::fs::read(file).with_context(|| format!("reading {}", file.display()))?;
    let mut feed = replay_feed(&bytes, setup)?;
    let mut pace = Pace {
        speed,
        paused: false,
        anchor: None,
    };
    let mut last_health = Instant::now();
    let mut batch = Vec::new();
    for frame in Deframer::new().feed(&bytes) {
        feed.frame(frame, &mut batch);
        // Pace on the newest sample the core got (after a stretch at full speed the batch
        // holds many); frames before the first Hello go at once.
        let due = batch.iter().rev().find_map(|u| match u {
            Update::Sample { t_us, .. } => Some(*t_us),
            _ => None,
        });
        if let Some(t_us) = due
            && let Some(stop) = pace.wait(t_us, control)
        {
            return Ok(stop);
        }
        if last_health.elapsed() >= HEALTH_EVERY {
            batch.push(Update::Health(feed.totals().clone()));
            last_health = Instant::now();
        }
        // At full speed, batch up to the next health tick instead of one send per sample.
        if pace.speed.is_finite() || matches!(batch.last(), Some(Update::Health(_))) {
            out.send(std::mem::take(&mut batch));
        }
    }
    batch.push(Update::Health(feed.totals().clone()));
    out.send(batch);
    Ok(Stop::End)
}

fn live(
    port: &PortArgs,
    setup: Setup,
    control: &Receiver<Control>,
    out: &Out<impl Fn()>,
) -> anyhow::Result<Stop> {
    let mut serial = serial::open(port)?;
    serial::settle(serial.as_mut())?;
    // No session to take a median from; the core refines the bias in every hold
    // (`docs/core.md`, bias update).
    let mut feed = Feed::new(setup, ([0.0; 3], "zero, no rest session given"));
    let mut deframer = Deframer::new();
    let mut buf = [0u8; READ_BUF];
    let mut last_health = Instant::now();
    loop {
        match control.try_recv() {
            Ok(_) | Err(TryRecvError::Empty) => {}
            Err(TryRecvError::Disconnected) => return Ok(Stop::Dropped),
        }
        let n = serial::read_some(serial.as_mut(), &mut buf)?;
        let mut batch = Vec::new();
        for frame in deframer.feed(&buf[..n]) {
            feed.frame(frame, &mut batch);
        }
        if last_health.elapsed() >= HEALTH_EVERY {
            batch.push(Update::Health(feed.totals().clone()));
            last_health = Instant::now();
        }
        out.send(batch);
    }
}

#[cfg(test)]
mod tests {
    use gesture_core::{Core, GcSample, Kind, Reject};
    use session::reference::{CoreArgs, ReferenceArgs, reference_config};
    use session::strokes;

    use super::*;

    /// `docs/todo.md` 3e.2: a replay of ruler-30 through the worker's path (frames ->
    /// `Feed`, its own config from the `Hello`) gives what `gesture-cli strokes` gets from
    /// the loaded recording: every sample's event and live point, every stroke.
    #[test]
    fn replay_of_ruler_30_matches_the_cli() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../data/sessions");
        let file = dir.join("20261030-211435_ruler-30.bin");
        let core = CoreArgs::vectors(&dir);

        let (rec, cfg, _) = reference_config(&ReferenceArgs {
            file: file.clone(),
            core: core.clone(),
        })
        .expect("config");
        let mut direct = Core::new(&cfg.gc()).expect("gc_init");
        let (mut samples, mut strokes) = (Vec::new(), Vec::new());
        for (t, s) in &rec.samples {
            let ev = direct.feed(&GcSample {
                t_us: s.t_us,
                gyro: s.gyro,
                accel: s.accel,
            });
            samples.push((*t, ev, direct.live()));
            if ev.kind == Kind::Stroke {
                strokes.push(direct.stroke().expect("gc_stroke"));
            }
        }

        let (control, control_rx) = mpsc::channel();
        let (tx, rx) = mpsc::channel();
        let stop = replay(
            &file,
            Setup::load(&core).expect("setup"),
            f64::INFINITY,
            &control_rx,
            &Out { tx, repaint: || {} },
        )
        .expect("replay");
        assert!(matches!(stop, Stop::End));
        drop(control);
        let updates: Vec<Update> = rx.try_iter().collect();

        assert_eq!(
            updates
                .iter()
                .filter(|u| matches!(u, Update::Started { config: Ok(_), .. }))
                .count(),
            1
        );
        let got: Vec<_> = updates
            .iter()
            .filter_map(|u| match u {
                Update::Sample { t_us, event, live } => Some((*t_us, *event, *live)),
                _ => None,
            })
            .collect();
        assert_eq!(got, samples);
        let got: Vec<_> = updates
            .iter()
            .filter_map(|u| match u {
                Update::Stroke { stroke, summary } => Some((stroke.clone(), *summary)),
                _ => None,
            })
            .collect();
        let summaries: Vec<_> = strokes::run(&rec, &cfg.gc())
            .expect("run")
            .into_iter()
            .filter_map(|(_, _, s)| s)
            .collect();
        assert_eq!(
            got.iter().map(|(s, _)| s.clone()).collect::<Vec<_>>(),
            strokes
        );
        assert_eq!(got.iter().map(|(_, s)| *s).collect::<Vec<_>>(), summaries);

        // As `core/contract/vectors/v1/README.md` lists for ruler-30.
        let count = |k: Kind| samples.iter().filter(|(_, e, _)| e.kind == k).count();
        assert_eq!(count(Kind::Stroke), 4);
        assert_eq!(count(Kind::Rejected(Reject::Short)), 3);
        assert_eq!(count(Kind::Rejected(Reject::Drift)), 1);
    }
}
