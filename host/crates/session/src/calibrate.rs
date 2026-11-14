//! The steps of `gesture-cli calibrate` (`docs/calibration.md`) without port or terminal:
//! decoded frames in, notes out. The live run and the replay of a recorded run both feed
//! this, so a replay tests what the live path does.
//!
//! Steps: connect (a `Hello` of the stream firmware), rest on the table (gyro bias), holds
//! in free orientations (accelerometer offset and gain by the gravity-norm fit), gates.

use std::collections::VecDeque;
use std::fmt;

use protocol::time::{self, Unwrapper};
use protocol::{Firmware, Hello, ImuStatus, Message, Sample};

use crate::calfile::CalFile;
use crate::calib::{self, Fit, HAND, Still, TABLE, WINDOW_S, rms};
use crate::config::{CLIP_LSB, Config, MAX_DT_US};
use crate::recording::{V3, mean, norm, std_dev};

/// Step lengths, MCU time.
#[derive(Debug, Clone, Copy)]
pub struct Params {
    /// Stillness before a measurement starts, us.
    pub settle_us: u64,
    /// The table rest measurement, us.
    pub rest_us: u64,
    /// One hold's measurement, us.
    pub hold_us: u64,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            settle_us: 500_000,
            rest_us: 10_000_000,
            hold_us: 2_000_000,
        }
    }
}

/// The sample rate shown by the connect step is measured over this, us.
pub const RATE_OVER_US: u64 = 1_000_000;
/// A hold is accepted this far, degrees, from every accepted hold.
pub const MIN_ANGLE_DEG: f64 = 20.0;
// The gates.
pub const MIN_HOLDS: usize = 12;
/// An axis is covered in one direction by a hold reading this much on it, g.
pub const COVERAGE_G: f64 = 0.85;
pub const MAX_RMS_MG: f64 = 3.0;
pub const MAX_WORST_MG: f64 = 6.0;
/// The gains `gc_init` accepts.
pub const GAIN: std::ops::RangeInclusive<f64> = 0.9..=1.1;
/// The fit is shown from this many holds on.
pub const FIT_FROM: usize = 9;

/// The six directions of the coverage rule, in the order of `Holds::coverage`.
pub const DIRECTIONS: [&str; 6] = ["+x", "-x", "+y", "-y", "+z", "-z"];

/// A still stretch, measured.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Measurement {
    /// MCU time of the first and last sample used, us.
    pub t_us: (u64, u64),
    pub samples: usize,
    /// Mean and per-axis std: gyro in dps, accel in mg.
    pub gyro: V3,
    pub accel: V3,
    pub gyro_std: V3,
    pub accel_std: V3,
}

/// Where the still tracker is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Moving,
    Settling,
    Measuring {
        done_us: u64,
        of_us: u64,
    },
    /// A measurement was taken; the board has to move before the next one.
    Spent,
}

#[derive(Debug)]
enum TrackState {
    Moving,
    Still {
        t0: u64,
        gyro: Vec<V3>,
        accel: Vec<V3>,
        first_t: u64,
    },
    Spent,
}

/// What a sample did to the tracker.
#[derive(Debug, PartialEq)]
enum Tracked {
    Nothing,
    /// A measurement in progress was lost to movement.
    Lost,
    Done(Measurement),
}

/// Finds a still stretch (the thresholds of `calib::Still` over a `WINDOW_S` window, as
/// `calib::still_mask`) and measures it: `settle_us` of stillness, then `measure_us`.
#[derive(Debug)]
struct Tracker {
    th: Still,
    /// The rate test needs the gyro bias; without one only the spreads are tested.
    bias: Option<V3>,
    window: usize,
    settle_us: u64,
    measure_us: u64,
    ring: VecDeque<(V3, V3)>,
    state: TrackState,
    now: u64,
}

impl Tracker {
    fn new(th: Still, bias: Option<V3>, window: usize, settle_us: u64, measure_us: u64) -> Self {
        Self {
            th,
            bias,
            window,
            settle_us,
            measure_us,
            ring: VecDeque::with_capacity(window),
            state: TrackState::Moving,
            now: 0,
        }
    }

    fn quiet(&self) -> bool {
        if self.ring.len() < self.window {
            return false;
        }
        let (g, a): (Vec<V3>, Vec<V3>) = self.ring.iter().copied().unzip();
        std_dev(&g).iter().all(|&s| s < self.th.gyro_std_dps)
            && std_dev(&a).iter().all(|&s| s < self.th.accel_std_mg)
            && self.bias.is_none_or(|b| {
                let m = mean(&g);
                (0..3).all(|i| (m[i] - b[i]).abs() < self.th.gyro_mean_dps)
            })
    }

    fn phase(&self) -> Phase {
        match &self.state {
            TrackState::Moving => Phase::Moving,
            TrackState::Spent => Phase::Spent,
            TrackState::Still { t0, .. } => {
                let held = self.now - t0;
                if held < self.settle_us {
                    Phase::Settling
                } else {
                    Phase::Measuring {
                        done_us: (held - self.settle_us).min(self.measure_us),
                        of_us: self.measure_us,
                    }
                }
            }
        }
    }

    fn measuring(&self) -> bool {
        matches!(self.phase(), Phase::Measuring { .. })
    }

    /// The stream broke (lost frame, time gap, clipped reading): the window and any
    /// measurement in progress are dropped. True if a measurement was lost.
    fn fault(&mut self) -> bool {
        let lost = self.measuring();
        self.ring.clear();
        if !matches!(self.state, TrackState::Spent) {
            self.state = TrackState::Moving;
        }
        lost
    }

    fn feed(&mut self, t_us: u64, gyro: V3, accel: V3) -> Tracked {
        self.now = t_us;
        if self.ring.len() == self.window {
            self.ring.pop_front();
        }
        self.ring.push_back((gyro, accel));
        // A window still filling (the start, or after a fault) says nothing yet: a spent
        // hold stays spent.
        if self.ring.len() < self.window {
            return Tracked::Nothing;
        }
        let quiet = self.quiet();
        let was_measuring = self.measuring();
        match &mut self.state {
            TrackState::Moving if quiet => {
                self.state = TrackState::Still {
                    t0: t_us,
                    gyro: Vec::new(),
                    accel: Vec::new(),
                    first_t: t_us,
                };
                Tracked::Nothing
            }
            TrackState::Still {
                t0,
                gyro: gs,
                accel: accs,
                first_t,
            } if quiet => {
                let held = t_us - *t0;
                if held < self.settle_us {
                    return Tracked::Nothing;
                }
                if gs.is_empty() {
                    *first_t = t_us;
                }
                gs.push(gyro);
                accs.push(accel);
                if held < self.settle_us + self.measure_us {
                    return Tracked::Nothing;
                }
                let m = Measurement {
                    t_us: (*first_t, t_us),
                    samples: gs.len(),
                    gyro: mean(gs),
                    accel: mean(accs),
                    gyro_std: std_dev(gs),
                    accel_std: std_dev(accs),
                };
                self.state = TrackState::Spent;
                Tracked::Done(m)
            }
            TrackState::Still { .. } | TrackState::Spent if !quiet => {
                self.state = TrackState::Moving;
                if was_measuring {
                    Tracked::Lost
                } else {
                    Tracked::Nothing
                }
            }
            _ => Tracked::Nothing,
        }
    }
}

/// The fit over the accepted holds and how well it fits them.
#[derive(Debug, Clone)]
pub struct Solution {
    pub fit: Fit,
    /// `| |a| - 1 g |` rms over the holds, mg: raw and fitted.
    pub before_mg: f64,
    pub after_mg: f64,
    /// `|a| - 1 g` after the fit per hold, mg.
    pub residual_mg: Vec<f64>,
}

impl Solution {
    /// The hold (index) with the largest residual, and that residual's size, mg.
    pub fn worst(&self) -> (usize, f64) {
        self.residual_mg
            .iter()
            .map(|r| r.abs())
            .enumerate()
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap_or((0, 0.0))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Gate {
    pub name: &'static str,
    pub pass: bool,
    pub detail: String,
}

/// The accepted holds.
#[derive(Debug, Default, Clone)]
pub struct Holds {
    list: Vec<Measurement>,
}

/// A hold was not accepted: it is this close, degrees, to hold `to` (index).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TooClose {
    pub to: usize,
    pub angle_deg: f64,
}

fn angle_deg(a: V3, b: V3) -> f64 {
    let dot: f64 = (0..3).map(|i| a[i] * b[i]).sum();
    (dot / (norm(a) * norm(b)))
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

impl Holds {
    pub fn list(&self) -> &[Measurement] {
        &self.list
    }

    /// Accepts a hold unless its gravity direction is within `MIN_ANGLE_DEG` of an accepted
    /// one. Returns its index.
    pub fn offer(&mut self, m: Measurement) -> Result<usize, TooClose> {
        let nearest = self
            .list
            .iter()
            .map(|h| angle_deg(h.accel, m.accel))
            .enumerate()
            .min_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((to, angle_deg)) = nearest
            && angle_deg < MIN_ANGLE_DEG
        {
            return Err(TooClose { to, angle_deg });
        }
        self.list.push(m);
        Ok(self.list.len() - 1)
    }

    /// Drops hold `index`; false if there is none.
    pub fn drop_hold(&mut self, index: usize) -> bool {
        let found = index < self.list.len();
        if found {
            self.list.remove(index);
        }
        found
    }

    pub fn clear(&mut self) {
        self.list.clear();
    }

    /// The largest reading seen in each of `DIRECTIONS`, g (0 if never on that side).
    pub fn coverage(&self) -> [f64; 6] {
        std::array::from_fn(|d| {
            let sign = if d % 2 == 0 { 1.0 } else { -1.0 };
            self.list
                .iter()
                .map(|h| sign * h.accel[d / 2] / 1000.0)
                .fold(0.0, f64::max)
        })
    }

    /// The directions not yet covered.
    pub fn missing(&self) -> Vec<&'static str> {
        let cov = self.coverage();
        (0..6)
            .filter(|&d| cov[d] < COVERAGE_G)
            .map(|d| DIRECTIONS[d])
            .collect()
    }

    /// The fit, once there are `FIT_FROM` holds and they span enough directions.
    pub fn solve(&self) -> Option<Solution> {
        if self.list.len() < FIT_FROM {
            return None;
        }
        let points: Vec<V3> = self
            .list
            .iter()
            .map(|h| h.accel.map(|x| x / 1000.0))
            .collect();
        let fit = calib::fit(&points)?;
        let residual_mg: Vec<f64> = points
            .iter()
            .map(|p| (norm(fit.apply(*p)) - 1.0) * 1000.0)
            .collect();
        Some(Solution {
            fit,
            before_mg: rms(points.iter().map(|p| (norm(*p) - 1.0) * 1000.0)),
            after_mg: rms(residual_mg.iter().copied()),
            residual_mg,
        })
    }

    /// Every gate of `docs/calibration.md`, passed or not. Hold numbers are 1-based.
    pub fn gates(&self) -> Vec<Gate> {
        let n = self.list.len();
        let missing = self.missing();
        let solution = self.solve();
        let mut gates = vec![
            Gate {
                name: "holds",
                pass: n >= MIN_HOLDS,
                detail: format!("{n} of at least {MIN_HOLDS}"),
            },
            Gate {
                name: "coverage",
                pass: missing.is_empty(),
                detail: if missing.is_empty() {
                    format!("every axis read {COVERAGE_G} g both ways")
                } else {
                    format!("missing: {}", missing.join(" "))
                },
            },
        ];
        match solution {
            None => gates.push(Gate {
                name: "fit",
                pass: false,
                detail: if n < FIT_FROM {
                    format!("needs {FIT_FROM} holds")
                } else {
                    "failed: the holds do not span enough directions".to_owned()
                },
            }),
            Some(s) => {
                let (worst, worst_mg) = s.worst();
                gates.push(Gate {
                    name: "residual rms",
                    pass: s.after_mg <= MAX_RMS_MG,
                    detail: format!("{:.1} mg, limit {MAX_RMS_MG}", s.after_mg),
                });
                gates.push(Gate {
                    name: "worst hold",
                    pass: worst_mg <= MAX_WORST_MG,
                    detail: format!("hold {}: {worst_mg:.1} mg, limit {MAX_WORST_MG}", worst + 1),
                });
                gates.push(Gate {
                    name: "gains",
                    pass: s.fit.gain.iter().all(|g| GAIN.contains(g)),
                    detail: format!(
                        "{:.4} {:.4} {:.4}, range {}..{}",
                        s.fit.gain[0],
                        s.fit.gain[1],
                        s.fit.gain[2],
                        GAIN.start(),
                        GAIN.end()
                    ),
                });
            }
        }
        gates
    }

    pub fn passed(&self) -> bool {
        self.gates().iter().all(|g| g.pass)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stage {
    Connect,
    Rest,
    Holds,
    /// All gates passed.
    Finished,
}

/// Why a measurement window was discarded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    BadFrame,
    LostFrame,
    TimeGap,
    Clipped,
}

impl fmt::Display for Fault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::BadFrame => "a frame did not decode",
            Self::LostFrame => "a frame was lost",
            Self::TimeGap => "a time gap between samples",
            Self::Clipped => "a clipped reading",
        })
    }
}

/// Why the run cannot go on with this stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fatal {
    NotStream(Firmware),
    Imu(ImuStatus),
    /// A scale or the sample rate is not configured.
    Scale,
    HelloChanged,
    Reset,
}

impl fmt::Display for Fatal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotStream(fw) => {
                write!(f, "the board runs the {fw:?} firmware; flash `stream`")
            }
            Self::Imu(imu) => write!(f, "the IMU is not ok ({imu:?}); check the wiring"),
            Self::Scale => f.write_str("the Hello names no known scale or sample rate"),
            Self::HelloChanged => f.write_str("the Hello changed during the run"),
            Self::Reset => f.write_str("the MCU restarted during the run"),
        }
    }
}

impl std::error::Error for Fatal {}

#[derive(Debug, Clone, PartialEq)]
pub enum Note {
    /// Connect passed: the `Hello` and the sample rate measured in MCU time, Hz.
    Connected {
        hello: Hello,
        rate_hz: f64,
    },
    /// A measurement in progress was lost.
    Moved,
    Discarded(Fault),
    /// The table rest is measured; its gyro mean is the bias.
    Rest(Measurement),
    /// A hold was accepted (index into `Holds::list`).
    Hold(usize),
    Rejected(Measurement, TooClose),
}

#[derive(Debug, Clone, Copy)]
struct Scale {
    mg_per_lsb: f64,
    dps_per_lsb: f64,
    window: usize,
}

/// One calibration run.
#[derive(Debug)]
pub struct Run {
    params: Params,
    stage: Stage,
    hello: Option<Hello>,
    scale: Option<Scale>,
    clock: Unwrapper,
    last: Option<Sample>,
    now: u64,
    /// MCU time of the first sample of the current step.
    step_t0: Option<u64>,
    /// Samples since `step_t0`, for the connect step's rate.
    step_samples: u64,
    tracker: Option<Tracker>,
    rest: Option<Measurement>,
    holds: Holds,
}

impl Run {
    pub fn new(params: Params) -> Self {
        Self {
            params,
            stage: Stage::Connect,
            hello: None,
            scale: None,
            clock: Unwrapper::new(),
            last: None,
            now: 0,
            step_t0: None,
            step_samples: 0,
            tracker: None,
            rest: None,
            holds: Holds::default(),
        }
    }

    pub fn stage(&self) -> Stage {
        self.stage
    }

    pub fn hello(&self) -> Option<&Hello> {
        self.hello.as_ref()
    }

    pub fn rest(&self) -> Option<&Measurement> {
        self.rest.as_ref()
    }

    pub fn holds(&self) -> &Holds {
        &self.holds
    }

    /// The still tracker of the rest or holds step.
    pub fn phase(&self) -> Phase {
        self.tracker.as_ref().map_or(Phase::Moving, Tracker::phase)
    }

    /// MCU time since the current step got its first sample, us.
    pub fn step_elapsed_us(&self) -> u64 {
        self.step_t0.map_or(0, |t0| self.now - t0)
    }

    fn enter(&mut self, stage: Stage) {
        self.stage = stage;
        self.step_t0 = None;
        self.step_samples = 0;
        let Some(scale) = self.scale else { return };
        let Params {
            settle_us,
            rest_us,
            hold_us,
        } = self.params;
        self.tracker = match stage {
            Stage::Rest => Some(Tracker::new(TABLE, None, scale.window, settle_us, rest_us)),
            Stage::Holds => Some(Tracker::new(
                HAND,
                self.rest.map(|r| r.gyro),
                scale.window,
                settle_us,
                hold_us,
            )),
            Stage::Connect | Stage::Finished => None,
        };
    }

    /// Starts the rest step again (a retry).
    pub fn restart_rest(&mut self) {
        if self.stage == Stage::Rest {
            self.enter(Stage::Rest);
        }
    }

    /// Drops a hold (index); false if there is none.
    pub fn drop_hold(&mut self, index: usize) -> bool {
        self.holds.drop_hold(index)
    }

    pub fn clear_holds(&mut self) {
        self.holds.clear();
    }

    /// Ends the holds step if every gate passes (after a hold was dropped, or on request);
    /// a new hold does this by itself. True once the run is finished.
    pub fn finish(&mut self) -> bool {
        if self.stage == Stage::Holds && self.holds.passed() {
            self.enter(Stage::Finished);
        }
        self.stage == Stage::Finished
    }

    /// A frame that failed its CRC or did not decode: a sample may be missing.
    pub fn bad_frame(&mut self) -> Option<Note> {
        self.fault(Fault::BadFrame)
    }

    fn fault(&mut self, fault: Fault) -> Option<Note> {
        let lost = self.tracker.as_mut().is_some_and(Tracker::fault);
        lost.then_some(Note::Discarded(fault))
    }

    pub fn message(&mut self, m: &Message) -> Result<Option<Note>, Fatal> {
        match m {
            Message::Hello(h) => self.on_hello(h).map(|()| None),
            Message::Sample(s) if self.hello.is_some() => self.on_sample(s),
            _ => Ok(None),
        }
    }

    fn on_hello(&mut self, h: &Hello) -> Result<(), Fatal> {
        if let Some(first) = self.hello {
            return if *h == first {
                Ok(())
            } else {
                Err(Fatal::HelloChanged)
            };
        }
        if h.fw != Firmware::Stream {
            return Err(Fatal::NotStream(h.fw));
        }
        if h.imu != ImuStatus::Ok {
            return Err(Fatal::Imu(h.imu));
        }
        let (Some(ug), Some(udps), Some(period_us)) = (
            h.accel_ug_per_lsb(),
            h.gyro_udps_per_lsb(),
            h.sample_period_us(),
        ) else {
            return Err(Fatal::Scale);
        };
        self.hello = Some(*h);
        self.scale = Some(Scale {
            mg_per_lsb: f64::from(ug) / 1e3,
            dps_per_lsb: f64::from(udps) / 1e6,
            window: ((WINDOW_S * 1e6 / f64::from(period_us)).round() as usize).max(2),
        });
        Ok(())
    }

    fn on_sample(&mut self, s: &Sample) -> Result<Option<Note>, Fatal> {
        let mut fault = None;
        if let Some(prev) = self.last {
            let dt = time::step(prev.t_us, s.t_us);
            if dt <= 0 {
                return Err(Fatal::Reset);
            }
            if s.seq.wrapping_sub(prev.seq) != 1 {
                fault = Some(Fault::LostFrame);
            } else if u64::from(dt.unsigned_abs()) > MAX_DT_US {
                fault = Some(Fault::TimeGap);
            }
        }
        self.last = Some(*s);
        self.now = self.clock.extend(s.t_us);
        self.step_t0.get_or_insert(self.now);
        self.step_samples += 1;
        let clipped = s
            .gyro
            .iter()
            .chain(&s.accel)
            .any(|&v| i32::from(v).abs() >= CLIP_LSB);
        if clipped {
            fault = Some(Fault::Clipped);
        }
        let mut note = fault.and_then(|f| self.fault(f));
        // A clipped sample is not measured; the sample after a gap starts a new window.
        if clipped {
            return Ok(note);
        }

        if self.stage == Stage::Connect {
            let elapsed = self.step_elapsed_us();
            if elapsed >= RATE_OVER_US
                && let Some(hello) = self.hello
            {
                let rate_hz = (self.step_samples - 1) as f64 * 1e6 / elapsed as f64;
                self.enter(Stage::Rest);
                return Ok(Some(Note::Connected { hello, rate_hz }));
            }
            return Ok(None);
        }
        let (Some(scale), Some(tracker)) = (self.scale, self.tracker.as_mut()) else {
            return Ok(note);
        };
        let gyro = s.gyro.map(|v| f64::from(v) * scale.dps_per_lsb);
        let accel = s.accel.map(|v| f64::from(v) * scale.mg_per_lsb);
        match tracker.feed(self.now, gyro, accel) {
            Tracked::Nothing => {}
            Tracked::Lost => note = Some(Note::Moved),
            Tracked::Done(m) => note = Some(self.measured(m)),
        }
        Ok(note)
    }

    fn measured(&mut self, m: Measurement) -> Note {
        match self.stage {
            Stage::Rest => {
                self.rest = Some(m);
                self.enter(Stage::Holds);
                Note::Rest(m)
            }
            _ => match self.holds.offer(m) {
                Ok(index) => {
                    if self.holds.passed() {
                        self.enter(Stage::Finished);
                    }
                    Note::Hold(index)
                }
                Err(close) => Note::Rejected(m, close),
            },
        }
    }

    /// The config the run gives: the fit and the rest bias in the integer formats, with the
    /// hand thresholds (they are not part of the file).
    pub fn config(&self) -> Option<Config> {
        let solution = self.holds.solve()?;
        Config::new(
            self.hello.as_ref()?,
            Some(&solution.fit),
            self.rest?.gyro,
            HAND,
        )
        .ok()
    }

    /// The calibration file of the run, whether or not the gates passed.
    pub fn calfile(&self) -> Option<CalFile> {
        Some(CalFile::from_gc(&self.config()?.gc()))
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use protocol::Deframer;

    use super::*;

    const PERIOD: u64 = 9600;
    const WINDOW: usize = 26;

    fn tracker() -> Tracker {
        Tracker::new(HAND, Some([0.0; 3]), WINDOW, 500_000, 2_000_000)
    }

    /// Feeds `n` samples from `*k` on; the accel x alternates by `wobble_mg`.
    fn feed(t: &mut Tracker, k: &mut u64, n: u64, wobble_mg: f64) -> Vec<Tracked> {
        (0..n)
            .map(|_| {
                let x = if k.is_multiple_of(2) {
                    wobble_mg
                } else {
                    -wobble_mg
                };
                let out = t.feed(*k * PERIOD, [0.1, 0.0, 0.0], [x, 0.0, 1000.0]);
                *k += 1;
                out
            })
            .filter(|o| *o != Tracked::Nothing)
            .collect()
    }

    #[test]
    fn tracker_measures_a_hold_once() {
        let mut t = tracker();
        let mut k = 0;
        assert!(feed(&mut t, &mut k, 100, 100.0).is_empty());
        assert_eq!(t.phase(), Phase::Moving);
        // The window fills with still samples, then 0.5 s settle + 2.0 s measured.
        let out = feed(&mut t, &mut k, 300, 1.0);
        let [Tracked::Done(m)] = out.as_slice() else {
            panic!("{out:?}");
        };
        let span = m.t_us.1 - m.t_us.0;
        assert!((2_000_000 - PERIOD..=2_000_000).contains(&span), "{span}");
        assert_eq!(m.samples as u64, span / PERIOD + 1);
        assert!((m.accel[2] - 1000.0).abs() < 1e-9 && m.accel[0].abs() < 0.1);
        assert!((m.accel_std[0] - 1.0).abs() < 0.01 && (m.gyro[0] - 0.1).abs() < 1e-9);
        // Still on: nothing more until the board moves.
        assert_eq!(t.phase(), Phase::Spent);
        assert!(feed(&mut t, &mut k, 300, 1.0).is_empty());
        assert!(feed(&mut t, &mut k, 30, 100.0).is_empty());
        assert_eq!(feed(&mut t, &mut k, 300, 1.0).len(), 1);
    }

    #[test]
    fn tracker_loses_a_measurement_to_movement_and_faults() {
        let mut t = tracker();
        let mut k = 0;
        assert!(feed(&mut t, &mut k, 150, 1.0).is_empty());
        assert!(matches!(t.phase(), Phase::Measuring { .. }));
        assert_eq!(feed(&mut t, &mut k, 20, 100.0), [Tracked::Lost]);
        // Settling only: movement loses nothing worth a note.
        assert!(feed(&mut t, &mut k, 40, 1.0).is_empty());
        assert_eq!(t.phase(), Phase::Settling);
        assert!(feed(&mut t, &mut k, 20, 100.0).is_empty());

        assert!(feed(&mut t, &mut k, 150, 1.0).is_empty());
        assert!(t.fault());
        assert_eq!(t.phase(), Phase::Moving);
        assert!(!t.fault());
        assert_eq!(feed(&mut t, &mut k, 300, 1.0).len(), 1);
        // A fault after the measurement does not measure the same hold again.
        assert!(!t.fault());
        assert!(feed(&mut t, &mut k, 300, 1.0).is_empty());
        assert_eq!(t.phase(), Phase::Spent);
    }

    #[test]
    fn tracker_rate_test_needs_the_bias() {
        // A steady 6 dps turn: quiet by spread, not by rate.
        let steady = |bias| {
            let mut t = Tracker::new(HAND, bias, WINDOW, 500_000, 2_000_000);
            (0..400u64)
                .filter(|k| {
                    t.feed(k * PERIOD, [6.0, 0.0, 0.0], [0.0, 0.0, 1000.0]) != Tracked::Nothing
                })
                .count()
        };
        assert_eq!(steady(Some([0.0; 3])), 0);
        assert_eq!(steady(Some([5.0, 0.0, 0.0])), 1);
        assert_eq!(steady(None), 1);
    }

    fn hold(accel_g: V3) -> Measurement {
        Measurement {
            t_us: (0, 0),
            samples: 1,
            gyro: [0.0; 3],
            accel: accel_g.map(|x| x * 1000.0),
            gyro_std: [0.0; 3],
            accel_std: [0.0; 3],
        }
    }

    /// 14 directions (faces and cube corners) read through this offset and gain.
    fn tumbled(offset: V3, gain: V3) -> Vec<Measurement> {
        let c = 1.0 / 3f64.sqrt();
        let mut dirs = vec![
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
        ];
        for k in 0..8 {
            let s = |bit: u32| if k >> bit & 1 == 0 { c } else { -c };
            dirs.push([s(0), s(1), s(2)]);
        }
        dirs.iter()
            .map(|d| hold(std::array::from_fn(|i| d[i] / gain[i] + offset[i])))
            .collect()
    }

    #[test]
    fn holds_separation_coverage_and_gates() {
        let all = tumbled([0.02, -0.01, 0.03], [1.02, 0.99, 0.97]);
        let mut holds = Holds::default();
        assert_eq!(holds.offer(all[0]), Ok(0));
        // 10 degrees off the first: too close.
        let near = hold([10f64.to_radians().cos(), 10f64.to_radians().sin(), 0.0]);
        let Err(close) = holds.offer(near) else {
            panic!("accepted");
        };
        assert_eq!(close.to, 0);
        assert!((close.angle_deg - 10.0).abs() < 1.5, "{}", close.angle_deg);

        for m in &all[1..5] {
            holds.offer(*m).expect("far enough");
        }
        assert_eq!(holds.missing(), ["-z"]);
        assert!(holds.solve().is_none());
        for m in &all[5..11] {
            holds.offer(*m).expect("far enough");
        }
        // 11 holds: everything but the count.
        let failing: Vec<_> = holds.gates().into_iter().filter(|g| !g.pass).collect();
        assert_eq!(failing.len(), 1, "{failing:?}");
        assert_eq!(failing[0].name, "holds");
        for m in &all[11..] {
            holds.offer(*m).expect("far enough");
        }
        assert!(holds.passed(), "{:?}", holds.gates());
        let s = holds.solve().expect("fit");
        assert!(s.after_mg < 1e-6 && s.before_mg > 10.0);
        assert!((s.fit.offset[2] - 0.03).abs() < 1e-9 && (s.fit.gain[0] - 1.02).abs() < 1e-9);

        // One disturbed hold fails the worst-hold gate and is named (1-based).
        let mut bad = holds.clone();
        bad.list[8].accel[0] += 40.0;
        let gates = bad.gates();
        let worst = gates.iter().find(|g| g.name == "worst hold").expect("gate");
        assert!(
            !worst.pass && worst.detail.starts_with("hold 9:"),
            "{worst:?}"
        );
        assert!(bad.drop_hold(8) && !bad.drop_hold(13));
        assert!(bad.passed());

        // A gain outside what gc_init takes.
        let mut wide = Holds::default();
        for m in tumbled([0.0; 3], [1.15, 1.0, 1.0]) {
            wide.offer(m).expect("far enough");
        }
        let gates = wide.gates();
        assert!(
            gates.iter().any(|g| g.name == "gains" && !g.pass),
            "{gates:?}"
        );
    }

    fn session(name: &str) -> Vec<u8> {
        let p = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../data/sessions")
            .join(name);
        std::fs::read(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
    }

    /// Feeds a recorded stream; panics on a fatal.
    fn replay(run: &mut Run, bytes: &[u8]) -> Vec<Note> {
        let mut notes = Vec::new();
        for frame in Deframer::new().feed(bytes) {
            let note = match frame.map(|p| Message::decode(&p)) {
                Ok(Ok(Some(m))) => run.message(&m).expect("no fatal"),
                Ok(Ok(None)) => None,
                _ => run.bad_frame(),
            };
            notes.extend(note);
        }
        notes
    }

    #[test]
    fn rest_step_on_the_table_session() {
        let mut run = Run::new(Params::default());
        let notes = replay(&mut run, &session("20261030-213745_rest-table-4g.bin"));
        // The board stays on the table after the rest: that is the first hold.
        let [
            Note::Connected { hello, rate_hz },
            Note::Rest(m),
            Note::Hold(0),
        ] = notes.as_slice()
        else {
            panic!("{notes:?}");
        };
        assert_eq!((hello.ctrl1_xl, hello.ctrl2_g), (0x48, 0x44));
        assert!((rate_hz - 104.2).abs() < 0.5, "{rate_hz}");
        // `stats` on the whole session: 1.817 -5.673 -0.240 dps.
        for (got, want) in m.gyro.iter().zip([1.817, -5.673, -0.240]) {
            assert!((got - want).abs() < 0.02, "{:?}", m.gyro);
        }
        let span = m.t_us.1 - m.t_us.0;
        assert!((9_980_000..=10_000_000).contains(&span), "{span}");
        assert!(m.gyro_std.iter().all(|&s| s < 0.1) && m.accel_std.iter().all(|&s| s < 2.0));
        assert_eq!(run.stage(), Stage::Holds);
        assert_eq!(run.holds().list().len(), 1);
    }

    /// `tumble-4g` is not a calibration run: no table rest, holds of 1 to 4 s, and x never
    /// near +-1 g. With the rest of `rest-table-4g` and shorter holds it gives the fit of
    /// `gesture-cli calib` and fails the coverage gate.
    #[test]
    fn holds_step_on_the_tumble_session() {
        let mut rest = Run::new(Params::default());
        replay(&mut rest, &session("20261030-213745_rest-table-4g.bin"));
        let bias = *rest.rest().expect("rest");

        let params = Params {
            settle_us: 200_000,
            hold_us: 400_000,
            ..Params::default()
        };
        let mut run = Run::new(params);
        // As if the rest step had just passed with that bias.
        run.params.rest_us = 0;
        let bytes = session("20261030-220345_tumble-4g.bin");
        let mut notes = Vec::new();
        for frame in Deframer::new().feed(&bytes) {
            let Ok(Ok(Some(m))) = frame.map(|p| Message::decode(&p)) else {
                continue;
            };
            notes.extend(run.message(&m).expect("no fatal"));
            if run.stage() == Stage::Holds && run.rest != Some(bias) {
                run.rest = Some(bias);
                run.enter(Stage::Holds);
            }
        }
        let holds = run.holds();
        let n = holds.list().len();
        assert!((12..=15).contains(&n), "{n} holds: {notes:?}");
        let s = holds.solve().expect("fit");
        // `calib`: 17.6 mg raw -> 1.4 mg fitted; offsets -4.3 +5.5 +25.1 mg.
        assert!((s.before_mg - 17.6).abs() < 2.0, "{}", s.before_mg);
        assert!(s.after_mg < 2.5, "{}", s.after_mg);
        for (got, want) in s.fit.offset.iter().zip([-4.3e-3, 5.5e-3, 25.1e-3]) {
            assert!((got - want).abs() < 4e-3, "{:?}", s.fit.offset);
        }
        let failing: Vec<_> = holds.gates().into_iter().filter(|g| !g.pass).collect();
        assert_eq!(failing.len(), 1, "{failing:?}");
        assert_eq!(failing[0].name, "coverage");
        assert!(failing[0].detail.contains("+x") && failing[0].detail.contains("-x"));
        assert_eq!(run.stage(), Stage::Holds);

        // The file of the run mirrors its config, and reads back.
        let file = run.calfile().expect("file");
        assert_eq!(CalFile::parse(&file.to_bytes()), Ok(file));
        let cfg = run.config().expect("config");
        let again = cfg.with_file(&file).expect("same scales").gc();
        assert_eq!(CalFile::from_gc(&again), file);
        assert!((cfg.bias_dps()[1] - bias.gyro[1]).abs() < 0.01);
    }
}
