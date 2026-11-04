//! Safe wrapper over the C ABI in `core/contract/include/gesture/core.h` (ABI v1, contract
//! in `docs/core.md`), linked to whichever implementation `GESTURE_CORE_IMPL` names
//! (`build.rs`). The MCU runs the same contract, so results must match bit for bit.
//!
//! The plain structs (`GcConfig`, `GcSample`, `GcPoint`, `GcStrokeInfo`) are their `core.h`
//! layouts, re-exported from [`ffi`]. Events, states and errors get Rust types here.
//!
//! Every output is checked against the contract on the way in: known codes, and the fields
//! `docs/core.md` says are 0 really 0. So the typed values hold everything the
//! implementation gave, and `GcEvent::from(event)` gives back its exact bytes (the
//! conformance digests in `gesture-cli` hash them). A violation is an implementation bug,
//! not an input error: it panics, naming the rule.

pub mod ffi;

use std::fmt;

use ffi::{GC_STATE_SIZE, GcEvent, GcState, GcStatus};
pub use ffi::{GcConfig, GcPoint, GcSample, GcStrokeInfo};

/// `gc_status` other than `GC_OK`: why `gc_init` refused the config.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// `GC_ERR_VERSION`: `abi_version` is not `GC_ABI_VERSION`.
    Version,
    /// `GC_ERR_SCALE`: an FS code not in the scale tables, or a ctrl value 0.
    Scale,
    /// `GC_ERR_CONFIG`: another value outside "Valid" (`docs/core.md`, `gc_config`).
    Config,
}

impl Error {
    /// The `gc_status` value.
    #[must_use]
    pub const fn status(self) -> GcStatus {
        match self {
            Self::Version => ffi::GC_ERR_VERSION,
            Self::Scale => ffi::GC_ERR_SCALE,
            Self::Config => ffi::GC_ERR_CONFIG,
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Version => "gc_init: wrong abi_version",
            Self::Scale => "gc_init: accel or gyro scale code not supported",
            Self::Config => "gc_init: a config value outside its valid range",
        })
    }
}

impl std::error::Error for Error {}

/// `GC_ST_*`: the state after a sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum State {
    /// No confirmed hold (after init, a time gap, or a long move).
    #[default]
    Warmup,
    /// In a hold.
    Still,
    /// In a stroke.
    Moving,
}

/// `GC_REJ_*`: why a move gave no stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reject {
    /// End hold: too short in time, or in extent.
    Short,
    /// At once: the move passed `max_stroke_us`.
    Long,
    /// End hold: a clipped reading up to the snapshot.
    Clipped,
    /// At once: a time gap during the move.
    TimeGap,
    /// End hold: leftover velocity above `max_v_end_mm_s`.
    Drift,
}

impl Reject {
    /// Reported at the end hold, with `v_end_mm_s` and `duration_us`; the others at once,
    /// with both 0.
    #[must_use]
    pub const fn at_end_hold(self) -> bool {
        matches!(self, Self::Short | Self::Clipped | Self::Drift)
    }
}

/// `GC_EV_*`, with the reason for `REJECTED`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Kind {
    /// Nothing changed.
    #[default]
    None,
    /// A hold confirmed in `Warmup`.
    Ready,
    /// A move started in `Still`.
    Moving,
    /// The end hold confirmed and the stroke passed the checks: read it with
    /// [`Core::stroke`].
    Stroke,
    /// The move gave no stroke.
    Rejected(Reject),
}

impl fmt::Display for Kind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::None => f.write_str("NONE"),
            Self::Ready => f.write_str("READY"),
            Self::Moving => f.write_str("MOVING"),
            Self::Stroke => f.write_str("STROKE"),
            Self::Rejected(r) => write!(f, "REJ {}", reject_name(*r)),
        }
    }
}

const fn reject_name(r: Reject) -> &'static str {
    match r {
        Reject::Short => "SHORT",
        Reject::Long => "LONG",
        Reject::Clipped => "CLIPPED",
        Reject::TimeGap => "TIME_GAP",
        Reject::Drift => "DRIFT",
    }
}

/// `gc_event`, checked. The numbers are 0 where `docs/core.md` says so: `points` outside
/// `Stroke`, `v_end_mm_s` and `duration_us` outside `Stroke` and the end-hold rejects.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Event {
    pub kind: Kind,
    pub state: State,
    /// `Stroke`: readable points.
    pub points: u16,
    /// `Stroke`, end-hold rejects: leftover velocity.
    pub v_end_mm_s: u16,
    /// `Stroke`, end-hold rejects: move length.
    pub duration_us: u32,
}

impl Event {
    #[must_use]
    pub fn is_none(&self) -> bool {
        self.kind == Kind::None
    }
}

/// A `gc_event` that breaks the contract: the rule it breaks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Violation(pub &'static str);

impl fmt::Display for Violation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "core.h contract violated: {}", self.0)
    }
}

impl std::error::Error for Violation {}

impl TryFrom<GcEvent> for Event {
    type Error = Violation;

    fn try_from(e: GcEvent) -> Result<Self, Violation> {
        let state = match e.state {
            ffi::GC_ST_WARMUP => State::Warmup,
            ffi::GC_ST_STILL => State::Still,
            ffi::GC_ST_MOVING => State::Moving,
            _ => return Err(Violation("gc_event.state not a GC_ST_* value")),
        };
        let reject = match e.reason {
            0 => None,
            ffi::GC_REJ_SHORT => Some(Reject::Short),
            ffi::GC_REJ_LONG => Some(Reject::Long),
            ffi::GC_REJ_CLIPPED => Some(Reject::Clipped),
            ffi::GC_REJ_TIME_GAP => Some(Reject::TimeGap),
            ffi::GC_REJ_DRIFT => Some(Reject::Drift),
            _ => return Err(Violation("gc_event.reason not 0 or a GC_REJ_* value")),
        };
        let kind = match (e.kind, reject) {
            (ffi::GC_EV_NONE, None) => Kind::None,
            (ffi::GC_EV_READY, None) => Kind::Ready,
            (ffi::GC_EV_MOVING, None) => Kind::Moving,
            (ffi::GC_EV_STROKE, None) => Kind::Stroke,
            (ffi::GC_EV_REJECTED, Some(r)) => Kind::Rejected(r),
            (ffi::GC_EV_REJECTED, None) => {
                return Err(Violation("gc_event.reason 0 for REJECTED"));
            }
            (ffi::GC_EV_NONE..=ffi::GC_EV_STROKE, Some(_)) => {
                return Err(Violation("gc_event.reason not 0 outside REJECTED"));
            }
            _ => return Err(Violation("gc_event.kind not a GC_EV_* value")),
        };
        if e.reserved != 0 {
            return Err(Violation("gc_event.reserved not 0"));
        }
        if kind != Kind::Stroke && e.points != 0 {
            return Err(Violation("gc_event.points not 0 outside STROKE"));
        }
        let ends = match kind {
            Kind::Stroke => true,
            Kind::Rejected(r) => r.at_end_hold(),
            _ => false,
        };
        if !ends && (e.v_end_mm_s != 0 || e.duration_us != 0) {
            return Err(Violation(
                "gc_event.v_end_mm_s or duration_us not 0 outside STROKE and end-hold rejects",
            ));
        }
        Ok(Self {
            kind,
            state,
            points: e.points,
            v_end_mm_s: e.v_end_mm_s,
            duration_us: e.duration_us,
        })
    }
}

impl From<Event> for GcEvent {
    /// The exact `gc_event` the event came from.
    fn from(e: Event) -> Self {
        let (kind, reason) = match e.kind {
            Kind::None => (ffi::GC_EV_NONE, 0),
            Kind::Ready => (ffi::GC_EV_READY, 0),
            Kind::Moving => (ffi::GC_EV_MOVING, 0),
            Kind::Stroke => (ffi::GC_EV_STROKE, 0),
            Kind::Rejected(r) => (
                ffi::GC_EV_REJECTED,
                match r {
                    Reject::Short => ffi::GC_REJ_SHORT,
                    Reject::Long => ffi::GC_REJ_LONG,
                    Reject::Clipped => ffi::GC_REJ_CLIPPED,
                    Reject::TimeGap => ffi::GC_REJ_TIME_GAP,
                    Reject::Drift => ffi::GC_REJ_DRIFT,
                },
            ),
        };
        Self {
            kind,
            state: match e.state {
                State::Warmup => ffi::GC_ST_WARMUP,
                State::Still => ffi::GC_ST_STILL,
                State::Moving => ffi::GC_ST_MOVING,
            },
            reason,
            reserved: 0,
            points: e.points,
            v_end_mm_s: e.v_end_mm_s,
            duration_us: e.duration_us,
        }
    }
}

/// The last stroke, from `gc_stroke`: its info and all its points.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Stroke {
    pub info: GcStrokeInfo,
    /// `info.points` of them, in the start frame, after the zero-velocity correction.
    pub points: Vec<GcPoint>,
}

/// One recognizer: the caller-owned `gc_state`, boxed (2 KB) so moves stay cheap and the
/// state never changes address (the contract does not say it may be copied).
#[derive(Debug)]
pub struct Core {
    state: Box<GcState>,
}

impl Core {
    /// `gc_init`: validates and copies `cfg`, resets everything.
    ///
    /// # Errors
    /// The `GC_ERR_*` status, as [`Error`].
    ///
    /// # Panics
    /// If the implementation returns a status `core.h` does not define.
    pub fn new(cfg: &GcConfig) -> Result<Self, Error> {
        let mut state = Box::new(GcState {
            opaque: [0; GC_STATE_SIZE],
        });
        // SAFETY: both pointers are valid for the call; GcState has the size and alignment
        // core.h requires (checked in ffi.rs).
        let status = unsafe { ffi::gc_init(&raw mut *state, cfg) };
        match status {
            ffi::GC_OK => Ok(Self { state }),
            ffi::GC_ERR_VERSION => Err(Error::Version),
            ffi::GC_ERR_SCALE => Err(Error::Scale),
            ffi::GC_ERR_CONFIG => Err(Error::Config),
            s => panic!("core.h contract violated: gc_init returned status {s}"),
        }
    }

    /// `gc_feed`: one sample, in sensor order.
    ///
    /// # Panics
    /// If the event breaks the contract ([`Violation`]).
    pub fn feed(&mut self, sample: &GcSample) -> Event {
        // SAFETY: valid pointers; the state was initialized by a successful gc_init.
        let raw = unsafe { ffi::gc_feed(&raw mut *self.state, sample) };
        Event::try_from(raw).unwrap_or_else(|v| panic!("{v}: {raw:?}"))
    }

    /// `gc_stroke`: the last stroke, after [`Kind::Stroke`] until the next [`Kind::Moving`].
    ///
    /// # Panics
    /// If `gc_stroke` breaks the contract (a nonzero info with no points, or a count other
    /// than `min(points, cap)`).
    #[must_use]
    pub fn stroke(&self) -> Option<Stroke> {
        // Info and the first point, then all of them: no buffer sized for the worst case.
        let (info, mut points) = self.stroke_into(1);
        if info.points == 0 {
            return None;
        }
        if info.points > 1 {
            (_, points) = self.stroke_into(info.points);
        }
        Some(Stroke { info, points })
    }

    /// One `gc_stroke` call with `cap` points of room, checked.
    fn stroke_into(&self, cap: u16) -> (GcStrokeInfo, Vec<GcPoint>) {
        let mut info = GcStrokeInfo::default();
        let mut points = vec![GcPoint::default(); usize::from(cap)];
        // SAFETY: valid pointers; `points` holds `cap` elements (cap >= 1, so non-dangling).
        let n = unsafe {
            ffi::gc_stroke(
                &raw const *self.state,
                &raw mut info,
                points.as_mut_ptr(),
                cap,
            )
        };
        if n == 0 && info != GcStrokeInfo::default() {
            panic!("core.h contract violated: gc_stroke returned 0 with a nonzero info: {info:?}");
        }
        if n != info.points.min(cap) {
            panic!(
                "core.h contract violated: gc_stroke returned {n} for {} points, cap {cap}",
                info.points
            );
        }
        points.truncate(usize::from(n));
        (info, points)
    }

    /// `gc_live`: the current point of the stroke (before the correction), `None` outside
    /// [`State::Moving`].
    ///
    /// # Panics
    /// If `gc_live` breaks the contract (a return other than 0 or 1, or a nonzero point
    /// with 0).
    #[must_use]
    pub fn live(&self) -> Option<GcPoint> {
        let mut p = GcPoint::default();
        // SAFETY: valid pointers; the state was initialized by a successful gc_init.
        let n = unsafe { ffi::gc_live(&raw const *self.state, &raw mut p) };
        match n {
            0 if p == GcPoint::default() => None,
            1 => Some(p),
            _ => panic!("core.h contract violated: gc_live returned {n} with {p:?}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ffi::GC_ABI_VERSION;

    #[test]
    fn init_checks_the_version_then_the_scales() {
        let cfg = GcConfig::default();
        assert_eq!(Core::new(&cfg).unwrap_err(), Error::Version);
        let cfg = GcConfig {
            abi_version: GC_ABI_VERSION,
            ..cfg
        };
        // Version right, the rest zero: a ctrl value 0 is the next failure.
        assert_eq!(Core::new(&cfg).unwrap_err(), Error::Scale);
    }

    #[test]
    fn errors_are_their_status() {
        assert_eq!(Error::Version.status(), ffi::GC_ERR_VERSION);
        assert_eq!(Error::Scale.status(), ffi::GC_ERR_SCALE);
        assert_eq!(Error::Config.status(), ffi::GC_ERR_CONFIG);
    }

    fn raw(kind: u8, reason: u8, points: u16, v_end_mm_s: u16, duration_us: u32) -> GcEvent {
        GcEvent {
            kind,
            state: ffi::GC_ST_STILL,
            reason,
            reserved: 0,
            points,
            v_end_mm_s,
            duration_us,
        }
    }

    #[test]
    fn every_valid_event_round_trips() {
        let valid = [
            raw(ffi::GC_EV_NONE, 0, 0, 0, 0),
            raw(ffi::GC_EV_READY, 0, 0, 0, 0),
            raw(ffi::GC_EV_MOVING, 0, 0, 0, 0),
            raw(ffi::GC_EV_STROKE, 0, 12, 34, 560_000),
            raw(ffi::GC_EV_REJECTED, ffi::GC_REJ_SHORT, 0, 3, 90_000),
            raw(ffi::GC_EV_REJECTED, ffi::GC_REJ_LONG, 0, 0, 0),
            raw(ffi::GC_EV_REJECTED, ffi::GC_REJ_CLIPPED, 0, 7, 400_000),
            raw(ffi::GC_EV_REJECTED, ffi::GC_REJ_TIME_GAP, 0, 0, 0),
            raw(ffi::GC_EV_REJECTED, ffi::GC_REJ_DRIFT, 0, 900, 700_000),
        ];
        for e in valid {
            for state in [ffi::GC_ST_WARMUP, ffi::GC_ST_STILL, ffi::GC_ST_MOVING] {
                let e = GcEvent { state, ..e };
                let typed = Event::try_from(e).expect("valid");
                assert_eq!(GcEvent::from(typed), e);
            }
        }
    }

    #[test]
    fn events_off_the_contract_are_refused() {
        let bad = [
            GcEvent {
                reserved: 1,
                ..raw(ffi::GC_EV_READY, 0, 0, 0, 0)
            },
            GcEvent {
                state: 3,
                ..raw(ffi::GC_EV_NONE, 0, 0, 0, 0)
            },
            raw(5, 0, 0, 0, 0),
            raw(ffi::GC_EV_MOVING, 0, 1, 0, 0),
            raw(ffi::GC_EV_MOVING, ffi::GC_REJ_SHORT, 0, 0, 0),
            raw(ffi::GC_EV_REJECTED, 0, 0, 0, 0),
            raw(ffi::GC_EV_REJECTED, 6, 0, 0, 0),
            raw(ffi::GC_EV_REJECTED, ffi::GC_REJ_SHORT, 2, 0, 0),
            raw(ffi::GC_EV_REJECTED, ffi::GC_REJ_LONG, 0, 0, 1),
            raw(ffi::GC_EV_REJECTED, ffi::GC_REJ_TIME_GAP, 0, 1, 0),
            raw(ffi::GC_EV_READY, 0, 0, 0, 1),
        ];
        for e in bad {
            assert!(Event::try_from(e).is_err(), "{e:?}");
        }
    }

    #[test]
    fn nothing_to_read_after_init() {
        // Unused scale bits zero: 2 g (CTRL1_XL FS 00) and 245 dps (CTRL2_G FS 00) at 104 Hz.
        let cfg = GcConfig {
            abi_version: GC_ABI_VERSION,
            ctrl1_xl: 0x40,
            ctrl2_g: 0x40,
            accel_gain_q30: [1 << 30; 3],
            still_gyro_std_lsb: 1,
            still_accel_std_lsb: 1,
            still_gyro_rate_lsb: 1,
            hold_n: 20,
            bias_gyro_std_lsb: 1,
            bias_accel_std_lsb: 1,
            clip_lsb: 1,
            max_dt_us: 20_000,
            min_stroke_us: 0,
            max_stroke_us: 1,
            ..GcConfig::default()
        };
        let core = Core::new(&cfg).expect("a valid config");
        assert_eq!(core.stroke(), None);
        assert_eq!(core.live(), None);
    }
}
