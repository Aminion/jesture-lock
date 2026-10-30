//! `check`: stream health from decoded messages (`docs/protocol.md` "Gaps"). No port
//! access, so it is tested with made-up streams; `main` reads the port and feeds it.
//!
//! Counting starts at the first `Hello`: bytes before it may be a stale backlog the
//! ST-LINK kept from before the port was opened.

use std::fmt;

use protocol::time::{self, Unwrapper};
use protocol::{FrameError, Hello, Message, MessageError, Sample, Status};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct FrameErrors {
    pub cobs: u64,
    pub too_short: u64,
    pub too_long: u64,
    pub crc: u64,
}

impl FrameErrors {
    pub fn total(&self) -> u64 {
        self.cobs + self.too_short + self.too_long + self.crc
    }
}

/// `Status` counters, summed over the session and across MCU resets.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct McuCounts {
    pub missed: u64,
    pub stuck: u64,
    pub i2c_err: u64,
    pub rx_err: u64,
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct Totals {
    pub samples: u64,
    pub frame_errors: FrameErrors,
    pub message_errors: u64,
    /// Frames of an unknown type, skipped.
    pub skipped: u64,
    /// `Sample` frames lost on the way, from `seq` skips.
    pub lost: u64,
    /// Samples absent with `seq` continuous, from `t_us` steps: not read in time by the
    /// MCU (`missed`) or released by a stuck INT1 recovery (`stuck`).
    pub t_gaps: u64,
    pub resets: u64,
    pub mcu: McuCounts,
    /// Unwrapped MCU time of the first and last `Sample`, us.
    pub first_t: Option<u64>,
    pub last_t: Option<u64>,
}

impl Totals {
    /// Mean sample rate in MCU time, Hz.
    pub fn rate_hz(&self) -> Option<f64> {
        let span = self.last_t? - self.first_t?;
        (span > 0).then(|| (self.samples - 1) as f64 * 1e6 / span as f64)
    }
}

/// Samples since the last `take_interval`, and the range of steps between them.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Interval {
    pub samples: u64,
    /// Min and max `t_us` step between consecutive samples, us.
    pub dt: Option<(u32, u32)>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    Started(Hello),
    HelloChanged(Hello),
    /// `t_us` went backwards: the MCU restarted.
    Reset,
}

impl fmt::Display for Event {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Started(h) => write!(f, "hello: {}", describe(h)),
            Self::HelloChanged(h) => write!(f, "hello changed: {}", describe(h)),
            Self::Reset => f.write_str("MCU reset"),
        }
    }
}

fn describe(h: &Hello) -> String {
    let period = h
        .sample_period_us()
        .map_or("-".to_owned(), |p| format!("{p} us"));
    format!(
        "{:?} firmware, IMU {:?} at 0x{:02x} WHO_AM_I 0x{:02x}, CTRL1_XL 0x{:02x} CTRL2_G 0x{:02x}, period {period}",
        h.fw, h.imu, h.imu_addr, h.who_am_i, h.ctrl1_xl, h.ctrl2_g
    )
}

#[derive(Debug, Default)]
pub struct Stats {
    hello: Option<Hello>,
    last_sample: Option<Sample>,
    last_status: Option<Status>,
    clock: Unwrapper,
    totals: Totals,
    interval: Interval,
}

impl Stats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn started(&self) -> bool {
        self.hello.is_some()
    }

    pub fn totals(&self) -> &Totals {
        &self.totals
    }

    pub fn take_interval(&mut self) -> Interval {
        std::mem::take(&mut self.interval)
    }

    pub fn frame_error(&mut self, e: FrameError) {
        if !self.started() {
            return;
        }
        let errors = &mut self.totals.frame_errors;
        *match e {
            FrameError::Cobs => &mut errors.cobs,
            FrameError::TooShort => &mut errors.too_short,
            FrameError::TooLong => &mut errors.too_long,
            FrameError::Crc => &mut errors.crc,
        } += 1;
    }

    pub fn message_error(&mut self, _: MessageError) {
        if self.started() {
            self.totals.message_errors += 1;
        }
    }

    /// A frame of a type this receiver does not know.
    pub fn skipped(&mut self) {
        if self.started() {
            self.totals.skipped += 1;
        }
    }

    pub fn message(&mut self, m: &Message) -> Option<Event> {
        match m {
            Message::Hello(h) => self.on_hello(h),
            _ if !self.started() => None,
            Message::Sample(s) => self.on_sample(s),
            Message::Status(s) => {
                self.on_status(s);
                None
            }
        }
    }

    fn on_hello(&mut self, h: &Hello) -> Option<Event> {
        let event = match self.hello {
            None => Event::Started(*h),
            Some(old) if old != *h => Event::HelloChanged(*h),
            Some(_) => return None,
        };
        self.hello = Some(*h);
        Some(event)
    }

    fn on_sample(&mut self, s: &Sample) -> Option<Event> {
        let mut event = None;
        if let Some(prev) = self.last_sample {
            let dt = time::step(prev.t_us, s.t_us);
            if dt <= 0 {
                self.totals.resets += 1;
                self.clock.reset();
                event = Some(Event::Reset);
            } else {
                let dt = dt.unsigned_abs();
                let lost = s.seq.wrapping_sub(prev.seq).wrapping_sub(1);
                self.totals.lost += u64::from(lost);
                // Across lost frames the step says nothing about missed reads.
                let period = self.hello.and_then(|h| h.sample_period_us());
                if let (0, Some(p)) = (lost, period) {
                    let periods = (dt + p / 2) / p;
                    self.totals.t_gaps += u64::from(periods.saturating_sub(1));
                }
                self.interval.dt = Some(match self.interval.dt {
                    None => (dt, dt),
                    Some((lo, hi)) => (lo.min(dt), hi.max(dt)),
                });
            }
        }
        let t = self.clock.extend(s.t_us);
        self.totals.first_t.get_or_insert(t);
        self.totals.last_t = Some(t);
        self.totals.samples += 1;
        self.interval.samples += 1;
        self.last_sample = Some(*s);
        event
    }

    fn on_status(&mut self, s: &Status) {
        let mcu = &mut self.totals.mcu;
        let fields = |s: &Status| [s.missed, s.stuck, s.i2c_err, s.rx_err];
        let sums = [
            &mut mcu.missed,
            &mut mcu.stuck,
            &mut mcu.i2c_err,
            &mut mcu.rx_err,
        ];
        if let Some(prev) = self.last_status {
            // After an MCU reset the counters start again from 0: all of them are new.
            let reset = time::step(prev.t_us, s.t_us) <= 0;
            for ((sum, old), new) in sums.into_iter().zip(fields(&prev)).zip(fields(s)) {
                *sum += u64::from(if reset { new } else { new.wrapping_sub(old) });
            }
        }
        self.last_status = Some(*s);
    }

    /// Why the check fails; empty if the stream is healthy. `rx_err` is only reported:
    /// it counts noise on the input line, which does not affect the stream.
    pub fn failures(&self) -> Vec<String> {
        let Some(hello) = self.hello else {
            return vec!["no Hello received".to_owned()];
        };
        let t = &self.totals;
        let mut out = Vec::new();
        if hello.imu == protocol::ImuStatus::Ok && t.samples == 0 {
            out.push("no samples although the IMU is ok".to_owned());
        }
        let mut fail = |n: u64, what: &str| {
            if n > 0 {
                out.push(format!("{what}: {n}"));
            }
        };
        fail(t.frame_errors.total(), "frame errors");
        fail(t.message_errors, "message errors");
        fail(t.lost, "frames lost (seq skips)");
        fail(t.t_gaps, "samples absent (t_us gaps)");
        fail(t.resets, "MCU resets");
        fail(t.mcu.missed, "MCU missed samples");
        fail(t.mcu.stuck, "MCU stuck INT1 recoveries");
        fail(t.mcu.i2c_err, "MCU I2C errors");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::{Firmware, ImuStatus};

    const PERIOD: u32 = 9615; // 104 Hz

    const HELLO: Hello = Hello {
        fw: Firmware::Stream,
        imu: ImuStatus::Ok,
        imu_addr: 0x6B,
        who_am_i: 0x69,
        ctrl1_xl: 0x40,
        ctrl2_g: 0x40,
    };

    fn sample(seq: u16, t_us: u32) -> Message {
        Message::Sample(Sample {
            seq,
            t_us,
            gyro: [0; 3],
            accel: [0; 3],
        })
    }

    fn status(t_us: u32, missed: u32, rx_err: u32) -> Message {
        Message::Status(Status {
            t_us,
            missed,
            stuck: 0,
            i2c_err: 0,
            rx_err,
        })
    }

    fn started() -> Stats {
        let mut st = Stats::new();
        assert_eq!(
            st.message(&Message::Hello(HELLO)),
            Some(Event::Started(HELLO))
        );
        st
    }

    #[test]
    fn ignores_everything_before_hello() {
        let mut st = Stats::new();
        st.frame_error(FrameError::Crc);
        assert_eq!(st.message(&sample(0, 0)), None);
        assert_eq!(st.totals(), &Totals::default());
        assert_eq!(st.failures(), ["no Hello received"]);
    }

    #[test]
    fn clean_stream() {
        let mut st = started();
        for i in 0..1000u16 {
            st.message(&sample(i, u32::from(i) * PERIOD));
        }
        assert_eq!(st.message(&Message::Hello(HELLO)), None); // repeated, unchanged
        let t = st.totals();
        assert_eq!(t.samples, 1000);
        assert!((t.rate_hz().unwrap() - 1e6 / f64::from(PERIOD)).abs() < 1e-9);
        assert_eq!(
            st.take_interval(),
            Interval {
                samples: 1000,
                dt: Some((PERIOD, PERIOD))
            }
        );
        assert_eq!(st.take_interval(), Interval::default());
        assert!(st.failures().is_empty());
    }

    #[test]
    fn lost_frames_and_late_reads() {
        let mut st = started();
        st.message(&sample(10, 0));
        st.message(&sample(13, 3 * PERIOD)); // 2 frames lost
        st.message(&sample(14, 6 * PERIOD)); // seq continuous, 2 samples not read
        st.message(&sample(15, 7 * PERIOD + 100)); // jitter, not a gap
        let t = st.totals();
        assert_eq!((t.lost, t.t_gaps), (2, 2));
        assert_eq!(
            st.failures(),
            [
                "frames lost (seq skips): 2",
                "samples absent (t_us gaps): 2"
            ]
        );
    }

    #[test]
    fn seq_and_time_wrap() {
        let mut st = started();
        let t0 = u32::MAX - PERIOD;
        st.message(&sample(u16::MAX, t0));
        st.message(&sample(0, t0.wrapping_add(PERIOD)));
        st.message(&sample(1, t0.wrapping_add(2 * PERIOD)));
        let t = st.totals();
        assert_eq!((t.lost, t.t_gaps, t.resets), (0, 0, 0));
        assert_eq!(
            t.last_t.unwrap() - t.first_t.unwrap(),
            2 * u64::from(PERIOD)
        );
    }

    #[test]
    fn mcu_reset() {
        let mut st = started();
        st.message(&status(1_000_000, 0, 0));
        st.message(&sample(500, 5_000_000));
        st.message(&status(5_000_000, 2, 3));
        assert_eq!(st.message(&sample(0, 20_000)), Some(Event::Reset));
        st.message(&sample(1, 20_000 + PERIOD));
        st.message(&status(1_000_000, 1, 0)); // counters restarted
        let t = st.totals();
        assert_eq!((t.resets, t.lost, t.t_gaps), (1, 0, 0));
        assert_eq!((t.mcu.missed, t.mcu.rx_err), (3, 3));
    }

    #[test]
    fn errors_and_hello_change() {
        let mut st = started();
        st.frame_error(FrameError::Crc);
        st.frame_error(FrameError::Cobs);
        st.message_error(MessageError::UnknownVersion(2));
        st.skipped();
        st.message(&status(1, 0, 0));
        st.message(&status(2, 0, 7)); // rx_err alone does not fail
        let changed = Hello {
            imu: ImuStatus::InitFailed,
            ..HELLO
        };
        assert_eq!(
            st.message(&Message::Hello(changed)),
            Some(Event::HelloChanged(changed))
        );
        let t = st.totals();
        assert_eq!((t.frame_errors.total(), t.skipped, t.mcu.rx_err), (2, 1, 7));
        assert_eq!(st.failures(), ["frame errors: 2", "message errors: 1"]);
    }
}
