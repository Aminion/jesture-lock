//! A recorded session: the wire bytes decoded into samples on the unwrapped MCU clock, and
//! the same in physical units (for reading only; `core/` works in raw LSB).

use anyhow::{Context, bail};
use protocol::time::{self, Unwrapper};
use protocol::{Deframer, Hello, Message, Sample};

/// Standard gravity, m/s^2.
pub const G: f64 = 9.806_65;

#[derive(Debug)]
pub struct Recording {
    pub hello: Hello,
    /// Unwrapped MCU time, us, and the sample.
    pub samples: Vec<(u64, Sample)>,
}

/// Decodes a session file (raw wire bytes). Starts at the first Hello, as `check` does;
/// fails on a scale change or an MCU reset mid-session.
pub fn load(bytes: &[u8]) -> anyhow::Result<Recording> {
    let mut hello: Option<Hello> = None;
    let mut clock = Unwrapper::new();
    let mut last_t: Option<u32> = None;
    let mut samples = Vec::new();
    for frame in Deframer::new().feed(bytes) {
        let Ok(Some(m)) = frame.map(|p| Message::decode(&p)).unwrap_or(Ok(None)) else {
            continue;
        };
        match (m, hello) {
            (Message::Hello(h), None) => hello = Some(h),
            (Message::Hello(h), Some(first)) if h != first => bail!("Hello changed mid-session"),
            (Message::Sample(s), Some(_)) => {
                if last_t.is_some_and(|t| time::step(t, s.t_us) <= 0) {
                    bail!("MCU reset mid-session");
                }
                last_t = Some(s.t_us);
                samples.push((clock.extend(s.t_us), s));
            }
            _ => {}
        }
    }
    let hello = hello.context("no Hello in the file")?;
    Ok(Recording { hello, samples })
}

pub type V3 = [f64; 3];

pub fn mean(v: &[V3]) -> V3 {
    let n = v.len() as f64;
    std::array::from_fn(|i| v.iter().map(|x| x[i]).sum::<f64>() / n)
}

/// Per-axis median (upper middle for an even count).
pub fn median(v: &[V3]) -> V3 {
    std::array::from_fn(|i| {
        let mut axis: Vec<f64> = v.iter().map(|x| x[i]).collect();
        axis.sort_by(f64::total_cmp);
        axis[axis.len() / 2]
    })
}

pub fn std_dev(v: &[V3]) -> V3 {
    let m = mean(v);
    let n = v.len() as f64;
    std::array::from_fn(|i| (v.iter().map(|x| (x[i] - m[i]).powi(2)).sum::<f64>() / n).sqrt())
}

pub fn norm(v: V3) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

/// A recording in physical units, for reading only.
#[derive(Debug)]
pub struct Scaled {
    pub mg_per_lsb: f64,
    pub dps_per_lsb: f64,
    pub period_us: u32,
    /// Time since the first sample, s.
    pub t_s: Vec<f64>,
    pub gyro: Vec<V3>,
    pub accel: Vec<V3>,
}

pub fn scaled(rec: &Recording) -> anyhow::Result<Scaled> {
    let ug = rec
        .hello
        .accel_ug_per_lsb()
        .context("accelerometer not configured")?;
    let udps = rec
        .hello
        .gyro_udps_per_lsb()
        .context("gyro not configured")?;
    let period_us = rec.hello.sample_period_us().context("unknown ODR")?;
    if rec.samples.len() < 2 {
        bail!("fewer than 2 samples");
    }
    let mg_per_lsb = f64::from(ug) / 1000.0;
    let dps_per_lsb = f64::from(udps) / 1e6;
    let t0 = rec.samples[0].0;
    let to = |v: [i16; 3], k: f64| -> V3 { v.map(|x| f64::from(x) * k) };
    Ok(Scaled {
        mg_per_lsb,
        dps_per_lsb,
        period_us,
        t_s: rec
            .samples
            .iter()
            .map(|(t, _)| (t - t0) as f64 / 1e6)
            .collect(),
        gyro: rec
            .samples
            .iter()
            .map(|(_, s)| to(s.gyro, dps_per_lsb))
            .collect(),
        accel: rec
            .samples
            .iter()
            .map(|(_, s)| to(s.accel, mg_per_lsb))
            .collect(),
    })
}
