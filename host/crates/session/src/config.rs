//! The core's config for a session: from its `Hello` (scales), an accelerometer fit, the
//! initial gyro bias and the still thresholds. Kept in f64 for the float reference (`path`
//! in gesture-cli), already rounded to the integer formats so both decide on the same
//! numbers; `gc()` gives the `gc_config` for the integer core.

use anyhow::Context;
use gesture_core::ffi::{GC_ABI_VERSION, GcConfig};

use crate::calib::{Fit, Still};
use crate::recording::{Recording, V3};

// docs/core.md, gc_config defaults that are not still thresholds.
pub const HOLD_N: u32 = 20;
pub const BIAS_GYRO_STD_DPS: f64 = 0.2;
pub const BIAS_ACCEL_STD_MG: f64 = 2.0;
pub const CLIP_LSB: i32 = 32700;
pub const MAX_DT_US: u64 = 14423;
pub const MIN_STROKE_US: u64 = 150_000;
pub const MAX_STROKE_US: u64 = 4_000_000;
pub const MIN_EXTENT_MM: f64 = 20.0;
pub const MAX_V_END_MM_S: f64 = 300.0;
/// `gc_config` in the session's LSB, as the caller would fill it.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub ctrl1_xl: u8,
    pub ctrl2_g: u8,
    /// Sensitivities, ug/LSB and udps/LSB.
    pub sa: f64,
    pub sg: f64,
    pub offset_lsb: V3,
    pub gain: V3,
    pub bias_lsb: V3,
    pub still_gyro_std: f64,
    pub still_accel_std: f64,
    pub still_gyro_rate: f64,
    pub bias_gyro_std: f64,
    pub bias_accel_std: f64,
}

impl Config {
    /// `bias_dps`: the initial gyro bias; `fit`: accel calibration (none: offset 0, gain 1).
    pub fn new(
        rec: &Recording,
        fit: Option<&Fit>,
        bias_dps: V3,
        still: Still,
    ) -> anyhow::Result<Self> {
        let sa = f64::from(
            rec.hello
                .accel_ug_per_lsb()
                .context("accelerometer not configured")?,
        );
        let sg = f64::from(
            rec.hello
                .gyro_udps_per_lsb()
                .context("gyro not configured")?,
        );
        // Already in the integer formats of gc_config (whole LSB, Q30 gain, LSB x 256
        // bias), so this reference and the integer core decide on the same numbers.
        let gyro_lsb = |dps: f64| (dps * 1e6 / sg).round();
        let accel_lsb = |mg: f64| (mg * 1e3 / sa).round();
        let q30 = f64::from(1u32 << 30);
        Ok(Self {
            ctrl1_xl: rec.hello.ctrl1_xl,
            ctrl2_g: rec.hello.ctrl2_g,
            sa,
            sg,
            offset_lsb: fit.map_or([0.0; 3], |f| f.offset.map(|g| (g * 1e6 / sa).round())),
            gain: fit.map_or([1.0; 3], |f| f.gain.map(|g| (g * q30).round() / q30)),
            bias_lsb: bias_dps.map(|d| (d * 1e6 / sg * 256.0).round() / 256.0),
            still_gyro_std: gyro_lsb(still.gyro_std_dps),
            still_accel_std: accel_lsb(still.accel_std_mg),
            still_gyro_rate: gyro_lsb(still.gyro_mean_dps),
            bias_gyro_std: gyro_lsb(BIAS_GYRO_STD_DPS),
            bias_accel_std: accel_lsb(BIAS_ACCEL_STD_MG),
        })
    }

    /// The same config for the integer core, rounded to its formats.
    pub fn gc(&self) -> GcConfig {
        let lsb = |x: f64| x.round() as u16;
        GcConfig {
            abi_version: GC_ABI_VERSION,
            ctrl1_xl: self.ctrl1_xl,
            ctrl2_g: self.ctrl2_g,
            accel_offset_lsb: self.offset_lsb.map(|x| x.round() as i16),
            accel_gain_q30: self
                .gain
                .map(|g| (g * f64::from(1u32 << 30)).round() as i32),
            gyro_bias_q8: self.bias_lsb.map(|b| (b * 256.0).round() as i32),
            still_gyro_std_lsb: lsb(self.still_gyro_std),
            still_accel_std_lsb: lsb(self.still_accel_std),
            still_gyro_rate_lsb: lsb(self.still_gyro_rate),
            hold_n: HOLD_N as u16,
            bias_gyro_std_lsb: lsb(self.bias_gyro_std),
            bias_accel_std_lsb: lsb(self.bias_accel_std),
            clip_lsb: CLIP_LSB as u16,
            max_dt_us: MAX_DT_US as u16,
            min_stroke_us: MIN_STROKE_US as u32,
            max_stroke_us: MAX_STROKE_US as u32,
            min_extent_mm: MIN_EXTENT_MM as u16,
            max_v_end_mm_s: MAX_V_END_MM_S as u16,
            ..GcConfig::default()
        }
    }

    /// Calibrated accel, ug.
    pub fn accel_ug(&self, raw: [i16; 3]) -> V3 {
        std::array::from_fn(|i| (f64::from(raw[i]) - self.offset_lsb[i]) * self.gain[i] * self.sa)
    }
}
