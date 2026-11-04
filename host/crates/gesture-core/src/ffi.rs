//! Raw `extern "C"` declarations mirroring `core/contract/include/gesture/core.h` (ABI v1,
//! contract in `docs/core.md`). Written by hand: every struct below has its size,
//! alignment and field offsets checked at compile time against the numbers in
//! `core.h`/`docs/core.md`, so a drift between the two fails the build.

use core::mem::{align_of, offset_of, size_of};

pub const GC_ABI_VERSION: u32 = 1;
pub const GC_STATE_SIZE: usize = 2048;
pub const GC_STATE_ALIGN: usize = 8;

pub type GcStatus = i32;
pub const GC_OK: GcStatus = 0;
pub const GC_ERR_VERSION: GcStatus = 1;
pub const GC_ERR_SCALE: GcStatus = 2;
pub const GC_ERR_CONFIG: GcStatus = 3;

pub const GC_EV_NONE: u8 = 0;
pub const GC_EV_READY: u8 = 1;
pub const GC_EV_MOVING: u8 = 2;
pub const GC_EV_STROKE: u8 = 3;
pub const GC_EV_REJECTED: u8 = 4;

pub const GC_ST_WARMUP: u8 = 0;
pub const GC_ST_STILL: u8 = 1;
pub const GC_ST_MOVING: u8 = 2;

pub const GC_REJ_SHORT: u8 = 1;
pub const GC_REJ_LONG: u8 = 2;
pub const GC_REJ_CLIPPED: u8 = 3;
pub const GC_REJ_TIME_GAP: u8 = 4;
pub const GC_REJ_DRIFT: u8 = 5;

/// `gc_state`: opaque, owned by the caller.
#[repr(C, align(8))]
#[derive(Debug)]
pub struct GcState {
    pub opaque: [u8; GC_STATE_SIZE],
}

/// `gc_sample`: the fields of the wire Sample (`docs/protocol.md`).
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GcSample {
    pub t_us: u32,
    pub gyro: [i16; 3],
    pub accel: [i16; 3],
}

/// `gc_config`: calibration and thresholds, in LSB of the selected scale.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GcConfig {
    pub abi_version: u32,
    pub ctrl1_xl: u8,
    pub ctrl2_g: u8,
    pub reserved0: u16,
    pub accel_offset_lsb: [i16; 3],
    pub reserved1: u16,
    pub accel_gain_q30: [i32; 3],
    pub gyro_bias_q8: [i32; 3],
    pub still_gyro_std_lsb: u16,
    pub still_accel_std_lsb: u16,
    pub still_gyro_rate_lsb: u16,
    pub hold_n: u16,
    pub bias_gyro_std_lsb: u16,
    pub bias_accel_std_lsb: u16,
    pub clip_lsb: u16,
    pub max_dt_us: u16,
    pub min_stroke_us: u32,
    pub max_stroke_us: u32,
    pub min_extent_mm: u16,
    pub max_v_end_mm_s: u16,
}

/// `gc_event`: `state` is the state after the sample.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GcEvent {
    pub kind: u8,
    pub state: u8,
    pub reason: u8,
    pub reserved: u8,
    pub points: u16,
    pub v_end_mm_s: u16,
    pub duration_us: u32,
}

/// `gc_point`: in the start frame (the board's axes at the stroke start).
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GcPoint {
    pub x_mm: i16,
    pub y_mm: i16,
    pub z_mm: i16,
    pub t_100us: u16,
}

/// `gc_stroke_info`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GcStrokeInfo {
    pub duration_us: u32,
    pub points: u16,
    pub v_end_mm_s: u16,
    pub gravity_ug: [i32; 3],
    pub extent_mm: u16,
    pub step: u16,
}

unsafe extern "C" {
    pub fn gc_init(s: *mut GcState, cfg: *const GcConfig) -> GcStatus;
    pub fn gc_feed(s: *mut GcState, input: *const GcSample) -> GcEvent;
    pub fn gc_stroke(
        s: *const GcState,
        info: *mut GcStrokeInfo,
        out: *mut GcPoint,
        cap: u16,
    ) -> u16;
    pub fn gc_live(s: *const GcState, out: *mut GcPoint) -> u16;
}

/// Compile-time layout check: size, alignment, then every field's offset.
macro_rules! layout {
    ($t:ty, size $size:expr, align $align:expr; $($f:ident @ $off:expr),* $(,)?) => {
        const _: () = assert!(size_of::<$t>() == $size);
        const _: () = assert!(align_of::<$t>() == $align);
        $(const _: () = assert!(offset_of!($t, $f) == $off);)*
    };
}

layout!(GcState, size GC_STATE_SIZE, align GC_STATE_ALIGN; opaque @ 0);
layout!(GcSample, size 16, align 4; t_us @ 0, gyro @ 4, accel @ 10);
layout!(GcConfig, size 68, align 4;
    abi_version @ 0, ctrl1_xl @ 4, ctrl2_g @ 5, reserved0 @ 6, accel_offset_lsb @ 8,
    reserved1 @ 14, accel_gain_q30 @ 16, gyro_bias_q8 @ 28, still_gyro_std_lsb @ 40,
    still_accel_std_lsb @ 42, still_gyro_rate_lsb @ 44, hold_n @ 46,
    bias_gyro_std_lsb @ 48, bias_accel_std_lsb @ 50, clip_lsb @ 52, max_dt_us @ 54,
    min_stroke_us @ 56, max_stroke_us @ 60, min_extent_mm @ 64, max_v_end_mm_s @ 66);
layout!(GcEvent, size 12, align 4;
    kind @ 0, state @ 1, reason @ 2, reserved @ 3, points @ 4, v_end_mm_s @ 6,
    duration_us @ 8);
layout!(GcPoint, size 8, align 2; x_mm @ 0, y_mm @ 2, z_mm @ 4, t_100us @ 6);
layout!(GcStrokeInfo, size 24, align 4;
    duration_us @ 0, points @ 4, v_end_mm_s @ 6, gravity_ug @ 8, extent_mm @ 20,
    step @ 22);
