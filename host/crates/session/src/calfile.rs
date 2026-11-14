//! The calibration file (`docs/calibration.md`): one board's accelerometer offset and gain
//! and its initial gyro bias, as the final integers of `gc_config`, for the scales of the
//! run that measured them.

use std::fmt;

use gesture_core::ffi::GcConfig;
use protocol::Hello;
use protocol::frame::crc16;

pub const SIZE: usize = 44;
pub const MAGIC: [u8; 4] = *b"GCAL";
pub const VERSION: u8 = 1;

// Field offsets.
const VERSION_AT: usize = 4;
const CTRL1_XL_AT: usize = 5;
const CTRL2_G_AT: usize = 6;
const RESERVED0_AT: usize = 7;
const OFFSET_AT: usize = 8;
const RESERVED1_AT: usize = 14;
const GAIN_AT: usize = 16;
const BIAS_AT: usize = 28;
const RESERVED2_AT: usize = 40;
const CRC_AT: usize = 42;

/// The full-scale bits: FS_XL in CTRL1_XL; FS_G and FS_125 in CTRL2_G. The rest is ODR
/// (and, in CTRL1_XL, the filter bandwidth), which does not change the LSB.
pub const FS_XL_MASK: u8 = 0x0C;
pub const FS_G_MASK: u8 = 0x0E;

/// `accel_gain_q30` accepted by `gc_init` (`docs/core.md`): 0.9..1.1.
pub const GAIN_Q30: std::ops::RangeInclusive<i32> = 966_367_642..=1_181_116_006;

/// The board's calibration file, linked into every tool (`data/calibration/board.cal`,
/// written by `gesture-cli calibrate`).
pub const BOARD: &[u8] = include_bytes!("../../../../data/calibration/board.cal");

/// The linked file. It is checked by a test, so a tool never sees it fail.
pub fn board() -> Result<CalFile, Error> {
    CalFile::parse(BOARD)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CalFile {
    pub ctrl1_xl: u8,
    pub ctrl2_g: u8,
    pub accel_offset_lsb: [i16; 3],
    pub accel_gain_q30: [i32; 3],
    pub gyro_bias_q8: [i32; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Size(usize),
    Magic,
    Version(u8),
    Reserved,
    Crc,
    /// A gain outside `GAIN_Q30`.
    Gain(i32),
    /// A ctrl byte is 0: no scale.
    Scale,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Size(n) => write!(f, "{n} bytes, a calibration file has {SIZE}"),
            Self::Magic => f.write_str("not a calibration file (magic)"),
            Self::Version(v) => write!(
                f,
                "calibration file version {v}, this reader knows {VERSION}"
            ),
            Self::Reserved => f.write_str("a reserved field is not 0"),
            Self::Crc => f.write_str("CRC mismatch"),
            Self::Gain(g) => write!(f, "accel gain {g} (Q30) is outside 0.9..1.1"),
            Self::Scale => f.write_str("a ctrl byte is 0: no scale"),
        }
    }
}

impl std::error::Error for Error {}

/// A ctrl pair selects scales the core knows, as `gc_init` checks (`GC_ERR_SCALE`): every
/// FS code is in the tables of `docs/core.md`, so only a ctrl value 0 is left to reject.
fn scales_known(ctrl1_xl: u8, ctrl2_g: u8) -> bool {
    let h = hello_with(ctrl1_xl, ctrl2_g);
    h.accel_ug_per_lsb().is_some() && h.gyro_udps_per_lsb().is_some()
}

fn hello_with(ctrl1_xl: u8, ctrl2_g: u8) -> Hello {
    Hello {
        fw: protocol::Firmware::Stream,
        imu: protocol::ImuStatus::Ok,
        imu_addr: 0,
        who_am_i: 0,
        ctrl1_xl,
        ctrl2_g,
    }
}

impl CalFile {
    /// The calibration fields of a `gc_config`.
    pub fn from_gc(c: &GcConfig) -> Self {
        Self {
            ctrl1_xl: c.ctrl1_xl,
            ctrl2_g: c.ctrl2_g,
            accel_offset_lsb: c.accel_offset_lsb,
            accel_gain_q30: c.accel_gain_q30,
            gyro_bias_q8: c.gyro_bias_q8,
        }
    }

    /// What `parse` checks beyond the bytes themselves.
    pub fn validate(&self) -> Result<(), Error> {
        if let Some(&g) = self.accel_gain_q30.iter().find(|g| !GAIN_Q30.contains(g)) {
            return Err(Error::Gain(g));
        }
        if !scales_known(self.ctrl1_xl, self.ctrl2_g) {
            return Err(Error::Scale);
        }
        Ok(())
    }

    pub fn to_bytes(&self) -> [u8; SIZE] {
        let mut b = [0u8; SIZE];
        b[..4].copy_from_slice(&MAGIC);
        b[VERSION_AT] = VERSION;
        b[CTRL1_XL_AT] = self.ctrl1_xl;
        b[CTRL2_G_AT] = self.ctrl2_g;
        for i in 0..3 {
            b[OFFSET_AT + 2 * i..][..2].copy_from_slice(&self.accel_offset_lsb[i].to_le_bytes());
            b[GAIN_AT + 4 * i..][..4].copy_from_slice(&self.accel_gain_q30[i].to_le_bytes());
            b[BIAS_AT + 4 * i..][..4].copy_from_slice(&self.gyro_bias_q8[i].to_le_bytes());
        }
        let crc = crc16(&b[..CRC_AT]);
        b[CRC_AT..].copy_from_slice(&crc.to_le_bytes());
        b
    }

    pub fn parse(bytes: &[u8]) -> Result<Self, Error> {
        let b: &[u8; SIZE] = bytes.try_into().map_err(|_| Error::Size(bytes.len()))?;
        if b[..4] != MAGIC {
            return Err(Error::Magic);
        }
        if b[VERSION_AT] != VERSION {
            return Err(Error::Version(b[VERSION_AT]));
        }
        let u16_at = |at: usize| u16::from_le_bytes([b[at], b[at + 1]]);
        if u16_at(CRC_AT) != crc16(&b[..CRC_AT]) {
            return Err(Error::Crc);
        }
        if b[RESERVED0_AT] != 0 || u16_at(RESERVED1_AT) != 0 || u16_at(RESERVED2_AT) != 0 {
            return Err(Error::Reserved);
        }
        let i32_at = |at: usize| i32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]]);
        let file = Self {
            ctrl1_xl: b[CTRL1_XL_AT],
            ctrl2_g: b[CTRL2_G_AT],
            accel_offset_lsb: std::array::from_fn(|i| u16_at(OFFSET_AT + 2 * i).cast_signed()),
            accel_gain_q30: std::array::from_fn(|i| i32_at(GAIN_AT + 4 * i)),
            gyro_bias_q8: std::array::from_fn(|i| i32_at(BIAS_AT + 4 * i)),
        };
        file.validate()?;
        Ok(file)
    }

    /// The scale match: the file's values are in the LSB of a stream with these ctrl bytes.
    pub fn applies_to(&self, ctrl1_xl: u8, ctrl2_g: u8) -> bool {
        (self.ctrl1_xl ^ ctrl1_xl) & FS_XL_MASK == 0 && (self.ctrl2_g ^ ctrl2_g) & FS_G_MASK == 0
    }

    /// The scales the file is for, for messages.
    pub fn scales(&self) -> String {
        let h = hello_with(self.ctrl1_xl, self.ctrl2_g);
        describe_scales(&h)
    }
}

/// "0.122 mg/LSB, 17.5 mdps/LSB", or "-" for a scale that is not configured.
pub fn describe_scales(h: &Hello) -> String {
    let per = |v: Option<u32>, unit: &str| {
        v.map_or("-".to_owned(), |v| {
            format!("{} {unit}/LSB", f64::from(v) / 1000.0)
        })
    };
    format!(
        "{}, {}",
        per(h.accel_ug_per_lsb(), "mg"),
        per(h.gyro_udps_per_lsb(), "mdps")
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    const FILE: CalFile = CalFile {
        ctrl1_xl: 0x48,
        ctrl2_g: 0x44,
        accel_offset_lsb: [-35, 45, 205],
        accel_gain_q30: [1_083_727_709, 1_072_668_082, 1_068_051_012],
        gyro_bias_q8: [26_581, -82_990, -3_511],
    };

    /// `bytes` with `change` applied and the CRC made right again.
    fn patched(change: impl FnOnce(&mut [u8; SIZE])) -> [u8; SIZE] {
        let mut b = FILE.to_bytes();
        change(&mut b);
        let crc = crc16(&b[..CRC_AT]);
        b[CRC_AT..].copy_from_slice(&crc.to_le_bytes());
        b
    }

    #[test]
    fn linked_board_file_is_valid() {
        let file = board().expect("data/calibration/board.cal");
        // The scales the firmware configures (`lsm6ds3.hpp`).
        assert!(file.applies_to(0x48, 0x44));
    }

    #[test]
    fn round_trip_and_layout() {
        let b = FILE.to_bytes();
        assert_eq!(CalFile::parse(&b), Ok(FILE));
        assert_eq!(&b[..8], b"GCAL\x01\x48\x44\x00");
        assert_eq!(b[8..10], (-35i16).to_le_bytes());
        assert_eq!(b[14..16], [0, 0]);
        assert_eq!(b[24..28], 1_068_051_012i32.to_le_bytes());
        assert_eq!(b[28..32], 26_581i32.to_le_bytes());
        assert_eq!(b[40..42], [0, 0]);
    }

    /// Bytes 8..39 are `gc_config` bytes 8..39.
    #[test]
    fn mirrors_gc_config() {
        let gc = GcConfig {
            ctrl1_xl: FILE.ctrl1_xl,
            ctrl2_g: FILE.ctrl2_g,
            accel_offset_lsb: FILE.accel_offset_lsb,
            accel_gain_q30: FILE.accel_gain_q30,
            gyro_bias_q8: FILE.gyro_bias_q8,
            ..GcConfig::default()
        };
        assert_eq!(CalFile::from_gc(&gc), FILE);
        assert_eq!(std::mem::offset_of!(GcConfig, accel_offset_lsb), OFFSET_AT);
        assert_eq!(std::mem::offset_of!(GcConfig, accel_gain_q30), GAIN_AT);
        assert_eq!(std::mem::offset_of!(GcConfig, gyro_bias_q8), BIAS_AT);
        assert_eq!(
            std::mem::offset_of!(GcConfig, still_gyro_std_lsb),
            RESERVED2_AT
        );
    }

    #[test]
    fn rejections() {
        let good = FILE.to_bytes();
        assert_eq!(CalFile::parse(&good[..43]), Err(Error::Size(43)));
        assert_eq!(
            CalFile::parse(&[good.as_slice(), &[0]].concat()),
            Err(Error::Size(45))
        );
        assert_eq!(CalFile::parse(&patched(|b| b[0] = b'X')), Err(Error::Magic));
        assert_eq!(
            CalFile::parse(&patched(|b| b[VERSION_AT] = 2)),
            Err(Error::Version(2))
        );
        for at in [RESERVED0_AT, RESERVED1_AT, RESERVED1_AT + 1, RESERVED2_AT] {
            assert_eq!(
                CalFile::parse(&patched(|b| b[at] = 1)),
                Err(Error::Reserved),
                "{at}"
            );
        }
        let mut flipped = good;
        flipped[20] ^= 1;
        assert_eq!(CalFile::parse(&flipped), Err(Error::Crc));
        let mut crc = good;
        crc[CRC_AT] ^= 1;
        assert_eq!(CalFile::parse(&crc), Err(Error::Crc));

        let lo = *GAIN_Q30.start() - 1;
        let hi = *GAIN_Q30.end() + 1;
        for g in [lo, hi] {
            let b = patched(|b| b[GAIN_AT + 4..][..4].copy_from_slice(&g.to_le_bytes()));
            assert_eq!(CalFile::parse(&b), Err(Error::Gain(g)));
        }
        for g in [*GAIN_Q30.start(), *GAIN_Q30.end()] {
            let b = patched(|b| b[GAIN_AT..][..4].copy_from_slice(&g.to_le_bytes()));
            assert!(CalFile::parse(&b).is_ok());
        }
        assert_eq!(
            CalFile::parse(&patched(|b| b[CTRL1_XL_AT] = 0)),
            Err(Error::Scale)
        );
        assert_eq!(
            CalFile::parse(&patched(|b| b[CTRL2_G_AT] = 0)),
            Err(Error::Scale)
        );
    }

    #[test]
    fn scale_match_ignores_odr() {
        assert!(FILE.applies_to(0x48, 0x44));
        assert!(FILE.applies_to(0x68, 0x54)); // other ODRs
        assert!(FILE.applies_to(0x4B, 0x44)); // accel bandwidth bits
        assert!(!FILE.applies_to(0x40, 0x44)); // +-2 g
        assert!(!FILE.applies_to(0x48, 0x40)); // 250 dps
        assert!(!FILE.applies_to(0x48, 0x46)); // FS_125
    }
}
