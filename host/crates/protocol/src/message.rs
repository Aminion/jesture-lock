//! Message payloads, `docs/protocol.md` "Payload header" and "Messages". Little-endian.

use std::fmt;

use crate::frame::{HEADER_BYTES, VERSION};

const TYPE_HELLO: u8 = 1;
const TYPE_SAMPLE: u8 = 2;
const TYPE_STATUS: u8 = 3;

/// Payload sizes, header included.
pub const HELLO_LEN: usize = 8;
pub const SAMPLE_LEN: usize = 20;
pub const STATUS_LEN: usize = 22;

/// `Hello.fw`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Firmware {
    Stream,
    Recognize,
    Unknown(u8),
}

impl From<u8> for Firmware {
    fn from(v: u8) -> Self {
        match v {
            1 => Self::Stream,
            2 => Self::Recognize,
            v => Self::Unknown(v),
        }
    }
}

impl From<Firmware> for u8 {
    fn from(fw: Firmware) -> Self {
        match fw {
            Firmware::Stream => 1,
            Firmware::Recognize => 2,
            Firmware::Unknown(v) => v,
        }
    }
}

/// `Hello.imu`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuStatus {
    /// No IMU answered.
    NotFound,
    Ok,
    UnknownWhoAmI,
    InitFailed,
    Unknown(u8),
}

impl From<u8> for ImuStatus {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::NotFound,
            1 => Self::Ok,
            2 => Self::UnknownWhoAmI,
            3 => Self::InitFailed,
            v => Self::Unknown(v),
        }
    }
}

impl From<ImuStatus> for u8 {
    fn from(imu: ImuStatus) -> Self {
        match imu {
            ImuStatus::NotFound => 0,
            ImuStatus::Ok => 1,
            ImuStatus::UnknownWhoAmI => 2,
            ImuStatus::InitFailed => 3,
            ImuStatus::Unknown(v) => v,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hello {
    pub fw: Firmware,
    pub imu: ImuStatus,
    /// 7-bit I2C address, 0 if none answered.
    pub imu_addr: u8,
    pub who_am_i: u8,
    /// LSM6DS3 CTRL1_XL as written (accel ODR, full scale), 0 if not configured.
    pub ctrl1_xl: u8,
    /// LSM6DS3 CTRL2_G as written (gyro ODR, full scale), 0 if not configured.
    pub ctrl2_g: u8,
}

impl Hello {
    /// Nominal time between `Sample`s from the gyro ODR in CTRL2_G (INT1 is gyro data
    /// ready). `None` if the gyro is off or the code is not a gyro ODR. The IMU clock
    /// is not exact: measured 9600 us against a nominal 9615 us.
    pub fn sample_period_us(&self) -> Option<u32> {
        // ODR_G[3:0] codes 1..8 (LSM6DS3 / LSM6DS3TR-C datasheets, CTRL2_G), in mHz.
        const ODR_MHZ: [u32; 8] = [
            12_500, 26_000, 52_000, 104_000, 208_000, 416_000, 833_000, 1_660_000,
        ];
        let code = usize::from(self.ctrl2_g >> 4);
        let odr = ODR_MHZ.get(code.checked_sub(1)?)?;
        Some(1_000_000_000 / odr)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sample {
    /// +1 per `Sample`, wraps; 0 after reset.
    pub seq: u16,
    /// MCU time of the INT1 rising edge, 1 us ticks, wraps.
    pub t_us: u32,
    /// x, y, z, raw LSB.
    pub gyro: [i16; 3],
    /// x, y, z, raw LSB.
    pub accel: [i16; 3],
}

/// Counters since reset, wrapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    pub t_us: u32,
    pub missed: u32,
    pub stuck: u32,
    pub i2c_err: u32,
    pub rx_err: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Message {
    Hello(Hello),
    Sample(Sample),
    Status(Status),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageError {
    /// Shorter than the header.
    TooShort,
    UnknownVersion(u8),
    /// Type 0, never sent.
    InvalidType,
    WrongLength {
        ty: u8,
        len: usize,
    },
}

impl fmt::Display for MessageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort => f.write_str("payload shorter than the header"),
            Self::UnknownVersion(v) => write!(f, "unknown protocol version {v}"),
            Self::InvalidType => f.write_str("message type 0"),
            Self::WrongLength { ty, len } => write!(f, "message type {ty} with length {len}"),
        }
    }
}

impl std::error::Error for MessageError {}

/// Reads little-endian fields in order; the caller has checked the length.
struct Reader<'a>(&'a [u8]);

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let (head, rest) = self.0.split_first_chunk().expect("length checked");
        self.0 = rest;
        *head
    }
    fn u8(&mut self) -> u8 {
        u8::from_le_bytes(self.take())
    }
    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }
    fn xyz(&mut self) -> [i16; 3] {
        [(); 3].map(|()| i16::from_le_bytes(self.take()))
    }
}

impl Message {
    /// `Ok(None)` for a message type this receiver does not know: skip it.
    pub fn decode(payload: &[u8]) -> Result<Option<Self>, MessageError> {
        let [ty, version, ..] = *payload else {
            return Err(MessageError::TooShort);
        };
        if version != VERSION {
            return Err(MessageError::UnknownVersion(version));
        }
        let expected = match ty {
            0 => return Err(MessageError::InvalidType),
            TYPE_HELLO => HELLO_LEN,
            TYPE_SAMPLE => SAMPLE_LEN,
            TYPE_STATUS => STATUS_LEN,
            _ => return Ok(None),
        };
        if payload.len() != expected {
            return Err(MessageError::WrongLength {
                ty,
                len: payload.len(),
            });
        }
        let mut r = Reader(&payload[HEADER_BYTES..]);
        Ok(Some(match ty {
            TYPE_HELLO => Self::Hello(Hello {
                fw: r.u8().into(),
                imu: r.u8().into(),
                imu_addr: r.u8(),
                who_am_i: r.u8(),
                ctrl1_xl: r.u8(),
                ctrl2_g: r.u8(),
            }),
            TYPE_SAMPLE => Self::Sample(Sample {
                seq: r.u16(),
                t_us: r.u32(),
                gyro: r.xyz(),
                accel: r.xyz(),
            }),
            _ => Self::Status(Status {
                t_us: r.u32(),
                missed: r.u32(),
                stuck: r.u32(),
                i2c_err: r.u32(),
                rx_err: r.u32(),
            }),
        }))
    }

    /// The payload, header included; pass it to `frame::encode`.
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(STATUS_LEN);
        match self {
            Self::Hello(h) => {
                out.extend([TYPE_HELLO, VERSION, h.fw.into(), h.imu.into()]);
                out.extend([h.imu_addr, h.who_am_i, h.ctrl1_xl, h.ctrl2_g]);
            }
            Self::Sample(s) => {
                out.extend([TYPE_SAMPLE, VERSION]);
                out.extend(s.seq.to_le_bytes());
                out.extend(s.t_us.to_le_bytes());
                out.extend(s.gyro.iter().chain(&s.accel).flat_map(|v| v.to_le_bytes()));
            }
            Self::Status(s) => {
                out.extend([TYPE_STATUS, VERSION]);
                for v in [s.t_us, s.missed, s.stuck, s.i2c_err, s.rx_err] {
                    out.extend(v.to_le_bytes());
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::tests::SAMPLE_PAYLOAD;

    const SAMPLE: Sample = Sample {
        seq: 258,
        t_us: 10_000_000,
        gyro: [250, -630, -55],
        accel: [-175, 50, 16900],
    };

    #[test]
    fn test_vector() {
        assert_eq!(
            Message::decode(&SAMPLE_PAYLOAD),
            Ok(Some(Message::Sample(SAMPLE)))
        );
        assert_eq!(Message::Sample(SAMPLE).encode(), SAMPLE_PAYLOAD);
    }

    // Same bytes as the "Messages" checks in proto/tests/proto_test.cpp.
    #[test]
    fn layouts_match_firmware() {
        let hello = Message::Hello(Hello {
            fw: Firmware::Stream,
            imu: ImuStatus::Ok,
            imu_addr: 0x6B,
            who_am_i: 0x69,
            ctrl1_xl: 0x40,
            ctrl2_g: 0x48,
        });
        let hello_bytes = [0x01, 0x01, 0x01, 0x01, 0x6B, 0x69, 0x40, 0x48];
        assert_eq!(hello.encode(), hello_bytes);
        assert_eq!(Message::decode(&hello_bytes), Ok(Some(hello)));

        let status = Message::Status(Status {
            t_us: 0x0403_0201,
            missed: 5,
            stuck: 0x0100,
            i2c_err: 0x00FF_0000,
            rx_err: 0xFFFF_FFFF,
        });
        let status_bytes = [
            0x03, 0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
            0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
        ];
        assert_eq!(status.encode(), status_bytes);
        assert_eq!(Message::decode(&status_bytes), Ok(Some(status)));
    }

    #[test]
    fn round_trips() {
        let messages = [
            Message::Hello(Hello {
                fw: Firmware::Stream,
                imu: ImuStatus::Ok,
                imu_addr: 0x6B,
                who_am_i: 0x6A,
                ctrl1_xl: 0x40,
                ctrl2_g: 0x40,
            }),
            Message::Hello(Hello {
                fw: Firmware::Unknown(7),
                imu: ImuStatus::Unknown(9),
                imu_addr: 0,
                who_am_i: 0,
                ctrl1_xl: 0,
                ctrl2_g: 0,
            }),
            Message::Sample(Sample {
                seq: u16::MAX,
                t_us: u32::MAX,
                gyro: [i16::MIN, 0, i16::MAX],
                accel: [-1, 1, i16::MIN],
            }),
            Message::Status(Status {
                t_us: 1,
                missed: 2,
                stuck: 3,
                i2c_err: 4,
                rx_err: u32::MAX,
            }),
        ];
        for m in messages {
            let payload = m.encode();
            let len = match m {
                Message::Hello(_) => HELLO_LEN,
                Message::Sample(_) => SAMPLE_LEN,
                Message::Status(_) => STATUS_LEN,
            };
            assert_eq!(payload.len(), len, "{m:?}");
            assert_eq!(Message::decode(&payload), Ok(Some(m)));
        }
    }

    #[test]
    fn errors() {
        assert_eq!(Message::decode(&[TYPE_SAMPLE]), Err(MessageError::TooShort));
        let mut payload = SAMPLE_PAYLOAD;
        payload[1] = 2;
        assert_eq!(
            Message::decode(&payload),
            Err(MessageError::UnknownVersion(2))
        );
        assert_eq!(
            Message::decode(&[0, VERSION]),
            Err(MessageError::InvalidType)
        );
        assert_eq!(
            Message::decode(&SAMPLE_PAYLOAD[..19]),
            Err(MessageError::WrongLength {
                ty: TYPE_SAMPLE,
                len: 19
            })
        );
        assert_eq!(
            Message::decode(&[TYPE_STATUS, VERSION]),
            Err(MessageError::WrongLength {
                ty: TYPE_STATUS,
                len: 2
            })
        );
    }

    #[test]
    fn sample_period() {
        let hello = |ctrl2_g| Hello {
            fw: Firmware::Stream,
            imu: ImuStatus::Ok,
            imu_addr: 0x6B,
            who_am_i: 0x69,
            ctrl1_xl: 0x40,
            ctrl2_g,
        };
        assert_eq!(hello(0x40).sample_period_us(), Some(9615)); // 104 Hz, +-250 dps
        assert_eq!(hello(0x4C).sample_period_us(), Some(9615)); // full scale bits ignored
        assert_eq!(hello(0x10).sample_period_us(), Some(80_000)); // 12.5 Hz
        assert_eq!(hello(0x80).sample_period_us(), Some(602)); // 1.66 kHz
        assert_eq!(hello(0x00).sample_period_us(), None); // power down
        assert_eq!(hello(0x90).sample_period_us(), None); // not a gyro ODR
    }

    // Unknown types are skipped, including Event (type 4, layout TBD).
    #[test]
    fn skips_unknown_types() {
        assert_eq!(Message::decode(&[4, VERSION, 1, 2, 3]), Ok(None));
        assert_eq!(Message::decode(&[200, VERSION]), Ok(None));
    }
}
