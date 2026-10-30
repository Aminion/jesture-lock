//! Frames on the wire: `COBS(payload || crc16) 0x00`, `docs/protocol.md` "Framing".
//! Mirrors `proto/include/gesture/proto/frame.hpp`: same limits, same errors.

use std::fmt;

use crc::{CRC_16_IBM_3740, Crc};

use crate::cobs::{self, CobsError};

pub const VERSION: u8 = 1;

/// type, version
pub const HEADER_BYTES: usize = 2;
pub const MAX_PAYLOAD: usize = 32;
pub const CRC_BYTES: usize = 2;
/// A frame after COBS decoding: payload + CRC.
pub const MAX_DECODED: usize = MAX_PAYLOAD + CRC_BYTES;
/// A frame before COBS decoding, without the delimiter.
pub const MAX_ENCODED: usize = cobs::max_encoded(MAX_DECODED);
/// Encoded payload + CRC + the 0x00 delimiter: 36 bytes.
pub const MAX_FRAME: usize = MAX_ENCODED + 1;

const CRC16: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_3740);

/// CRC-16/IBM-3740 (CRC-16/CCITT-FALSE) over a payload.
pub fn crc16(data: &[u8]) -> u16 {
    CRC16.checksum(data)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// COBS decoding failed.
    Cobs,
    /// Fewer than header + CRC bytes.
    TooShort,
    /// More than `MAX_PAYLOAD` + CRC bytes.
    TooLong,
    /// CRC mismatch.
    Crc,
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Cobs => "bad COBS encoding",
            Self::TooShort => "frame too short",
            Self::TooLong => "frame too long",
            Self::Crc => "CRC mismatch",
        })
    }
}

impl std::error::Error for FrameError {}

/// The whole frame, delimiter included. `TooLong` if the payload exceeds `MAX_PAYLOAD`.
pub fn encode(payload: &[u8]) -> Result<Vec<u8>, FrameError> {
    if payload.len() > MAX_PAYLOAD {
        return Err(FrameError::TooLong);
    }
    let raw = [payload, &crc16(payload).to_le_bytes()].concat();
    let mut out = cobs::encode(&raw);
    out.push(0);
    Ok(out)
}

/// Decodes one frame, given without its 0x00 delimiter. Returns the payload, CRC stripped.
pub fn decode(encoded: &[u8]) -> Result<Vec<u8>, FrameError> {
    let mut raw = cobs::decode(encoded, MAX_DECODED).map_err(|e| match e {
        CobsError::Overflow => FrameError::TooLong,
        CobsError::ZeroByte | CobsError::Truncated => FrameError::Cobs,
    })?;
    if raw.len() < HEADER_BYTES + CRC_BYTES {
        return Err(FrameError::TooShort);
    }
    let len = raw.len() - CRC_BYTES;
    if crc16(&raw[..len]).to_le_bytes() != raw[len..] {
        return Err(FrameError::Crc);
    }
    raw.truncate(len);
    Ok(raw)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    // docs/protocol.md "Test vector": Sample, seq 258, t_us 10 000 000,
    // gyro (250, -630, -55), accel (-175, 50, 16900).
    pub(crate) const SAMPLE_PAYLOAD: [u8; 20] = [
        0x02, 0x01, 0x02, 0x01, 0x80, 0x96, 0x98, 0x00, 0xfa, 0x00, 0x8a, 0xfd, 0xc9, 0xff, 0x51,
        0xff, 0x32, 0x00, 0x04, 0x42,
    ];
    pub(crate) const SAMPLE_WIRE: [u8; 24] = [
        0x08, 0x02, 0x01, 0x02, 0x01, 0x80, 0x96, 0x98, 0x02, 0xfa, 0x08, 0x8a, 0xfd, 0xc9, 0xff,
        0x51, 0xff, 0x32, 0x05, 0x04, 0x42, 0x8d, 0xe9, 0x00,
    ];

    /// Decodes wire bytes that end in the delimiter.
    fn decode_wire(wire: &[u8]) -> Result<Vec<u8>, FrameError> {
        decode(&wire[..wire.len() - 1])
    }

    #[test]
    fn crc() {
        assert_eq!(crc16(b"123456789"), 0x29B1); // catalogue check value
        assert_eq!(crc16(&[]), 0xFFFF);
        assert_ne!(crc16(&[0]), crc16(&[0, 0])); // init 0xFFFF: leading zeros count
        assert_eq!(crc16(&SAMPLE_PAYLOAD), 0xE98D);
    }

    #[test]
    fn sizes() {
        assert_eq!(MAX_FRAME, 36);
    }

    #[test]
    fn test_vector() {
        assert_eq!(encode(&SAMPLE_PAYLOAD), Ok(SAMPLE_WIRE.to_vec()));
        assert_eq!(decode_wire(&SAMPLE_WIRE), Ok(SAMPLE_PAYLOAD.to_vec()));
    }

    // Every single-bit error in the encoded bytes is caught (as COBS or CRC error).
    #[test]
    fn bit_flips() {
        for i in 0..SAMPLE_WIRE.len() - 1 {
            for bit in 0..8 {
                let mut wire = SAMPLE_WIRE;
                wire[i] ^= 1 << bit;
                assert!(decode_wire(&wire).is_err(), "byte {i} bit {bit}");
            }
        }
    }

    #[test]
    fn limits() {
        let mut payload = [0u8; MAX_PAYLOAD + 1];
        payload[0] = 3;
        assert_eq!(encode(&payload), Err(FrameError::TooLong));
        let wire = encode(&payload[..MAX_PAYLOAD]).unwrap();
        assert_eq!(wire.len(), MAX_FRAME);
        assert_eq!(decode_wire(&wire), Ok(payload[..MAX_PAYLOAD].to_vec()));
        assert_eq!(
            decode_wire(&encode(&payload[..1]).unwrap()),
            Err(FrameError::TooShort)
        );
        assert_eq!(decode(&[]), Err(FrameError::TooShort)); // empty
    }

    // One byte longer than any valid frame decodes: TooLong.
    #[test]
    fn rejects_long() {
        let enc = cobs::encode(&[0x55; MAX_DECODED + 1]);
        assert_eq!(decode(&enc), Err(FrameError::TooLong));
    }
}
