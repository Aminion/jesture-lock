//! COBS encode/decode, same canonical encoding as `proto/include/proto/cobs.hpp`.
//! The encoded bytes contain no 0x00; the 0x00 delimiter is not part of them.
//!
//! Canonical encoding: a run of 254 non-zero bytes that ends the input gets no extra empty
//! block, so the output is at most `n + n / 254 + 1` bytes.

use std::fmt;

/// Data bytes in a full block; its code byte is `MAX_BLOCK + 1` (0xFF).
pub const MAX_BLOCK: usize = 254;
const FULL_CODE: u8 = MAX_BLOCK as u8 + 1;

/// Upper bound on the encoded size of `n` input bytes.
pub const fn max_encoded(n: usize) -> usize {
    n + n / MAX_BLOCK + 1
}

pub fn encode(input: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(max_encoded(input.len()));
    let mut code_at = 0; // slot of the current block's code byte
    out.push(0);
    let mut code = 1u8; // 1 + data bytes in the current block
    for (i, &byte) in input.iter().enumerate() {
        if byte != 0 {
            out.push(byte);
            code += 1;
            if code != FULL_CODE || i + 1 == input.len() {
                continue;
            }
        }
        // A zero, or a full block with more input to come: close the block.
        out[code_at] = code;
        code_at = out.len();
        out.push(0);
        code = 1;
    }
    out[code_at] = code;
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CobsError {
    /// 0x00 inside the encoded bytes.
    ZeroByte,
    /// A code byte promises more bytes than are left.
    Truncated,
    /// Decoded data is longer than the limit.
    Overflow,
}

impl fmt::Display for CobsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::ZeroByte => "0x00 inside COBS data",
            Self::Truncated => "COBS block runs past the end",
            Self::Overflow => "COBS data decodes past the limit",
        })
    }
}

impl std::error::Error for CobsError {}

/// Decodes at most `max_len` bytes; errors are checked in the same order as `proto`,
/// so both sides report the same error for the same input. Empty input decodes to nothing.
pub fn decode(input: &[u8], max_len: usize) -> Result<Vec<u8>, CobsError> {
    let mut out = Vec::with_capacity(input.len().min(max_len));
    let mut rest = input;
    while let Some((&code, tail)) = rest.split_first() {
        if code == 0 {
            return Err(CobsError::ZeroByte);
        }
        let len = usize::from(code - 1);
        if len > tail.len() {
            return Err(CobsError::Truncated);
        }
        if len > max_len - out.len() {
            return Err(CobsError::Overflow);
        }
        let (block, after) = tail.split_at(len);
        if block.contains(&0) {
            return Err(CobsError::ZeroByte);
        }
        out.extend_from_slice(block);
        rest = after;
        // Every block but a full one stands for a zero, except at the very end.
        if code != FULL_CODE && !rest.is_empty() {
            if out.len() == max_len {
                return Err(CobsError::Overflow);
            }
            out.push(0);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `first`, `first + 1`, ... (wrapping), `n` bytes.
    fn bytes_from(first: u8, n: usize) -> Vec<u8> {
        (0..n).map(|i| first.wrapping_add(i as u8)).collect()
    }

    fn cat(parts: &[&[u8]]) -> Vec<u8> {
        parts.concat()
    }

    /// Encodes to exactly `enc` and decodes back to `raw`.
    fn check_vector(raw: &[u8], enc: &[u8]) {
        assert_eq!(encode(raw), enc, "encode {raw:02x?}");
        assert_eq!(
            decode(enc, raw.len()).as_deref(),
            Ok(raw),
            "decode {enc:02x?}"
        );
    }

    // Known vectors (Wikipedia, "Consistent Overhead Byte Stuffing"), as in proto_test.cpp.
    #[test]
    fn vectors() {
        check_vector(&[], &[0x01]);
        check_vector(&[0x00], &[0x01, 0x01]);
        check_vector(&[0x00, 0x00], &[0x01, 0x01, 0x01]);
        check_vector(&[0x00, 0x11, 0x00], &[0x01, 0x02, 0x11, 0x01]);
        check_vector(&[0x11, 0x22, 0x00, 0x33], &[0x03, 0x11, 0x22, 0x02, 0x33]);
        check_vector(&[0x11, 0x22, 0x33, 0x44], &[0x05, 0x11, 0x22, 0x33, 0x44]);
        check_vector(&[0x11, 0x00, 0x00, 0x00], &[0x02, 0x11, 0x01, 0x01, 0x01]);
    }

    #[test]
    fn block_boundaries() {
        // 254 non-zero bytes: one full block, no empty block after it.
        check_vector(
            &bytes_from(0x01, 254),
            &cat(&[&[0xFF], &bytes_from(0x01, 254)]),
        );
        check_vector(
            &cat(&[&[0x00], &bytes_from(0x01, 254)]),
            &cat(&[&[0x01, 0xFF], &bytes_from(0x01, 254)]),
        );
        // 255 non-zero bytes: a full block, then a block of one.
        check_vector(
            &bytes_from(0x01, 255),
            &cat(&[&[0xFF], &bytes_from(0x01, 254), &[0x02, 0xFF]]),
        );
        check_vector(
            &cat(&[&bytes_from(0x02, 254), &[0x00]]),
            &cat(&[&[0xFF], &bytes_from(0x02, 254), &[0x01, 0x01]]),
        );
        check_vector(
            &cat(&[&bytes_from(0x03, 253), &[0x00, 0x01]]),
            &cat(&[&[0xFE], &bytes_from(0x03, 253), &[0x02, 0x01]]),
        );
    }

    #[test]
    fn errors() {
        assert_eq!(decode(&[0x00, 0x01], 8), Err(CobsError::ZeroByte));
        assert_eq!(decode(&[0x03, 0x11, 0x00], 8), Err(CobsError::ZeroByte));
        assert_eq!(decode(&[0x03, 0x11], 8), Err(CobsError::Truncated));
        assert_eq!(
            decode(&[0x05, 0x11, 0x22, 0x33, 0x44], 3),
            Err(CobsError::Overflow)
        );
        // The implicit zero between blocks needs room too.
        assert_eq!(
            decode(&[0x02, 0x11, 0x02, 0x22], 1),
            Err(CobsError::Overflow)
        );
        assert_eq!(decode(&[], 0), Ok(vec![]));
    }

    const SWEEP_MAX_LEN: usize = 600; // past two full blocks

    // Every length 0..600, with no zeros, all zeros, and zeros at block-sized spacing.
    #[test]
    fn round_trips() {
        for zero_every in [0, 1, 7, 254, 255] {
            let raw: Vec<u8> = (0..SWEEP_MAX_LEN)
                .map(|i| {
                    if zero_every != 0 && i % zero_every == 0 {
                        0
                    } else {
                        1 + ((i * 37) % 255) as u8
                    }
                })
                .collect();
            for len in 0..=SWEEP_MAX_LEN {
                let raw = &raw[..len];
                let enc = encode(raw);
                assert!(enc.len() <= max_encoded(len), "len {len}");
                assert!(!enc.contains(&0), "len {len}");
                assert_eq!(decode(&enc, len).as_deref(), Ok(raw), "len {len}");
            }
        }
    }
}
