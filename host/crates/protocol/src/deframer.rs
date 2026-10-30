//! Splits a byte stream into frames on 0x00 (`docs/protocol.md` "Framing").

use crate::frame::{self, FrameError, MAX_ENCODED};

/// Collects bytes up to each 0x00 and decodes them as one frame. A bad frame costs only
/// itself: the next 0x00 starts over.
///
/// Starts unsynced: bytes before the first 0x00 are the tail of a frame whose start was
/// missed (port opened mid-stream) and are dropped without an error.
#[derive(Debug, Default)]
pub struct Deframer {
    buf: Vec<u8>,
    synced: bool,
    /// More than `MAX_ENCODED` bytes since the last 0x00; dropped until the next one.
    overlong: bool,
}

impl Deframer {
    pub fn new() -> Self {
        Self::default()
    }

    /// A payload or an error at each frame end; `None` mid-frame, on an empty frame
    /// (two 0x00 in a row), and while unsynced.
    pub fn push(&mut self, byte: u8) -> Option<Result<Vec<u8>, FrameError>> {
        if byte != 0 {
            if self.synced {
                if self.buf.len() < MAX_ENCODED {
                    self.buf.push(byte);
                } else {
                    self.overlong = true;
                }
            }
            return None;
        }
        if !self.synced {
            self.synced = true;
            return None;
        }
        if std::mem::take(&mut self.overlong) {
            self.buf.clear();
            return Some(Err(FrameError::TooLong));
        }
        if self.buf.is_empty() {
            return None;
        }
        let result = frame::decode(&self.buf);
        self.buf.clear();
        Some(result)
    }

    /// Every frame that ends within `bytes`; a partial frame at the end waits for more.
    pub fn feed<'a>(
        &'a mut self,
        bytes: &'a [u8],
    ) -> impl Iterator<Item = Result<Vec<u8>, FrameError>> + 'a {
        bytes.iter().filter_map(|&b| self.push(b))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::tests::{SAMPLE_PAYLOAD, SAMPLE_WIRE};

    fn collect(d: &mut Deframer, bytes: &[u8]) -> Vec<Result<Vec<u8>, FrameError>> {
        d.feed(bytes).collect()
    }

    #[test]
    fn drops_bytes_before_first_delimiter() {
        let mut d = Deframer::new();
        // Tail of a missed frame, then a whole one.
        assert_eq!(collect(&mut d, &SAMPLE_WIRE[5..]), []);
        assert_eq!(collect(&mut d, &SAMPLE_WIRE), [Ok(SAMPLE_PAYLOAD.to_vec())]);
    }

    #[test]
    fn frames_split_anywhere() {
        let stream = [&[0][..], &SAMPLE_WIRE, &[0, 0], &SAMPLE_WIRE].concat(); // empty frames
        for split in 0..=stream.len() {
            let mut d = Deframer::new();
            let mut got = collect(&mut d, &stream[..split]);
            got.extend(collect(&mut d, &stream[split..]));
            assert_eq!(
                got,
                [Ok(SAMPLE_PAYLOAD.to_vec()), Ok(SAMPLE_PAYLOAD.to_vec())]
            );
        }
    }

    #[test]
    fn resyncs_after_bad_frame() {
        let mut corrupt = SAMPLE_WIRE;
        corrupt[10] ^= 0x01;
        let stream = [&[0][..], &corrupt, &SAMPLE_WIRE].concat();
        let mut d = Deframer::new();
        assert_eq!(
            collect(&mut d, &stream),
            [Err(FrameError::Crc), Ok(SAMPLE_PAYLOAD.to_vec())]
        );
    }

    #[test]
    fn overlong_frame() {
        let mut d = Deframer::new();
        let stream = [&[0][..], &[0x55; 1000], &[0], &SAMPLE_WIRE].concat();
        assert_eq!(
            collect(&mut d, &stream),
            [Err(FrameError::TooLong), Ok(SAMPLE_PAYLOAD.to_vec())]
        );
        // Exactly MAX_ENCODED bytes still go to the decoder.
        let max = frame::encode(&[3; frame::MAX_PAYLOAD]).unwrap();
        assert_eq!(max.len() - 1, MAX_ENCODED);
        assert_eq!(collect(&mut d, &max), [Ok(vec![3; frame::MAX_PAYLOAD])]);
    }
}
