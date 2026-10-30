//! MCU timestamps, `docs/protocol.md` "Timestamps": u32 microseconds that wrap every
//! 2^32 us (~71.6 min), unwrapped to 64 bits on the receiver.

/// Extends successive `t_us` values to a 64-bit time, assuming consecutive values are
/// less than half a wrap (~35.8 min) apart. A step backwards stays a step backwards.
#[derive(Debug, Default, Clone)]
pub struct Unwrapper {
    last: Option<u64>,
}

impl Unwrapper {
    pub fn new() -> Self {
        Self::default()
    }

    /// The first value is taken as is; later ones follow it by the shortest signed step.
    pub fn extend(&mut self, t_us: u32) -> u64 {
        let t = match self.last {
            None => u64::from(t_us),
            Some(last) => {
                // The low 32 bits of `last` are its wrapped value.
                last.saturating_add_signed(i64::from(step(last as u32, t_us)))
            }
        };
        self.last = Some(t);
        t
    }

    /// Forget the history, e.g. after an MCU reset restarted the clock.
    pub fn reset(&mut self) {
        self.last = None;
    }
}

/// Signed step from `a` to `b` under the same half-wrap assumption.
pub fn step(a: u32, b: u32) -> i32 {
    b.wrapping_sub(a) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn across_wrap() {
        let mut u = Unwrapper::new();
        assert_eq!(u.extend(u32::MAX - 9), u64::from(u32::MAX) - 9);
        assert_eq!(u.extend(5), (1u64 << 32) + 5);
        assert_eq!(u.extend(1000), (1u64 << 32) + 1000);
        // A second wrap, in steps under half a wrap.
        assert_eq!(u.extend(0x8000_0000), (1u64 << 32) + 0x8000_0000);
        assert_eq!(u.extend(u32::MAX), (1u64 << 32) + u64::from(u32::MAX));
        assert_eq!(u.extend(0), 2u64 << 32);
    }

    #[test]
    fn backwards_and_reset() {
        let mut u = Unwrapper::new();
        assert_eq!(u.extend(1000), 1000);
        assert_eq!(u.extend(400), 400);
        assert_eq!(u.extend(u32::MAX), 0); // would go below zero: saturates
        u.reset();
        assert_eq!(u.extend(u32::MAX), u64::from(u32::MAX));
    }

    #[test]
    fn signed_step() {
        assert_eq!(step(10, 20), 10);
        assert_eq!(step(20, 10), -10);
        assert_eq!(step(u32::MAX, 3), 4);
        assert_eq!(step(3, u32::MAX), -4);
    }
}
