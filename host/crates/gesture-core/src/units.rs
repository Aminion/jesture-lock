//! Units of the numbers `core.h` gives, as types. Each wraps the integer of its field
//! unchanged, so a value converts back to the exact `core.h` bytes. `Display` prints the
//! number alone (width and precision apply): the unit belongs in the caller's label.
//! Time is `std::time::Duration`.

use std::fmt;

/// Millimetres: point coordinates (`i16`, the default) and extents (`u16`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Mm<T = i16>(pub T);

/// Millimetres per second: leftover velocity at the stroke end.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct MmPerS(pub u16);

/// Micro-g: gravity in the start frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct MicroG(pub i32);

impl<T: fmt::Display> fmt::Display for Mm<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

impl fmt::Display for MmPerS {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

impl fmt::Display for MicroG {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

/// The number, for float math in the same unit.
impl<T: Into<f64>> From<Mm<T>> for f64 {
    fn from(v: Mm<T>) -> Self {
        v.0.into()
    }
}

impl From<MmPerS> for f64 {
    fn from(v: MmPerS) -> Self {
        v.0.into()
    }
}

impl From<MicroG> for f64 {
    fn from(v: MicroG) -> Self {
        v.0.into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_is_the_number_with_its_format() {
        assert_eq!(format!("{:>6}", Mm(-12)), "   -12");
        assert_eq!(format!("{}", Mm(40_u16)), "40");
        assert_eq!(format!("{:<4}|", MmPerS(7)), "7   |");
        assert_eq!(MicroG(-5).to_string(), "-5");
    }

    #[test]
    fn same_unit_compares() {
        assert!(Mm(-1) < Mm(0));
        assert!(MmPerS(30) > MmPerS(29));
        assert_eq!(f64::from(Mm(250_u16)), 250.0);
    }
}
