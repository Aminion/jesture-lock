//! Safe wrapper over the C ABI in `core/include/gesture/core.h`.
//! The same C++ sources are compiled for the MCU, so results must match bit for bit.
//!
//! So far `gc_init` and `gc_feed` only (3c.1 in progress); the full wrapper is 3d.1.

pub mod ffi;

use ffi::{GC_OK, GC_STATE_SIZE, GcConfig, GcEvent, GcSample, GcState, GcStatus};

/// One recognizer: the caller-owned `gc_state`, boxed (2 KB) so moves stay cheap.
#[derive(Debug)]
pub struct Core {
    state: Box<GcState>,
}

impl Core {
    /// `gc_init`. Err: the `GC_ERR_*` status.
    pub fn new(cfg: &GcConfig) -> Result<Self, GcStatus> {
        let mut state = Box::new(GcState {
            opaque: [0; GC_STATE_SIZE],
        });
        // SAFETY: both pointers are valid for the call; GcState has the size and alignment
        // core.h requires (checked in ffi.rs).
        let status = unsafe { ffi::gc_init(&raw mut *state, cfg) };
        if status == GC_OK {
            Ok(Self { state })
        } else {
            Err(status)
        }
    }

    /// `gc_feed`: one sample, in sensor order.
    pub fn feed(&mut self, sample: &GcSample) -> GcEvent {
        // SAFETY: valid pointers; the state was initialized by a successful gc_init.
        unsafe { ffi::gc_feed(&raw mut *self.state, sample) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ffi::{GC_ABI_VERSION, GC_ERR_VERSION};

    #[test]
    fn init_checks_the_version() {
        let cfg = GcConfig::default();
        assert_eq!(Core::new(&cfg).unwrap_err(), GC_ERR_VERSION);
        let cfg = GcConfig {
            abi_version: GC_ABI_VERSION,
            ..cfg
        };
        // Version right, the rest zero: a ctrl value 0 is the next failure.
        assert_eq!(Core::new(&cfg).unwrap_err(), ffi::GC_ERR_SCALE);
    }
}
