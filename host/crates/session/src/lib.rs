//! What the host tools share (`gesture-cli`, `viewer`): recorded sessions, the serial port
//! of the stream firmware, stream health, accelerometer calibration, and the core's config
//! for a session, the core's strokes on a recording.

pub mod calib;
pub mod check;
pub mod config;
pub mod recording;
pub mod reference;
pub mod serial;
pub mod strokes;
