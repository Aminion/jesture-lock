//! What the host tools share (`gesture-cli`, `viewer`): recorded sessions, the serial port
//! of the stream firmware, stream health, accelerometer calibration, and the core's config
//! for a session.

pub mod calib;
pub mod check;
pub mod config;
pub mod recording;
pub mod reference;
pub mod serial;
