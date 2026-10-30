//! Host side of the wire protocol. The contract is `docs/protocol.md`;
//! the firmware side is `proto/`.
//!
//! Bytes from the port go through a [`Deframer`], which yields frame payloads (or a
//! [`FrameError`] per dropped frame); [`Message::decode`] turns a payload into a message.

pub mod cobs;
pub mod deframer;
pub mod frame;
pub mod message;

pub use deframer::Deframer;
pub use frame::FrameError;
pub use message::{Firmware, Hello, ImuStatus, Message, MessageError, Sample, Status};

/// Serial line rate, 8N1 (`docs/protocol.md` "Transport").
pub const BAUD: u32 = 115_200;
