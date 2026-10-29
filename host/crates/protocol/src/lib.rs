//! Host side of the wire protocol. The contract is `docs/protocol.md`;
//! the firmware side is `fw/proto/`.

/// Serial line rate, 8N1 (`docs/protocol.md` "Transport").
pub const BAUD: u32 = 115_200;
