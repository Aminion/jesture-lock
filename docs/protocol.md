# Wire protocol

The contract between `fw/proto` (C++) and `host/crates/protocol` (Rust).
Change this document first, then both sides.

## Transport
- USART2 through the ST-LINK virtual COM port. Baud rate: TBD.
- Byte order: little-endian.

## Framing
- Frame on the wire: `COBS(payload || crc16) 0x00`.
- CRC16: variant TBD (polynomial, init, reflection, xorout), computed over `payload`.
- Maximum payload length: TBD (it sizes the MCU RX/TX buffers; count the RAM).

## Payload header
| Offset | Size | Field   | Notes |
|--------|------|---------|-------|
| 0      | 1    | type    | message type, see below |
| 1      | 1    | version | protocol version |
| ...    |      |         | TBD |

## Timestamps
- Source: MCU timer only, never the PC receive time.
- Unit, width and wraparound handling: TBD.

## Messages

### MCU -> PC
- `Hello`: firmware id (`fw_stream` / `fw_recognize`), protocol version, IMU `WHO_AM_I`, IMU config.
- `Sample` (fw_stream): timestamp, raw accel x/y/z and gyro x/y/z as int16 LSB.
- `Event` (fw_recognize): timestamp, recognition result.
- `Error`: TBD.

### PC -> MCU
- TBD (e.g. start/stop streaming, set ODR/full scale).

## Versioning
TBD.
