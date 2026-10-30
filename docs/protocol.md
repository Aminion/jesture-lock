# Wire protocol

The contract between `proto` (C++) and `host/crates/protocol` (Rust).
Change this document first, then both sides.

Protocol version: **1**.

## Transport
- USART2 through the ST-LINK virtual COM port, 115200 baud, 8N1.
- Byte order: little-endian for every multi-byte field.
- The `stream` firmware sends only frames; the bring-up text lines are gone.

## Framing
Frame on the wire: `COBS(payload || crc16) 0x00`.

- **CRC16**: CRC-16/IBM-3740 (also known as CRC-16/CCITT-FALSE): polynomial 0x1021,
  init 0xFFFF, no input or output reflection, xorout 0x0000. Check value: the ASCII bytes
  `123456789` give 0x29B1. Computed over `payload` only, appended low byte first.
- **COBS**: standard Consistent Overhead Byte Stuffing (Cheshire and Baker). The encoded
  bytes contain no 0x00; a single 0x00 ends every frame. For up to 254 input bytes the
  encoding adds exactly 1 byte.
- **Maximum payload**: 32 bytes. Largest frame on the wire: 32 + 2 (CRC) + 1 (COBS)
  + 1 (delimiter) = 36 bytes.

The sender may emit extra 0x00 bytes (for example one at boot, to end a frame cut short by a
reset). The receiver:
- ignores empty frames (two 0x00 in a row);
- drops and counts a frame whose COBS decoding fails, that decodes to fewer than 4 bytes
  (header + CRC) or more than 34 bytes, or whose CRC does not match;
- rejects a frame whose `version` it does not know;
- skips a frame whose `type` it does not know.

## Payload header
| Offset | Size | Field   | Notes |
|--------|------|---------|-------|
| 0      | 1    | type    | message type, see below |
| 1      | 1    | version | protocol version, currently 1 |

Message types:

| type | Message | Direction |
|------|---------|-----------|
| 0    | (invalid, never sent) | |
| 1    | `Hello`  | MCU -> PC |
| 2    | `Sample` | MCU -> PC (`stream`) |
| 3    | `Status` | MCU -> PC |
| 4    | `Event`  | MCU -> PC (`recognize`), reserved, layout TBD |

A message with a length other than the one given for its type is dropped and counted as an error.

## Timestamps
- Source: TIM2 on the MCU, 1 MHz (1 us per tick), free-running u32. Never the PC receive time.
- Wraps every 2^32 us (about 71.6 min). The receiver unwraps to 64 bits, assuming that
  consecutive timestamps are less than half a wrap apart.
- The MCU runs on HSI16, the internal RC oscillator (±1 % over 0..85 °C): MCU time drifts
  against the PC clock (measured about 0.15 % against the IMU clock). The board's 32.768 kHz
  LSE crystal is not used. Durations within a session are in MCU time.

## Messages: MCU -> PC

### `Hello` (type 1), 8 bytes
Sent once at boot and then once per second, so a PC that connects later still gets it.

| Offset | Size | Field     | Notes |
|--------|------|-----------|-------|
| 0      | 1    | type      | 1 |
| 1      | 1    | version   | |
| 2      | 1    | fw        | 1 = `stream`, 2 = `recognize` |
| 3      | 1    | imu       | 0 = none answered, 1 = ok, 2 = unknown `WHO_AM_I`, 3 = init failed |
| 4      | 1    | imu_addr  | 7-bit I2C address, 0 if none answered |
| 5      | 1    | who_am_i  | `WHO_AM_I` value as read, 0 if none answered |
| 6      | 1    | ctrl1_xl  | LSM6DS3 CTRL1_XL as written (accel ODR, full scale), 0 if not configured |
| 7      | 1    | ctrl2_g   | LSM6DS3 CTRL2_G as written (gyro ODR, full scale), 0 if not configured |

The raw register values are the session's IMU configuration: the recorder stores them, the
viewer derives the scale (g, deg/s per LSB) and the ODR from them.

### `Sample` (type 2), 20 bytes
One per IMU data-ready edge (104 Hz at the current ODR).

| Offset | Size | Field   | Notes |
|--------|------|---------|-------|
| 0      | 1    | type    | 2 |
| 1      | 1    | version | |
| 2      | 2    | seq     | u16, +1 per `Sample`, wraps; starts at 0 after reset |
| 4      | 4    | t_us    | u32, time of the INT1 rising edge (see Timestamps) |
| 8      | 6    | gyro    | x, y, z, int16 raw LSB |
| 14     | 6    | accel   | x, y, z, int16 raw LSB |

Axis directions on the board: `pinout.md`, "IMU axes".

Gaps, as seen by the receiver:
- `seq` skips: frames were sent but lost on the way (UART, USB, CRC error).
- `seq` continuous, `t_us` step about 2 or more sample periods: the MCU did not read an IMU
  sample in time (counted in `Status.missed`).
- `t_us` gap with no data-ready edge (latched INT1 recovered, `Status.stuck` incremented):
  the receiver must not assume a fixed period across it.

### `Status` (type 3), 22 bytes
Sent once per second. Counters are u32, since reset, wrapping.

| Offset | Size | Field   | Notes |
|--------|------|---------|-------|
| 0      | 1    | type    | 3 |
| 1      | 1    | version | |
| 2      | 4    | t_us    | u32, time the message was built |
| 6      | 4    | missed  | IMU samples made but never read (from `t_us` steps) |
| 10     | 4    | stuck   | INT1 found high with no edge, recovered by a read |
| 14     | 4    | i2c_err | failed IMU reads |
| 18     | 4    | rx_err  | UART receive errors (overrun, framing, noise, ring full) |

### `Event` (type 4)
`recognize` firmware only. Layout TBD.

## Messages: PC -> MCU
None yet. Later: start/stop streaming, set ODR and full scale.

## Bandwidth
At 115200 baud 8N1 a byte takes about 87 us. One `Sample` is 24 bytes on the wire
(20 + 2 + 1 + 1), about 2.1 ms, about 22 % of the link at 104 Hz. `Hello` and `Status` add
under 0.5 %.

## Versioning
`version` is in every frame. Any change to the layout or meaning of an existing message
increments it. A new message type does not: receivers skip types they do not know.

## Test vector
A `Sample` with seq 258, t_us 10 000 000, gyro (250, -630, -55), accel (-175, 50, 16900):

```
payload  02 01 02 01 80 96 98 00 fa 00 8a fd c9 ff 51 ff 32 00 04 42
crc16    0xe98d  (appended as 8d e9)
wire     08 02 01 02 01 80 96 98 02 fa 08 8a fd c9 ff 51 ff 32 05 04 42 8d e9 00
```

Both `proto/tests` and `host/crates/protocol` check this frame.
