# Gesture template format

How gesture templates are stored and consumed by `core/` on both host and MCU.

## Requirements
- Integer data only, on raw IMU LSB.
- Same bytes on host and MCU, so recognition matches bit for bit.
- Fits the flash budget (64 KB total, shared with the firmware).

## Layout
TBD: header (magic, version, count), per-template record, endianness.

## Generation
TBD: produced by `gesture-cli` from sessions in `data/`.

## Embedding in firmware
TBD: how templates reach the `recognize` firmware (compiled-in table, flash page, ...).
