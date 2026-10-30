# Recorded sessions

Sessions recorded from the `stream` firmware by `gesture-cli record`. Timestamps are MCU time.

`sessions/<YYYYMMDD-HHMMSS>_<label>.bin`: the raw bytes as received from the port
(`docs/protocol.md` framing), so a replay goes through the same deframer. Bytes before the
first Hello may be a stale backlog; readers skip them (as `gesture-cli check` does). The
sensor scale is in the Hello frames (`ctrl1_xl`, `ctrl2_g`).

`sessions/<same stem>.toml`: label, note (how the board was held, what moved), start time,
duration, and the stream health from the recording (samples, rate, failures).
