# Gesture IMU

Gesture un/lock with IMU: data capture, gesture recognition, validation.
The puzzle box with a motion-based lock is out of scope for now.

Two firmwares share the same IMU driver and the same recognition core (`core/`):
- `stream` (`firmware/apps/stream/`): IMU -> UART -> PC. The PC records sessions and visualizes them;
  recognition runs on the PC through the host build of `core/` (Rust FFI).
  Used for building the dataset and developing the algorithm.
- `recognize` (`firmware/apps/recognize/`): IMU -> `core/` on the MCU -> events over UART.
  Used to verify the algorithm on the real target.

Flash one or the other. Results of both paths on the same recording must match bit for bit.

## Stack
- Firmware: C++26 (limited to what arm-none-eabi-gcc supports), no exceptions, no RTTI, no heap. Not OOP: plain structs plus
  free functions, with `std::expected`/`optional`/`variant`/`span`.
  CMSIS + ST LL drivers, no ST HAL.
- Desktop: Rust (stable), egui/eframe for the viewer. No Python.
- Build: CMake + Ninja, presets `firmware` and `host`.
- Environment: Nix flakes only. No IDEs, no CubeMX, no global installs.

## Hardware
- MCU: STM32G031K8 on Nucleo-32 (Cortex-M0+, 64 KB flash, 8 KB RAM,
  no FPU, no hardware divide, no USB).
- Flashing and logs go through the on-board ST-LINK (virtual COM port).
- IMU: GY-LSM6DS3 module (I2C, SPI, INT1/INT2). Detect the exact chip via
  `WHO_AM_I`; never hardcode 0x69 as the only valid value.

## Structure
- `core/`: pure recognition core (C++) plus a C ABI. Builds for host and MCU.
- `proto/`: protocol framing (COBS, CRC16), no hardware dependencies. Builds for host and MCU;
  `host/crates/protocol` mirrors it.
- `firmware/board/`: clocks, pins, I2C/UART/LSM6DS3 drivers. Register-free logic (ring buffer,
  baud/I2C timing math, WHO_AM_I map, IMU sample decode) lives in headers, `constexpr` where possible, and is
  tested on the host in `firmware/board/tests/`.
- `firmware/apps/stream/`, `firmware/apps/recognize/`: the two firmwares, `main.cpp` only.
- `test_support/`: shared helpers for the host test executables (`gesture/test/check.hpp`).
- C++ public headers use one prefix: `gesture/core.h`, `gesture/proto/`, `gesture/board/`,
  `gesture/test/`.
- `host/`: Rust workspace: `protocol`, `gesture-core` (FFI), `gesture-cli`, `viewer`.
- `docs/`: `protocol.md` (the C++/Rust contract), `template_format.md`, `pinout.md` (MCU pins and IMU wiring),
  `todo.md` (current step plan, deferred adjustments),
  `datasheets/` (vendor PDFs, gitignored; manifest + checksums committed).
- `data/`: recorded sessions.

## Commands
- Environment: `nix develop`
- Firmware: `cmake --preset firmware && cmake --build --preset firmware`
- Host tests: `cmake --preset host && cmake --build --preset host && ctest --preset host`
- Rust (run in `host/`): `cargo test`, `cargo clippy -- -D warnings`, `cargo fmt`

## Rules
- `core/` has no I/O, no platform code, no clock access, no global state.
  Integer arithmetic only, on raw IMU LSB (widen int16 to int32/uint32 immediately).
  Where a result can overflow or must fit back into int16, use C++26 saturation
  (`std::add_sat`/`sub_sat`/`mul_sat`, `std::saturate_cast`) instead of hand-written clamps.
  Physical units (g, deg/s) are for the viewer only.
- Time comes from the MCU, never from the PC receive time.
- Protocol changes: update `docs/protocol.md` first, then both sides.
- List source files explicitly in CMake. `git add` new files (flakes ignore untracked ones).
- Check register usage against RM0444. Vendor docs: `docs/datasheets/` (local PDFs + `.txt`
  copies, see its README); grep the `.txt` instead of fetching from the web.
- Pin changes: update `docs/pinout.md` in the same commit.
- Count RAM before adding any buffer (only 8 KB total).
- Prefer standard facilities over hand-rolled code (`<bit>`, `<charconv>`, `<ranges>`,
  `std::to_underlying`, `std::span`, `[[nodiscard]]`, `constexpr`/`consteval` for tables
  and checks). Before relying on a C++26 library feature, build it with the firmware
  toolchain (GCC 15 lacks contracts and `<inplace_vector>`). `std::atomic` on Cortex-M0+
  is load/store only: read-modify-write needs libatomic, which is not linked.
- clang-tidy runs on every first-party C++ file during the build (both presets), warnings
  are errors. Fix the code; disable a check only in `.clang-tidy` (or `firmware/.clang-tidy`
  for firmware-only exceptions) with a comment, or `NOLINTNEXTLINE(check)` with a reason.
