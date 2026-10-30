---
name: refactor
description: Behavior-preserving refactoring in this repo (renames, moves, splitting or merging files, extracting helpers, removing magic numbers and duplication, replacing hand-rolled code with std/LL/hardware features, simplifying C++ or Rust). Use when asked to refactor, rename, restructure folders, clean up, modernize, or deduplicate code without changing what the firmware or host tools do.
---

# Refactor

A refactor changes structure, not behavior: same wire bytes, same timing, same results on
the same recording. If behavior has to change, stop and say so; that is a separate step
with its own plan in `docs/todo.md`.

## Before editing

1. Start from a clean tree (`git status`). Commit or ask about unrelated changes first.
2. Record the baseline, all inside `nix develop`:
   - `cmake --preset host && cmake --build --preset host && ctest --preset host`
   - `cmake --preset firmware && cmake --build --preset firmware`, and note FLASH/RAM
     for `stream` and `recognize` from the link output.
   - In `host/`: `cargo test`, `cargo clippy -- -D warnings`.
3. Pin current behavior where the code to change has no test (characterization tests):
   add `static_assert`s or host-test cases that record what it does today, golden bytes
   included (as `proto_test.cpp` does for the frame layouts), and commit them first. A
   test that passes before and after is the proof the refactor kept behavior.
4. For firmware, keep the baseline images:
   `cp build/firmware/firmware/apps/{stream,recognize}.elf <scratchpad>/`.
5. State the plan in a few lines: what moves or changes, and what stays. For a rename,
   list the old and new names and say which are public (headers under `gesture/`, CMake
   targets, namespaces, Rust crate items).

## What to look for

Report each finding with `file:line`, then fix the ones in scope. Firmware changes must
not cost flash, RAM or time: check the link output, and the disassembly
(`arm-none-eabi-objdump -dC`) if in doubt.

- **Magic numbers.** Every literal gets a named `constexpr` with its source (RM0444
  section, datasheet table, protocol doc), or is derived from other constants
  (`micros_hz / lsm6ds3_odr_hz`, not `9615`). Register bits come from CMSIS/LL names
  (`USART_ISR_RXNE_RXFNE`), not hex. Tie a derived value to its source with a
  `static_assert` (as `i2c.cpp` does for TIMINGR). Allowed bare: 0, 1, array indices,
  bit shifts in an obvious layout.
- **Duplication.** The same logic in two places becomes one function or table. Across
  languages it cannot be shared: `proto/` vs `host/crates/protocol` and `core/` vs its
  Rust FFI stay two copies by design, kept equal by shared test vectors. Two lines that
  only look alike are not duplication; merge only what must change together.
- **Hand-rolled code with a ready-made replacement**, in this order:
  1. C++ standard library: `<algorithm>`/`<ranges>` instead of raw loops, `<bit>`
     (`std::bit_cast`, `std::byteswap`, `std::popcount`), `<charconv>`, `std::span`
     instead of pointer + length, `std::to_underlying`, `std::add_sat` and friends,
     `std::min`/`max`/`clamp`. Rust: iterators, `u16::from_le_bytes`, the `crc` crate.
  2. ST LL and CMSIS: `LL_*` setters and `__LL_*` macros instead of open-coded register
     math, `NVIC_*` and `__DSB()` instead of inline asm.
  3. Hardware: a peripheral instead of a CPU loop when it frees time on the hot path
     (DMA for transfers, TIM for timing, EXTI for edges; G031 also has a CRC unit with a
     programmable polynomial). Count the trade-off: a hardware version only builds for
     the MCU, so the host test must still cover the logic.
  Before relying on a C++26 library feature, build it with the firmware toolchain.
- **Modern language features where they remove code or errors:** `constexpr`/`consteval`
  tables and checks, `static_assert` tests, designated initializers, structured
  bindings, `std::expected` + `.and_then`/`.transform` instead of error codes,
  `std::optional` instead of sentinel values, `enum class`, `[[nodiscard]]`, `auto` only
  where the type is obvious.
- **Declarative, not OOP:** plain data (`struct`, `std::array`) plus free functions; no
  classes with behavior, no inheritance, no virtual. Prefer pure functions: inputs in,
  result out, no hidden state; push I/O and globals to the edges (`main.cpp`, ISRs,
  drivers) so the logic in between is `constexpr` and host-tested. Table lookups over
  `switch` chains, ranges pipelines over index loops, early returns over nesting.
- **No overhead for style.** On the MCU: no heap, no exceptions, no RTTI, no
  `std::function` (use templates or function pointers), no type-erasure or ranges
  adaptors that grow code on the hot path. If the declarative version is bigger or
  slower in the link output or a timing check, keep the plain loop and say why.

## Rules while editing

- One kind of change per commit: a move or rename alone, then simplifications. Mixing
  them hides real changes in the diff.
- CLAUDE.md "Structure" and "Rules" apply as written; this list adds only what is
  specific to refactoring.
- Moves: use `git mv` so history follows.
- `proto/` and `host/crates/protocol` mirror each other: refactor one, check the other
  still matches. `docs/protocol.md` must not change in a refactor.
- Match the comment density and tone of the file being edited.
- Run `clang-format -i` on touched C++ files and `cargo fmt` in `host/`.

## Firmware hazards a refactor can break silently

- **Shared with an ISR.** State touched by an interrupt handler keeps its `volatile`,
  `std::atomic_signal_fence`, or IRQ masking (`imu_int.cpp`, `uart.cpp`). Do not merge
  it into a plain struct or move it into a helper that drops these. Cortex-M0+ has no
  read-modify-write atomics: `x++` on shared state needs masking or a single writer.
  "Works with a debug print, fails without" means a missing `volatile` or barrier.
- **ISRs stay short** and never block, wait on a flag, or call `delay_*`.
- **Critical sections stay short** and mask as little as possible: prefer masking one
  IRQ (`NVIC_DisableIRQ`) to PRIMASK so the INT1 edge timestamp keeps its timing.
- **Write-1-to-clear registers** (`ICR`, `DMA_IFCR`) are written with the bits to clear
  (`ICR = X`), never `|=` or `&= ~`. Keep that form when rewriting register code.
- **Init order is part of behavior:** clock enable before register access; some
  registers are writable only with the peripheral off (USART UE = 0, I2C PE = 0, DMA
  channel EN = 0). Reordering init means re-checking RM0444 for each register.
- **No unaligned access:** M0+ faults on it. Parse bytes byte-wise or with
  `std::bit_cast` of a `std::array`, never by casting a byte pointer to a wider type.
- **Stack:** M0+ has no stack guard. Moving a buffer onto the stack (or making it larger)
  counts against RAM like a static one; check with `-fstack-usage` if in doubt.
- **Errata:** if register code changes, grep `docs/datasheets/es0486*.txt` for the
  peripheral.

## Find what the refactor left stale

Grep the whole repo (not `build/`, `host/target/`, `docs/datasheets/`) for every old
name, path and number you touched:

```
grep -rn "<old name>" --exclude-dir={build,target,datasheets,.git} .
```

Check comments, `CLAUDE.md`, `docs/*.md`, `.clang-tidy` (`HeaderFilterRegex`),
CMake targets and flash targets, include guards and `#include` lines, Rust `use`
paths and doc comments. A comment that cites a number (a size, a time, a frequency)
must still be true.

## Verify

1. Repeat every baseline command. All tests pass, no new warnings.
2. FLASH/RAM equal to the baseline, or explain the difference. For a pure rename or
   move, compare the code itself: `arm-none-eabi-objcopy -O binary` the old and new
   `.elf` and `cmp` them. Identical bytes end the question; if not, `arm-none-eabi-nm
   --size-sort -C` both and explain which symbols changed.
3. If the hot path changed (ISRs, `poll_imu`, `send`, I2C/UART drivers), compare worst
   case INT1 edge -> sample read and edge -> frame queued against the table in
   `docs/todo.md`: temporary `volatile` maxima in `poll_imu`, read over SWD with the
   core halted (one `openocd ... -c init -c halt -c "mdw ..." -c resume -c exit`).
   Remove the instrumentation afterwards.
4. If firmware code changed (anything under `firmware/` or `proto/` that the MCU
   links), flash and check the live stream:
   `cmake --build --preset firmware --target flash_stream`, then in `host/`
   `cargo run -q -p gesture-cli -- check --seconds 30`. It must end with `ok` and all
   counters at 0. If the board is not connected, say the hardware check was skipped.
5. Read the final `git diff --stat` and the diff itself once: no stray edits, no
   behavior change slipped in.

## Report

Say what moved or changed, the baseline vs. final sizes, which checks ran (and which
did not), and anything stale you found and fixed. Do not commit or push unless asked.
