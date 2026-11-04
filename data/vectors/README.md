# Board test vectors

`air-lines.gcv`: the first 2400 samples (23 s) of `sessions/20261030-212525_air-lines.bin`
with its `gc_config` and the digest of what the host build of `core/` gives on them
(19 events: READY, 9 MOVING, 4 SHORT, 1 DRIFT, 4 STROKE). Layout and digest:
`host/crates/gesture-cli/src/vector.rs`. The `recognize` firmware embeds it and replays it at
boot; `cargo test` fails if the core no longer gives the stored digest.

Regenerate after a core change (in `host/`):

```
cargo run -q -p gesture-cli -- vector ../data/sessions/20261030-212525_air-lines.bin \
  --calib ../data/sessions/20261030-205757_tumble.bin \
  --bias ../data/sessions/20261030-183544_rest-table.bin \
  --samples 2400 --out ../data/vectors/air-lines.gcv
```

Run on the board: `cmake --build --preset firmware --target flash_recognize`, wait ~4 s
(LED: slow blink = match, fast = mismatch), then read `bench_result` (14 words, fields in
`firmware/apps/recognize/main.cpp`):

```
addr=$(arm-none-eabi-nm build/firmware/firmware/apps/recognize.elf | awk '/ bench_result$/ {print $1}')
openocd -f interface/stlink.cfg -f target/stm32g0x.cfg -c init -c halt -c "mdw 0x$addr 14" -c resume -c exit
```
