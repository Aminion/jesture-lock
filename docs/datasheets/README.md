# Reference documents

Vendor PDFs kept locally, not in git: the repo is public and the documents are ST copyright.
Only the list (`manifest.txt`) and the checksums (`SHA256SUMS`) are committed.

## Setup

1. Open each URL from `manifest.txt` in a browser and save the PDF here under its
   default name (st.com rejects scripted downloads).
2. Run `./sync.sh` inside `nix develop`. It reports missing files, verifies `SHA256SUMS`,
   and writes a `pdftotext -layout` copy (`*.txt`) next to each PDF for grep.

When ST publishes a new revision: download it, run `./sync.sh --update-sums`,
update the revision column below, commit both.

## Documents

| Id          | Document                                   | Use it for                                    | Rev |
|-------------|--------------------------------------------|-----------------------------------------------|-----|
| rm0444      | STM32G0x1 reference manual                 | registers, peripherals (I2C, USART, EXTI, RCC) | Rev 6 |
| ds12992     | STM32G031x4/x6/x8 datasheet                | pinout, alternate function table, electrical  | Rev 4 |
| es0486      | STM32G031/G041 errata sheet                | silicon bugs, check before blaming the code   | Rev 7 |
| pm0223      | Cortex-M0+ programming manual              | core, NVIC, SysTick, instruction timing       | Rev 10 |
| um2591      | Nucleo-32 STM32G0 board user manual (MB1455) | headers, solder bridges, LEDs, ST-LINK VCP  | Rev 2 |
| lsm6ds3     | LSM6DS3 datasheet                          | the fitted IMU (WHO_AM_I 0x69): register map  | DocID026899 Rev 10 |
| an4650      | LSM6DS3 application note                   | FIFO, interrupts, power modes, startup        | DocID027415 Rev 1 |
| lsm6ds3tr-c | LSM6DS3TR-C datasheet                      | the common 0x6A variant on other GY modules   | DocID030071 Rev 3 |
