# Pinout: Nucleo-G031K8 + GY-LSM6DS3

Board: Nucleo-32 with STM32G031K8. Checked against `docs/datasheets/`: header pinout
UM2591 Rev 2 (Table 9), alternate functions DS12992 Rev 4, SYSCFG remap RM0444 Rev 6.
Keep this file in sync with the code: change a pin there, change it here in the same commit.

## IMU wiring (jumpers)

| IMU pin | Nucleo header | Arduino | MCU  | Function             | Status  |
|---------|---------------|---------|------|----------------------|---------|
| VCC     | CN4-14        | 3V3     | -    | power, 3.3 V         | wired   |
| GND     | CN4-2         | GND     | -    | ground               | wired   |
| SCL     | CN3-8         | D5      | PA9  | I2C1_SCL, AF6        | in use  |
| SDA     | CN3-7         | D4      | PA10 | I2C1_SDA, AF6        | in use  |
| INT1    | CN3-5         | D2      | PA15 | EXTI15, data ready   | wired, not used in code yet |
| INT2    | CN3-6         | D3      | PB1  | EXTI1, spare (e.g. wake on motion) | not wired, pin reserved |
| CS      | -             | -       | -    | high on module: I2C mode         | |
| SDO/SA0 | -             | -       | -    | high on module: address 0x6B     | |

- I2C1 at 100 kHz, open-drain, pull-ups on the module (no internal pull-ups).
- Detected chip: WHO_AM_I 0x69 (LSM6DS3) at 0x6B. Firmware also probes 0x6A and
  accepts 0x69/0x6A/0x6C (see `fw/hal/src/lsm6ds3.cpp`).

## On-board connections (fixed, not on the headers)

| MCU  | Function                       | Notes                                   |
|------|--------------------------------|-----------------------------------------|
| PA2  | USART2_TX -> ST-LINK VCP       | planned; SB3/SB4 ON; `/dev/ttyACM0`     |
| PA3  | USART2_RX <- ST-LINK VCP       | planned                                 |
| PA13 | SWDIO                          | ST-LINK flashing/debug; do not reuse    |
| PA14 | SWCLK                          | ST-LINK flashing/debug; do not reuse    |
| PC6  | LD3 green user LED             | default SB13 ON; `fw/hal/src/gpio.cpp`  |
| NRST | reset                          | also on CN3-3 and CN4-3                 |

LD1 (ST-LINK COM) and LD2 (power) are not MCU-controlled.

## Gotchas

- **Never set `SYSCFG_CFGR1.PA11_RMP/PA12_RMP`.** PA9/PA10 (D5/D4) are their own pins;
  the remap makes the PA11/PA12 pins (A5/A4) act as PA9/PA10 and forces the real
  PA9/PA10 to analog, which disconnects the IMU. A4/A5 are otherwise free (I2C2 capable).
- Power the IMU from 3V3, not 5V: the MCU and the IMU share 3.3 V logic levels.

## Header map (used pins marked)

```
 CN3 (left)                      CN4 (right)
 1  D1  PB6                      1  VIN
 2  D0  PB7                      2  GND   <- IMU GND
 3  NRST                         3  NRST
 4  GND                          4  +5V
 5  D2  PA15  <- IMU INT1        5  A7  PA7
 6  D3  PB1   (reserved: INT2)   6  A6  PA6
 7  D4  PA10  <- IMU SDA         7  A5  PA11
 8  D5  PA9   <- IMU SCL         8  A4  PA12
 9  D6  PB0                      9  A3  PA5
10  D7  PB2                     10  A2  PA4
11  D8  PB8                     11  A1  PA1
12  D9  PA8                     12  A0  PA0
13  D10 PB9                     13  AREF
14  D11 PB5                     14  3V3   <- IMU VCC
15  D12 PB4                     15  D13 PB3
```
