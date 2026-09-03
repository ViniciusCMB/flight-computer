# Bench Self-Test (`test/bench/`)

Interactive self-test for every flight-computer sensor, module and actuator.
Run on the bench with the full stack wired as for flight.

## Flash & run

**Board variant (2026-08-29):** this DevKit S3 has an external CH343 USB-serial
bridge (QinHeng 1a86:55d3), NOT the native S3 USB-CDC. Therefore:

- Do **NOT** use `CDCOnBoot=cdc` — it remaps `Serial` to the native USB-CDC
  (GPIO19/20), which is not wired to this board's connector, and ALL app
  prints disappear (only the ROM bootloader + system log remain visible).
- Use `PSRAM=disabled` (module has no PSRAM chip) and `DebugLevel=none` to
  keep the boot log clean.

```bash
# Firmware
~/bin/arduino-cli compile -b "esp32:esp32:esp32s3:PSRAM=disabled,DebugLevel=none" firmware/firmware.ino
~/bin/arduino-cli upload -p /dev/ttyACM0 -b "esp32:esp32:esp32s3:PSRAM=disabled,DebugLevel=none" firmware/firmware.ino

# Bench self-test
~/bin/arduino-cli compile -b "esp32:esp32:esp32s3:PSRAM=disabled,DebugLevel=none" test/bench
~/bin/arduino-cli upload -p /dev/ttyACM0 -b "esp32:esp32:esp32s3:PSRAM=disabled,DebugLevel=none" test/bench

# Serial monitor at 115200
~/bin/arduino-cli monitor -p /dev/ttyACM0 --config 115200
```

Bench tips (learned the hard way):

- **Close any serial monitor before uploading** — the port is exclusive
  (`Errno 11 Could not exclusively lock port` otherwise).
- Upload does NOT erase NVS — a stale FSM snapshot survives re-flash and the
  FSM boots straight into ASCENT/DESCENT. For a clean start:
  ```bash
  ~/.arduino15/packages/esp32/tools/esptool_py/5.3.0/esptool \
    --chip esp32s3 --port /dev/ttyACM0 --baud 921600 erase-flash
  ```
  (Full-flight procedure never erases — use the `ARM` serial command instead.)
- If upload fails with `No serial data received`: hold BOOT, pulse EN, release
  BOOT, run the upload immediately.

## Commands

Send a single letter over Serial (115200):

| Cmd | Test |
|-----|------|
| `a` | **ALL** — runs every test sequentially, prints grand total |
| `b` | Barometer: BMP585 @0x7E primary, BMP280 @0x76/0x77 fallback; pressure/temp ranges + repeatability (sd < 0.5 hPa over 10 samples) |
| `i` | IMU: LSM6DS3 @0x6B; accel ~1g at rest, gyro < 0.35 rad/s at rest, temp, bus health (20 reads < 200 ms) |
| `g` | GPS: NEO-8M UART1 RX=17/TX=18 @9600; NMEA flow, checksum validity, fix |
| `l` | LoRa: RFM95W SPI 12/13/11 CS=10 RST=4 DIO0=5 @915 MHz; init, params, TX packet (endPacket==1), 3 s RX window |
| `f` | Filesystem: SD @CS=14 (type + write/read), falls back to LittleFS |
| `s` | Servo: 3x CLOSED->OPEN->CLOSED sweeps on GPIO7 (**actuator moves**) |
| `z` | Buzzer: 3 beeps on GPIO6 (audible verdict is manual) |
| `?` | List commands |

## Latest bench result (2026-08-27)

**18 PASS / 5 FAIL** — full log:

- BMP585 @0x7E: **FAIL (hardware absent)** — BMP280 fallback active, all checks
  PASS (920.47 hPa, 22.01 C, repeatability sd=0.0066 hPa).
- LSM6DS3: begin/accel/temp/bus PASS; gyro offset FAIL at rest
  (0.22/-0.88/-0.65 rad/s — re-test with board still; possible static bias).
- GPS: **FAIL was wiring** — GPS TX physically on GPIO17 (raw sniffer
  `test/gps_diag/` proved it). After the `config.h` swap (RX_GPS=17,
  TX_GPS=18): 4/4 PASS, 11 sats, 3D fix.
- LoRa: all PASS (TX endPacket==1; RX window empty without a second radio — normal).
- SD: all PASS (SDHC 7.7 GB, write/read round-trip).
- Servo: attach + 3 sweeps PASS (mechanical verdict manual).
- Buzzer: executed (audible verdict manual; user reports low volume —
  hardware review in progress: resistor/transistor drive, passive-vs-active type).

## Notes

- Pinout/constants in `bench.ino` are a **mirror of `firmware/config.h`** —
  keep in sync (this bit us once already: the GPS swap had to be applied in
  both places).
- The SD test writes/removes `/bench.txt`; LittleFS test writes `/bench.txt`.
- Servo test is the same actuator as the parachute door — keep fingers clear.
