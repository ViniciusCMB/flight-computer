# Avionics — Onboard Computer

> **v2.0 Flight Computer** for Serra Rocketry (#11 - Dedalo).
>
> OOP + FreeRTOS + 4-state FSM, validated against real flight data.
> Built for the ESP32-S3.

## Overview

Real-time avionics firmware for a sounding rocket. Runs on an **ESP32-S3**
under FreeRTOS, reading barometric (BMP585), inertial (LSM6DS3) and GPS
(NEO-8M) sensors to detect flight events and deploy the parachute at apogee.

- **Flight state machine**: IDLE → ASCENT → DESCENT → LANDED, with sub-event
  flags (liftoff, burnout, apogee, freefall, parachute)
- **Parachute deployment**: Apogee + stable negative Vz, 3-cycle
  confirmation, 50 m ground guard — validated against RocketPy + real flight data points
- **Telemetry**: 22-field CSV over LoRa @ 915 MHz to ground receiver
- **Logging**: SD card (primary) with LittleFS flash fallback
- **Safety**: NaN/Inf rejection, sensor fallback, TWDT watchdog, multi-condition parachute logic

## Architecture

```
firmware/
├── firmware.ino          # setup()/loop() — orchestrates init*Task()
├── config.h              # Pins, thresholds, radio parameters
├── sensors/              # ISensor implementations (BMP585, LSM6DS3, GPS)
├── flight/               # FreeRTOS tasks + FlightStateMachine
├── modules/              # Actuators/peripherals (servo, LoRa, buzzer, FS)
└── docs/architecture.md  # v2.0 architecture (consolidated from REFACTORING_PLAN.md)
```

Two FreeRTOS cores with queue-based communication:

| Core  | Task                | Priority | Rate  | Responsibility                        |
| ----- | ------------------- | -------- | ----- | ------------------------------------- |
| **1** | `FlightControlTask` | 20       | 5 Hz  | Sensors + FSM + parachute + watchdog  |
| **0** | `TelemetryTask`     | 5        | 5 Hz  | GPS enrichment + LoRa + file + Serial |
| **0** | `LoggerTask`        | 1        | —     | Async log queue with level filter     |

## Quick Start

### Arduino IDE (recommended)

1. Board: **ESP32-S3 Dev Module** (enable "USB CDC On Boot")
2. Open `firmware/firmware.ino`
3. Install libraries: `Adafruit BMP5xx`, `Adafruit LSM6DS3`, `TinyGPS++`,
   `ESP32Servo`, `LoRa by Sandeep Mistry`
4. Compile (`Ctrl+R`) and upload (`Ctrl+U`)

### Validate without hardware

```bash
python3 extras/FSM_tester/FSM_Tester.py       # FSM against 1,873 real data points
python3 extras/validate_telemetry_format.py    # 22-field telemetry alignment
```

## Key Specifications

| Parameter          | Value                                    |
| ------------------ | ---------------------------------------- |
| FlightControl rate | 5 Hz (200 ms)                            |
| Telemetry rate     | 5 Hz (200 ms)                            |
| Parachute confirm  | 3 consecutive Vz < −2 m/s                |
| Ground guard       | 50 m AGL                                 |
| LoRa frequency     | 915 MHz (Brazil/Americas ISM)            |
| LoRa config        | SF7, BW 125 kHz, CR 4/5, CRC on, +17 dBm |
| Storage            | SD card (SPI) → LittleFS fallback        |
| Sensor queue       | 25 slots (∼2.4 KB)                       |
| Log queue          | 50 slots (∼7.2 KB)                       |

## Documentation

- [`docs/software.md`](docs/software.md) — Software architecture
|- [`docs/hardware.md`](docs/hardware.md) — Hardware specs, pinout, BOM
|- [`docs/architecture.md`](docs/architecture.md) — Architecture spec (consolidated)
|- [`docs/modules.md`](docs/modules.md) — Module reference
|- [`docs/flowchart.md`](docs/flowchart.md) — FreeRTOS + FSM flow diagram
- [`AGENTS.md`](AGENTS.md) — AI agent coding guide
- [`CONTRIBUTING.md`](CONTRIBUTING.md) — How to contribute

## Repository Layout

```
firmware/   v2.0 firmware (OOP + FreeRTOS + FSM)
test/       Hardware validation sketches (sensors, servo, FSM)
docs/       software.md, hardware.md, telemetry-format.md, flowchart.md
hardware/   KiCad schematic + PCB + BOM
extras/     Scripts, FSM tester, format validator, emergency deploy
```

## Status

All refactoring plan phases are **complete** — v2.0 is fully implemented and
documented. See [`CHANGELOG.md`](CHANGELOG.md) for the full release history.

## Team

Serra Rocketry — IPRJ/UERJ

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
