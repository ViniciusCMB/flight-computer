# Software Documentation - Onboard Computer

## Overview

The Flight Computer v2.0 firmware runs on an **ESP32-S3** (the v2.0 target
platform; the earlier ESP32-C3 SuperMini was used for the prototype/dev
firmware and is still used for the emergency backup board — see
[`extras/emergency/`](../extras/emergency)). It uses a FreeRTOS multi-task
architecture, managing sensors, communication and parachute control during
flight.

**Version**: 2.0.0 (all phases complete)
**Architecture**: FreeRTOS-based OOP (Phases 1-10 complete)
**Hardware**: ESP32-S3 (v2.0 target; ESP32-C3 SuperMini used for prototype/dev)
**Team**: #11 - Serra Rocketry

## Architecture (v2.0)

The v2.0 refactoring introduces:

- **Object-oriented sensor abstraction** (`ISensor` interface)
- **Multi-task real-time architecture** (FreeRTOS, 2 cores)
- **Type-safe data sharing** (`SensorData` struct + queues)
- **Flight State Machine** — 4 outer states (IDLE → ASCENT → DESCENT → LANDED)
  with 7 internal sub-event flags (liftoff, burnout, apogee, freefall,
  parachute); validated with real flight data (1,873 points in
  `13_30_11-Dados.csv`) and RocketPy simulation.

```mermaid
graph TB
    subgraph "Core 1 - Flight Critical"
        FC[FlightControlTask<br/>5 Hz, Priority 20]
        FSM[FlightStateMachine<br/>4 states + 7 sub-events]
        SENS[Sensor Updates<br/>BMP585, LSM6DS3]
        PARA[ParachuteServo<br/>deploy at apogee]

        FC --> FSM
        FC --> SENS
        FSM -->|deploy on apogee| PARA
    end

    subgraph "Core 0 - Non-Critical"
        TEL[TelemetryTask<br/>5Hz, Priority 5]
        LOG[LoggerTask<br/>Low Priority]
        GPS[GPSModule<br/>non-blocking UART]
    end

    subgraph "Shared Resources"
        QUEUE[(Sensor Data Queue<br/>SENSOR_DATA_QUEUE_LEN slots)]
        LOGQUEUE[(Log Queue<br/>LOG_QUEUE_LEN slots)]
    end

    FC -->|xQueueSend| QUEUE
    TEL -->|xQueueReceive| QUEUE
    TEL --> GPS
    FC -->|xQueueSend| LOGQUEUE
    LOG -->|xQueueReceive| LOGQUEUE

    TEL -->|Serial + LoRa + storage (SD/LittleFS)| OUT[(Telemetry sinks)]

    style FC fill:#f96,stroke:#333,stroke-width:2px
    style TEL fill:#9cf,stroke:#333,stroke-width:2px
    style LOG fill:#9cf,stroke:#333,stroke-width:2px
```

## Project Structure (v2.0)

> **Build note**: the Arduino IDE / arduino-cli only compiles `.cpp`/`.h` files
> located **at the sketch root** (`firmware/`). Headers (`.h`) are kept in
> subfolders (`sensors/`, `flight/`, `modules/`) for organization; the matching
> implementation files (`.cpp`) live at the root so they are picked up by the
> Arduino build (subfolders are not recursed for sources).

```text
firmware/
├── firmware.ino                # Entry point (FreeRTOS setup + init*Task())
├── config.cpp                  # Config helpers (sketch-root source)
├── sensors/                    # OOP sensor abstraction (ISensor) — headers
│   ├── ISensor.h               # Abstract interface (begin/update/getData/isReady)
│   ├── BMP585Sensor.h          # Barometer (altitude, pressure, temp, Vz)
│   ├── LSM6DS3Sensor.h         # IMU (accel + gyro)
│   └── GPSModule.h             # GNSS (lat/lon/alt/sats, non-blocking)
├── modules/                    # Actuators & peripherals — headers
│   ├── parachute_module.h      # ParachuteServo + setupServo() (servo owner)
│   ├── lora_module.h           # setupLoRa() / sendLoRa() (915 MHz)
│   ├── buzzer_module.h         # Status buzzer
│   └── filesystem_module.h     # Storage SD + LittleFS fallback (setupStorage)
├── flight/                     # Flight logic — headers
│   ├── SensorData.h            # SensorData struct + FlightState enum
│   ├── FlightStateMachine.h    # FSM (4 states + 7 sub-events)
│   ├── FlightControlTask.h     # Task 1 — 5 Hz (FSM + deploy + queue)
│   ├── TelemetryTask.h         # Task 2 — 5 Hz (assemble + LoRa + file)
│   └── LoggerTask.h            # Task 3 — low priority (log Serial)
├── BMP585Sensor.cpp            # sensor impl (root — compiled by Arduino)
├── LSM6DS3Sensor.cpp
├── GPSModule.cpp
├── FlightStateMachine.cpp
├── FlightControlTask.cpp
├── TelemetryTask.cpp
├── LoggerTask.cpp
├── parachute_module.cpp
├── docs/architecture.md        # v2.0 architecture (consolidated)
└── docs -> ../docs             # telemetry-format.md (FSM/format reference)
```

## FreeRTOS Tasks

| Task | Core | Rate | Priority | Responsibility |
|------|------|------|----------|----------------|
|| `taskFlightControl` | 1 | 5 Hz | 20 | Update sensors + FSM, deploy parachute at apogee, push `SensorData` to `sensorDataQueue`, feed TWDT |
| `taskTelemetry` | 0 | 5 Hz | 5 | Drain `sensorDataQueue` (newest sample), enrich with GPS, assemble CSV v2.0, fan-out to Serial + LoRa + storage (SD/LittleFS) |
| `taskLogger` | 0 | event | 1 | Consume `logQueue`, print to Serial (level filter) |

Queues (defined in `config.h`):

- `sensorDataQueue` — between FlightControl and Telemetry.
- `logQueue` — between any task and Logger.

## Flight State Machine

See [`docs/architecture.md`](architecture.md) Phase 6 for the full specification.

- **Outer states**: `IDLE → ASCENT → DESCENT → LANDED` (enum `FlightState`).
- **Sub-event flags** (diagnostic, set once): `liftoff`, `burnout`, `apogee`,
  `freefall`, `parachute`.
- **Parachute deploy (Option A)**: `detectParachute()` confirms apogee + stable
  negative `Vz` for `PARACHUTE_CONFIRM_CYCLES` cycles, never below
  `PARACHUTE_MIN_ALTITUDE` (50 m ground guard). FlightControlTask actuates the
  servo via `parachute_module`.

## Telemetry

The v2.0 telemetry format is defined in [`docs/telemetry-format.md`](telemetry-format.md)
(single source of truth). Summary:

- **Satellite → Receiver**: 22-field CSV
  `TEAM_ID,millis,count,altp,temp,umi,p,gx,gy,gz,ax,ay,az,vz,maxAltitude,state,alt,lat,lon,sat,parachute,rssi`
- **Receiver → WebUI**: 24-field CSV (inserts local GPS `hora`/`data` + real `rssi`).
- **Radio**: 915 MHz, SYNC 0xF3, SF7, BW 125 kHz, CR 4/5, TX +17 dBm, CRC on.
- **Local storage (SD card, LittleFS fallback)**: same 22-field CSV header as the transmitted line.

## Storage (SD card with LittleFS fallback)

`filesystem_module.h` provides a transparent storage abstraction:

- `setupStorage()` — tries the **SD card** (SPI, `SD_CS_PIN`) first; on failure
  falls back to **LittleFS** (internal flash, auto-format). If both fail,
  telemetry continues without file logging (the system does not halt).
- `writeFile()` / `appendFile()` — dispatch to whichever backend is active
  (`g_storage_type`), so application code never picks a backend explicitly.
- Helpers: `getStorageName()`, `isStorageReady()`.

- **Format**: CSV (22-field telemetry header, see `docs/telemetry-format.md`).
- **File name**: `HH_MM_SS-Dados.csv` (GPS time) or `{millis}-Dados.csv` if no fix.
- **Functions**: `setupStorage()`, `writeFile()`, `appendFile()`
  (`filesystem_module.h`).

## Communication

### Serial UART

- **Baud Rate**: 115200 (debug + real-time monitoring).

### LoRa (RFM95W)

- **Frequency**: 915 MHz (Americas/Brazil ISM).
- **Sync Word**: 0xF3.
- **Spreading Factor**: 7, **Bandwidth**: 125 kHz, **Coding Rate**: 4/5,
  **TX Power**: +17 dBm, **CRC**: on.
- **Range**: up to ~4 km (open field).

## Parachute (Option A)

- **Actuator**: `ParachuteServo` (owned by `parachute_module.h`); positions set
  by `setupServo()` and `deployParachute()`.
- **Decision**: FSM `detectParachute()` at apogee (validated: RocketPy apogee
  951 m → deploy 949.5 m; real flight apogee 272 m → deploy 268 m).
- **Guards**: `PARACHUTE_MIN_ALTITUDE = 50 m` (ground guard only);
  `PARACHUTE_CONFIRM_VZ = -2.0 m/s`; `PARACHUTE_CONFIRM_CYCLES = 3`.

## Build & Test

- **Arduino IDE**: Board `ESP32-S3 Dev Module` (the v2.0 target). The
  ESP32-C3 SuperMini build (`ESP32-C3 Dev Module`) works for the prototype
  firmware but pin assignments in `config.h` are C3-specific and must be
  re-mapped for the S3.
- **PlatformIO** (available): `platformio run -e esp32-c3`.
- **FSM validation**: `python3 extras/FSM_tester/FSM_Tester.py` (real data).
- **Telemetry validation**: `python3 extras/validate_telemetry_format.py`.

## Tests

Hardware tests in [`test/`](../test/):

- `test/basico/basico.ino` — basic init
- `test/buzzer/buzzer.ino` — buzzer
- `test/lora/lora.ino` — LoRa
- `test/testeGPS/testeGPS.ino` — GPS
- `test/servo/servo.ino` — servo
- `test/LittleFS/LittleFS.ino` — filesystem
- `test/FSM/FSM.ino` — FSM reference implementation (validated)

## Dependencies — Arduino Libraries

| Library | Use |
|---------|-----|
| Adafruit BMP585 | Pressure/altitude sensor |
| Adafruit LSM6DS3 | IMU |
| TinyGPS++ | GPS decoding |
| LoRa | RFM95W LoRa module |
| ESP32Servo | Servo control |
| Arduino_JSON / ArduinoJson | JSON (if used) |

## Development Notes

- **Safety**: parachute uses apogee + confirmed negative Vz; ground guard only.
- **Determinism**: FlightControlTask feeds the TWDT; telemetry is best-effort.
- **Logging**: all telemetry stored locally (SD card, LittleFS fallback) before/with transmission.
- All code comments in English; UI strings in English; telemetry keys per
  firmware convention.
