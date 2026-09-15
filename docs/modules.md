# Modules — Flight Computer v2.0

Documentation for the firmware module architecture (OOP sensors, FreeRTOS tasks,
actuator modules). Replaces the former `firmware/MODULOS.md`.

## Project Structure

```text
firmware/
├── firmware.ino                # Entry point (FreeRTOS setup + init*Task())
├── config.h                    # Pins, thresholds, radio parameters
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
│   └── LoggerTask.h          # Task 3 — low priority (log Serial)
├── BMP585Sensor.cpp            # sensor impl (root — compiled by Arduino)
├── LSM6DS3Sensor.cpp
├── GPSModule.cpp
├── FlightStateMachine.cpp
├── FlightControlTask.cpp
├── TelemetryTask.cpp
├── LoggerTask.cpp
├── parachute_module.cpp
├── config.cpp
└── REFACTORING_PLAN.md         # v2.0 architecture specification
```

> **Build note**: Arduino IDE / arduino-cli only compiles `.cpp`/`.h` files at the
> sketch root (`firmware/`). Headers live in subfolders for organization; matching
> `.cpp` files live at the root so they are picked up by the Arduino build.

## sensors/ — ISensor abstraction layer

All sensors implement `ISensor` (`begin`, `update`, `getData`, `isReady`).

- **BMP585Sensor** — barometric altitude (m), pressure (hPa), temperature (°C),
  and vertical velocity `Vz` (m/s) via numerical differentiation. Methods:
  `getAltitude()`, `getVerticalVelocity()`, `getPressure()`, `getMaxAltitude()`,
  `getPreviousAltitude()`.
- **LSM6DS3Sensor** — acceleration `ax,ay,az` (m/s²) and gyroscope `gx,gy,gz`
  (rad/s). `getTotalAccel()` returns the magnitude `sqrt(ax²+ay²+az²)`.
- **GPSModule** — non-blocking fix via UART1. `hasValidFix()`, `getLatitude()`,
  `getLongitude()`, `getGPSAltitude()`, `getSatellites()`, `getTimeString()`.

## modules/ — actuators & peripherals

- **parachute_module.h** — sole owner of the actuator: `Servo ParachuteServo`
  and `bool setupServo()`. The deploy *decision* lives in the FSM +
  FlightControlTask (Option A: deploy at apogee). This module only positions
  the servo.
- **lora_module.h** — `setupLoRa()` applies 915E6 / SYNC 0xF3 / SF7 / BW125k /
  CR5 / TP17 / CRC, and `sendLoRa(String)`.
- **buzzer_module.h** — audio status tones (init / flight / landed).
- **filesystem_module.h** — storage abstraction with fallback:
  `setupStorage()` tries SD card (SPI) first; on failure uses LittleFS
  (internal flash). `writeFile()`/`appendFile()` dispatch to the active
  backend (`g_storage_type`). Helpers: `getStorageName()`, `isStorageReady()`.

## flight/ — task layer

- **SensorData.h** — `struct SensorData` (queue payload) and `enum FlightState`
  `{IDLE, ASCENT, DESCENT, LANDED}`. Sub-event flags (`liftoff`, `burnout`,
  `apogee`, `freefall`, `parachute`) are tracked as booleans in the FSM.
- **FlightStateMachine** — consumes BMP585 + LSM6DS3; detects liftoff, burnout,
  apogee, freefall, parachute (flags). Parachute deploy via Option A
  (apogee + stable negative Vz, `PARACHUTE_CONFIRM_CYCLES` cycles, 50m ground
  guard).
- **FlightControlTask** — Task 1 @5 Hz (Core 1, priority 20). Updates sensors,
  advances FSM, calls `deployParachute()` on Option A confirmation, pushes
  `SensorData` to `sensorDataQueue`, feeds TWDT.
- **TelemetryTask** — Task 2 @5 Hz (Core 0, priority 5). Drains
  `sensorDataQueue` (keeps newest sample), enriches with GPS, assembles CSV v2.0
  via `assembleTelemetry()`, fans out to Serial + LoRa + storage.
- **LoggerTask** — Task 3 (low priority). Consume `logQueue`, print to Serial
  (level filter).

## Queues (`config.h`)

- `SENSOR_DATA_QUEUE_LEN` — between FlightControl and Telemetry (25 slots).
- `LOG_QUEUE_LEN` — between any task and Logger (50 slots).

## Telemetry format

See [`docs/telemetry-format.md`](telemetry-format.md) (single source of truth).
Summary — 22-field CSV from satellite:

```
TEAM_ID,millis,count,altp,temp,umi,p,gx,gy,gz,ax,ay,az,vz,maxAltitude,state,alt,lat,lon,sat,parachute,rssi
```

Receiver extends to 24 fields (inserts local GPS `hora`/`data` + real `rssi`).
