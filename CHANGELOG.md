# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased] — LASC 2026 Post-Flight

### Flight-Critical Fix
- **50 Hz → 5 Hz flight loop** (`d0865ee`): Thonyan flight #51 (2026-09-03)
  descended ballistically — parachute never deployed. Root cause: the 50 Hz
  loop outran the BMP585's ~25 Hz effective conversion rate, causing `vz` to
  alternate between 0.00 (stale cycle) and 2×real (double step over the 20 ms dt).
  Both `PARACHUTE_CONFIRM_CYCLES=3` and `FREEFALL_BACKSTOP_CYCLES=50` —
  consecutive-cycle gates — never survived the interleaved zeros.
  At 5 Hz, every cycle sees a fresh baro conversion → `vz` continuous →
  deploy confirmed at 317 m (+0.6 s after apogee, apogee 359 m; MC: 968 m).
  Validated: `extras/FSM_tester/validate_fsm_5hz.py` reproduces the failure
  numerically (50 Hz stale model → deploy=None; 5 Hz → deploy at 317 m).
- All cycle-count constants rescaled to preserve absolute times:
  backstop 50→5, baro-stale 125→13, arming rezero 150→15, glitch 50→5,
  liftoff-alt-confirm 3→1. `LIFTOFF_CONFIRM_MS` (100 ms) and
  `PARACHUTE_CONFIRM_CYCLES` (3 = 600 ms) unchanged.

### Fixed
- **Servo pin + team ID alignment** (`4dc4aca`): `SERVO_PIN` 7→39 (pin 7
  was floating on ESP32-S3 DevKitC — chute would not actuate even with the
  FSM calling deploy). `TEAM_ID` #51→#11 (Serra Rocketry canonical).
- **Hardware schematic**: R1 (servo signal) 1k→220Ω (bench measurement —
  1k was clamping GPIO39 output swing, causing servo jitter).
- **Sensor init retry** (`c86a324`): BMP585/LSM6DS3 sometimes fail `begin()`
  on first power-up; retry within 10 s window with buzzer feedback per attempt.
- Buffer overflow fix in GPSModule (`sprintf` → `snprintf`).
- Sensor fallback on corrupted readings (NaN/range validation, last-good
  value preservation).

### Added
- **Emergency deploy board** (`25cee6b`): standalone ESP32-C3 SuperMini
  firmware with barometer-only FSM (BMP280 + servo GPIO19, 10 Hz loop).
  Deploys chute if main S3 computer fails to init or FSM gets stuck.
  Includes `bench_emergency.ino` self-test.
- **Thonyan #51 post-flight data** (`211b0b2`, `110657f`): raw RX log (38
  packets), parsed CSV, Jupyter analysis notebook, vz comparison chart.
- Random TX jitter on telemetry period (satellite #213 collision avoidance).

### Changed
- FlightControlTask rate: 50 Hz → 5 Hz (all cycle constants rescaled).
- `LIFTOFF_CONFIRM_MAX_GAP_MS`: 60→200 ms (one 5 Hz frame tolerance).
- `LIFTOFF_ALT_CONFIRM_CYCLES`: 3→1 (one 5 Hz frame = 200 ms).
- Private member naming aligned with v2.0 convention (snake_case → _camelCase).

### Documentation
- Consolidated `firmware/REFACTORING_PLAN.md` → `docs/architecture.md`
  (1,313 lines → living reference with safety validation summary).
- Consolidated `firmware/MODULOS.md` → `docs/modules.md`.
- Removed legacy `firmware/MODULOS.md` and `firmware/REFACTORING_PLAN.md`
  (content preserved in docs/).
- Updated `README.md`, `docs/software.md`, `docs/flowchart.md`, `AGENTS.md`
  for 5 Hz and new doc structure.

### Flight Records
- **`flight_records/` directory**: centralized post-flight data per mission.
- **Thonyan #51** (Dedalo): ballistic descent, no deploy. Apogee 359 m
  (MC: 968 m). Deploy altitude if fix had been active: 317 m.
- **Dedalo** (mission data): `extras/FSM_tester/flight_results_dedalo.csv`.
- Golden RocketPy simulation: apogee 951 m → deploy 949 m (PASS).

---

## [2.0.0] - 2026-07-19

### Added
- **Phase 1-2: Project structure and base interfaces**
  - `firmware/sensors/ISensor.h` — Abstract interface for all sensors (BMP585, LSM6DS3, GPS)
  - `firmware/flight/SensorData.h` — Shared data structures (SensorData, LogMessage)
  - Sensor abstraction layer enabling polymorphic sensor implementations
  - FreeRTOS-compatible data structures for inter-task communication via queues
  - Complete Doxygen documentation for all interfaces

- **Phase 3: BMP585Sensor class**
  - Barometric altitude, pressure, temperature reading
  - Vertical velocity (Vz) via numerical differentiation, clipped to ±200 m/s
  - NaN/Inf and range validation with fallback to last known good value
  - IIR low-pass filter (alpha=0.2) for accelerometer data

- **Phase 4: LSM6DS3Sensor class**
  - 6-axis accelerometer + gyroscope with safety validations
  - NaN/Inf rejection and range checking (±200 m/s² accel, ±2000 °/s gyro)
  - Vector accessors via pointer parameters (`getAcceleration`, `getGyroscope`)

- **Phase 5: GPSModule class**
  - Non-blocking NMEA parsing via TinyGPS++ at 5Hz
  - HardwareSerial dependency injection for testability
  - GPS time used for CSV file naming (NOFIX fallback)

- **Phase 6: 4-state Flight State Machine**
  - IDLE → ASCENT → DESCENT → LANDED with sub-event flags (liftoff, burnout, apogee, freefall, parachute)
  - Thresholds validated against real flight data (1,873 points from `13_30_11-Dados.csv`)
  - IIR filter with seed on first reading (avoids transient)
  - Parachute deploy with multi-cycle confirmation (3 cycles of stable negative Vz)
  - Ground guard (50m) preventing deployment near terrain

- **Phase 7: FreeRTOS multi-task architecture**
  - **FlightControlTask** (Core 1, Priority 20, 5 Hz): sensor reads + FSM + parachute + watchdog
  - **TelemetryTask** (Core 0, Priority 5, 5Hz): GPS + queue drain + LoRa + file logging
  - **LoggerTask** (Core 0, Priority 1): async log queue with level filtering
  - Queue-based inter-task communication (no shared variables)
  - ESP32 TWDT armed inside the running task (not in init)
  - Performance metrics (cycle count, overruns, queue drops, exec time)

- **Phase 8: firmware.ino integration**
  - `setup()` delegates to `initFlightControlTask()` / `initTelemetryTask()` / `initLoggerTask()`
  - Safe-hold on critical failure (infinite loop + buzzer) — no `ESP.restart()`
  - `loop()` blocks on `vTaskDelay(portMAX_DELAY)`; all work in FreeRTOS tasks

- **Phase 9: Module adaptation**
  - `parachute_module` — Servo actuator (one-shot, idempotent deploy)
  - `lora_module` — RFM95W at 915 MHz, SF7, BW125k, CR4/5, CRC enabled
  - `filesystem_module` — SD card primary + LittleFS fallback, transparent dispatch
  - `buzzer_module` — Status tones (init success/failure)

- **Phase 10: Telemetry format (22 fields)**
  - CSV format aligned with `recovery-webui` receiver: `TEAM_ID,millis,count,altp,temp,umi,p,gx,gy,gz,ax,ay,az,vz,maxAltitude,state,alt,lat,lon,sat,parachute,rssi`
  - Single `snprintf` call (no heap fragmentation)
  - GPS enrichment in TelemetryTask (not FlightControlTask)

### Changed
- Migrated from v1.0 procedural architecture to v2.0 Object-Oriented design
- Restructured firmware directory: `sensors/`, `flight/`, `modules/`
- **FSM simplified to 4 main states** with internal event flags (was 7-state model)
- Replaced BMP280 with BMP585 (I2C, improved accuracy)
- Replaced MPU6050 with LSM6DS3 (6-axis, better range)
- Replaced NEO-6M GPS with NEO-8M (multi-constellation)
- Removed WiFi AP + Web server (v1.0 feature, not needed in v2.0)
- Updated `docs/software.md` with new module structure and component mapping
- Pin table in `docs/hardware.md` updated for ESP32-S3 pinout
- `docs/flowchart.md` rewritten for FreeRTOS + 4-state FSM
- `CONTRIBUTING.md` rewritten for v2.0 workflows

### Fixed
- **Critical Safety Initializations**:
  - All SensorData struct fields now have safe default values
  - LogMessage buffer initialized with zero-terminator
  - Prevents undefined behavior from uninitialized variables
  - Ensures parachute_deployed flag cannot be random on startup
  - Protects against NaN propagation in FSM transitions

### Documentation Added
- `docs/adr/002-sensor-abstraction.md` — Architectural Decision Record for ISensor interface

---

## [1.0.0] - 2026-01-27

### Added
- Initial project structure
- Base firmware for ESP32-C3 Super Mini
- Altitude monitoring system (MPU6050 sensor)
- GPS integration
- LoRa communication with operational base
- Telemetry storage in LittleFS
- Web interface for data access
- Component unit tests
- Software and hardware documentation

### Release Notes
- First functional version of the onboard computer
- Parachute deployment system still under testing
- Sensor calibration required before flight

---

## Versioning Guide

### MAJOR (X.0.0)
- Incompatible changes to firmware API or data structure

### MINOR (0.X.0)
- New features backward compatible with previous version
- Functionality improvements

### PATCH (0.0.X)
- Bug fixes
- Performance optimizations
- Documentation updates

## How to Report Changes

When making commits or pull requests, use the following categories:

- `feat:` for new features
- `fix:` for bug fixes
- `docs:` for documentation
- `test:` for tests
- `refactor:` for code refactoring
- `perf:` for performance improvements
- `chore:` for maintenance tasks

Example: `feat: add temperature sensor`
