# Architecture — Flight Computer v2.0

Architecture specification for the v2.0 firmware (OOP + FreeRTOS + 4-state FSM).
Consolidates the former `firmware/REFACTORING_PLAN.md` into a living reference.

## Overview

The v2.0 firmware migrates from the v1.0 procedural architecture to an
object-oriented design with selective POO (sensors only) and a strict
FreeRTOS multi-task separation between safety-critical (FSM + deploy) and
non-critical (telemetry/logging) paths.

**Hardware**: ESP32-S3 DevKitC-1-N8R8 (primary). ESP32-C3 SuperMini for the
emergency backup board (`extras/emergency/`).

## Design Decisions

| Aspect | Decision | Rationale |
|--------|----------|-----------|
| OOP | Selective | Only sensors that change became classes |
| FSM | Enum/Switch | Simple, low overhead |
| FreeRTOS | 3 Tasks | Separation: FSM (5 Hz) / Telemetry (5 Hz) / Logger |
| Migration | Big Bang | Full refactor in one pass |
| Tests | Hardware + Python | No C++ unit-test infra; FSM validated via Python port |
| Queue sizes | 25 (sensor), 50 (log) | Covers worst case with margin |
| Watchdog | 5s, armed inside FlightControlTask | Avoids TWDT armed with no task to reset it |
| Logging | Queue-based + Serial filter | Verbose dev, filtered in production |

## Architecture (post-5Hz fix)

### Directory layout

```
firmware/
├── firmware.ino                    # Entry point (setup + init*Task)
├── config.h                        # Pins, thresholds, radio/storage params
├── config.cpp                      # Config helpers
├── sensors/                        # OOP sensor abstraction (headers)
│   ├── ISensor.h                   # Abstract interface
│   ├── BMP585Sensor.h              # Barometer (alt, press, temp, Vz)
│   ├── LSM6DS3Sensor.h             # IMU (6-axis)
│   └── GPSModule.h                 # GNSS (non-blocking NMEA)
├── flight/                         # FSM + FreeRTOS tasks (headers)
│   ├── SensorData.h                # Shared structs + FlightState enum
│   ├── FlightStateMachine.h        # 4-state FSM + 7 sub-event flags
│   ├── FlightControlTask.h         # Task 1 — 5 Hz, Core 1, priority 20
│   ├── TelemetryTask.h             # Task 2 — 5 Hz, Core 0, priority 5
│   └── LoggerTask.h                # Task 3 — low priority
├── modules/                        # Actuators/peripherals (headers)
│   ├── parachute_module.h          # Servo owner
│   ├── lora_module.h               # 915 MHz radio
│   ├── buzzer_module.h             # Status tones
│   └── filesystem_module.h         # SD + LittleFS fallback
├── *.cpp                           # Implementations at sketch root
└── REFACTORING_PLAN.md             # This document (legacy source)
```

> `.cpp` files live at the sketch root because Arduino IDE only compiles
> sources at the root of the `.ino` sketch folder. Headers are organized in
> subfolders; PlatformIO can use `src/` with subdirectories natively.

### FreeRTOS task layout

| Core | Task | Priority | Rate | Responsibility |
|------|------|----------|------|----------------|
| 1 | `taskFlightControl` | 20 (HIGH) | **5 Hz** (200 ms) | Sensors + FSM + deploy + queue + TWDT |
| 0 | `taskTelemetry` | 5 (LOW) | 5 Hz (200 ms) | GPS + queue drain + LoRa + storage + Serial |
| 0 | `taskLogger` | 1 | event | Async log queue consumer |

Inter-task communication via queues only — no shared variables:

- `sensorDataQueue` (25 slots) — FlightControl → Telemetry
- `logQueue` (50 slots) — any task → Logger

### Key tuning parameter: 5 Hz flight loop

> **Context**: Flight #51 (Thonyan, 2026-09-03) descended ballistically —
> parachute never deployed. Root cause: the flight loop ran at 50 Hz while the
> BMP585 effective conversion rate was ~25 Hz. `vz` alternated between 0.00
> (stale cycle) and 2×real (double step over the 20 ms dt). Both
> `PARACHUTE_CONFIRM_CYCLES=3` and `FREEFALL_BACKSTOP_CYCLES=50` — both
> consecutive-cycle gates — never survived the interleaved zeros.

**Fix** (`d0865ee`, `c5dd591`): `FLIGHT_CONTROL_PERIOD_MS` 20 → 200 ms.
All cycle-count constants rescaled to preserve the same **absolute times**:

| Constant | 50 Hz (old) | 5 Hz (new) | Absolute time |
|----------|------------|------------|---------------|
| `FREEFALL_BACKSTOP_CYCLES` | 50 | 5 | 1.0 s |
| `BARO_STALE_SUSTAIN_CYCLES` | 125 | 13 | 2.6 s |
| `ARM_REZERO_SUSTAIN_CYCLES` | 150 | 15 | 3.0 s |
| `BARO_GLITCH_SUSTAIN_CYCLES` | 50 | 5 | 1.0 s |
| `LIFTOFF_ALT_CONFIRM_CYCLES` | 3 | 1 | 200 ms |
| `PARACHUTE_CONFIRM_CYCLES` | 3 | 3 | 0.6 s (unchanged — time-based) |

`LIFTOFF_CONFIRM_MS` (100 ms) and `LIFTOFF_CONFIRM_MAX_GAP_MS` (60 → 200 ms)
are per-time; `PARACHUTE_CONFIRM_CYCLES` is a count (3 = 600 ms at 5 Hz).

**Validation**: `extras/FSM_tester/validate_fsm_5hz.py` reproduces the failure
numerically — at 50 Hz with stale interleaving, deploy = `None` (blocked);
at 5 Hz, deploy at 317 m, +0.6 s after apogee, matching the real flight.

## Flight State Machine

4 outer states with internal sub-event flags (set once, diagnostic only):

| Outer state | Sub-events tracked |
|-------------|--------------------|
| `IDLE` | — |
| `ASCENT` | `liftoff`, `burnout` |
| `DESCENT` | `apogee`, `freefall` |
| `LANDED` | `parachute` |

### Transitions

| From | To | Trigger |
|------|----|---------|
| IDLE | ASCENT | `detectLiftoff()`: totalAccel > 15 m/s² sustained via `LIFTOFF_CONFIRM_MS` (100 ms) |
| ASCENT | DESCENT | `detectApogee()`: \|vz\| < 1.0 m/s (zero crossing / peak) |
| DESCENT | LANDED | `detectLanded()`: \|vz\| < 0.5 m/s and altitude < 2 m |

### Deploy logic (Option A — at apogee, not descent)

Parachute deploy is **not** a state transition — it is queried by
`FlightControlTask` via `detectParachute()`:

```cpp
bool FlightStateMachine::detectParachute(float height, float vz) const {
  if (height < PARACHUTE_MIN_ALTITUDE) return false;   // 50 m ground guard
  if (vz < PARACHUTE_CONFIRM_VZ) {                     // -2.0 m/s
    return (_parachuteConfirmCount >= PARACHUTE_CONFIRM_CYCLES);  // 3 cycles
  }
  return false;
}
```

Deploy is invoked by `FlightControlTask` when the FSM signals confirmation,
via `parachute_module`'s `ParachuteServo`. Validated against RocketPy
(apogee 951 m → deploy 949.5 m) and real flight (apogee 272 m → deploy 268 m).

### Safety validation

| Test | Scenario | Result |
|------|----------|--------|
| `validate_fsm_5hz.py` | 50 Hz stale-sensor model reproduces ballistic failure | PASS (deploy = None at 50 Hz, deploy at 317 m at 5 Hz) |
| `validate_50hz_noise.py` | Quantization noise up to 20× real | PASS (0 false apogees, 0 premature deploys) |
| `validate_freefall_backstop.py` | FSM stuck in IDLE/ASCENT | PASS (backstop deploys at apogee) |
| `validate_baro_stale.py` | Barometer frozen mid-flight | PASS (IMU-only deploy at t=19.6s) |
| `validate_watchdog_reboot.py` | TWDT reset during flight | PASS (NVS restores state, deploy on apogee) |
| `validate_arming.py` | Bench vibration false-liftoff | PASS (50 m arming guard survives 22–122 m/s² spikes) |

See also [`docs/modules.md`](modules.md) for the module-by-module API.

## Safety: Dual-layer deploy (FSM-independent backstop)

### Layer 1: FSM Option A
Apogee + 3 consecutive cycles of Vz < -2 m/s, 50 m ground guard.

### Layer 2: Freefall backstop (independent of FSM state)

Implemented in `FlightControlTask::checkFreefallBackstop()` — fires even if
the FSM is alive but stuck in the wrong state (e.g. NVS-restored ASCENT mid-descent,
or FSM in IDLE with no snapshot). Uses its own IIR-filtered accelerometer
(α=0.2), separate from FSM state:

```
totalAccel (IIR α=0.2) < 3.0 m/s²   per 5 consecutive cycles (1.0 s)
        AND vz < -5.0 m/s            (real descent, excludes burnout)
        AND altitude > 50 m          (ground guard)
        → deployParachute()         (idempotent via g_parachuteActuated)
```

**Decisions grounded in real flight data:**
- `vz < -5 m/s` is mandatory: accelerometer drops to ~0g shortly after burnout
  (rocket still ascending). Without this guard, the chute would deploy at ~190 m
  on the way up.
- 1.0 s window: real zero-g windows last 8–131 s; bench vibration spikes are
  transient (22–122 m/s²) and rejected by the window.
- Own IIR filter: works even when FSM state is corrupted.

**Behavior**: fires 1–3 s after FSM deploy in normal flight (never before);
with FSM stuck, deploys at 922 m (simulated) / 255 m (real) — always above
the 50 m floor.

## Safety: Post-watchdog recovery (NVS)

If the TWDT resets the system mid-flight:

1. `FlightStateMachine::begin()` calls `restoreFromNVS()`:
   - No valid snapshot (first boot, or previous flight ended LANDED → clean
     snapshot) → FSM starts fresh in IDLE.
   - Valid snapshot in ASCENT/DESCENT → FSM **resumes from saved state**
     (flags + confirmation counter), and `base_pressure` from launch site is
     restored to the BMP585 → altitude remains **absolute to launch elevation**
     (no re-zeroing at the reboot point).
2. Snapshot saved on every state transition, at deploy, and once at boot.
   `reset()` clears it.
3. `setupServo(keepOpen)`: if the snapshot says chute already deployed, the
   servo is **not** re-closed on boot (closing with chute out mid-air would
   release it).

Validated: `extras/FSM_tester/validate_watchdog_reboot.py` — PASS in 5
simulated flight phases + 4 real flight phases.

**Known limitations**: a reboot during the powered phase re-arms only if
acceleration > 15 m/s² persists after boot; a snapshot from an aborted flight
(before LANDED) can restore ASCENT on the ground — the `PARACHUTE_MIN_ALTITUDE`
guard prevents false deploy.

## Resource usage

| Component | RAM | Flash |
|-----------|-----|-------|
| Base code | ~50 KB | ~200 KB |
| Adafruit libs | ~10 KB | ~50 KB |
| FreeRTOS overhead | ~2 KB | ~20 KB |
| Task stacks (3 × 8 KB avg) | 28 KB | — |
| Queues (25 + 50 slots) | ~8 KB | — |
| Sensor/FSM objects | ~5 KB | — |
| **Total** | **~103 KB** | **~270 KB** |

Available: 512 KB RAM (~20%); 8 MB flash (~3%).

## Change log (from REFACTORING_PLAN.md)

### 2026-08-04 — Freefall backstop (FSM-independent)
`checkFreefallBackstop()`: acc < 3 m/s² (own IIR) for 1.0 s + vz < -5 m/s +
h > 50 m → idempotent deploy. Validated in Python before C++
(`validate_freefall_backstop.py` — scenarios A–D, all PASS).

### 2026-08-04 — Watchdog post-reset recovery (NVS)
FSM persists snapshot to NVS (`Preferences`, namespace `flight`/key `fsm`):
state, flags, confirmation counter, `base_pressure`, `maxAltitude`.
`restoreFromNVS()` in `begin()`. `setupServo(keepOpen)` prevents re-closing
an already-deployed chute. Validated: `validate_watchdog_reboot.py` (9 scenarios).

### 2026-08-05 — Real-flight risk review (5 risks)
- **Risk 1** (barometer freezes mid-flight): resolved — `getLastReadingAgeMs()`
  + IMU-only `checkBaroStaleContingency()` (deploy if baro stale >2 s + acc <3
  m/s² + h > 50 m, validated `validate_baro_stale.py` A–D PASS).
- **Risk 3** (apogee lost to 50 Hz vz noise): not confirmed —
  `validate_50hz_noise.py` shows 100% apogee detection, 0 premature deploys,
  0 missed deploys up to 5 s after apogee.
- **Risk 5** (post-burnout oscillation loses apogee): resolved —
  apogee gate simplified to `|vz| < 1.0` only (removed `az` gate with ~0.81 m/s²
  margin). vz from barometer is immunized against oscillation by construction.

### 2026-05-09 — 5 Hz flight loop (Thonyan #51 fix)
`FLIGHT_CONTROL_PERIOD_MS` 20 → 200. See "Key tuning parameter: 5 Hz flight loop"
above. This is the single change that unblocked apogee deployment for
Dédalo II / Thonyan II.

### 2026-06-24 — All 10 phases complete
v2.0 firmware fully migrated to OOP + FreeRTOS + 4-state FSM. See `CHANGELOG.md`.
