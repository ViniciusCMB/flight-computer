/**
 * @file FlightStateMachine.cpp
 * @brief Flight State Machine implementation
 *
 * All detection logic is a direct port from test/FSM/FSM.ino (the
 * validated Arduino test sketch). Threshold values are identical to
 * those validated against real flight data in
 * extras/FSM_tester/FSM_Tester.py and extras/FSM_tester/13_30_11-Dados.csv.
 *
 * State machine:
 *   IDLE → (liftoff) → ASCENT → (apogee) → DESCENT → (landed) → LANDED
 *
 * Within ASCENT: burnout event is flagged (no state change).
 * Within DESCENT: freefall and parachute events are flagged (no state change).
 *
 * @see FlightStateMachine.h for class and threshold documentation
 * @see test/FSM/FSM.ino lines 76–117 — original detection functions
 */

#include "flight/FlightStateMachine.h"
#include "config.h"
#include <Preferences.h>

// NVS keys (defined out-of-line — static const char* const members)
const char* const FlightStateMachine::NVS_NAMESPACE = "flight";
const char* const FlightStateMachine::NVS_KEY       = "fsm";

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / lifecycle
// ─────────────────────────────────────────────────────────────────────────────

FlightStateMachine::FlightStateMachine(BMP585Sensor* baro, LSM6DS3Sensor* imu)
    : _baro(baro),
      _imu(imu),
      _state(IDLE),
      _ready(false),
      _liftoffDetected(false),
      _burnoutDetected(false),
      _apogeeDetected(false),
      _freefallDetected(false),
      _parachuteDeployed(false),
      _parachuteConfirmCount(0),
      _filtAx(0.0f),
      _filtAy(0.0f),
      _filtAz(0.0f),
      _firstReading(true),
      _stateEntryMs(0),
      _altAboveCount(0) {}

bool FlightStateMachine::begin() {
  if (!_baro || !_imu) {
    Serial.println("[FSM] ERROR: null sensor pointer");
    return false;
  }
  _ready = true;
  // After a watchdog reboot mid-flight, resume where the FSM was instead of
  // starting over at IDLE (which would never re-arm and never deploy).
  restoreFromNVS();
  // Always persist the current state + launch reference so a later reboot
  // has a valid snapshot to restore.
  persistToNVS();
  Serial.println("[FSM] Ready");
  return true;
}

bool FlightStateMachine::isReady() {
  return _ready;
}

void FlightStateMachine::reset() {
  _state            = IDLE;
  _liftoffDetected  = false;
  _burnoutDetected  = false;
  _apogeeDetected   = false;
  _freefallDetected = false;
  _parachuteDeployed = false;
  _parachuteConfirmCount = 0;
  _filtAx = _filtAy = _filtAz = 0.0f;
  _firstReading = true;
  _liftoffAccelAbove = false;
  _liftoffAccelStartMs = 0;
  _stateEntryMs = millis();
  _altAboveCount = 0;
  // Clear the persisted snapshot so the next boot starts fresh at IDLE.
  Preferences prefs;
  if (prefs.begin(NVS_NAMESPACE, false)) {
    prefs.remove(NVS_KEY);
    prefs.end();
  }
  Serial.println("[FSM] Reset -> IDLE (NVS cleared)");
}

// ─────────────────────────────────────────────────────────────────────────────
// Main update — called at 5 Hz from FlightControlTask
// ─────────────────────────────────────────────────────────────────────────────

void FlightStateMachine::update() {
  if (!_ready || !_baro->isReady() || !_imu->isReady()) return;

  const float height = _baro->getAltitude();
  const float vz     = _baro->getVerticalVelocity();

  float ax, ay, az;
  _imu->getAcceleration(&ax, &ay, &az);

  // Reject corrupt sensor frames before any computation
  if (!std::isfinite(height) || !std::isfinite(vz) ||
      !std::isfinite(ax)     || !std::isfinite(ay)  || !std::isfinite(az)) {
    return;
  }

  // IIR low-pass filter on accelerometer (ALPHA=0.2 — matches test/FSM/FSM.ino)
  // On first reading, seed the filter with the raw value so the filter state
  // starts at a real measurement rather than 0.
  if (_firstReading) {
    _filtAx = ax;
    _filtAy = ay;
    _filtAz = az;
    _firstReading = false;
  } else {
    _filtAx = smoothFilter(ax, _filtAx);
    _filtAy = smoothFilter(ay, _filtAy);
    _filtAz = smoothFilter(az, _filtAz);
  }

  const float acc = totalAccel(_filtAx, _filtAy, _filtAz);
  if (!std::isfinite(acc)) return;

  // ── State machine ────────────────────────────────────────────────────────

  switch (_state) {

    case IDLE:
      // Sustained-altitude guard: shaking the board produces accel spikes AND
      // a baro pressure puff that crosses LIFTOFF_MIN_HEIGHT for 1-2 samples.
      // A real ascent holds the height for many consecutive cycles, so require
      // both the timed accel confirmation AND sustained height above the guard.
      if (!_liftoffDetected) {
        _altAboveCount = (height > LIFTOFF_MIN_HEIGHT) ? _altAboveCount + 1 : 0;
        if (_altAboveCount >= LIFTOFF_ALT_CONFIRM_CYCLES &&
            detectLiftoffTimed(_filtAx, _filtAy, _filtAz)) {
          _liftoffDetected = true;
          transitionTo(ASCENT);
        }
      }
      break;

    case ASCENT:
      // Burnout is an informational sub-event; it does not change the state.
      if (!_burnoutDetected && detectBurnout(_filtAx, _filtAy, _filtAz, height, vz)) {
        _burnoutDetected = true;
        Serial.printf("[FSM] BURNOUT  h=%.1f vz=%.2f acc=%.2f\n", height, vz, acc);
      }
      // Apogee by vz only, single cycle. The az gate was removed (risk #5):
      // apparent acceleration includes centripetal/pendulum terms and the
      // accelerometer may have bias (real flight: az at rest = +2.81 m/s²),
      // so filtered az can stay above -0.1 at the true apogee and the gate
      // would lose it entirely. Sampling quantization noise makes |vz| briefly
      // cross zero before the real apogee — benign: the deploy gate below
      // (vz < -2 m/s sustained) still only fires on the real descent.
      // Validated: extras/FSM_tester/validate_50hz_noise.py (0 premature
      // deploys) and analyze_apogee_robustness.py.
      if (!_apogeeDetected && detectApogee(vz)) {
        _apogeeDetected = true;
        _parachuteConfirmCount = 0;  // Reset deploy confirmation at apogee
        transitionTo(DESCENT);
      }
      break;

    case DESCENT:
      // Backstop: DESCENT that never confirms LANDED (baro drift keeps the
      // relative height above the ground guard, or vz noise). After
      // DESCENT_TIMEOUT_MS at rest (no thrust, no vertical motion), force
      // LANDED — the parachute decision already happened, so no safety loss.
      if ((millis() - _stateEntryMs) >= DESCENT_TIMEOUT_MS &&
          fabsf(vz) < STUCK_REST_MAX_VZ && acc < STUCK_REST_MAX_ACC) {
        Serial.printf("[FSM] DESCENT timeout (%lus) — forcing LANDED\n",
                      (unsigned long)(DESCENT_TIMEOUT_MS / 1000UL));
        transitionTo(LANDED);
        break;
      }
      // Freefall is an informational sub-event within descent.
      if (!_freefallDetected && detectFreefall(vz, height, acc)) {
        _freefallDetected = true;
        Serial.printf("[FSM] FREEFALL h=%.1f vz=%.2f acc=%.2f\n", height, vz, acc);
      }
      // Option A: deploy immediately after apogee, but only once descent is
      // confirmed by a stable negative Vz (PARACHUTE_CONFIRM_CYCLES samples)
      // and above the ground guard. Never deploys on ascent or near ground.
      if (!_parachuteDeployed && detectParachute(height, vz)) {
        if (++_parachuteConfirmCount >= PARACHUTE_CONFIRM_CYCLES) {
          _parachuteDeployed = true;
          persistToNVS();  // safety-critical: survive a reboot right after deploy
          Serial.printf("[FSM] PARACHUTE DEPLOYED h=%.1f vz=%.2f\n", height, vz);
        }
      } else {
        _parachuteConfirmCount = 0;
      }
      if (detectLanded(vz, height)) {
        transitionTo(LANDED);
      }
      break;

    case LANDED:
      // Terminal state — persists until a reboot (restoreFromNVS() clears the
      // stale snapshot and starts fresh at IDLE) or an explicit reset().
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// ISensor::getData
// ─────────────────────────────────────────────────────────────────────────────

String FlightStateMachine::getData() {
  // Format mirrors the Serial output style of test/FSM/FSM.ino
  return String("FSM:") + getStateName()
       + ",liftoff="   + (_liftoffDetected  ? "1" : "0")
       + ",burnout="   + (_burnoutDetected  ? "1" : "0")
       + ",apogee="    + (_apogeeDetected   ? "1" : "0")
       + ",freefall="  + (_freefallDetected ? "1" : "0")
       + ",parachute=" + (_parachuteDeployed ? "1" : "0");
}

// ─────────────────────────────────────────────────────────────────────────────
// Accessors
// ─────────────────────────────────────────────────────────────────────────────

FlightState FlightStateMachine::getState() const         { return _state; }
const char* FlightStateMachine::getStateName() const     { return getFlightStateName(_state); }
bool FlightStateMachine::isLiftoffDetected() const       { return _liftoffDetected; }
bool FlightStateMachine::isBurnoutDetected() const       { return _burnoutDetected; }
bool FlightStateMachine::isApogeeDetected() const        { return _apogeeDetected; }
bool FlightStateMachine::isFreefallDetected() const      { return _freefallDetected; }
bool FlightStateMachine::isParachuteDeployed() const     { return _parachuteDeployed; }

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

void FlightStateMachine::transitionTo(FlightState next) {
  Serial.printf("[FSM] %s -> %s\n", getFlightStateName(_state), getFlightStateName(next));
  _state = next;
  _stateEntryMs = millis();
  persistToNVS();
}

// ── NVS persistence ─────────────────────────────────────────────────────────

void FlightStateMachine::persistToNVS() {
  NvsSnapshot snap;
  snap.magic         = NVS_MAGIC;
  snap.version       = NVS_VERSION;
  snap.state         = static_cast<int32_t>(_state);
  snap.flags         = (_liftoffDetected  ? FLAG_LIFTOFF   : 0)
                     | (_burnoutDetected  ? FLAG_BURNOUT   : 0)
                     | (_apogeeDetected   ? FLAG_APOGEE    : 0)
                     | (_freefallDetected ? FLAG_FREEFALL  : 0)
                     | (_parachuteDeployed ? FLAG_PARACHUTE : 0);
  snap.confirmCount  = _parachuteConfirmCount;
  snap.reserved      = 0;
  snap.basePressure  = _baro->getBasePressure();
  snap.maxAltitude   = _baro->getMaxAltitude();

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    Serial.println("[FSM] NVS open failed (write)");
    return;
  }
  const size_t written = prefs.putBytes(NVS_KEY, &snap, sizeof(snap));
  prefs.end();
  if (written != sizeof(snap)) {
    Serial.println("[FSM] WARN: NVS snapshot write size mismatch");
  }
}

void FlightStateMachine::restoreFromNVS() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, true)) {
    Serial.println("[FSM] NVS open failed (read) — fresh IDLE");
    return;
  }
  NvsSnapshot snap;
  const size_t got = prefs.getBytes(NVS_KEY, &snap, sizeof(snap));
  prefs.end();

  if (got != sizeof(snap) || snap.magic != NVS_MAGIC || snap.version != NVS_VERSION) {
    Serial.println("[FSM] No valid NVS state — fresh IDLE");
    return;
  }

  if (snap.state == static_cast<int32_t>(LANDED)) {
    // Previous flight already completed; clear the stale snapshot and start fresh.
    Preferences w;
    if (w.begin(NVS_NAMESPACE, false)) {
      w.remove(NVS_KEY);
      w.end();
    }
    Serial.println("[FSM] NVS state was LANDED — cleared, fresh IDLE");
    return;
  }

  if (snap.state < static_cast<int32_t>(IDLE) ||
      snap.state > static_cast<int32_t>(LANDED)) {
    Serial.println("[FSM] NVS state out of range — fresh IDLE");
    return;
  }

  _state = static_cast<FlightState>(snap.state);
  _stateEntryMs = millis();  // restart stuck-state backstop timers after reboot
  _liftoffDetected   = snap.flags & FLAG_LIFTOFF;
  _burnoutDetected   = snap.flags & FLAG_BURNOUT;
  _apogeeDetected    = snap.flags & FLAG_APOGEE;
  _freefallDetected  = snap.flags & FLAG_FREEFALL;
  _parachuteDeployed = snap.flags & FLAG_PARACHUTE;
  _parachuteConfirmCount = snap.confirmCount;
  _firstReading = true;  // re-seed the IIR filter from the first raw sample

  // Altitude continuity: restore the LAUNCH-site reference (captured at boot
  // mid-flight it would be the reboot-point pressure, making the FSM think
  // it is at ground level).
  if (snap.basePressure > 100.0f && snap.basePressure < 1200.0f) {
    _baro->setBasePressure(snap.basePressure);
    _baro->setMaxAltitude(snap.maxAltitude);
  }

  Serial.printf("[FSM] Restored from NVS: %s parachute=%d h=%.1f m\n",
                getStateName(), _parachuteDeployed ? 1 : 0,
                _baro->getAltitude());
}

// ── Detection functions ───────────────────────────────────────────────────────
// Each function is a 1-to-1 port of the corresponding function in
// test/FSM/FSM.ino.  Comments reference the original line numbers.

// test/FSM/FSM.ino lines 85-88
bool FlightStateMachine::detectLiftoff(float ax, float ay, float az) const {
  return totalAccel(ax, ay, az) > LIFTOFF_ACCEL_THRESHOLD;
}

// Per-time liftoff confirmation (risk #2 hardening, 2026-08-29):
// the total accel must stay above LIFTOFF_ACCEL_THRESHOLD for
// LIFTOFF_CONFIRM_MS (100 ms = 5 cycles @ 50Hz). A single-cycle spike
// (bench/hand vibration, servo jerk) no longer false-arms the FSM — which
// mattered because at 1-cycle liftoff, IDLE->ASCENT->DESCENT happened in
// 40 ms on the pad (|vz|~0 -> apogee-by-vz fires immediately).
// Short gaps below threshold (<= LIFTOFF_CONFIRM_MAX_GAP_MS, e.g. one stale
// IMU frame) do not reset the accumulator, so real burns with brief dips
// still confirm on schedule.
// Validated: extras/FSM_tester/validate_liftoff_confirm.py (6/6 scenarios:
// bench stays IDLE, real flight + both RocketPy sims confirm in 100-160 ms,
// single 30 m/s² spike rejected, drone-drop backstop unaffected).
bool FlightStateMachine::detectLiftoffTimed(float ax, float ay, float az) {
  const bool above = detectLiftoff(ax, ay, az);
  const uint32_t now = millis();
  if (above) {
    if (!_liftoffAccelAbove) {
      _liftoffAccelStartMs = now;  // start of a new above-threshold run
    }
    _liftoffAccelAbove = true;
    return (now - _liftoffAccelStartMs) >= LIFTOFF_CONFIRM_MS;
  }
  // Below threshold: tolerate a short gap (stale frame), otherwise reset.
  if (_liftoffAccelAbove &&
      (now - _liftoffAccelStartMs) < LIFTOFF_CONFIRM_MAX_GAP_MS) {
    return false;  // keep the run alive; do not reset the start time
  }
  _liftoffAccelAbove = false;
  return false;
}

// test/FSM/FSM.ino lines 90-98
bool FlightStateMachine::detectBurnout(float ax, float ay, float az,
                                        float height, float vz) const {
  if (height < BURNOUT_MIN_HEIGHT || vz <= BURNOUT_MIN_VZ) return false;
  const float acc = totalAccel(ax, ay, az);
  return (az < BURNOUT_AZ_THRESHOLD || acc < BURNOUT_ACC_THRESHOLD);
}

// test/FSM/FSM.ino lines 100-102 (az gate removed 2026-08-05, risk #5)
bool FlightStateMachine::detectApogee(float vz) const {
  return (fabsf(vz) < APOGEE_MAX_VZ);
}

// test/FSM/FSM.ino lines 104-109  (az param unused in the original too)
bool FlightStateMachine::detectFreefall(float vz, float height, float totalAcc) const {
  if (height < FREEFALL_MIN_HEIGHT || vz >= FREEFALL_MAX_VZ) return false;
  return totalAcc < FREEFALL_ACC_THRESHOLD;
}

// test/FSM/FSM.ino lines 111-113
// Option A: deploy trigger = after apogee (state is DESCENT) AND a stable
// negative Vz confirming descent AND still above the ground guard. There is
// intentionally NO upper ceiling: the chute opens at apogee, not at 100 m.
bool FlightStateMachine::detectParachute(float height, float vz) const {
  return (height > PARACHUTE_MIN_ALTITUDE && vz < PARACHUTE_CONFIRM_VZ);
}

// test/FSM/FSM.ino lines 115-117
bool FlightStateMachine::detectLanded(float vz, float height) const {
  return (fabsf(vz) < LANDED_MAX_VZ && height < LANDED_MAX_HEIGHT);
}

// ── Filter / math helpers ────────────────────────────────────────────────────

// test/FSM/FSM.ino lines 80-83  (exponential moving average, alpha=0.2)
float FlightStateMachine::smoothFilter(float value, float prev) {
  return FILTER_ALPHA * value + (1.0f - FILTER_ALPHA) * prev;
}

// test/FSM/FSM.ino lines 76-78
float FlightStateMachine::totalAccel(float ax, float ay, float az) {
  return sqrtf(ax * ax + ay * ay + az * az);
}
