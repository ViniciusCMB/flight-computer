/**
 * @file FlightControlTask.cpp
 * @brief Implementation of the 5Hz flight control task
 *
 * @see FlightControlTask.h for API and configuration documentation
 * @see firmware/REFACTORING_PLAN.md - Fase 7
 */

#include "flight/FlightControlTask.h"

#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <ESP32Servo.h>
#include <string.h>
#include <Wire.h>

#include "config.h"
#include "sensors/BMP585Sensor.h"
#include "sensors/LSM6DS3Sensor.h"
#include "modules/parachute_module.h"
#include "flight/FlightStateMachine.h"
#include "flight/LoggerTask.h"

TaskHandle_t  g_flightControlTaskHandle = nullptr;
QueueHandle_t sensorDataQueue           = nullptr;

namespace {

// ── Sensor init retry (single shared loop for IMU + baro) ────────────────────
// Tries begin() on one sensor for up to SENSOR_INIT_RETRY_WINDOW_MS, beeping
// the buzzer on every attempt (bench-validated: the LSM6DS3/BMP585 sometimes
// only come up on a retry). Returns true on the first success.
static bool beginSensorWithRetry(ISensor* sensor, const char* name) {
  const uint32_t start = millis();
  uint32_t attempt = 0;
  for (;;) {
    attempt++;
    if (sensor->begin()) {
      if (attempt > 1) {
        Serial.printf("[FlightControl] %s init OK on attempt %lu\n",
                      name, (unsigned long)attempt);
        logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_WARN,
                   "Sensor init recovered on retry");
      }
      return true;
    }
    if (millis() - start >= SENSOR_INIT_RETRY_WINDOW_MS) {
      Serial.printf("[FlightControl] %s init failed after %lu attempts "
                    "(retry window exhausted)\n",
                    name, (unsigned long)attempt);
      return false;
    }
    Serial.printf("[FlightControl] %s init failed, retrying in %lu ms...\n",
                  name, (unsigned long)SENSOR_INIT_RETRY_PERIOD_MS);
    // Audible tick per attempt (passive piezo -> square wave via tone()).
    tone(BUZZER_PIN, BUZZER_TONE_HZ, 100);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_INIT_RETRY_PERIOD_MS));
  }
}

BMP585Sensor*       g_baro = nullptr;
LSM6DS3Sensor*       g_imu = nullptr;
FlightStateMachine*  g_fsm = nullptr;

bool  g_parachuteActuated = false;

// ── Free-fall backstop state (FSM-independent safety net) ───────────────────
// Own IIR-filtered acceleration so the detector never depends on the FSM
// internals: even if the state machine is stuck (e.g. restored to ASCENT
static uint16_t g_ffSustainedCycles = 0;
static bool     g_ffFilterSeeded    = false;
static float    g_ffFiltAx = 0.0f, g_ffFiltAy = 0.0f, g_ffFiltAz = 0.0f;

// ── Barometer-staleness contingency state ───────────────────────────────────
// Own IIR filter + liftoff latch: completely independent of FSM and backstop
// state. Covers the common-mode failure where the barometer freezes mid-flight
// (frozen last-good values are plausible, never NaN) — without it, neither the
// FSM nor the free-fall backstop would deploy.
static bool     g_baroStaleFilterSeeded = false;
static float    g_bsFiltAx = 0.0f, g_bsFiltAy = 0.0f, g_bsFiltAz = 0.0f;
static bool     g_bsLiftoffLatched = false;
static uint16_t g_bsSustainedCycles = 0;

// ── Pad arming state (risk #2) ──────────────────────────────────────────────
static char     armBuffer[16] = {0};
static int      armBufferLen  = 0;

/**
 * @brief FSM-independent free-fall detector (safety backstop)
 *
 * Fires once when total acceleration (IIR-filtered, alpha=0.2) stays below
 * FREEFALL_BACKSTOP_ACC_THRESHOLD for FREEFALL_BACKSTOP_CYCLES consecutive
 * cycles while the rocket is descending faster than FREEFALL_BACKSTOP_VZ and
 * still above FREEFALL_BACKSTOP_MIN_HEIGHT. Returns true exactly once, then
 * stays latched until the next reset() (via g_ffSustainedCycles saturation).
 *
 * Covers the failure mode the NVS persistence cannot: FSM alive but stuck in
 * the wrong state (IDLE after a reboot with no valid snapshot, or ASCENT
 * while actually falling) — the chute still opens at apogee.
 *
 * @note Validated offline: extras/FSM_tester/validate_freefall_backstop.py
 *       (never fires before apogee, fires 1-3s after the FSM deploy, never
 *       on pad vibration).
 * @return true when the backstop has just fired (deploy now)
 */
bool checkFreefallBackstop() {
  // Latch: once fired, keep returning true until a reboot/reset.
  if (g_ffSustainedCycles > FREEFALL_BACKSTOP_CYCLES) {
    return true;
  }

  float ax, ay, az;
  g_imu->getAcceleration(&ax, &ay, &az);

  if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
    return false;
  }

  // IIR low-pass, same alpha as the FSM (seed on first reading).
  if (!g_ffFilterSeeded) {
    g_ffFiltAx = ax;
    g_ffFiltAy = ay;
    g_ffFiltAz = az;
    g_ffFilterSeeded = true;
  } else {
    g_ffFiltAx += FILTER_ALPHA * (ax - g_ffFiltAx);
    g_ffFiltAy += FILTER_ALPHA * (ay - g_ffFiltAy);
    g_ffFiltAz += FILTER_ALPHA * (az - g_ffFiltAz);
  }

  const float acc = sqrtf(g_ffFiltAx * g_ffFiltAx +
                          g_ffFiltAy * g_ffFiltAy +
                          g_ffFiltAz * g_ffFiltAz);
  if (!std::isfinite(acc)) {
    return false;
  }

  const float height = g_baro->getAltitude();
  const float vz     = g_baro->getVerticalVelocity();

  const bool freefalling = (acc < FREEFALL_BACKSTOP_ACC_THRESHOLD &&
                            vz  < FREEFALL_BACKSTOP_VZ &&
                            height > FREEFALL_BACKSTOP_MIN_HEIGHT);
  if (freefalling) {
    g_ffSustainedCycles++;
    if (g_ffSustainedCycles == FREEFALL_BACKSTOP_CYCLES) {
      Serial.printf("[FlightControl] BACKSTOP: free-fall acc=%.2f vz=%.2f h=%.1f\n",
                    acc, vz, height);
      return true;
    }
  } else {
    g_ffSustainedCycles = 0;
  }
  return false;
}

/**
 * @brief Barometer-staleness contingency (IMU-only, FSM-independent)
 *
 * Fires when the barometer has been frozen for BARO_STALE_AGE_MS while the
 * rocket is in a sustained IMU-only free fall. The flight must have actually
 * started (accel > LIFTOFF_ACCEL_THRESHOLD seen at least once since boot —
 * never opens on the pad) and the last-good maxAltitude must be above the
 * ground guard (replaces the height check while the barometer is dead).
 *
 * @return true once armed and a BARO_STALE_SUSTAIN_CYCLES window of
 *         near-zero-g is confirmed; latches until reboot (one-shot via
 *         g_parachuteActuated in the caller)
 */
bool checkBaroStaleContingency() {
  if (g_bsSustainedCycles > BARO_STALE_SUSTAIN_CYCLES) {
    return true;  // already fired, latched
  }

  // Barometer not frozen? Nothing to do (normal flight path).
  if (g_baro->getLastReadingAgeMs() < BARO_STALE_AGE_MS) {
    g_bsSustainedCycles = 0;
    return false;
  }

  // Safety guards: the flight started and climbed above the ground guard
  // (maxAltitude from the last good baro reading).
  if (!g_bsLiftoffLatched ||
      g_baro->getMaxAltitude() <= BARO_STALE_MIN_HEIGHT) {
    g_bsSustainedCycles = 0;
    return false;
  }

  float ax, ay, az;
  g_imu->getAcceleration(&ax, &ay, &az);
  if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
    return false;
  }

  // Own IIR low-pass, same alpha as everywhere else (seed on first reading).
  if (!g_baroStaleFilterSeeded) {
    g_bsFiltAx = ax;
    g_bsFiltAy = ay;
    g_bsFiltAz = az;
    g_baroStaleFilterSeeded = true;
  } else {
    g_bsFiltAx += FILTER_ALPHA * (ax - g_bsFiltAx);
    g_bsFiltAy += FILTER_ALPHA * (ay - g_bsFiltAy);
    g_bsFiltAz += FILTER_ALPHA * (az - g_bsFiltAz);
  }

  const float acc = sqrtf(g_bsFiltAx * g_bsFiltAx +
                          g_bsFiltAy * g_bsFiltAy +
                          g_bsFiltAz * g_bsFiltAz);
  if (!std::isfinite(acc)) {
    return false;
  }

  if (acc > LIFTOFF_ACCEL_THRESHOLD) {
    g_bsLiftoffLatched = true;
  }

  if (acc < BARO_STALE_ACC_THRESHOLD) {
    g_bsSustainedCycles++;
    if (g_bsSustainedCycles >= BARO_STALE_SUSTAIN_CYCLES) {
      Serial.printf("[FlightControl] CONTINGENCY: baro stale %lu ms, "
                    "IMU-only free-fall acc=%.2f\n",
                    (unsigned long)(g_baro->getLastReadingAgeMs()), acc);
      return true;
    }
  } else {
    g_bsSustainedCycles = 0;
  }
  return false;
}

// ── Pad arming (risk #2) ────────────────────────────────────────────────────
// Bench vibration can false-liftoff the FSM into ASCENT; a reboot then
// restores ASCENT from NVS and the FSM lands on the pad without ever
// deploying (LANDED on the ramp, parachute=False). The ARM command clears
// the NVS snapshot, re-captures base_pressure and zeroes maxAltitude.
// Refused once the flight really started (maxAltitude above the pad guard
// or parachute already open).
static void handleArmCommand() {
  // Non-blocking read of one complete line (e.g. "ARM\n" from the pad).
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (armBufferLen > 0) {
        armBuffer[armBufferLen] = '\0';
        if (strcmp(armBuffer, "ARM") == 0) {
          const bool flightStarted =
              g_baro->getMaxAltitude() >= ARM_MAX_ARM_ALTITUDE;
          if (flightStarted || g_parachuteActuated ||
              g_fsm->isParachuteDeployed()) {
            Serial.println("[FlightControl] ARM refused: flight already "
                           "started or parachute open");
            logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_WARN,
                       "ARM refused (flight started)");
          } else {
            g_fsm->reset();                       // IDLE + NVS cleared
            g_baro->setBasePressure(g_baro->getPressure());
            g_baro->setMaxAltitude(0.0f);
            Serial.println("[FlightControl] ARM OK: NVS cleared, "
                           "base_pressure re-captured");
            logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_WARN,
                       "Pad armed (NVS cleared, base re-captured)");
          }
        }
        armBufferLen = 0;
      }
    } else if (armBufferLen < (int)sizeof(armBuffer) - 1) {
      armBuffer[armBufferLen++] = c;
    }
  }
}

// ── Baro glitch recovery (IDLE only) ─────────────────────────────────────────
// On the bench the BMP585 was observed to lose calibration (suspected chip
// reset from a supply glitch) and read a self-consistent but wrong pressure:
// alt jumped to ~2400 m while vz stayed ~0. On the pad (IDLE + at rest) that
// is unambiguously a sensor fault, never a real flight — a real liftoff has
// acc > LIFTOFF_ACCEL_THRESHOLD. Sustained impossible altitude at rest =>
// full sensor reinit + re-zero of the pad reference.
static void checkBaroGlitch() {
  static uint16_t glitchCycles = 0;
  const bool atRest = g_baro->getVerticalVelocity() > -STUCK_REST_MAX_VZ &&
                      g_baro->getVerticalVelocity() < STUCK_REST_MAX_VZ &&
                      g_imu->getTotalAccel() < STUCK_REST_MAX_ACC;
  if (g_fsm->getState() != IDLE || g_parachuteActuated || !atRest) {
    glitchCycles = 0;
    return;
  }
  if (fabsf(g_baro->getAltitude()) > BARO_GLITCH_ALTITUDE) {
    if (++glitchCycles >= BARO_GLITCH_SUSTAIN_CYCLES) {
      Serial.println("[FlightControl] Baro glitch on pad (alt at rest "
                     "impossible) — reinitializing sensor");
      logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_WARN,
                 "Baro glitch recovery (reinit)");
      if (g_baro->reinit()) {
        g_baro->setMaxAltitude(0.0f);
      }
      glitchCycles = 0;
    }
  } else {
    glitchCycles = 0;
  }
}

// Complement to ARM: while the FSM sits on the pad in IDLE, a baro pressure
// drift pulls the relative altitude negative. After ARM_REZERO_SUSTAIN_CYCLES
// of sustained drift below ARM_REZERO_THRESHOLD, re-capture base_pressure and
// zero maxAltitude automatically so a low flight still crosses the ground
// guard.
static void checkAutoRezero() {
  static uint16_t rezeroCycles = 0;
  if (g_fsm->getState() != IDLE || g_parachuteActuated) {
    rezeroCycles = 0;
    return;
  }
  if (g_baro->getAltitude() < ARM_REZERO_THRESHOLD) {
    rezeroCycles++;
    if (rezeroCycles >= ARM_REZERO_SUSTAIN_CYCLES) {
      g_baro->setBasePressure(g_baro->getPressure());
      g_baro->setMaxAltitude(0.0f);
      Serial.println("[FlightControl] Auto re-zero: base_pressure "
                     "re-captured (pad drift)");
      logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_WARN,
                 "Auto re-zero on pad (baro drift)");
      rezeroCycles = 0;
    }
  } else {
    rezeroCycles = 0;
  }
}

FlightControlStats g_stats = {0, 0, 0, 0, 0};

/**
 * @brief Consolida as leituras atuais dos sensores e o estado da FSM
 * @note Campos de GPS ficam nos valores default (0/false) — GPS e'
 *       responsabilidade da TelemetryTask (5Hz), fora do escopo desta task
 */
SensorData buildSensorData() {
  SensorData data;

  data.timestamp     = millis();
  data.packet_count  = static_cast<uint16_t>(g_stats.cycleCount);

  data.altitude         = g_baro->getAltitude();
  data.pressure         = g_baro->getPressure();
  data.temperature      = g_baro->getTemperature();
  data.verticalVelocity = g_baro->getVerticalVelocity();
  data.maxAltitude      = g_baro->getMaxAltitude();

  g_imu->getAcceleration(&data.accelX, &data.accelY, &data.accelZ);
  g_imu->getGyroscope(&data.gyroX, &data.gyroY, &data.gyroZ);
  data.totalAccel = g_imu->getTotalAccel();

  data.state              = g_fsm->getState();
  data.parachute_deployed = g_fsm->isParachuteDeployed();

  return data;
}

/**
 * @brief Aciona o servo de liberacao do paraquedas (one-shot, idempotente)
 * @note Chamada apenas quando a FSM confirma as condicoes de deploy
 *       (FlightStateMachine::detectParachute — apogeu + Vz negativo estavel,
 *       Opcao A). O servo (ParachuteServo) e' dono deste modulo.
 */
void deployParachute() {
  ParachuteServo.write(SERVO_OPEN);
}

}  // namespace

bool initFlightControlTask() {
  // I2C bus init is owned here (sensor bus user). See firmware.ino setup().
  Wire.begin(I2C_SDA, I2C_SCL);

  g_baro = new BMP585Sensor();
  g_imu  = new LSM6DS3Sensor();

  // IMU first: bench bring-up showed the LSM6DS3 must init before the
  // BMP280 fallback driver configures the bus (order validated 2026-08-27).
  // Distinguish which sensor failed so pad-side troubleshooting (buzzer
  // alarm is the only visible symptom) doesn't require re-flashed firmware.
  // Each gets a 10 s retry window (bench: sometimes begin() only succeeds
  // on a retry); the buzzer ticks once per failed attempt.
  if (!beginSensorWithRetry(g_imu, "LSM6DS3 (IMU)")) {
    Serial.println("[FlightControl] FATAL: LSM6DS3 (IMU) init failed "
                   "(wiring/address? see config.h I2C_ADDR_LSM6DS3)");
    return false;
  }
  if (!beginSensorWithRetry(g_baro, "BMP585/BMP280 (baro)")) {
    Serial.println("[FlightControl] FATAL: BMP585/BMP280 (baro) init failed "
                   "(wiring/address? see config.h I2C_ADDR_BMP585)");
    return false;
  }

  g_fsm = new FlightStateMachine(g_baro, g_imu);
  if (!g_fsm->begin()) {
    Serial.println("[FlightControl] FATAL: FSM initialization failed");
    return false;
  }
  // g_fsm->begin() restored the NVS snapshot after a watchdog reboot:
  // mark the actuator as already fired so the task does not re-deploy,
  // and keep the servo open if the chute was already released mid-flight.
  g_parachuteActuated = g_fsm->isParachuteDeployed();

  sensorDataQueue = xQueueCreate(SENSOR_DATA_QUEUE_LEN, sizeof(SensorData));
  if (sensorDataQueue == nullptr) {
    Serial.println("[FlightControl] FATAL: failed to create sensorDataQueue");
    return false;
  }

  if (!setupServo(g_fsm->isParachuteDeployed())) {
    Serial.println("[FlightControl] FATAL: failed to close parachute");
    return false;
  }

  

  const BaseType_t created = xTaskCreatePinnedToCore(
      taskFlightControl, "FlightControl", FLIGHT_CONTROL_STACK_SIZE,
      nullptr, FLIGHT_CONTROL_PRIORITY, &g_flightControlTaskHandle,
      FLIGHT_CONTROL_CORE);

  if (created != pdPASS) {
    Serial.println("[FlightControl] FATAL: failed to create task");
    return false;
  }

  return true;
}

void taskFlightControl(void* pvParameters) {
  (void)pvParameters;

  // Inicializado aqui (nao em initFlightControlTask()) para que o watchdog
  // so seja armado quando a task que o alimenta esta de fato rodando — se
  // xTaskCreatePinnedToCore falhasse com o init la, o TWDT ficaria armado
  // globalmente sem nenhuma task para chamar esp_task_wdt_reset().
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = FLIGHT_CONTROL_WDT_TIMEOUT_S * 1000,
      .idle_core_mask = 0,
      .trigger_panic = true
  };
  // ESP_ERR_INVALID_STATE is OK here: the IDF already arms the TWDT at boot
  // (CONFIG_ESP_TASK_WDT_INIT). Only treat unexpected failures as errors.
  esp_err_t wdt_err = esp_task_wdt_init(&twdt_config);
  if (wdt_err == ESP_ERR_INVALID_STATE) {
    // Already armed by the IDF at boot (CONFIG_ESP_TASK_WDT_INIT) with its
    // own default timeout — apply ours instead of silently keeping theirs.
    wdt_err = esp_task_wdt_reconfigure(&twdt_config);
  }
  if (wdt_err != ESP_OK) {
    Serial.println("[FlightControl] ERROR: watchdog init failed");
    logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_ERROR, "Watchdog init failed");
  }

  if (esp_task_wdt_add(nullptr) != ESP_OK) {
    Serial.println("[FlightControl] ERROR: failed to register with watchdog");
    logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_ERROR,
               "Failed to register with watchdog");
  }

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(FLIGHT_CONTROL_PERIOD_MS);

  for (;;) {
    vTaskDelayUntil(&lastWakeTime, period);

    const int64_t t0 = esp_timer_get_time();

    // 1) Sensores (ordem sequencial, ambos non-blocking)
    g_baro->update();
    g_imu->update();

    // 1b) Pad arming (risk #2): ARM via Serial + auto re-zero do barometro
    handleArmCommand();
    checkAutoRezero();
    checkBaroGlitch();

    // 2) FSM
    g_fsm->update();

    // 3) Consolidar dados
    const SensorData data = buildSensorData();

    // 4) Deploy do paraquedas (safety-critical, one-shot)
    if (data.parachute_deployed && !g_parachuteActuated) {
      deployParachute();
      g_parachuteActuated = true;
      Serial.println("[FlightControl] PARACHUTE DEPLOYED");
      logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_WARN, "Parachute deployed");
    }

    // 4b) Free-fall backstop (FSM-independent safety net). Fires only when the
    //     FSM has not deployed yet; the parachute opens at apogee even if the
    //     state machine is stuck. Idempotent via g_parachuteActuated.
    if (!g_parachuteActuated && checkFreefallBackstop()) {
      deployParachute();
      g_parachuteActuated = true;
      Serial.println("[FlightControl] BACKSTOP: parachute deployed (free-fall)");
      logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_WARN,
                 "Backstop free-fall deploy");
    }

    // 4c) Barometer-staleness contingency. Covers the common-mode failure
    //     where the barometer freezes mid-flight: both the FSM and the
    //     backstop depend on its vz/height, so neither would deploy.
    if (!g_parachuteActuated && checkBaroStaleContingency()) {
      deployParachute();
      g_parachuteActuated = true;
      Serial.println("[FlightControl] CONTINGENCY: parachute deployed "
                     "(baro stale, IMU-only free-fall)");
      logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_WARN,
                 "Baro-stale contingency deploy");
    }

    // 5) Queue -> TelemetryTask
    if (xQueueSend(sensorDataQueue, &data, 0) != pdPASS) {
      g_stats.queueDropCount++;
    }

    // 6) Watchdog
    esp_task_wdt_reset();

    // 7) Metricas de tempo de execucao
    const int64_t execTimeUs = esp_timer_get_time() - t0;
    g_stats.cycleCount++;
    g_stats.lastExecTimeUs = static_cast<int32_t>(execTimeUs);
    if (execTimeUs > g_stats.maxExecTimeUs) {
      g_stats.maxExecTimeUs = static_cast<int32_t>(execTimeUs);
    }
    if (execTimeUs > static_cast<int64_t>(FLIGHT_CONTROL_PERIOD_MS) * 1000) {
      g_stats.overrunCount++;
      logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_WARN, "Cycle overrun");
    }
  }
}

const FlightControlStats& getFlightControlStats() {
  return g_stats;
}
