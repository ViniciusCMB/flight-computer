/**
 * @file TelemetryTask.cpp
 * @brief Implementation of the 5Hz telemetry task
 *
 * CSV field order does NOT yet match the v2.0 draft table in
 * firmware/REFACTORING_PLAN.md (Fase 10, still pending): pressure is
 * emitted in hPa here vs. Pa in the draft, and the last column ("pqd" in
 * the CSV header) actually carries `parachute_deployed` (0/1) — it is not
 * `packetQuality`. Real `rssi`/`packetQuality` are not emitted at all yet;
 * that alignment is Fase 10's job.
 *
 * @see TelemetryTask.h for API and configuration documentation
 * @see firmware/REFACTORING_PLAN.md - Fase 7
 */

#include "flight/TelemetryTask.h"

#include "config.h"
#include "modules/filesystem_module.h"
#include "modules/lora_module.h"
#include "modules/buzzer_module.h"
#include "sensors/GPSModule.h"
#include "flight/FlightControlTask.h"
#include "flight/LoggerTask.h"

#include <SPI.h>

TaskHandle_t g_telemetryTaskHandle = nullptr;

namespace {

GPSModule* g_gps = nullptr;

TelemetryStats g_stats = {0, 0, 0};

/**
 * @brief Deriva o caminho do arquivo CSV a partir da hora do GPS
 * @note Sem fix disponivel no momento da chamada, usa "NOFIX" (nao bloqueia
 *       esperando o GPS — ver REFACTORING_PLAN.md, tabela de riscos)
 */
String buildDataFilePath() {
  String stamp = g_gps->getTimeString();  // "HH:MM:SS" ou "nan"
  if (stamp == "nan") {
    stamp = "NOFIX";
  } else {
    stamp.replace(":", "");
  }
  return "/" + stamp + "-" + file_name;
}

/**
 * @brief Monta a linha CSV de telemetria (Serial file + LoRa)
 * @note Formato v2.0 (Fase 10) — alinhado ao parser do receiver
 *       (recovery-webui/components/receiver-lora). Ordem canônica de 22
 *       campos:
 *       TEAM_ID,millis,count,altp,temp,umi,p,gx,gy,gz,ax,ay,az,vz,
 *       maxAltitude,state,alt,lat,lon,sat,parachute,rssi
 *       - "umi" (umidade) e' 0 fixo: ainda nao ha sensor de umidade.
 *       - "rssi" e' placeholder 0 aqui; o receiver SOBRESCREVE com o
 *         RSSI real medido no link descendente (LoRa.packetRssi()).
 * @note Monta em um buffer fixo via snprintf (uma unica conversao para
 *       String no retorno) em vez de ~20 concatenacoes com `+`, que a 5Hz
 *       fragmentavam o heap em voos longos (>20min) ate causar OOM.
 */
String assembleTelemetry(const SensorData& data) {
  char gpsAltBuf[16];
  char latBuf[16];
  char lonBuf[16];

  if (data.gps_valid) {
    snprintf(gpsAltBuf, sizeof(gpsAltBuf), "%.2f", data.gpsAltitude);
    snprintf(latBuf, sizeof(latBuf), "%.6f", data.latitude);
    snprintf(lonBuf, sizeof(lonBuf), "%.6f", data.longitude);
  } else {
    strncpy(gpsAltBuf, "nan", sizeof(gpsAltBuf));
    strncpy(latBuf, "nan", sizeof(latBuf));
    strncpy(lonBuf, "nan", sizeof(lonBuf));
  }

  char buf[256];
  snprintf(buf, sizeof(buf),
           "%s,%lu,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
           "%.2f,%.2f,%d,%s,%s,%s,%u,%d,%d",
           TEAM_ID, data.timestamp, data.packet_count, data.altitude,
           data.temperature,
           0.0f,  // umi: sem sensor de umidade ainda (placeholder 0)
           data.pressure, data.gyroX, data.gyroY, data.gyroZ, data.accelX,
           data.accelY, data.accelZ, data.verticalVelocity, data.maxAltitude,
           static_cast<int>(data.state), gpsAltBuf, latBuf, lonBuf,
           data.satellites, data.parachute_deployed ? 1 : 0,
           0);  // rssi: placeholder; receiver substitui por rxRssi real

  return String(buf);
}

/**
 * @brief Monta uma linha legivel para o Serial Monitor
 */
String formatForSerial(const SensorData& data) {
  char buf[192];
  snprintf(buf, sizeof(buf),
           "[T+%lums #%u] %s | alt=%.1fm vz=%.2fm/s maxAlt=%.1fm | "
           "p=%.1fhPa t=%.1fC acc=%.2fm/s2 | GPS: %s (%u sats) | chute=%s",
           data.timestamp, data.packet_count, getFlightStateName(data.state),
           data.altitude, data.verticalVelocity, data.maxAltitude,
           data.pressure, data.temperature,
           data.totalAccel,
           data.gps_valid ? "fix" : "no fix", data.satellites,
           data.parachute_deployed ? "YES" : "no");
  return String(buf);
}

}  // namespace

bool initTelemetryTask() {
  if (sensorDataQueue == nullptr) {
    Serial.println(
        "[Telemetry] FATAL: sensorDataQueue not created — call "
        "initFlightControlTask() first");
    return false;
  }

  // Remap the shared SPI bus (LoRa SCK=12/MISO=13/MOSI=11, CS=10) BEFORE
  // setupStorage(): SD.begin uses the global SPI object, and without this
  // remap the SD card is probed on the ESP32-S3 default SPI pins and always
  // falls back to LittleFS. setupLoRa() re-issues SPI.begin() (idempotent).
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, SS_LORA);

  g_gps = new GPSModule(&Serial1);
  if (!g_gps->begin()) {
    Serial.println("[Telemetry] WARNING: GPS init failed — continuing without fix");
    logMessage(TASK_ID_TELEMETRY, LOG_LEVEL_WARN, "GPS init failed");
  }
  g_gps->update();  // Non-blocking best-effort read before building the filename

  if (!setupStorage()) {
    Serial.println("[Telemetry] WARNING: No storage available — continuing without file logging");
  }

  file_dir = buildDataFilePath();
  Serial.print("[Telemetry] Saving data to: ");
  Serial.println(file_dir);

  const String header =
      "TEAM_ID,millis,count,altp,temp,umi,p,gx,gy,gz,ax,ay,az,vz,"
      "maxAltitude,state,alt,lat,lon,sat,parachute,rssi";
  if (isStorageReady() && !writeFile(file_dir, header)) {
    Serial.println("[Telemetry] FATAL: failed to write CSV header");
    return false;
  }

  Serial.print("[Telemetry] Storage: ");
  Serial.println(getStorageName());

  if (!setupLoRa()) {
    Serial.println("[Telemetry] WARNING: LoRa init failed — continuing without radio");
  }

  const BaseType_t created = xTaskCreatePinnedToCore(
      taskTelemetry, "Telemetry", TELEMETRY_STACK_SIZE, nullptr,
      TELEMETRY_PRIORITY, &g_telemetryTaskHandle, TELEMETRY_CORE);

  if (created != pdPASS) {
    Serial.println("[Telemetry] FATAL: failed to create task");
    return false;
  }

  return true;
}

void taskTelemetry(void* pvParameters) {
  (void)pvParameters;

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);

  SensorData data;

  for (;;) {
    vTaskDelayUntil(&lastWakeTime, period);

    // 1) GPS (non-blocking NMEA feed)
    g_gps->update();

    // 2) Queue receive — bounded wait, never blocks indefinitely. FlightControl
    //    produces samples ~10x faster than we consume them (50Hz vs 5Hz), so
    //    drain any backlog and keep only the freshest sample — otherwise the
    //    queue fills up and FlightControl silently drops every subsequent
    //    sample once it's full.
    if (xQueueReceive(sensorDataQueue, &data, pdMS_TO_TICKS(TELEMETRY_QUEUE_TIMEOUT_MS)) == pdPASS) {
      SensorData newer;
      while (xQueueReceive(sensorDataQueue, &newer, 0) == pdPASS) {
        data = newer;
      }

      // Enrich the sample with the current GPS fix (owned by this task only)
      data.gps_valid = g_gps->hasValidFix();
      if (data.gps_valid) {
        data.latitude    = g_gps->getLatitude();
        data.longitude   = g_gps->getLongitude();
        data.gpsAltitude = g_gps->getGPSAltitude();
      }
      data.satellites = g_gps->getSatellites();

      const String telemetry = assembleTelemetry(data);

      // 3) Multi-channel fan-out
      Serial.println(formatForSerial(data));
      sendLoRa(telemetry);

      // Flash writes disable the flash cache on BOTH cores; while it lasts,
      // the I2C transaction inside taskFlightControl (Core 1) stalls and the
      // esp_driver_i2c spinlock can time out -> panic (bench crash 2026-08-27).
      // Writing at full 5 Hz made this near-certain within ~13 s. 1 Hz keeps
      // the log useful for post-flight reconstruction while drastically
      // reducing cache-off windows. LoRa TX still runs at 5 Hz.
      static uint8_t flashWriteDivider = 0;
      if (++flashWriteDivider >= 5) {  // 5 Hz task -> 1 Hz flash write
        flashWriteDivider = 0;
        appendFile(file_dir, telemetry);
      }

      g_stats.packetsSent++;
    } else {
      g_stats.queueTimeoutCount++;
    }

    g_stats.cycleCount++;

    // Recovery beacon: 1 Hz short beep; LONG beep when LANDED (post-flight
    // findability). Non-blocking tone(); millis()-paced so the 5 Hz loop is
    // never delayed.
    static uint32_t lastBeepMs = 0;
    if (data.state == LANDED) {
      // Always pace from the last beep in LANDED, even if the last one was short
      if (millis() - lastBeepMs >= BUZZER_BEACON_PERIOD_MS) {
        buzzRecoveryBeep(true);
        lastBeepMs = millis();
      }
    } else if (millis() - lastBeepMs >= BUZZER_BEACON_PERIOD_MS) {
      buzzRecoveryBeep(false);
      lastBeepMs = millis();
    }
  }
}

const TelemetryStats& getTelemetryStats() {
  return g_stats;
}
