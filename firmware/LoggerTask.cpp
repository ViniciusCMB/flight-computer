/**
 * @file LoggerTask.cpp
 * @brief Implementation of the low-priority async logging task
 *
 * @see LoggerTask.h for API and configuration documentation
 * @see firmware/REFACTORING_PLAN.md - Fase 7
 */

#include "flight/LoggerTask.h"

#include <cstring>

#include "modules/lora_module.h"

QueueHandle_t logQueue            = nullptr;
TaskHandle_t  g_loggerTaskHandle  = nullptr;

namespace {

uint8_t g_minLogLevel = LOG_LEVEL_INFO;

LoggerStats g_stats = {0, 0, 0};

}  // namespace

bool initLoggerTask() {
  logQueue = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogMessage));
  if (logQueue == nullptr) {
    Serial.println("[Logger] FATAL: failed to create logQueue");
    return false;
  }

  const BaseType_t created = xTaskCreatePinnedToCore(
      taskLogger, "Logger", LOGGER_STACK_SIZE, nullptr, LOGGER_PRIORITY,
      &g_loggerTaskHandle, LOGGER_CORE);

  if (created != pdPASS) {
    Serial.println("[Logger] FATAL: failed to create task");
    return false;
  }

  return true;
}

void taskLogger(void* pvParameters) {
  (void)pvParameters;

  LogMessage log;

  for (;;) {
    // Blocks indefinitely: safe here because this is the lowest-priority
    // task in the system and has nothing else to do while idle — it never
    // holds up FlightControlTask or TelemetryTask.
    if (xQueueReceive(logQueue, &log, portMAX_DELAY) != pdPASS) {
      continue;
    }

    g_stats.receivedCount++;

    if (log.level < g_minLogLevel) {
      continue;
    }

    Serial.printf("[%lu]%s[T%u] %s\n", log.timestamp,
                  getLogLevelName(log.level), log.taskId, log.message);
    g_stats.printedCount++;

    // Forward errors over LoRa (same radio/frequency as telemetry) so the
    // ground station sees them even without a serial link. Non-fatal if the
    // radio is unavailable: sendLoRa() no-ops when LoRa was not initialized.
    if (log.level == LOG_LEVEL_ERROR && isLoRaAvailable()) {
      sendLoRa(String("ERR|") + log.timestamp + "|" + log.message);
    }
  }
}

bool logMessage(uint8_t taskId, uint8_t level, const String& message) {
  if (logQueue == nullptr) {
    return false;
  }

  LogMessage log;
  log.timestamp = millis();
  log.taskId    = taskId;
  log.level     = level;
  strncpy(log.message, message.c_str(), sizeof(log.message) - 1);
  log.message[sizeof(log.message)-1] = '\0';
  if (xQueueSend(logQueue, &log, 0) != pdPASS) {
    g_stats.droppedCount++;
    return false;
  }

  return true;
}

void setLoggerMinLevel(uint8_t level) { g_minLogLevel = level; }

uint8_t getLoggerMinLevel() { return g_minLogLevel; }

const LoggerStats& getLoggerStats() { return g_stats; }
