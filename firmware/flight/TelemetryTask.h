/**
 * @file TelemetryTask.h
 * @brief FreeRTOS task that transmits telemetry at 5Hz (Core 0)
 *
 * Owns the GPSModule instance. Every 200ms it updates GPS, drains one
 * SensorData sample from sensorDataQueue (produced by FlightControlTask),
 * enriches it with the current GPS fix, and fans it out to Serial, LoRa
 * and a CSV file on LittleFS.
 *
 * Runs at low priority on Core 0, sharing the core with WiFi/BT stacks
 * and other non-critical work. Never blocks indefinitely on the queue —
 * a bounded timeout keeps the task responsive even without new data.
 *
 * @author #11 - Serra Rocketry
 * @date 2026-07-06
 * @version 1.0.0
 *
 * @see firmware/REFACTORING_PLAN.md - Fase 7 (FreeRTOS Tasks) / Fase 10 (Receiver format)
 * @see AGENTS.md - FreeRTOS Guidelines
 * @see firmware/modules/lora_module.h - sendLoRa() (current CSV-over-LoRa protocol)
 */

#ifndef TELEMETRY_TASK_H
#define TELEMETRY_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "SensorData.h"

//==============================================================================
// TASK CONFIGURATION (AGENTS.md - FreeRTOS Guidelines)
//==============================================================================

constexpr uint32_t    TELEMETRY_STACK_SIZE       = 4096;
constexpr UBaseType_t TELEMETRY_PRIORITY         = 5;
constexpr BaseType_t  TELEMETRY_CORE             = 0;
constexpr TickType_t  TELEMETRY_PERIOD_MS        = 200;  // 5Hz
// Jitter aleatorio (±ms) no periodo de TX — reduz colisao com o satellite
// (#213) que transmite no mesmo canal/sync word a ~5Hz.
constexpr TickType_t  TELEMETRY_JITTER_MS        = 40;
constexpr TickType_t  TELEMETRY_QUEUE_TIMEOUT_MS = 50;   // bounded wait on sensorDataQueue

// Handle da task (definido no .cpp)
extern TaskHandle_t g_telemetryTaskHandle;

/**
 * @brief Inicializa GPS, LittleFS (arquivo CSV com header) e cria a task Telemetry
 *
 * @note Depende de sensorDataQueue (criada por initFlightControlTask()) ja
 *       existir — chame initFlightControlTask() antes desta funcao.
 * @return true se GPS, filesystem e task foram criados com sucesso
 */
bool initTelemetryTask();

/**
 * @brief Entrypoint FreeRTOS - loop de telemetria a 5Hz
 * @param pvParameters Nao utilizado
 */
void taskTelemetry(void* pvParameters);

/**
 * @brief Metricas de diagnostico da task de telemetria
 */
struct TelemetryStats {
  uint32_t cycleCount;         ///< Total de ciclos executados
  uint32_t packetsSent;        ///< Pacotes efetivamente transmitidos (Serial+LoRa+CSV)
  uint32_t queueTimeoutCount;  ///< Ciclos sem dado novo em sensorDataQueue
};

const TelemetryStats& getTelemetryStats();

#endif  // TELEMETRY_TASK_H
