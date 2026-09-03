/**
 * @file FlightControlTask.h
 * @brief FreeRTOS task that runs the flight FSM at 5Hz (Core 1)
 *
 * Owns the BMP585 barometer, LSM6DS3 IMU, FlightStateMachine and the
 * parachute release servo. Every 20ms it updates both sensors, advances
 * the FSM, deploys the parachute the moment the FSM confirms deploy
 * conditions, consolidates a SensorData snapshot and pushes it to
 * sensorDataQueue for TelemetryTask to consume.
 *
 * This is the only safety-critical task in the system: highest
 * priority, pinned to its own core, and monitored by the ESP32
 * task watchdog.
 *
 * @author #11 - Serra Rocketry
 * @date 2026-07-06
 * @version 1.0.0
 *
 * @see firmware/REFACTORING_PLAN.md - Fase 7 (FreeRTOS Tasks)
 * @see AGENTS.md - FreeRTOS Guidelines
 */

#ifndef FLIGHT_CONTROL_TASK_H
#define FLIGHT_CONTROL_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "SensorData.h"

//==============================================================================
// TASK CONFIGURATION (AGENTS.md - FreeRTOS Guidelines)
//==============================================================================

constexpr uint32_t    FLIGHT_CONTROL_STACK_SIZE     = 4096;
constexpr UBaseType_t FLIGHT_CONTROL_PRIORITY       = 20;
constexpr BaseType_t  FLIGHT_CONTROL_CORE           = 1;
constexpr TickType_t  FLIGHT_CONTROL_PERIOD_MS      = 200;  // 5Hz (matches baro effective conversion rate; Thonyan 2026-09-03: 50Hz loop + 25Hz sensor = vz alternating 0/2x-real, no deploy)
constexpr uint8_t     FLIGHT_CONTROL_WDT_TIMEOUT_S  = 5;
constexpr UBaseType_t SENSOR_DATA_QUEUE_LEN         = 25;

// Handle da task (definido no .cpp)
extern TaskHandle_t g_flightControlTaskHandle;

// Queue de saida: FlightControlTask -> TelemetryTask
extern QueueHandle_t sensorDataQueue;

/**
 * @brief Inicializa sensores, FSM e queue, e cria a task FlightControl
 *
 * Cria BMP585Sensor, LSM6DS3Sensor e FlightStateMachine internamente,
 * cria sensorDataQueue e agenda taskFlightControl pinned no Core 1.
 *
 * @return true se sensores, FSM, queue e task foram criados com sucesso
 */
bool initFlightControlTask();

/**
 * @brief Entrypoint FreeRTOS - loop de controle de voo a 5Hz
 * @param pvParameters Nao utilizado
 */
void taskFlightControl(void* pvParameters);

/**
 * @brief Metricas de diagnostico/telemetria da task
 */
struct FlightControlStats {
  uint32_t cycleCount;      ///< Total de ciclos executados
  uint32_t overrunCount;    ///< Ciclos com tempo de execucao > 20ms
  uint32_t queueDropCount;  ///< Falhas ao enviar em sensorDataQueue
  int32_t  lastExecTimeUs;  ///< Tempo de execucao do ultimo ciclo (us)
  int32_t  maxExecTimeUs;   ///< Maior tempo de execucao observado (us)
};

const FlightControlStats& getFlightControlStats();

#endif  // FLIGHT_CONTROL_TASK_H
