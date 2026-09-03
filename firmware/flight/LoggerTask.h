/**
 * @file LoggerTask.h
 * @brief FreeRTOS task that prints debug logs asynchronously (Core 0)
 *
 * Consumes LogMessage entries from logQueue and prints them to Serial
 * with a timestamp and level filter. Runs at the lowest FreeRTOS priority
 * so it never delays FlightControlTask (50Hz, Core 1) or TelemetryTask
 * (5Hz, Core 0) — it only runs when no higher-priority task is ready.
 *
 * Any task can call logMessage() to enqueue a message without blocking;
 * if logQueue is full the message is dropped and counted in
 * LoggerStats::droppedCount rather than stalling the caller.
 *
 * @author #11 - Serra Rocketry
 * @date 2026-07-06
 * @version 1.0.0
 *
 * @see firmware/REFACTORING_PLAN.md - Fase 7 (FreeRTOS Tasks)
 * @see AGENTS.md - Error Handling / FreeRTOS Guidelines
 * @see firmware/flight/SensorData.h - LogMessage struct, getLogLevelName()
 */

#ifndef LOGGER_TASK_H
#define LOGGER_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "SensorData.h"

//==============================================================================
// TASK CONFIGURATION (AGENTS.md - FreeRTOS Guidelines)
//==============================================================================

// 4096 bytes: Serial.printf (IDF printf + float formatting + UART driver)
// peaks at ~2 KB; 2048 tripped the stack canary in flight (A15=0xcdcd).
constexpr uint32_t    LOGGER_STACK_SIZE = 4096;
constexpr UBaseType_t LOGGER_PRIORITY   = 1;
constexpr BaseType_t  LOGGER_CORE       = 0;
constexpr UBaseType_t LOG_QUEUE_LEN     = 50;

// Queue de entrada: qualquer task -> LoggerTask
extern QueueHandle_t logQueue;

// Handle da task (definido no .cpp)
extern TaskHandle_t g_loggerTaskHandle;

/**
 * @brief Cria a logQueue e a task Logger (Core 0, prioridade minima)
 * @return true se a queue e a task foram criadas com sucesso
 */
bool initLoggerTask();

/**
 * @brief Entrypoint FreeRTOS - consome logQueue e imprime no Serial
 * @param pvParameters Nao utilizado
 */
void taskLogger(void* pvParameters);

/**
 * @brief Enfileira uma mensagem de log (chamavel por qualquer task)
 *
 * Nao bloqueia (timeout 0): se logQueue estiver cheia, a mensagem e'
 * descartada e contabilizada em getLoggerStats().droppedCount.
 *
 * @param taskId  Identificador da task de origem (ver enum TaskId em SensorData.h)
 * @param level   LOG_LEVEL_DEBUG..LOG_LEVEL_ERROR
 * @param message Texto da mensagem (truncado em 127 caracteres)
 * @return true se enfileirada com sucesso
 */
bool logMessage(uint8_t taskId, uint8_t level, const String& message);

/**
 * @brief Define/consulta o nivel minimo impresso no Serial
 * @note Mensagens abaixo do nivel minimo ainda sao recebidas da queue
 *       (contam em receivedCount) mas nao sao impressas
 */
void setLoggerMinLevel(uint8_t level);
uint8_t getLoggerMinLevel();

/**
 * @brief Metricas de diagnostico da task de logging
 */
struct LoggerStats {
  uint32_t receivedCount;  ///< Mensagens recebidas da logQueue
  uint32_t printedCount;   ///< Mensagens efetivamente impressas (passaram no filtro)
  uint32_t droppedCount;   ///< Mensagens descartadas por logQueue cheia (logMessage())
};

const LoggerStats& getLoggerStats();

#endif  // LOGGER_TASK_H
