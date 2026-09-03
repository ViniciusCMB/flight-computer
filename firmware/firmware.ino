/**
 * @file firmware.ino
 * @brief Main firmware entry point for #11 Flight Computer (Avionics System)
 * 
 * This is the main program file for a rocket flight computer that handles
 * sensor data collection, parachute deployment, telemetry transmission, and
 * data storage. The system uses a modular architecture with separate header files
 * for each subsystem.
 * 
 * System capabilities:
 * - Altitude and atmospheric pressure monitoring (BMP585)
 * - Inertial measurement unit for orientation (LSM6DS3)
 * - GPS position and time tracking
 * - Long-range telemetry via LoRa radio
 * - Autonomous parachute deployment based on flight profile (apogee, Option A)
 * - Data logging to onboard flash storage (LittleFS)
 * - Audio feedback via buzzer for system status
 * 
 * Flight phases:
 * 1. Initialization: System startup and sensor calibration
 * 2. Pre-launch: Monitoring and data collection on ground
 * 3. Ascent: High-frequency data logging during powered flight
 * 4. Apogee detection: Tracking maximum altitude
 * 5. Descent: Parachute deployment at apogee and controlled landing
 * 6. Recovery: Post-flight data access via storage retrieval
 * 
 * @note All configuration parameters are in config.h
 * @note FlightControlTask runs at 50Hz (FLIGHT_CONTROL_PERIOD_MS);
 *       TelemetryTask and LoggerTask run at 5Hz.
 * 
 * @author Serra Rocketry
 * @date 2026
 */

//==============================================================================
// MODULE INCLUDES
//==============================================================================

#include "config.h"             // Global configuration and constants

#include "sensors/BMP585Sensor.h"
#include "sensors/LSM6DS3Sensor.h"
#include "sensors/GPSModule.h"
#include "flight/FlightStateMachine.h"
#include "flight/FlightControlTask.h"
#include "flight/TelemetryTask.h"
#include "flight/LoggerTask.h"


#include "modules/buzzer_module.h"
#include "modules/filesystem_module.h"
#include "modules/lora_module.h"
#include "modules/parachute_module.h"

//==============================================================================
// SETUP - ONE-TIME INITIALIZATION
//==============================================================================

/**
 * @brief Initialize all system components and prepare for flight
 * 
 * This function runs once at power-on and delegates subsystem startup to the
 * task init functions (each owns its objects, buses, queues and watchdog):
 *  - initFlightControlTask(): Wire (I2C) + BMP585 + LSM6DS3 + FSM +
 *    sensorDataQueue + servo
 *  - initTelemetryTask(): SPI remap + GPS + telemetry queue consumer +
 *    LoRa/file fan-out
 *  - initLoggerTask(): logQueue + Serial/file logger
 * 
 * @note Serial monitor must be set to 115200 baud
 * @note The watchdog (TWDT) is armed inside taskFlightControl once it is
 *       actually running, not here, to avoid a dangling armed watchdog.
 *
 * On any critical init failure the system prints the error, flushes Serial,
 * and enters an infinite loop with the buzzer blinking — it does NOT call
 * ESP.restart() to avoid losing state in flight.
 * 
 * @see setup() is called automatically once by Arduino framework
 */
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);

  // I2C/SPI bus init is owned by the subsystems that use it:
  //   - Wire.begin()      -> initFlightControlTask() (BMP585 + LSM6DS3)
  //   - SPI.begin() remap -> initTelemetryTask(), BEFORE setupStorage()
  //     (SD.begin uses the global SPI object; without the pin remap the SD
  //     card is probed on the ESP32-S3 default SPI pins and always falls
  //     back to LittleFS. setupLoRa() re-issues SPI.begin() — idempotent.)

  bool initOk = true;
  String initFail = "";

  if (!initFlightControlTask()) {
    initFail = "FATAL: FlightControl init failed (sensors/servo?)";
    initOk = false;
  } else if (!initTelemetryTask()) {
    initFail = "FATAL: Telemetry init failed (LoRa/FS/GPS?)";
    initOk = false;
  } else if (!initLoggerTask()) {
    initFail = "FATAL: Logger init failed";
    initOk = false;
  }

  if (!initOk) {
    // Safe-hold: do NOT reboot in a loop (would lose state in flight and
    // hides the error). Print clearly, flush, and blink the buzzer as alarm.
    Serial.println(initFail);
    Serial.flush();
    Serial.println("Halting — check wiring/sensors. Buzzer alarm active.");
    Serial.flush();
    for (;;) {
      // Alarm tone at the piezo resonance — a passive piezo needs a square
      // wave (digitalWrite DC is silent). Same frequency as the normal Beep.
      tone(BUZZER_PIN, BUZZER_TONE_HZ, 400);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

void loop() {
  vTaskDelay(portMAX_DELAY);

}

//==============================================================================
// MAIN LOOP - CONTINUOUS OPERATION
//==============================================================================

/**
 * @brief Main flight computer control loop
 * 
 * Intentionally empty: all real-time work happens in the FreeRTOS tasks
 * created by the init functions (FlightControl @50Hz, Telemetry @5Hz,
 * Logger @5Hz). The loop blocks forever on portMAX_DELAY so the Arduino
 * framework's loop() does not spin.
 * 
 * Parachute deployment (safety-critical) is decided by FlightStateMachine
 * on apogee detection (Option A) and actuated by taskFlightControl — not
 * here.
 */
