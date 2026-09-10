/**
 * @file emergency_deploy.ino
 * @brief EMERGENCY parachute deployment - barometer only
 *
 * No FSM, no logging. BMP280 only. Servo on GPIO19.
 * Deploy condition: altitude > 50 m AND descending (vz < 0).
 *
 * @author Team #100 - Serra Rocketry
 * @date 2026
 */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>

#define SERVO_PIN 19
#define BUZZER_PIN 1          // passive piezo, cyclic beep (never stops)
#define DEPLOY_ALT_MIN 50.0   // m  - minimum altitude to allow deploy
#define LIFTOFF_ALT 10.0      // m  - altitude that arms descent detection
#define DESCENT_VZ -1.5       // m/s - must be falling faster than this
#define LOOP_PERIOD_MS 100    // 10 Hz

// Angles mirror firmware/config.h (main flight computer)
const int SERVO_CLOSED = 50;   // door held closed
const int SERVO_OPEN = 135;    // parachute ejection position

Adafruit_BMP280 bmp;
Servo deployServo;

float basePressure = 1013.25;
float lastAltitude = 0.0;
bool airborne = false;
bool deployed = false;

void releaseParachute() {
  deployServo.write(SERVO_OPEN);
  Serial.println("🪂 PARACHUTE DEPLOYED");
}

// Cyclic beep: 200 ms tone every 2 s, driven from loop() (never stops).
// Same passive-piezo approach as the main bench (PN2222A stage).
#define BUZZER_TONE_HZ 2700
#define BEEP_ON_MS 200
#define BEEP_PERIOD_MS 2000

void beepTask() {
  static uint32_t lastBeep = 0;
  static bool beeping = false;
  uint32_t now = millis();
  if (!beeping && now - lastBeep >= BEEP_PERIOD_MS - BEEP_ON_MS) {
    ledcWriteTone(BUZZER_PIN, BUZZER_TONE_HZ);
    beeping = true;
    lastBeep = now;
  } else if (beeping && now - lastBeep >= BEEP_ON_MS) {
    ledcWriteTone(BUZZER_PIN, 0);
    beeping = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Buzzer (passive piezo on GPIO1, cyclic beep)
  ledcAttach(BUZZER_PIN, BUZZER_TONE_HZ, 10);  // core 3.x API: pin, freq, resolution

  // Servo (angles same as main code: 500-2400 us, closed 50 / open 135)
  ESP32PWM::allocateTimer(0);
  deployServo.setPeriodHertz(50);
  if (!deployServo.attach(SERVO_PIN, 500, 2400)) {
    Serial.println("❌ Servo attach failed");
  }
  deployServo.write(SERVO_CLOSED);  // hold closed

  // BMP280
  if (!bmp.begin(0x76)) {  // try 0x77 as fallback
    if (!bmp.begin(0x77)) {
      Serial.println("❌ BMP280 not found, halting");
      while (true) delay(1000);
    }
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,   // temperature
                  Adafruit_BMP280::SAMPLING_X16,  // pressure
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_63);

  // Zero base pressure (average 20 readings)
  float sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += bmp.readPressure() / 100.0F;
    delay(50);
  }
  basePressure = sum / 20.0;
  Serial.print("✅ Base pressure: ");
  Serial.print(basePressure);
  Serial.println(" hPa");
}

void loop() {
  beepTask();  // cyclic beep, always running

  static uint32_t lastLoop = 0;
  if (millis() - lastLoop < LOOP_PERIOD_MS) return;
  lastLoop = millis();

  float pressure = bmp.readPressure() / 100.0F;
  if (isnan(pressure) || pressure <= 300 || pressure > 1200) {
    Serial.println("⚠️ Invalid pressure reading");
    return;
  }

  float altitude = 44330.0 * (1.0 - pow(pressure / basePressure, 0.1903));

  if (deployed) {
    Serial.printf("🪂 %.1f m\n", altitude);
    return;
  }

  if (!airborne) {
    if (altitude > LIFTOFF_ALT) {
      airborne = true;
      Serial.println("🚀 LIFTOFF detected");
    }
    return;
  }

  // Airborne: descent detection (finite difference over ~200 ms window)
  static uint32_t lastDiff = 0;
  static float prevAlt = altitude;
  if (millis() - lastDiff >= 200) {
    float vz = (altitude - prevAlt) * 1000.0 / (float)(millis() - lastDiff);
    prevAlt = altitude;
    lastDiff = millis();

    Serial.printf("alt=%.1f m vz=%.2f m/s\n", altitude, vz);

    if (vz < DESCENT_VZ && altitude > DEPLOY_ALT_MIN) {
      releaseParachute();
      deployed = true;
    }
  }
}
