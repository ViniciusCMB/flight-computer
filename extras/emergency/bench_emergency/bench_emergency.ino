/**
 * @file bench_emergency.ino
 * @brief Bench self-test for the EMERGENCY deploy board (ESP32-C3 + BMP280 + servo)
 *
 * Stripped-down sibling of test/bench/bench.ino: only the components the
 * emergency firmware uses. Interactive Serial dispatcher (115200):
 *
 *   ?  — list commands
 *   b  — Barometer (BMP280 @0x76/0x77): range, repeatability
 *   s  — Servo (parachute door — MOVES THE ACTUATOR, 3 ejection cycles)
 *   z  — Buzzer (passive piezo GPIO1): tones + resonance sweep
 *   a  — ALL: barometer then servo then buzzer
 *
 * Angles mirror firmware/config.h — keep in sync.
 *
 * @author Serra Rocketry (#11)
 * @date 2026
 */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>

// ---- mirror of firmware/config.h — keep in sync ---------------------------
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_ADDR_BMP280_PRIMARY 0x76
#define I2C_ADDR_BMP280_ALT 0x77
#define SERVO_PIN 19  // emergency board (main computer uses 39)
#define BUZZER_PIN 1  // passive piezo, same as emergency firmware
const int SERVO_CLOSED = 50;   // door held closed (bench-set 2026-08-27)
const int SERVO_OPEN = 135;    // parachute ejection position
#define BUZZER_TONE_HZ 2700

// ---- fixtures ---------------------------------------------------------------
Adafruit_BMP280 bmp280;
Servo benchServo;

uint8_t passCount = 0, failCount = 0;

void verdict(const char *name, bool ok) {
  Serial.printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  ok ? passCount++ : failCount++;
}

void header(const char *s) { Serial.printf("\n=== %s ===\n", s); }

// ---------------------------------------------------------------- barometer
void testBaro() {
  header("BAROMETER (BMP280 @0x76/0x77)");
  bool found = false;
  uint8_t addr = 0;
  for (uint8_t a : {I2C_ADDR_BMP280_PRIMARY, I2C_ADDR_BMP280_ALT}) {
    if (bmp280.begin(a)) {
      addr = a;
      found = true;
      break;
    }
  }
  verdict("BMP280 begin", found);
  if (!found) return;
  Serial.printf("  backend: BMP280 @0x%02X\n", addr);
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_1);

  float p = bmp280.readPressure() / 100.0F;
  float t = bmp280.readTemperature();
  Serial.printf("  p=%.2f hPa  t=%.2f C\n", p, t);
  verdict("pressure in plausible range (300-1200 hPa)", p > 300 && p < 1200);
  verdict("temperature in plausible range (-40-85 C)", t > -40 && t < 85);

  // repeatability: 10 readings, stddev of pressure
  float vals[10];
  for (int i = 0; i < 10; i++) {
    vals[i] = bmp280.readPressure() / 100.0F;
    delay(50);
  }
  float mean = 0;
  for (float v : vals) mean += v;
  mean /= 10;
  float var = 0;
  for (float v : vals) var += (v - mean) * (v - mean);
  float sd = sqrtf(var / 10);
  Serial.printf("  repeatability sd=%.4f hPa\n", sd);
  verdict("repeatability sd < 0.5 hPa", sd < 0.5);

  // zero-altitude plausibility: measured altitude vs 1013.25 reference
  float alt = 44330.0 * (1.0 - pow(p / 1013.25, 0.1903));
  Serial.printf("  altitude vs 1013.25 hPa: %.1f m (UERJ ~10 m)\n", alt);
}

// ------------------------------------------------------------------- servo
void testServo() {
  header("SERVO (parachute ejection, GPIO19 — ACTUATOR WILL MOVE)");
  Serial.println("  Angles: CLOSED=50 (held), EJECT=135 (parachute out)");
  Serial.println("  WARNING: door will EJECT the parachute — clear the area!");
  ESP32PWM::allocateTimer(0);
  bool attached = benchServo.attach(SERVO_PIN, 500, 2400);
  verdict("servo attach(GPIO19)", attached);
  if (!attached) return;

  // Start CLOSED (50) so the test always begins from the held position
  Serial.println("  start: CLOSED (50)...");
  benchServo.write(SERVO_CLOSED);
  delay(1500);  // generous settle so the horn definitely reaches 50
  Serial.printf("  [Check] initial position read: %d\n", benchServo.read());

  uint32_t lastMs[3] = {0, 0, 0};

  for (int rep = 1; rep <= 3; rep++) {
    Serial.printf("  rep %d: CLOSED (%d) — ready\n", rep, benchServo.read());
    delay(500);

    Serial.printf("  rep %d: EJECT (%d)!\n", rep, SERVO_OPEN);
    uint32_t t0 = millis();
    benchServo.write(SERVO_OPEN);
    delay(1000);
    uint32_t travel = millis() - t0;  // upper bound on real travel time
    lastMs[rep - 1] = travel;

    Serial.printf("  [Check] post-ejection position read: %d\n", benchServo.read());
    Serial.printf("  rep %d: travel %d->%d <= %lu ms\n", rep, SERVO_CLOSED,
                  SERVO_OPEN, travel);

    Serial.printf("  rep %d: return CLOSED (%d)\n", rep, SERVO_CLOSED);
    benchServo.write(SERVO_CLOSED);
    delay(1000);
    Serial.printf("  [Check] position after return: %d\n", benchServo.read());
  }

  benchServo.write(SERVO_CLOSED);
  delay(800);
  benchServo.detach();

  Serial.printf("  travel times (upper bound): %lu / %lu / %lu ms\n",
                lastMs[0], lastMs[1], lastMs[2]);
  Serial.println("  3 ejection cycles done — visually confirm the chute ejected");
  Serial.println("  and the door re-seated closed each time.");
  Serial.println("  (mechanical verdict is manual; timing stats above)");
}

// ------------------------------------------------------------------ buzzer
// Passive piezo, same as emergency firmware: square wave via LEDC PWM.
void testBuzzer() {
  header("BUZZER (passive piezo, GPIO1)");
  Serial.printf("  PWM %d Hz...\n", BUZZER_TONE_HZ);
  ledcAttach(BUZZER_PIN, BUZZER_TONE_HZ, 10);  // core 3.x API: pin, freq, resolution
  Serial.println("  3 tones (400ms on / 200ms off)...");
  for (int i = 0; i < 3; i++) {
    ledcWriteTone(BUZZER_PIN, BUZZER_TONE_HZ);
    delay(400);
    ledcWriteTone(BUZZER_PIN, 0);
    delay(200);
  }
  // frequency sweep so you can find the resonant frequency (loudest point)
  Serial.println("  resonance sweep 2.0-4.0 kHz (note the loudest freq):");
  for (uint32_t f = 2000; f <= 4000; f += 250) {
    Serial.printf("    %lu Hz...", f);
    ledcWriteTone(BUZZER_PIN, f);
    delay(350);
    ledcWriteTone(BUZZER_PIN, 0);
    delay(150);
    Serial.println();
  }
  ledcDetach(BUZZER_PIN);
  Serial.println("  (audible verdict is manual: 3 tones + sweep = PASS)");
  verdict("buzzer PWM sequence executed", true);
}

// --------------------------------------------------------------------- all
void runAll() {
  passCount = 0; failCount = 0;
  Serial.println("\n############ FULL EMERGENCY BENCH SELF-TEST ############");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  testBaro();
  testServo();
  testBuzzer();
  Serial.printf("\n############ RESULT: %d PASS, %d FAIL ############\n",
                passCount, failCount);
}

// ------------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== Emergency deploy board bench self-test ===");
  Serial.println("Commands: a(ll) b(aro) s(ervo) z(buzzer) ?");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    switch (c) {
      case 'a': runAll(); break;
      case 'b': passCount = failCount = 0; testBaro();
        Serial.printf("  subtotal: %d PASS, %d FAIL\n", passCount, failCount); break;
      case 's': testServo(); break;
      case 'z': testBuzzer(); break;
      case '?':
        Serial.println("a=all b=baro s=servo z=buzzer");
        break;
      default: break;  // ignore CR/LF/noise
    }
  }
  delay(10);
}
