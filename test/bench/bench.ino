/**
 * @file bench.ino
 * @brief Sequential bench self-test for every flight-computer sensor/module/actuator
 *
 * Interactive dispatcher: send a single-letter command over Serial (115200)
 * to run that component's test. Results print PASS/FAIL per sub-check.
 *
 * Commands:
 *   ?  — list commands
 *   a  — ALL: run every test sequentially
 *   b  — Barometer (BMP585 primary, BMP280 fallback)
 *   i  — IMU (LSM6DS3)
 *   g  — GPS (NEO-8M via UART1)
 *   l  — LoRa (RFM95W SPI + TX loopback window)
 *   f  — Filesystem (SD card, LittleFS)
 *   s  — Servo (parachute door — MOVES THE ACTUATOR)
 *   z  — Buzzer
 *
 * Pinout/constants mirror firmware/config.h — keep in sync.
 *
 * @author Serra Rocketry (#11)
 * @date 2026
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LSM6DS3.h>
#include <TinyGPSPlus.h>
#include <LoRa.h>
#include <SD.h>
#include <ESP32Servo.h>
#include <LittleFS.h>
#include "FS.h"

// ---- mirror of firmware/config.h — keep in sync ---------------------------
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_ADDR_BMP585 0x7E
#define I2C_ADDR_BMP280_PRIMARY 0x76
#define I2C_ADDR_BMP280_ALT 0x77
#define I2C_ADDR_LSM6DS3 0x6B
#define LORA_FREQ 915E6
#define LORA_SCK 12
#define LORA_MISO 13
#define LORA_MOSI 11
#define SS_LORA 10
#define RST_LORA 4
#define DIO0_LORA 5
#define SYNC_WORD 0xF3
#define LORA_SF 7
#define LORA_BW 125E3
#define LORA_CR 5
#define LORA_TX_POWER 17
#define SD_CS_PIN 14
#define RX_GPS 17  // bench-measured: GPS TX physically on GPIO17 (mirror of config.h 2026-08-27)
#define TX_GPS 18
#define SERVO_PIN 7
#define BUZZER_PIN 6
const int SERVO_CLOSED = 50;  // door held closed (was 90; bench-set 2026-08-27)
const int SERVO_EJECT = 160;  // parachute ejection position
static const int GPS_BAUD = 9600;

// ---- fixtures ---------------------------------------------------------------
Adafruit_BMP5xx bmp585;
Adafruit_BMP280 bmp280;
Adafruit_LSM6DS3 lsm;
Servo benchServo;
HardwareSerial GpsSerial(1);

uint8_t passCount = 0, failCount = 0;

void verdict(const char *name, bool ok) {
  Serial.printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  ok ? passCount++ : failCount++;
}

void header(const char *s) { Serial.printf("\n=== %s ===\n", s); }

// ---------------------------------------------------------------- barometer
void testBaro() {
  header("BAROMETER (BMP585 primary / BMP280 fallback)");
  bool use585 = bmp585.begin(I2C_ADDR_BMP585, &Wire);
  if (use585) {
    Serial.println("  backend: BMP585 @0x7E");
    verdict("BMP585 begin", true);
    bool ok = bmp585.performReading();
    verdict("BMP585 performReading", ok);
    if (ok) {
      float p = bmp585.pressure / 100.0F;
      float t = bmp585.temperature;
      Serial.printf("  p=%.2f hPa  t=%.2f C\n", p, t);
      verdict("pressure in plausible range (300-1200 hPa)", p > 300 && p < 1200);
      verdict("temperature in plausible range (-40-85 C)", t > -40 && t < 85);
      // repeatability: 10 readings, stddev of pressure
      float vals[10];
      for (int i = 0; i < 10; i++) {
        bmp585.performReading();
        vals[i] = bmp585.pressure / 100.0F;
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
    }
  } else {
    Serial.println("  BMP585 @0x7E not found — trying BMP280 fallback");
    verdict("BMP585 begin (expected present)", false);
    bool found = false;
    for (uint8_t addr : {I2C_ADDR_BMP280_PRIMARY, I2C_ADDR_BMP280_ALT}) {
      if (bmp280.begin(addr)) {
        Serial.printf("  backend: BMP280 @0x%02X\n", addr);
        found = true;
        break;
      }
    }
    verdict("BMP280 fallback begin", found);
    if (found) {
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
    }
  }
}

// ---------------------------------------------------------------------- IMU
void testImu() {
  header("IMU (LSM6DS3 @0x6B)");
  bool ok = lsm.begin_I2C(I2C_ADDR_LSM6DS3, &Wire);
  verdict("LSM6DS3 begin_I2C", ok);
  if (!ok) return;

  sensors_event_t a, g, t;
  lsm.getEvent(&a, &g, &t);
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float total = sqrtf(ax * ax + ay * ay + az * az);
  Serial.printf("  accel: %.2f %.2f %.2f  |total|=%.2f m/s2\n", ax, ay, az, total);
  verdict("accel |total| ~1g (8.5-11.5)", total > 8.5 && total < 11.5);
  Serial.printf("  gyro: %.3f %.3f %.3f rad/s\n", g.gyro.x, g.gyro.y, g.gyro.z);
  verdict("gyro |xyz| < 0.35 rad/s at rest",
          fabsf(g.gyro.x) < 0.35 && fabsf(g.gyro.y) < 0.35 && fabsf(g.gyro.z) < 0.35);
  Serial.printf("  temp: %.2f C\n", t.temperature);
  verdict("IMU temp plausible (-10-70 C)", t.temperature > -10 && t.temperature < 70);

  // data rate sanity: 20 reads should complete quickly (non-blocking bus)
  uint32_t t0 = millis();
  for (int i = 0; i < 20; i++) lsm.getEvent(&a, &g, &t);
  uint32_t dt = millis() - t0;
  Serial.printf("  20 reads in %lu ms (%.1f us/read)\n", dt, dt * 1000.0 / 20);
  verdict("20 reads < 200 ms (bus healthy)", dt < 200);
}

// --------------------------------------------------------------------- GPS
void testGps() {
  header("GPS (NEO-8M UART1 RX=17 TX=18 @9600)");
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, RX_GPS, TX_GPS);
  verdict("UART1 begin", true);

  TinyGPSPlus gps;
  uint32_t t0 = millis();
  int bytesIn = 0;
  while (millis() - t0 < 5000) {  // listen 5 s
    while (GpsSerial.available()) {
      gps.encode(GpsSerial.read());
      bytesIn++;
    }
    delay(10);
  }
  Serial.printf("  bytes received: %d\n", bytesIn);
  verdict("GPS sending NMEA (bytes > 0)", bytesIn > 0);
  Serial.printf("  chars processed: %d, sentences: %d, checksum fail: %d\n",
                (int)gps.charsProcessed(), (int)gps.sentencesWithFix(),
                (int)gps.failedChecksum());
  if (bytesIn == 0) {
    Serial.println("  NOTE: no NMEA at 9600. Some NEO modules run 38400. Retrying...");
    GpsSerial.begin(38400, SERIAL_8N1, RX_GPS, TX_GPS);
    t0 = millis(); bytesIn = 0;
    TinyGPSPlus gps2;
    while (millis() - t0 < 3000) {
      while (GpsSerial.available()) { gps2.encode(GpsSerial.read()); bytesIn++; }
      delay(10);
    }
    Serial.printf("  @38400 bytes: %d\n", bytesIn);
    verdict("GPS NMEA at 38400 baud", bytesIn > 0);
    if (gps2.sentencesWithFix() > 0) {
      Serial.printf("  FIX: lat=%.6f lon=%.6f alt=%.1f sats=%d\n",
                    gps2.location.lat(), gps2.location.lng(), gps2.altitude.meters(),
                    gps2.satellites.value());
    }
    verdict("GPS has fix (needs sky view)", gps2.sentencesWithFix() > 0);
    return;
  }
  verdict("GPS sentences valid (checksum fail < processed)", gps.failedChecksum() < gps.charsProcessed());
  Serial.printf("  sats: %d  fix: %d\n", gps.satellites.value(), gps.sentencesWithFix());
  verdict("GPS has fix (needs sky view)", gps.sentencesWithFix() > 0);
}

// -------------------------------------------------------------------- LoRa
void testLora() {
  header("LoRa (RFM95W SPI 12/13/11 CS=10 RST=4 DIO0=5 @915MHz)");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, SS_LORA);
  LoRa.setPins(SS_LORA, RST_LORA, DIO0_LORA);
  bool ok = LoRa.begin(LORA_FREQ);
  verdict("LoRa.begin (module responds on SPI)", ok);
  if (!ok) return;
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.enableCrc();
  verdict("radio params applied (SF7 BW125 CR5 P17 sync 0xF3)", true);

  // TX a packet (visible on a nearby receiver-lora running the same params)
  Serial.println("  transmitting test packet...");
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print("#11BENCH,TEST,1");
  ok = LoRa.endPacket();  // synchronous
  verdict("LoRa TX packet (endPacket==1)", ok == 1);
  LoRa.receive();
  // 3 s RX window: report anything heard (echo/ground station)
  Serial.println("  RX window 3 s (any received packet prints below)...");
  uint32_t t0 = millis();
  bool heard = false;
  while (millis() - t0 < 3000) {
    int ps = LoRa.parsePacket();
    if (ps > 0) {
      heard = true;
      Serial.printf("  RX %d bytes RSSI=%d: ", ps, LoRa.packetRssi());
      while (LoRa.available()) Serial.write(LoRa.read());
      Serial.println();
    }
    delay(5);
  }
  if (!heard) Serial.println("  (nothing received — normal without a second radio)");
}

// -------------------------------------------------------------- filesystem
void testFs() {
  header("FILESYSTEM (SD @CS=14, fallback LittleFS)");
  bool sd = SD.begin(SD_CS_PIN);
  if (sd) {
    Serial.println("  backend: SD card");
    verdict("SD.begin", true);
    uint8_t type = SD.cardType();
    const char *tn = (type == CARD_MMC) ? "MMC" : (type == CARD_SD) ? "SDSC"
                     : (type == CARD_SDHC) ? "SDHC" : "UNKNOWN";
    Serial.printf("  card type: %s, size: %llu MB\n", tn, SD.cardSize() / (1024ULL * 1024ULL));
    verdict("card type known", type != CARD_NONE);
    File f = SD.open("/bench.txt", FILE_WRITE);
    verdict("SD open write", f);
    if (f) { f.println("bench test"); f.close(); }
    f = SD.open("/bench.txt", FILE_READ);
    verdict("SD read back", f && f.readStringUntil('\n').startsWith("bench"));
    if (f) f.close();
    SD.remove("/bench.txt");
  } else {
    Serial.println("  SD not found — falling back to LittleFS");
    verdict("SD.begin (card optional on bench)", false);
    bool lfs = LittleFS.begin(true);
    verdict("LittleFS.begin(true)", lfs);
    if (lfs) {
      Serial.printf("  LittleFS: %llu of %llu bytes used\n",
                    LittleFS.usedBytes(), LittleFS.totalBytes());
      File f = LittleFS.open("/bench.txt", FILE_WRITE);
      verdict("LittleFS open write", f);
      if (f) { f.println("bench test"); f.close(); }
      f = LittleFS.open("/bench.txt", FILE_READ);
      verdict("LittleFS read back", f && f.readStringUntil('\n').startsWith("bench"));
      if (f) f.close();
      LittleFS.remove("/bench.txt");
    }
  }
}

// ------------------------------------------------------------------- servo
void testServo() {
  header("SERVO (parachute ejection, GPIO7 — ACTUATOR WILL MOVE)");
  Serial.println("  Angles: CLOSED=50 (held), EJECT=180 (parachute out)");
  Serial.println("  WARNING: door will EJECT the parachute — clear the area!");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  bool attached = benchServo.attach(SERVO_PIN, 500, 2400);
  verdict("servo attach(GPIO7)", attached);
  if (!attached) return;

  // Start CLOSED (50) so the test always begins from the held position
  Serial.println("  start: CLOSED (50)...");
  benchServo.write(SERVO_CLOSED);
  delay(1500);  // generous settle so the horn definitely reaches 50
  
  // LOG DE VERIFICAÇÃO INICIAL
  Serial.printf("  [Check] Posição inicial lida: %d\n", benchServo.read());

  uint32_t lastMs[3] = {0,0,0};
  
  // Alterado para rep <= 3 para fazer sentido com o array de 3 posições (lastMs[3])
  for (int rep = 1; rep <= 3; rep++) {
    Serial.printf("  rep %d: CLOSED (%d) — ready\n", rep, benchServo.read());
    delay(500);
    
    Serial.printf("  rep %d: EJECT (180)!\n", rep);
    uint32_t t0 = millis();
    benchServo.write(SERVO_EJECT);
    delay(1000);
    uint32_t travel = millis() - t0;   // upper bound on real travel time
    lastMs[rep - 1] = travel;
    
    // LOG DE VERIFICAÇÃO PÓS-EJEÇÃO
    Serial.printf("  [Check] Posição pós-ejeção lida: %d\n", benchServo.read());
    Serial.printf("  rep %d: travel 50->180 <= %lu ms\n", rep, travel);
    
    Serial.printf("  rep %d: return CLOSED (50)\n", rep);
    benchServo.write(SERVO_CLOSED);
    delay(1000);
    
    // LOG DE VERIFICAÇÃO PÓS-RETORNO
    Serial.printf("  [Check] Posição após retornar: %d\n", benchServo.read());
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
// Piezo is PASSIVE: it needs a square wave (~2-4 kHz), not DC. Driven via
// LEDC PWM through a PN2222A (220 ohm base resistor).
#define BUZZER_TONE_HZ 2700

void testBuzzer() {
  header("BUZZER (passive piezo, GPIO6 via PN2222A)");
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
  Serial.println("\n############ FULL BENCH SELF-TEST ############");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, SS_LORA);
  testBaro();
  testImu();
  testGps();
  testLora();
  testFs();
  testServo();
  testBuzzer();
  Serial.printf("\n############ RESULT: %d PASS, %d FAIL ############\n",
                passCount, failCount);
}

// ------------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== Flight Computer bench self-test ===");
  Serial.println("Commands: a(ll) b(aro) i(mu) g(ps) l(ora) f(s) s(ervo) z(buzzer) ?");
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
      case 'i': passCount = failCount = 0; testImu();
        Serial.printf("  subtotal: %d PASS, %d FAIL\n", passCount, failCount); break;
      case 'g': passCount = failCount = 0; testGps();
        Serial.printf("  subtotal: %d PASS, %d FAIL\n", passCount, failCount); break;
      case 'l': passCount = failCount = 0; testLora();
        Serial.printf("  subtotal: %d PASS, %d FAIL\n", passCount, failCount); break;
      case 'f': passCount = failCount = 0; testFs();
        Serial.printf("  subtotal: %d PASS, %d FAIL\n", passCount, failCount); break;
      case 's': testServo(); break;
      case 'z': testBuzzer(); break;
      case '?':
        Serial.println("a=all b=baro i=imu g=gps l=lora f=fs s=servo z=buzzer");
        break;
      default: break;  // ignore CR/LF/noise
    }
  }
  delay(10);
}
