/**
 * @file I2CScanner.ino
 * @brief Brute-force I2C bus scan + stability stress test
 *
 * Diagnoses intermittent sensor init failures ("sometimes IMU, sometimes
 * baro"): scans all 7-bit addresses every second, reporting which of the
 * expected devices (LSM6DS3 0x6B, BMP585 0x46/0x47, BMP280 0x76/0x77)
 * respond on each pass. If devices drop out between passes, the bus
 * (wiring, pull-ups, power) is the problem, not the drivers.
 *
 * @author Team #100 (Serra Rocketry)
 * @date 2026
 */

#include <Wire.h>

#define I2C_SDA 8
#define I2C_SCL 9
#define SCAN_PASSES 15
#define PASS_DELAY_MS 1000

const uint8_t EXPECTED[] = {0x46, 0x47, 0x6B, 0x76, 0x77};

const char* addrName(uint8_t a) {
  switch (a) {
    case 0x46: return "BMP585";
    case 0x47: return "BMP585 (ALT)";
    case 0x6B: return "LSM6DS3 (IMU)";
    case 0x76: return "BMP280";
    case 0x77: return "BMP280 (ALT)";
    default:   return "";
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== I2C Scanner (flight-computer pinout: SDA=8 SCL=9) ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // slow mode: rules out signal-integrity issues
  Wire.setTimeOut(50);

  Serial.println("Scanning all 7-bit addresses...");
  int found = scanAndReport();
  if (found == 0) {
    Serial.println(">>> NO DEVICES FOUND. Check wiring/power, then re-seat cables.");
    Serial.println(">>> SDA must be GPIO8, SCL must be GPIO9 (config.h).");
  }
}

void loop() {
  static int pass = 0;
  if (pass >= SCAN_PASSES) {
    Serial.println("=== Scan complete. Stable results above ===");
    while (true) { delay(1000); }
  }
  delay(PASS_DELAY_MS);
  pass++;
  Serial.printf("\n--- pass %d ---\n", pass);
  scanAndReport();
}

int scanAndReport() {
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X FOUND %s\n", addr, addrName(addr));
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  (nothing responding this pass)");
  }
  return found;
}
