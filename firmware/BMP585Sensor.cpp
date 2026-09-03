/**
 * @file BMP585Sensor.cpp
 * @brief Implementation of BMP585 barometric sensor driver
 * 
 * @see BMP585Sensor.h for class definition
 * @see firmware/REFACTORING_PLAN.md Fase 3
 */

#include "sensors/BMP585Sensor.h"
#include "config.h"

BMP585Sensor::BMP585Sensor()
    : _ready(false), _useBMP585(true), _basePressure(0.0F), _altitude(0.0F), _temperature(0.0F),
      _pressure(0.0F), _maxAltitude(0.0F), _prevAltitude(0.0F), _prevTime(0UL),
      _spikeStreak(0),
      _verticalVelocity(0.0F) {}

/**
 * @brief Initializes BMP585 sensor and calibrates base pressure
 * 
 * Performs I2C communication test, first reading validation, and
 * base pressure calibration (single sample at startup).
 * 
 * @return true if initialization successful, false on error
 * @note Blocking: performs one sensor reading during calibration
 */
bool BMP585Sensor::begin() {
  // Primary: BMP585 at the factory-default address (0x46; CSB strap = 0x47).
  // Post-begin config mirrors the working satellite implementation
  // (satellite/src/sensors/BME280Sensor.cpp) — without setPowerMode(NORMAL)
  // the chip stays in standby and performReading() can fail.
  bool found585 = false;
  for (uint8_t a : {I2C_ADDR_BMP585, I2C_ADDR_BMP585_ALT}) {
    if (!_bmp.begin(a, &Wire)) {
      continue;
    }
    _bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);
    _bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
    _bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    _bmp.setOutputDataRate(BMP5XX_ODR_25_HZ);
    _bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    _useBMP585 = true;
    found585 = true;
    break;
  }
  if (!found585) {
    // Fallback: BMP280 (satellite BME280Sensor.cpp pattern) — try both
    // strap-selected addresses. Sampling mirrors the satellite config.
    Serial.println("BMP585 not found, trying BMP280 fallback...");
    bool found280 = false;
    for (uint8_t addr : {I2C_ADDR_BMP280_PRIMARY, I2C_ADDR_BMP280_ALT}) {
      if (_bmp280.begin(addr)) {
        _bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                            Adafruit_BMP280::SAMPLING_X2,
                            Adafruit_BMP280::SAMPLING_X16,
                            Adafruit_BMP280::FILTER_X16,
                            Adafruit_BMP280::STANDBY_MS_1);  // fastest for 50 Hz loop
        _useBMP585 = false;
        found280 = true;
        Serial.println("BMP280 fallback active (backend=bmp280).");
        break;
      }
    }
    if (!found280) {
      Serial.println("BMP585 initialization failed (no BMP280 fallback either).");
      _ready = false;
      return false;
    }
  }

  if (!_firstReading()) {
    Serial.println("Barometer first reading failed.");
    _ready = false;
    return false;
  }

  _ready = true;
  return true;
}

bool BMP585Sensor::reinit() {
  Serial.println("[BMP585] reinit: sensor glitch recovery");
  _ready = false;
  if (!begin()) {
    Serial.println("[BMP585] reinit FAILED");
    return false;
  }
  Serial.printf("[BMP585] reinit OK: p=%.2f hPa t=%.1f C\n",
                _pressure, _temperature);
  return true;
}

/**
 * @brief Take the first (blocking) reading and seed the state
 *
 * Shared by both backends (BMP585 and BMP280 fallback): fills
 * pressure/temperature/altitude, calibrates base pressure, and resets
 * the Vz derivative state.
 *
 * Base pressure calibration is median-filtered and range-validated: a
 * single corrupted sample at boot (I2C glitch, sensor mid-reset during
 * power-up) was observed to seed base_pressure ~1305 hPa, which made the
 * altitude read ~+2400 m on the pad permanently. The median of N valid
 * samples is immune to isolated corruption.
 *
 * @return true if the reading was valid
 */
bool BMP585Sensor::_firstReading() {
  float rawPressure = 0.0F;
  float rawTemp = 0.0F;

  if (_useBMP585) {
    if (!_bmp.performReading()) {
      return false;
    }
    // NOTE: performReading() already converts Pa -> hPa internally.
    // Do NOT divide by 100 again (double conversion caused alt=-62157m).
    rawPressure = _bmp.pressure;
    rawTemp = _bmp.temperature;
  } else {
    rawPressure = _bmp280.readPressure() / 100.0F;
    rawTemp = _bmp280.readTemperature();
  }

  // ── Median-filtered base pressure calibration ─────────────────────────────
  // Reject samples outside the physically plausible atmosphere (300 hPa ~
  // 9000 m; 1100 hPa ~ -900 m — the analog of the setBasePressure guard).
  // Up to N attempts; require at least MIN_VALID to accept.
  float samples[FIRST_READ_SAMPLES];
  uint8_t valid = 0;
  for (uint8_t attempt = 0;
       attempt < FIRST_READ_MAX_ATTEMPTS && valid < FIRST_READ_SAMPLES;
       attempt++) {
    if (rawPressure > 300.0F && rawPressure < 1100.0F) {
      samples[valid++] = rawPressure;
    }
    delay(FIRST_READ_SAMPLE_PERIOD_MS);
    if (_useBMP585) {
      if (!_bmp.performReading()) continue;
      rawPressure = _bmp.pressure;
      rawTemp = _bmp.temperature;
    } else {
      rawPressure = _bmp280.readPressure() / 100.0F;
      rawTemp = _bmp280.readTemperature();
    }
  }

  if (valid < FIRST_READ_MIN_VALID) {
    Serial.printf("[BMP585] Calibration failed: only %u valid pressure "
                  "samples\n", valid);
    return false;
  }

  // Insertion sort (N <= 9, trivial cost) and take the median
  for (uint8_t i = 1; i < valid; i++) {
    const float key = samples[i];
    int8_t j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }
  _basePressure = samples[valid / 2];
  _pressure = rawPressure;
  _temperature = rawTemp;
  _altitude = _useBMP585 ? _bmp.readAltitude(_basePressure)
                         : _bmp280.readAltitude(_basePressure);

  Serial.printf("[BMP585] Base pressure: %.2f hPa (%u/%u valid samples)\n",
                _basePressure, valid, FIRST_READ_SAMPLES);

  _prevAltitude = _altitude;
  _maxAltitude = _altitude;
  _prevTime = millis();
  _verticalVelocity = 0.0F;
  _spikeStreak = 0;
  return !isnan(_altitude) && !isnan(_pressure);
}

/**
 * @brief Updates sensor readings and calculates vertical velocity
 * 
 * Non-blocking sensor read with numerical differentiation for Vz calculation.
 * Vertical velocity is clipped to ±200 m/s to reject noise spikes.
 * Invalid readings (NaN, out of range) are silently discarded,
 * preserving the last known good values as fallback.
 * 
 * @return void
 * @note Called by FlightControlTask at 50Hz
 * @note Calls checkHighest() to update max altitude
 */
void BMP585Sensor::update() {
  if (!isReady()) {
    return;
  }

  const unsigned long current_time = millis();
  float current_altitude = NAN;

  if (_useBMP585) {
    if (!_bmp.performReading()) {
      return;
    }
    // performReading() already returns hPa (Pa->hPa done internally)
    _pressure = _bmp.pressure;
    _temperature = _bmp.temperature;
    current_altitude = _bmp.readAltitude(_basePressure);
  } else {
    _pressure = _bmp280.readPressure() / 100.0F;
    _temperature = _bmp280.readTemperature();
    current_altitude = _bmp280.readAltitude(_basePressure);
  }

  // Validate reading — fallback to previous values on corruption
  if (isnan(current_altitude) || current_altitude < -500.0F ||
      current_altitude > 50000.0F) {
    return;
  }

  // Pressure-spike rejection: moving the board through the air (bench shake,
  // EMI) causes momentary pressure puffs that read as tens of meters of
  // altitude change between consecutive samples — an implied climb rate no
  // real flight produces. Discard the sample and keep the last good state
  // (altitude, Vz and maxAltitude are all protected).
  //
  // Ratchet escape: a single ACCEPTED glitch latches _prevAltitude high, and
  // from then on every legitimate return-to-zero reading looks like a spike
  // (a -15 m step is 750 m/s) — the altitude would never come back down.
  // After BARO_SPIKE_STREAK_RESEED consecutive rejections the reference is
  // re-seeded: either the sensor reference is corrupt (sustained glitch ->
  // checkBaroGlitch then reinits on the pad) or the flight genuinely exceeds
  // BARO_MAX_ALT_RATE (in which case tracking reality is the right choice).
  if (_prevTime != 0UL && current_time > _prevTime) {
    const float spikeRate =
        fabsf(current_altitude - _prevAltitude) * 1000.0F /
        static_cast<float>(current_time - _prevTime);
    if (spikeRate > BARO_MAX_ALT_RATE) {
      if (++_spikeStreak >= BARO_SPIKE_STREAK_RESEED) {
        _spikeStreak = 0;
        _altitude = current_altitude;
        _prevAltitude = current_altitude;
        _prevTime = current_time;
        _verticalVelocity = 0.0F;
        checkHighest();
        Serial.printf("[BMP585] Spike streak (%u samples) — reference "
                      "re-seeded to %.1f m\n", BARO_SPIKE_STREAK_RESEED,
                      current_altitude);
      }
      return;
    }
  }
  _spikeStreak = 0;

  _altitude = current_altitude;

  const float dt = (current_time - _prevTime) / 1000.0F;
  if (dt > 0.001F) {
    float vz = (current_altitude - _prevAltitude) / dt;

    if (vz > 200.0F) {
      vz = 200.0F;
    } else if (vz < -200.0F) {
      vz = -200.0F;
    }

    _verticalVelocity = vz;
    _prevAltitude = current_altitude;
    _prevTime = current_time;
  }

  checkHighest();
}

String BMP585Sensor::getData() {
  return String(_altitude) + "," + String(_temperature) + ",nan," +
         String(_pressure);
}

bool BMP585Sensor::isReady() { return _ready; }

float BMP585Sensor::getAltitude() const { return _altitude; }

float BMP585Sensor::getPressure() const { return _pressure; }

float BMP585Sensor::getTemperature() const { return _temperature; }

float BMP585Sensor::getMaxAltitude() const { return _maxAltitude; }

float BMP585Sensor::getVerticalVelocity() const { return _verticalVelocity; }

uint32_t BMP585Sensor::getLastReadingAgeMs() const {
  // _prevTime only advances on valid readings (update()/setBasePressure),
  // so millis() - _prevTime is the age of the last GOOD sample. UINT32_MAX
  // marks a sensor that never produced a valid reading.
  return (_ready) ? (millis() - _prevTime) : 0xFFFFFFFFUL;
}

void BMP585Sensor::setBasePressure(float basePressure) {
  if (!(basePressure > 100.0F && basePressure < 1200.0F)) {
    Serial.println("BMP585: invalid base pressure rejected");
    return;
  }
  _basePressure = basePressure;
  _altitude = _useBMP585 ? _bmp.readAltitude(_basePressure)
                         : _bmp280.readAltitude(_basePressure);

  // Validate like update(): fall back to previous value on corruption
  if (isnan(_altitude) || _altitude < -500.0F || _altitude > 50000.0F) {
    return;
  }

  // Reset derivative state so the first Vz after restore is a real reading
  _prevAltitude = _altitude;
  _prevTime = millis();
  _spikeStreak = 0;
}

void BMP585Sensor::setMaxAltitude(float maxAltitude) {
  _maxAltitude = maxAltitude;
}

float BMP585Sensor::getBasePressure() const { return _basePressure; }

void BMP585Sensor::checkHighest() {
  if (_altitude > _maxAltitude) {
    _maxAltitude = _altitude;
  }
}
