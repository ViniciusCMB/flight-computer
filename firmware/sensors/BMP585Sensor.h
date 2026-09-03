/**
 * @file BMP585Sensor.h
 * @brief BMP585 barometric pressure sensor driver
 *
 * @see BMP585Sensor.cpp for implementation
 * @see firmware/REFACTORING_PLAN.md Phase 3
 */

#ifndef BMP585_SENSOR_H
#define BMP585_SENSOR_H

#include "ISensor.h"
#include <Adafruit_BMP5xx.h>
#include <Adafruit_BMP280.h>
#include <Arduino.h>

/**
 * @brief BMP585 barometric sensor implementation
 *
 * Implements ISensor interface for BMP585 barometric sensor.
 * Calculates altitude using barometric formula and computes
 * vertical velocity via numerical differentiation.
 *
 * Fallback (mirrors satellite/src/sensors/BME280Sensor.cpp pattern):
 * if the BMP585 is not found, tries Adafruit_BMP280 at 0x76 and 0x77
 * (pressure/temperature only). UseBMP585() selects the active backend.
 */
class BMP585Sensor : public ISensor {
public:
  BMP585Sensor();

  // ISensor interface
  bool begin() override;
  void update() override;
  String getData() override;
  bool isReady() override;

  // Barometer-specific getters
  float getAltitude() const;
  float getPressure() const;
  float getTemperature() const;
  float getMaxAltitude() const;
  float getVerticalVelocity() const;

  /**
   * @brief Full re-initialization and re-calibration (glitch recovery)
   *
   * Re-runs the begin() sequence. Recovers the sensor after an I2C bus
   * corruption or a chip reset (e.g. brownout during a LoRa TX burst) that
   * leaves the BMP585 self-consistent but reading a wrong pressure (observed
   * on the bench: sustained ~-26% pressure offset => alt jumps to ~2400 m
   * while vz stays ~0). Re-captures base pressure from the fresh reading.
   *
   * @return true if re-initialization succeeded
   * @warning Only call while the FSM is IDLE and the rocket is at rest
   */
  bool reinit();

  /** @brief true if the primary BMP585 backend is active (not the BMP280 fallback) */
  bool useBMP585() const { return _useBMP585; }

  /**
   * @brief Age (ms) of the last valid barometer reading
   * @return 0xFFFFFFFF if the sensor never produced a valid reading
   * @note Used by the barometer-staleness contingency: a frozen barometer
   *       keeps last-good altitude/vz values (never NaN), so staleness must
   *       be tracked by time, not by value checks.
   */
  uint32_t getLastReadingAgeMs() const;
  void checkHighest();

  /**
   * @brief Restore the launch-site reference pressure after a watchdog reboot
   *
   * Overrides the base pressure captured at boot (which, mid-flight, would be
   * the pressure at the reboot point) with the value persisted to NVS at
   * launch, so altitude stays relative to the launch site across resets.
   * Recomputes altitude, resets _prevAltitude so the first Vz after restore
   * is a real reading (no derivative spike).
   *
   * @param basePressure Reference sea-level pressure in hPa (from NVS)
   */
  void setBasePressure(float basePressure);

  /**
   * @brief Restore the peak-altitude tracker after a watchdog reboot
   * @param maxAltitude Peak altitude reached so far, in meters (from NVS)
   */
  void setMaxAltitude(float maxAltitude);

  /**
   * @brief Current base pressure reference (hPa)
   * @note Used by FlightStateMachine to persist the launch reference to NVS
   */
  float getBasePressure() const;

private:
  bool _firstReading();

  Adafruit_BMP5xx _bmp;
  Adafruit_BMP280 _bmp280;  // fallback backend (BMP585 missing)
  bool _useBMP585;
  bool _ready;
  float _basePressure;
  float _altitude;
  float _temperature;
  float _pressure;
  float _maxAltitude;
  float _prevAltitude;
  unsigned long _prevTime;
  float _verticalVelocity;
  uint16_t _spikeStreak;  // consecutive spike-rejected samples (ratchet escape)
};

#endif // BMP585_SENSOR_H
