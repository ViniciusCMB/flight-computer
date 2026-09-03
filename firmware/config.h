/**
 * @file config.h
 * @brief Global configuration file for the avionics system
 * 
 * This file contains all pin definitions, constants, and parameters
 * used throughout the system. Centralizing configurations here makes
 * maintenance and parameter modifications easier.
 * 
 * @author #11
 * @date 2026
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

//==============================================================================
// LORA COMMUNICATION CONFIGURATION
//==============================================================================

/**
 * LoRa module operating frequency in Hz
 * 915 MHz is the ISM frequency for Brazil/Americas (matches receiver-lora).
 * (868 MHz is Europe — do not use here.)
 */
#define LORA_FREQ 915E6

/**
 * SPI bus pins shared between LoRa (RFM95W) and SD card.
 * Wired on the ESP32-S3-DEVKITC-1-N8R8 schematic (2026-08):
 *   SCK  = GPIO12, MISO = GPIO13, MOSI = GPIO11, CS_LORA = GPIO10
 * @note The LoRa 0.8.0 library uses the global SPI object. We remap the SPI
 *       bus to these pins via SPI.begin(SCK, MISO, MOSI, SS) in setupLoRa()
 *       (same approach as the receiver-lora firmware, which compiles/runs
 *       clean). Only CS/RST/DIO0 are passed to LoRa.setPins().
 */
#define LORA_SCK  12
#define LORA_MISO 13
#define LORA_MOSI 11
#define SS_LORA 10

/**
 * Reset pin for LoRa module (RFM95W).
 * Schematic: GPIO4 on ESP32-S3-DEVKITC-1-N8R8.
 */
#define RST_LORA 4

/**
 * DIO0 pin of LoRa module (IRQ / interrupt).
 * Schematic: GPIO5 on ESP32-S3-DEVKITC-1-N8R8.
 */
#define DIO0_LORA 5

/**
 * Synchronization word for LoRa communication
 * Ensures only devices with the same word communicate
 * Change this value to create isolated LoRa networks
 */
#define SYNC_WORD 0xF3

// Spreading factor / bandwidth / coding rate / TX power.
// Explicitly matched to the receiver (recovery-webui/components/receiver-lora)
// so the link connects. These are also the LoRa.h defaults, but we set them
// explicitly to avoid relying on library defaults.
#define LORA_SF       7      // Spreading Factor 7–12
#define LORA_BW       125E3  // Bandwidth Hz
#define LORA_CR       5      // Coding Rate (4/5)
#define LORA_TX_POWER 17     // dBm

//==============================================================================
// PIN DEFINITIONS - SD CARD (SPI, shares bus with LoRa)
//==============================================================================

/**
 * Chip Select pin for SD card module.
 * Shares the SPI bus with LoRa (SCK=12, MISO=13, MOSI=11, CS_LORA=10).
 * Schematic: GPIO14 on ESP32-S3-DEVKITC-1-N8R8.
 * @note If SD fails, data is saved to LittleFS (internal flash) automatically.
 */
#define SD_CS_PIN 14

/**
 * Flush file buffer every N samples when using SD card
 * Balances data safety vs. write endurance.
 */
static constexpr uint8_t FLUSH_EVERY_N = 10;

//==============================================================================
// PIN DEFINITIONS - ACTUATORS
//==============================================================================

/**
 * Digital pin connected to parachute servo motor.
 * Uses PWM to control servo position.
 * Schematic: GPIO7 on ESP32-S3-DEVKITC-1-N8R8.
 * @note Two servo footprints (M1, M2) on the schematic share this same PWM
 *       signal — confirm with the assembler whether the build uses one or both.
 */
#define SERVO_PIN 7

/**
 * Digital pin connected to piezoelectric buzzer.
 * Emits sound signals for status indication.
 * Schematic: GPIO6 on ESP32-S3-DEVKITC-1-N8R8.
 * @note Avoid GPIO0 (boot strap).
 */
#define BUZZER_PIN 6

/**
 * Tone frequency for the passive piezo buzzer (Hz).
 * @note Bench resonance sweep (test/bench `z`, 2026-08-27): loudest point
 *       of the 2.0-4.0 kHz scan. The piezo is PASSIVE — drive with tone()/
 *       LEDC square wave; digitalWrite DC produces no sound.
 */
#define BUZZER_TONE_HZ 2700

//==============================================================================
// PIN DEFINITIONS - I2C (SENSORS: BMP585 barometer, LSM6DS3 IMU)
//==============================================================================

/**
 * I2C data pin (SDA) for the sensor bus.
 * @note ESP32-S3 Arduino core default is SDA=8 / SCL=9. The BMP585 and
 *       LSM6DS3 drivers call begin_I2C() with no pins, so they use this
 *       default. Wire.begin() in initFlightControlTask() also uses these.
 *       Keep the schematic wired to 8/9 (or change both here and the calls).
 */
#define I2C_SDA 8

/**
 * I2C clock pin (SCL) for the sensor bus.
 */
#define I2C_SCL 9

/**
 * I2C 7-bit address of the LSM6DS3 IMU.
 * Bench-measured via brute-force I2C scan (2026-08-27): responds at 0x6B
 * (the LSM6DS3 alternates 0x6A/0x6B depending on the SDO/SA0 strap).
 */
#define I2C_ADDR_LSM6DS3 0x6B

/**
 * I2C 7-bit address of the BMP585 barometer.
 * 0x46 is the BMP58x factory default (CSB tied low selects 0x47).
 * The earlier 0x7E here was a ghost ACK from the brute-force scan
 * (ACK present but CHIP_ID=0x50 never validated — 2026-08-27 bench).
 */
#define I2C_ADDR_BMP585 0x46
/** Alternate BMP58x address when the CSB pin is strapped high. */
#define I2C_ADDR_BMP585_ALT 0x47

/**
 * BMP280 fallback addresses (strap-selected: SDO low = 0x76, high = 0x77).
 * Used only when the BMP585 is not found at I2C_ADDR_BMP585.
 */
#define I2C_ADDR_BMP280_PRIMARY 0x76
#define I2C_ADDR_BMP280_ALT 0x77

//==============================================================================
// PIN DEFINITIONS - GPS
//==============================================================================

/**
 * RX pin for serial communication with GPS module.
 * Connects to GPS module TX.
 * Bench-measured 2026-08-27 (test/gps_diag raw sniffer): the GPS TX line
 * physically arrives on GPIO17 (NMEA at 9600 confirmed), NOT on GPIO18 as
 * the schematic comment said. Swapped with TX_GPS accordingly.
 */
#define RX_GPS 17

/**
 * TX pin for serial communication with GPS module.
 * Connects to GPS module RX (bench: the remaining GPS line is on GPIO18).
 */
#define TX_GPS 18

//==============================================================================
// PARACHUTE CONTROL CONSTANTS
//==============================================================================

/**
 * Servo position: parachute EJECTED (deploy actuation)
 * @note Bench-validated 2026-08-27 (test/bench `s`, physical ejection test):
 *       135° ejects the parachute; the test starts at SERVO_CLOSED (50).
 */
const int SERVO_OPEN = 135;

/**
 * Servo position: door held closed (parachute retained)
 * @note Bench-validated 2026-08-27. Firmware keeps the servo at this
 *       position from boot until apogee deploy.
 */
const int SERVO_CLOSED = 50;

//==============================================================================
// TEAM IDENTIFICATION
//==============================================================================

/**
 * Team unique identifier
 * Used as prefix in all telemetry transmissions
 * Allows identifying data from different teams
 */
constexpr const char* TEAM_ID = "#51";

//==============================================================================
// FILESYSTEM CONFIGURATION
//==============================================================================

/**
 * CSV data file name
 * Stores all telemetry readings
 */
extern String file_name;

/**
 * Complete data file path
 * Will be filled during setup() with GPS timestamp
 * Format: /HH_MM_SS-Dados.csv
 */
extern String file_dir;


static constexpr float LIFTOFF_ACCEL_THRESHOLD  = 15.0f;  ///< m/s²  total accel
static constexpr uint16_t LIFTOFF_CONFIRM_MS    = 100;    ///< ms  accel must stay above LIFTOFF_ACCEL_THRESHOLD (per-time confirmation; 5 cycles @50Hz — rejects bench/hand vibration spikes; burn sustains >15 m/s² for 1.2-19.6s in both missions)
static constexpr uint16_t LIFTOFF_CONFIRM_MAX_GAP_MS = 60; ///< ms  max gap below threshold that does NOT reset the accumulator (tolerates single stale IMU frames)
static constexpr float LIFTOFF_MIN_HEIGHT       =  5.0f;  ///< m     height guard for liftoff
static constexpr uint16_t LIFTOFF_ALT_CONFIRM_CYCLES = 3; ///< consecutive cycles above LIFTOFF_MIN_HEIGHT (60ms @50Hz — a bench pressure puff is 1-2 samples; a real ascent crosses 5m climbing)
static constexpr float BARO_MAX_ALT_RATE        = 200.0f; ///< m/s   |dAlt/dt| above this = sample is a pressure spike (hand shake / EMI), discarded; matches the Vz clip bound
static constexpr uint16_t BARO_SPIKE_STREAK_RESEED = 3;    ///< consecutive rejected samples (60ms @50Hz) before re-seeding the reference — prevents the ratchet effect where one accepted glitch latches the altitude high forever (every return-to-zero then looks like a spike)
static constexpr float BURNOUT_AZ_THRESHOLD     = -8.0f;  ///< m/s²  vertical accel
static constexpr float BURNOUT_ACC_THRESHOLD    =  2.0f;  ///< m/s²  total accel
static constexpr float BURNOUT_MIN_HEIGHT       =  5.0f;  ///< m     minimum altitude
static constexpr float BURNOUT_MIN_VZ           =  0.5f;  ///< m/s   minimum climb speed
static constexpr float APOGEE_MAX_VZ            =  1.0f;  ///< m/s   |vz| below this (single cycle)
// Az gate removed 2026-08-05 (risk #5): apparent accel includes pendulum
// terms + sensor bias; the real flight had only ~0.81 m/s² margin and lost
// the apogee under >=1.1 m/s² pendulum motion. vz-only is immune because vz
// comes from the barometer, not the accelerometer. See
// extras/FSM_tester/analyze_apogee_robustness.py.
static constexpr float FREEFALL_ACC_THRESHOLD   = 11.5f;  ///< m/s²  total accel
static constexpr float FREEFALL_MIN_HEIGHT      =  5.0f;  ///< m     minimum altitude
static constexpr float FREEFALL_MAX_VZ          = -5.0f;  ///< m/s   vz must be below this

// ── Free-fall backstop (FSM-independent safety net) ─────────────────────────
// Detects a real free fall WITHOUT relying on the FSM state. Runs in the
// FlightControlTask loop; fires only when total accel (IIR-filtered) stays
// below near-zero-g for a full second WHILE descending fast and well above
// the ground guard. The vz < -5 m/s condition is what separates real descent
// from burnout/coasting (where vz is still positive) — without it the chute
// would deploy on ascent right after motor cutoff (accel dips to ~0).
// Sized with real flight data: zero-g windows last 8-131s, pad vibration
// spikes (22-122 m/s²) are transient so the 1s window rejects them.
static constexpr float   FREEFALL_BACKSTOP_ACC_THRESHOLD = 3.0f;    ///< m/s²  near zero-g (≈0.3g)
static constexpr float   FREEFALL_BACKSTOP_VZ            = -5.0f;   ///< m/s   must be descending this fast
static constexpr float FREEFALL_BACKSTOP_MIN_HEIGHT   = 50.0f;  ///< m     ground guard (same as PARACHUTE_MIN_ALTITUDE)
static constexpr uint16_t FREEFALL_BACKSTOP_CYCLES     = 50;     ///< ~1.0s at 50Hz (FLIGHT_CONTROL_PERIOD_MS=20ms)

// ── Barometer-staleness contingency (IMU-only, FSM-independent) ─────────────
// If the barometer freezes mid-flight (I2C glitch, bad solder, EMI), both the
// FSM and the free-fall backstop lose vz/height — neither would deploy and the
// frozen values are plausible, so no NaN check catches it. This contingency
// detects a sustained IMU-only free fall once the flight actually started
// (accel > 15 m/s² seen at least once since boot) and the rocket climbed above
// the ground guard (maxAltitude from the last good baro reading). The window
// is longer than the backstop (2.5s vs 1.0s) because there is no vz<0 gate
// nor a live height check without the barometer.
static constexpr uint32_t BARO_STALE_AGE_MS          = 2000;  ///< ms without a valid baro reading => frozen
static constexpr float    BARO_STALE_ACC_THRESHOLD   = 3.0f;  ///< m/s²  near zero-g (same as backstop)
static constexpr float    BARO_STALE_MIN_HEIGHT      = 50.0f; ///< m     ground guard via last-good maxAltitude
static constexpr uint16_t BARO_STALE_SUSTAIN_CYCLES  = 125;   ///< 2.5s @ 50Hz (FLIGHT_CONTROL_PERIOD_MS=20ms)

// ── Pad arming (risk #2) ─────────────────────────────────────────────────────
// Bench vibration (13_30_11: spikes 22-122 m/s²) can false-liftoff the FSM
// into ASCENT; a reboot then restores ASCENT from NVS and the FSM lands on
// the pad without ever deploying. An explicit ARM command on the pad clears
// the NVS snapshot, re-captures base_pressure from the current reading and
// zeroes maxAltitude. Complement: auto re-zero of base_pressure on the pad
// when the relative altitude drifts below the threshold (1 hPa ~ 8.4 m).
static constexpr float    ARM_MAX_ARM_ALTITUDE    = 10.0f;  ///< m   refuse ARM once the flight really started
static constexpr float    ARM_REZERO_THRESHOLD    = -10.0f; ///< m   baro drift guard on the pad (relative alt)
static constexpr uint16_t ARM_REZERO_SUSTAIN_CYCLES = 150;  ///< 3.0s @ 50Hz
static constexpr float PARACHUTE_MIN_ALTITUDE    = 50.0f;  ///< m  minimum altitude (ground guard — never deploy below)
static constexpr float PARACHUTE_CONFIRM_VZ      = -2.0f;  ///< m/s negative Vz required to confirm descent after apogee
static constexpr uint8_t PARACHUTE_CONFIRM_CYCLES = 3;     ///< consecutive cycles of (vz < CONFIRM_VZ) before deploy
static constexpr float LANDED_MAX_VZ            =  0.5f;  ///< m/s   |vz| below this
static constexpr float LANDED_MAX_HEIGHT        =  2.0f;  ///< m     altitude below this
static constexpr float FILTER_ALPHA             =  0.2f;  ///< IIR low-pass coefficient

// ── Stuck-state backstop (time-in-state) ─────────────────────────────────────
// DESCENT never confirming LANDED (baro drift keeps height above the ground
// guard, or vz noise) would hang the FSM forever. After the timeout at rest
// (no thrust, no vertical motion), force LANDED — the parachute decision was
// already made, so there is no safety loss. LANDED itself remains terminal
// until a reboot or an explicit reset().
static constexpr uint32_t DESCENT_TIMEOUT_MS   = 120000UL; ///< 2 min in DESCENT -> force LANDED
static constexpr float    STUCK_REST_MAX_VZ    = 1.0f;    ///< m/s  |vz| guard for the backstop
static constexpr float    STUCK_REST_MAX_ACC   = 15.0f;   ///< m/s² accel guard (no motor/thrust active)

// ── Baro glitch recovery (IDLE only) ─────────────────────────────────────────
// Bench observation: the BMP585 can lose its configuration (suspected chip
// reset from a supply glitch, e.g. LoRa TX burst) and afterwards read a
// SELF-CONSISTENT but wrong pressure — alt jumps to ~2400 m on the table with
// vz ~ 0. On the pad at rest that is impossible, so after the sustain window
// reinit the sensor and re-capture the base pressure. IDLE only: in flight a
// sustained altitude with low apparent accel can be real (coasting).
static constexpr float    BARO_GLITCH_ALTITUDE       = 50.0f; ///< m   alt at rest above this on the pad = sensor fault
static constexpr uint16_t BARO_GLITCH_SUSTAIN_CYCLES = 50;    ///< ~1.0s @ 50Hz sustained before reinit

// ── Baro base-pressure calibration (boot / reinit) ───────────────────────────
// A single corrupted pressure sample at boot was observed to seed
// base_pressure ~1305 hPa (real pad pressure ~1013 hPa), making the altitude
// read ~+2400 m permanently. The median of N range-validated samples is
// immune to isolated corruption.
static constexpr uint8_t  FIRST_READ_SAMPLES          = 9;   ///< samples taken for the median
static constexpr uint8_t  FIRST_READ_MIN_VALID        = 5;   ///< fewer valid samples => calibration fails
static constexpr uint8_t  FIRST_READ_MAX_ATTEMPTS     = 30;  ///< total read attempts before giving up
static constexpr uint16_t FIRST_READ_SAMPLE_PERIOD_MS = 20;  ///< between calibration samples

#endif // CONFIG_H
