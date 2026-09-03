/**
 * @file lora_module.h
 * @brief LoRa wireless communication module
 * 
 * This module manages long-range wireless communication using LoRa technology.
 * LoRa (Long Range) provides:
 * - Long-distance communication (several kilometers)
 * - Low power consumption
 * - Good penetration through obstacles
 * - Resilient to interference
 * 
 * Used for:
 * - Real-time telemetry transmission during flight
 * - Ground station communication
 * - Remote monitoring and tracking
 * 
 * Communication: SPI bus
 * Frequency: 915 MHz (Americas/Brazil) - configured in config.h (matches receiver-lora)
 * 
 * @author #11
 * @date 2026
 */

#ifndef LORA_MODULE_H
#define LORA_MODULE_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "config.h"

// Set to 1 to enable verbose LoRa transmission logs.
#ifndef LORA_DEBUG_LOGS
#define LORA_DEBUG_LOGS 0
#endif

//==============================================================================
// AVAILABILITY FLAG
//==============================================================================

// True only after setupLoRa() succeeded. Guards every transmission so
// callers (telemetry, error logging) can send unconditionally without
// touching an uninitialized radio.
inline bool& loraAvailabilityRef() {
  static bool loraAvailable = false;
  return loraAvailable;
}

inline void markLoRaAvailable(bool available) { loraAvailabilityRef() = available; }
inline bool isLoRaAvailable() { return loraAvailabilityRef(); }

//==============================================================================
// INITIALIZATION FUNCTIONS
//==============================================================================

/**
 * Initialize LoRa communication module
 * 
 * Configures the LoRa module with pins and operating parameters.
 * Sets up SPI communication and initializes the radio at specified frequency.
 * 
 * Configuration:
 * - Frequency: Defined in config.h (LORA_FREQ)
 * - Sync word: 0xF3 (ensures only matching devices communicate)
 * - SPI pins: SS, RST, DIO0 defined in config.h
 * 
 * @return true if initialization was successful
 * @return false if module not found or communication failed
 * 
 * @note Ensure correct frequency for your region (868MHz EU, 915MHz Americas)
 * @warning Operating on wrong frequency may violate local regulations
 * @see config.h for pin and frequency configuration
 */
inline bool setupLoRa()
{
  // Remap the SPI bus to the RFM95W wiring (same as receiver-lora firmware).
  // The LoRa 0.8.0 lib uses the global SPI object, so we must call SPI.begin()
  // with the custom pins BEFORE LoRa.setPins().
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, SS_LORA);

  // Configure SPI pins for LoRa module
  LoRa.setPins(SS_LORA, RST_LORA, DIO0_LORA);
  
  // Initialize LoRa at configured frequency
  if (!LoRa.begin(LORA_FREQ))
  {
    Serial.println("LoRa initialization failed.");
    markLoRaAvailable(false);
    return false;
  }
  
  // Set synchronization word (network ID)
  // Only devices with same sync word can communicate
  LoRa.setSyncWord(SYNC_WORD);

  // Explicitly match the receiver (recovery-webui/components/receiver-lora)
  // so the link connects. These equal the LoRa.h defaults, but we set them
  // explicitly to avoid relying on library defaults.
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.enableCrc();

  markLoRaAvailable(true);
  return true;
}

//==============================================================================
// TRANSMISSION FUNCTIONS
//==============================================================================

/**
 * Send message via LoRa wireless communication
 * 
 * Transmits a text message over LoRa radio. The function:
 * 1. Begins a new packet
 * 2. Writes message content
 * 3. Ends packet and transmits
 * 4. Reports transmission status
 * 
 * @param message String message to transmit via LoRa
 * 
 * @note Maximum packet size depends on LoRa configuration (typically 255 bytes)
 * @note Transmission time increases with message length
 * @note Function blocks until transmission completes
 * @note TelemetryTask builds the message with snprintf and transmits via
 *       sendLoRa() + Serial (telemetry_module.h was removed in v2.0)
 */
inline void sendLoRa(const String &message)
{
  // No-op if the radio never initialized (setupLoRa() failed or was skipped)
  if (!isLoRaAvailable())
  {
    return;
  }

  // Start a new LoRa packet
  LoRa.beginPacket();
  
  // Write message to packet buffer
  LoRa.print(message);
  
  // Finalize and transmit packet
  if (LoRa.endPacket())
  {
    #if LORA_DEBUG_LOGS
    Serial.println("LoRa message sent.");
    #endif
  }
  else
  {
    // Transmission failed (unlikely if initialization succeeded)
    Serial.println("ERROR sending LoRa message!");
  }
}

#endif // LORA_MODULE_H
