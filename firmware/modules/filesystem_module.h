/**
 * @file filesystem_module.h
 * @brief Storage abstraction layer with SD card + LittleFS fallback
 *
 * Provides transparent data logging with automatic fallback:
 * 1. Primary: SD card via SPI
 * 2. Fallback: LittleFS (internal flash)
 *
 * All file operations dispatch through storage_type at runtime,
 * so application code never needs to know which backend is active.
 *
 * @author Team #100
 * @date 2026
 */

#ifndef FILESYSTEM_MODULE_H
#define FILESYSTEM_MODULE_H

#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "LittleFS.h"
#include "config.h"

//==============================================================================
// STORAGE TYPE ENUM
//==============================================================================

enum StorageType { STORAGE_NONE, STORAGE_SD, STORAGE_LITTLEFS };

// C++17 inline global — guaranteed single instance across all TUs
inline StorageType g_storage_type = STORAGE_NONE;

//==============================================================================
// INITIALIZATION FUNCTIONS
//==============================================================================

/**
 * Initialize storage subsystem — SD card preferred, LittleFS fallback
 *
 * Tenta SD primeiro, se falhar usa LittleFS (internal flash).
 * Requer que o barramento SPI ja tenha sido configurado com os pinos do
 * LoRa (LORA_SCK/MISO/MOSI) — initTelemetryTask() chama SPI.begin() antes
 * de setupStorage()
 * pois SD.begin() usa o objeto SPI global. Sem isso, o SD e' procurado nos
 * pinos SPI default do ESP32-S3 e cai sempre no LittleFS.
 *
 * @return true se SD ou LittleFS foi montado com sucesso
 * @return false se ambos falharam (dados serao apenas Serial)
 */
inline bool setupStorage()
{
  // Try SD card first — uses the same SPI bus as LoRa (SCK=4, MISO=2, MOSI=3)
  Serial.println("[FS] Initializing SD card...");
  if (SD.begin(SD_CS_PIN) && SD.cardType() != CARD_NONE)
  {
    g_storage_type = STORAGE_SD;
    Serial.printf("[FS] SD card OK — type: %s | size: %llu MB\n",
      SD.cardType() == CARD_MMC  ? "MMC" :
      SD.cardType() == CARD_SD   ? "SDSC" :
      SD.cardType() == CARD_SDHC ? "SDHC" : "UNKNOWN",
      SD.cardSize() * 512ULL / 1048576ULL);
    return true;
  }
  Serial.println("[FS] SD card failed. Falling back to LittleFS...");

  // Fallback to LittleFS (internal flash, auto-format if needed)
  if (LittleFS.begin(true))
  {
    g_storage_type = STORAGE_LITTLEFS;
    Serial.printf("[FS] LittleFS OK — %u bytes of %u bytes free\n",
      (unsigned)(LittleFS.totalBytes() - LittleFS.usedBytes()),
      (unsigned)LittleFS.totalBytes());
    return true;
  }

  g_storage_type = STORAGE_NONE;
  Serial.println("[FS] ERROR: No storage available — continuing without file logging");
  return false;
}

//==============================================================================
// FILE OPERATIONS
//==============================================================================

/**
 * Write data to file (creates new or overwrites existing)
 *
 * Dispatches to the active storage backend. Use for writing CSV headers.
 */
inline bool writeFile(const String &path, const String &data_string)
{
  File file;

  if (g_storage_type == STORAGE_SD)
  {
    file = SD.open(path, FILE_WRITE);
  }
  else if (g_storage_type == STORAGE_LITTLEFS)
  {
    file = LittleFS.open(path, FILE_WRITE);
  }
  else
  {
    return false;
  }

  if (!file)
  {
    Serial.println("[FS] Failed to open file for writing.");
    return false;
  }

  if (file.println(data_string))
  {
    Serial.println("[FS] File written.");
    file.close();
    return true;
  }

  Serial.println("[FS] File write failed.");
  file.close();
  return false;
}

/**
 * Append data to existing file
 *
 * Dispatches to the active storage backend. Creates file if it doesn't exist.
 * Each call opens, appends, and closes the file.
 */
inline void appendFile(const String &path, const String &message)
{
  File file;

  if (g_storage_type == STORAGE_SD)
  {
    file = SD.open(path, FILE_APPEND);
  }
  else if (g_storage_type == STORAGE_LITTLEFS)
  {
    file = LittleFS.open(path, FILE_APPEND);
  }
  else
  {
    return;
  }

  if (!file)
  {
    Serial.println("[FS] Failed to open file for appending.");
    return;
  }

  if (!file.println(message))
  {
    Serial.println("[FS] Failed to append message.");
  }

  file.close();
}

//==============================================================================
// HELPER FUNCTIONS
//==============================================================================

/**
 * @return StorageType currently in use
 */
inline StorageType getStorageType()
{
  return g_storage_type;
}

/**
 * @return true if any storage backend is available
 */
inline bool isStorageReady()
{
  return g_storage_type != STORAGE_NONE;
}

/**
 * @return Human-readable storage name for logging
 */
inline const char* getStorageName()
{
  switch (g_storage_type)
  {
    case STORAGE_SD:       return "SD";
    case STORAGE_LITTLEFS: return "LittleFS";
    default:               return "NONE";
  }
}

#endif // FILESYSTEM_MODULE_H
