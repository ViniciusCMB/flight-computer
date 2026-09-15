# AGENTS.md - Flight Computer Development Guide

This guide is for AI coding agents working on the Flight Computer project (#11 - Serra Rocketry). It contains build commands, code style guidelines, and development workflows.

## Project Context

- **Hardware**: ESP32-S3 (primary), ESP32-C3 SuperMini (legacy)
- **Language**: C/C++ (Arduino framework)
- **Architecture**: v2.0 (OOP + FreeRTOS + FSM - implemented), v1.0 (procedural, EOL)
|- **Key Docs**: `docs/architecture.md`, `docs/modules.md`, `docs/software.md`, `docs/hardware.md`, `CONTRIBUTING.md`

---

## Build, Test, and Lint Commands

### Compile Firmware (Arduino IDE)
```bash
# Open firmware/firmware.ino in Arduino IDE
# Board: ESP32-S3 Dev Module (primary) or ESP32-C3 Dev Module (legacy)
# Verify/Compile: Ctrl+R or Sketch → Verify/Compile
# Upload: Ctrl+U or Sketch → Upload
```

### Test Commands

#### Run FSM Hardware Test
```bash
# Flash test firmware to ESP32
cd test/FSM
# Open FSM.ino in Arduino IDE and upload

# Optional: Feed test data via Python
cd extras/FSM_tester
python flight_inserter.py
```

#### Run FSM Python Simulator (Validation)
```bash
cd extras/FSM_tester
python FSM_Tester.py
# Uses real flight data from 13_30_11-Dados.csv (1,873 points)
# Validates FSM state transitions and thresholds
```

#### Run Single Test
```bash
# For Arduino tests: Open specific test .ino file and upload
# Example: test/FSM/FSM.ino

# For Python tests: Run specific file
python extras/FSM_tester/FSM_Tester.py
```

### Linting (Manual)
```bash
# No automated linting configured yet
# Manual checks:
# - Arduino IDE: Check for compiler warnings (Tools → Preferences → Show verbose output)
# - Follow code style guidelines below
```

---

## Code Style Guidelines

### General Principles
- **Indentation**: 2 spaces (Arduino IDE standard)
- **Line Length**: Aim for 80 characters, max 120
- **Comments**: English only
- **File Encoding**: UTF-8

### Naming Conventions

#### Variables
```cpp
// Global variables: snake_case
float base_pressure = 1013.25;
volatile int sensor_value = 0;

// Local variables: camelCase
float currentAltitude = 0;
int loopCounter = 0;
```

#### Functions
```cpp
// Functions: camelCase
void setupSensors() { }
float calculateAltitude(float pressure) { }
bool detectLiftoff() { }
```

#### Constants
```cpp
// Constants: UPPER_CASE
const int MAX_ALTITUDE = 50000;
const float LIFTOFF_THRESHOLD = 15.0;  // m/s²

#define LORA_FREQ 915E6
```

#### Classes (v2.0)
```cpp
// Classes: PascalCase
class BMP585Sensor { };
class FlightStateMachine { };

// Private members: _camelCase
class Sensor {
private:
  float _lastReading;
  bool _isInitialized;
};
```

### File Structure

#### Header Files
```cpp
/**
 * @file module_name.h
 * @brief Brief description of module purpose
 * 
 * Detailed description of what this module does,
 * its responsibilities, and how it fits in the system.
 * 
 * @author Team #100
 * @date 2026
 */

#ifndef MODULE_NAME_H
#define MODULE_NAME_H

// Includes (Arduino libs first, then third-party)
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP5xx.h>

// Global variables (documented)

// Function declarations

#endif  // MODULE_NAME_H
```

### Imports/Includes Order
1. Arduino core (`<Arduino.h>`)
2. Standard libraries (`<Wire.h>`, `<SPI.h>`)
3. Third-party libraries (`<Adafruit_BMP5xx.h>`)
4. Project headers (`"config.h"`)

### Function Documentation (Doxygen Style)
```cpp
/**
 * @brief Calculate altitude from pressure reading
 * 
 * Uses international barometric formula to convert
 * atmospheric pressure to altitude above sea level.
 * 
 * @param pressure Current pressure in hPa
 * @param basePressure Reference sea-level pressure in hPa
 * @return float Altitude in meters
 * 
 * @note Accuracy decreases above 10,000m
 * @warning Pressure must be > 0 and < 1200 hPa
 */
float calculateAltitude(float pressure, float basePressure) {
  // Validate inputs
  if (pressure <= 0 || basePressure <= 0) {
    return NAN;
  }
  
  // International barometric formula
  return 44330.0 * (1.0 - pow(pressure / basePressure, 0.1903));
}
```

### Error Handling

#### Sensor Initialization
```cpp
bool setupSensor() {
  if (!sensor.begin()) {
    Serial.println("❌ Sensor initialization failed");
    return false;
  }
  return true;
}
```

#### Data Validation
```cpp
// Always validate sensor readings
float altitude = bmp.readAltitude(base_pressure);
if (isnan(altitude) || altitude < -500 || altitude > 50000) {
  Serial.println("⚠️ Invalid altitude reading");
  altitude = previous_altitude;  // Use last known good value
}
```

#### Memory Safety
```cpp
// Use snprintf instead of sprintf
char buffer[64];
snprintf(buffer, sizeof(buffer), "Altitude: %.2f m", altitude);

// Check array bounds
if (index >= 0 && index < ARRAY_SIZE) {
  data[index] = value;
}
```

### Formatting

#### Braces
```cpp
// Opening brace on same line (K&R style)
if (condition) {
  // code
} else {
  // code
}

void function() {
  // code
}
```

#### Spacing
```cpp
// Space after keywords, around operators
if (x > 0) {
  y = x + 1;
}

// No space before function parentheses
void setupBMP() {
  sensor.begin();
}

// Space after commas
float calculate(float a, float b, float c);
```

---

## Development Workflow

### Before Implementing
1. Read `docs/architecture.md` for v2.0 architecture
2. Check `CONTRIBUTING.md` for project guidelines
3. Review existing similar code in `firmware/` modules

### Implementation Checklist
- [ ] Follow naming conventions (snake_case vars, camelCase functions)
- [ ] Add Doxygen comments to public functions
- [ ] Validate all inputs (NaN, range checks)
- [ ] Use `config.h` for constants (no magic numbers)
- [ ] Test on hardware if possible
- [ ] Update documentation if behavior changes

### Safety-Critical Code (Parachute Logic)
```cpp
// ❌ WRONG: Single condition
if (altitude < 750) {
  deployParachute();
}

// ✅ CORRECT: Multi-condition with state validation
if (data.parachute_deployed && 
    !g_parachuteActuated) {
  deployParachute();
  g_parachuteActuated = true;
  logMessage(TASK_ID_FLIGHT_CONTROL, LOG_LEVEL_WARN, "Parachute deployed");
  Serial.println("🪂 PARACHUTE DEPLOYED");
}
```

### FreeRTOS Guidelines (v2.0)
- **Task naming**: `task` + `PascalCase` (e.g., `taskFlightControl`)
- **Priorities**: FlightControl=20, Telemetry=5, Logger=1
- **Mutexes**: Protect shared variables (`maxAltitude`, etc.)
- **Queues**: Use for data passing between tasks
- **Watchdog**: Feed in critical tasks only

---

## Key Files Reference

- `firmware/config.h` - Pin definitions, thresholds, constants
|- `docs/architecture.md` - Complete v2.0 architecture spec (consolidated from firmware/REFACTORING_PLAN.md)
- `extras/FSM_tester/FSM_Tester.py` - FSM validation with real data
- `extras/FSM_tester/explicacao.md` - FSM detailed explanation (541 lines)
- `test/FSM/FSM.ino` - Hardware FSM test (322 lines)
- `CONTRIBUTING.md` - Code standards and PR process

---

## Related Documentation

### Organizational Workflows & Team Structure
- `.opencode/README.md` - Skills overview and workflows
- `.opencode/TEAM.md` - Complete team structure and RACI matrix
- `.opencode/opencode.yaml` - OpenCode configuration and skill definitions
- `.opencode/skills/` - Detailed specialist guides (6 skills available)
  - `embedded-architect/SKILL.md` - System architecture specialist
  - `firmware-developer/SKILL.md` - Firmware implementation expert
  - `fsm-specialist/SKILL.md` - Flight state machine specialist
  - `code-reviewer/SKILL.md` - Safety-critical code review
  - `test-engineer/SKILL.md` - Testing and validation expert
  - `documentation-specialist/SKILL.md` - Technical documentation expert

### Project Documentation
|- `CONTRIBUTING.md` - Contribution guidelines and PR process
|- `docs/architecture.md` - v2.0 architecture specification (consolidated from REFACTORING_PLAN.md)
|- `docs/modules.md` - Module reference (moved from firmware/MODULOS.md)
|- `docs/software.md` - Software architecture overview
|- `docs/hardware.md` - Hardware specifications

---

**Remember**: This is safety-critical aerospace code. Validate all sensor data, check edge cases, and prioritize deterministic behavior over clever optimizations.
