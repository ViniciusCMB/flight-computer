# System Flowchart — Flight Computer v2.0

FreeRTOS multi-task architecture with 4-state FSM.

```mermaid
graph TB
    subgraph "Core 1 — FlightControl Task (5 Hz, Priority 20)"
        FC_START([Start Cycle]) --> FC_SENSORS[Update BMP585 + LSM6DS3]
        FC_SENSORS --> FC_FSM[Run FlightStateMachine.update]
        FC_FSM --> FC_ACTUATE{Parachute<br>deployed?}
        FC_ACTUATE -- Yes & not actuated --> FC_SERVO[deployParachute<br>Servo.write OPEN]
        FC_ACTUATE -- No / already done --> FC_QUEUE[push SensorData<br>→ sensorDataQueue]
        FC_SERVO --> FC_QUEUE
        FC_QUEUE --> FC_WDT[esp_task_wdt_reset]
        FC_WDT --> FC_METRICS[Update exec stats]
        FC_METRICS --> FC_END([vTaskDelayUntil +20ms])
    end

    subgraph "Core 0 — Telemetry Task (5Hz, Priority 5)"
        TEL_START([Start Cycle]) --> TEL_GPS[GPSModule.update<br>non-blocking NMEA]
        TEL_GPS --> TEL_QUEUE[Receive SensorData<br>from sensorDataQueue]
        TEL_QUEUE --> TEL_ENRICH[Enrich with GPS fix]
        TEL_ENRICH --> TEL_ASSEMBLE[assembleTelemetry<br>22-field CSV]
        TEL_ASSEMBLE --> TEL_FANOUT{Multi-channel}
        TEL_FANOUT --> TEL_SERIAL[Serial.println]
        TEL_FANOUT --> TEL_LORA[sendLoRa]
        TEL_FANOUT --> TEL_FILE[appendFile<br>SD / LittleFS]
        TEL_SERIAL --> TEL_END([vTaskDelayUntil +200ms])
        TEL_LORA --> TEL_END
        TEL_FILE --> TEL_END
    end

    subgraph "Core 0 — Logger Task (Low Priority)"
        LOG_START([Wait for log]) --> LOG_RECV[Receive LogMessage<br>from logQueue]
        LOG_RECV --> LOG_FILTER{Level >=<br>minLevel?}
        LOG_FILTER -- Yes --> LOG_PRINT[Serial.printf]
        LOG_FILTER -- No --> LOG_START
        LOG_PRINT --> LOG_START
    end

    subgraph "4-State FSM"
        direction LR
        IDLE -->|detectLiftoff| ASCENT
        ASCENT -->|detectApogee| DESCENT
        DESCENT -->|detectLanded| LANDED
    end

    subgraph "FSM Sub-Events (boolean flags)"
        direction TB
        LIFTOFF[Liftoff: totalAccel > 15 m/s²]
        BURNOUT[Burnout: az < -8 OR acc < 2]
        APOGEE[Apogee: |vz| < 1 AND az < -0.1]
        FREEFALL[Freefall: acc < 11.5 AND vz < -5]
        PARACHUTE[Parachute: height > 50m<br>AND vz < -2 for 3 cycles]
    end

    FC_QUEUE -->|sensorDataQueue| TEL_QUEUE

    style FC_START fill:#f96,stroke:#333,stroke-width:2px
    style FC_ACTUATE fill:#ff6,stroke:#333,stroke-width:2px
    style FC_SERVO fill:#f96,stroke:#333,stroke-width:2px
    style TEL_START fill:#9cf,stroke:#333,stroke-width:2px
    style LOG_START fill:#9f9,stroke:#333,stroke-width:2px
```

## Key Parameters

| Parameter | Value |
|-----------|-------|
| FlightControl period | 200 ms (5 Hz) |
|| Telemetry period | 200 ms (5 Hz) |
| Parachute confirm cycles | 3 consecutive |
| Parachute ground guard | 50 m AGL |
| Sensor data queue | 25 slots |
| Log queue | 50 slots |

## Parachute Deployment

1. FSM detects apogee: `|vz| < 1 m/s AND az < -0.1 m/s²`
2. FSM transitions to `DESCENT`
3. `detectParachute` waits for `vz < -2 m/s` for 3 consecutive cycles (600 ms @5Hz)
4. FlightControlTask actuates servo via `deployParachute()`
5. One-shot: `g_parachuteActuated` flag prevents re-actuation
