# Plumbing Board Safety Features

## High Humidity Leak Detection

### Overview
The board monitors the BME280 humidity sensor for sudden spikes that may indicate a water leak in the RV. If humidity rises rapidly by 15% or more within 60 seconds, the system triggers an emergency shutdown.

### Trigger Conditions
- **Threshold**: +15% humidity increase
- **Time Window**: Within 60 seconds
- **Sensor**: BME280 environmental sensor

### Emergency Actions
When high humidity alarm triggers, the system automatically:

1. **Stops the hot water pump** immediately
2. **Closes all loop valves**:
   - Rear loop valve → CLOSED
   - Engine loop valve → CLOSED (saves to EEPROM)
   - Front loop valve → CLOSED (saves to EEPROM)
   - Grey water heater valve → CLOSED
3. **Publishes MQTT alerts**:
   - `AlertHighHumidity` = "ACTIVE - High Humidity, Possible Leak"
   - `HotWaterPumpStatus` = "ALARM_HIGH_HUMIDITY"
   - `PlumbingSystemStatus` = "LEAK_DETECTED_SHUTDOWN"

### Lockout Behavior
- **Pump cannot restart** - All auto-start and manual start attempts are blocked
- **System stays locked** - Alarm persists until manually cleared via MQTT
- **Serial output** - Instructions displayed for clearing the alarm

### Clearing the Alarm
To reset the system after investigating the leak:

**MQTT Command:**
```
Topic: ClearHighHumidityCommand
Payload: clear  (or CLEAR, true, TRUE)
```

**What happens when cleared:**
- Alarm flag is reset
- Humidity baseline is updated to current reading
- Pump can be restarted (manual or auto mode)
- MQTT status published: `AlertHighHumidity` = "CLEARED"

---

## High Pressure Safety

### Overview
Monitors pump pressure continuously and triggers emergency shutdown if pressure exceeds safe limits.

### Pressure Limits
- **Low Pressure Alarm**: < 2.0 PSI (pump may be running dry)
- **High Pressure Alarm**: > 40.0 PSI (excessive pressure buildup)
- **Monitoring Interval**: Every 100ms

### High Pressure Response
When pressure exceeds 40 PSI:

1. **Stops pump immediately**
2. **Opens DHW (Domestic Hot Water) solenoid valve** for emergency pressure relief
3. **Activates pressure relief timer** (2 second duration)
4. **Publishes alerts**:
   - `AlertPumpError` = "PRESSURE_HIGH"
   - `HotWaterPumpStatus` = "ALARM_HIGH_PRESSURE"

### Pressure Relief Operation
- DHW valve opens automatically to release trapped pressure
- Remains open for 2 seconds
- Closes automatically after relief period (unless flow is active)

### Low Pressure Response
When pressure drops below 2 PSI while pump is running:

1. **Stops pump immediately** (possible dry run condition)
2. **Publishes alerts**:
   - `AlertPumpError` = "PRESSURE_LOW"
   - `HotWaterPumpStatus` = "ALARM_LOW_PRESSURE"

### Auto-Clear
Pressure alarms automatically clear when pressure returns to safe range (2-40 PSI) and pump is stopped.

---

## Safety Monitoring Topics

### Subscribe (Commands)
- `ClearHighHumidityCommand` - Clear high humidity leak alarm

### Publish (Alerts)
- `AlertHighHumidity` - High humidity leak alarm status
- `AlertPumpError` - Pump pressure alarm status
- `HotWaterPumpStatus` - Detailed pump status including alarms
- `PlumbingSystemStatus` - Overall system status

---

## Testing & Troubleshooting

### Testing High Humidity Alarm
⚠️ **Only test in controlled conditions**

1. Monitor current humidity: Subscribe to `EnvironmentHumidity`
2. Create rapid humidity increase (careful - may trigger real alarm!)
3. Verify system shutdown occurs
4. Clear alarm: Publish `clear` to `ClearHighHumidityCommand`
5. Verify normal operation resumes

### Testing High Pressure Safety
⚠️ **Only test with pressure gauge monitoring**

1. Close all loop valves except one
2. Start pump manually
3. Monitor pressure via `HotWaterPumpPressure`
4. System should stop pump and open DHW valve at threshold
5. Pressure should drop via relief valve

### False Alarm Prevention
**High Humidity:**
- 15% threshold prevents triggering from normal humidity changes
- 60-second window filters slow environmental changes
- Requires BME280 sensor to be functioning

**High Pressure:**
- Threshold set at 40 PSI (well above normal operating pressure)
- DHW relief valve provides automatic pressure release
- Pump stops immediately to prevent damage

---

## Implementation Notes

### Code Location
`Plumbing/src/main_esp32s2.cpp`

### Key Variables
```cpp
// Humidity Detection
bool highHumidityAlarmActive = false;
float lastHumidity = -999.0;
const float HUMIDITY_SPIKE_THRESHOLD = 15.0;  // % change
const unsigned long HUMIDITY_RATE_WINDOW = 60000;  // 1 minute

// Pressure Limits
const float PRESSURE_MIN_ALARM = 2.0;   // PSI
const float PRESSURE_MAX_ALARM = 40.0;  // PSI
```

### Firmware Version
Added in: v2.0.0-ESP32S2

### Dependencies
- BME280 sensor (for humidity monitoring)
- Pressure sensor on ADC1_CH0 (GPIO1)
- DHW solenoid valve control (GPIO10)
- MQTT connection for alerts and clear commands

---

## Safety Philosophy

These features follow a **fail-safe approach**:

1. ✅ **Immediate shutdown** - No delays when hazards detected
2. ✅ **Multiple protection layers** - Sensor monitoring + pressure relief + valve closure
3. ✅ **Manual recovery** - Requires human verification before restart
4. ✅ **Clear diagnostics** - Detailed MQTT alerts and serial output
5. ✅ **State persistence** - Valve positions saved to EEPROM during emergency

The system prioritizes **preventing water damage** over convenience. All safety alarms require manual intervention to ensure issues are investigated before resuming operation.
