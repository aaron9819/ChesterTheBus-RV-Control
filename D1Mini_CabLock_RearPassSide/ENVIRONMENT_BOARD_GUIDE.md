# D1 Mini Environmental & Hot Water Controller

## Board Overview
This D1 Mini board is a multi-function controller that handles:
- **Cabinet Lock Control** - Servo-based locking mechanism
- **Environmental Monitoring** - BME280 sensor (temperature, humidity, pressure)
- **Water Flow Detection** - Hall Effect flow sensor
- **Hot Water Control** - Solenoid valve automation
- **OTA Updates** - Wireless firmware updates

## Hardware Configuration

### Pin Assignments
| Pin | GPIO | Function | Hardware |
|-----|------|----------|----------|
| D1 | GPIO5 | I2C SCL | BME280 Clock |
| D2 | GPIO4 | I2C SDA | BME280 Data |
| D4 | GPIO2 | PWM | Cabinet Lock Servo |
| D5 | GPIO14 | Digital Out | Hot Water Solenoid Relay |
| D8 | GPIO15 | Digital In | Flow Sensor (Interrupt) |

### Components Required
1. **BME280 Sensor** - I2C environmental sensor (0x76 or 0x77 address)
2. **Hall Effect Flow Sensor** - YF-S201 or compatible (~450 pulses/liter)
3. **Servo Motor** - Standard 180° servo for cabinet lock
4. **Relay Module** - For hot water solenoid control
5. **Hot Water Solenoid** - 12V DC solenoid valve

### Wiring Diagram
```
BME280 Sensor:
  VCC  → 3.3V
  GND  → GND
  SCL  → D1 (GPIO5)
  SDA  → D2 (GPIO4)

Flow Sensor:
  VCC  → 5V
  GND  → GND
  DATA → D8 (GPIO15)

Hot Water Solenoid Relay:
  VCC  → 5V
  GND  → GND
  IN   → D5 (GPIO14)
  COM  → 12V Power Supply +
  NO   → Solenoid +
  Solenoid - → 12V Power Supply GND

Cabinet Lock Servo:
  VCC  → 5V
  GND  → GND
  SIG  → D4 (GPIO2)
```

## MQTT Topics

### Environmental Sensors (Publish Only)
| Topic | Data Type | Description |
|-------|-----------|-------------|
| `EnvironmentTempAmbient` | float | Temperature in °F |
| `EnvironmentHumidity` | float | Relative humidity % |
| `EnvironmentPressure` | float | Atmospheric pressure in hPa |

### Water Flow (Publish Only)
| Topic | Data Type | Description |
|-------|-----------|-------------|
| `HotWaterFlowRate` | float | Flow rate in L/min |
| `HotWaterFlowStatus` | string | FLOWING or STOPPED |

### Hot Water Solenoid (Subscribe & Publish)
| Topic | Direction | Values | Description |
|-------|-----------|--------|-------------|
| `HotWaterSolenoidCommand` | Subscribe | ON, OFF | Manual control |
| `HotWaterSolenoidStatus` | Publish | ON, OFF | Current state |
| `HotWaterSolenoidMode` | Subscribe | AUTO, MANUAL | Operation mode |

### Cabinet Lock (Subscribe & Publish)
| Topic | Direction | Values | Description |
|-------|-----------|--------|-------------|
| `CabLockRearPassSideCommand` | Subscribe | LOCK, UNLOCK, TOGGLE, STATUS | Control commands |
| `CabLockRearPassSideStatus` | Publish | LOCKED, UNLOCKED, UNKNOWN | Lock state |
| `CabLockAllCommand` | Subscribe | LOCK, UNLOCK, TOGGLE | Group control |

### System Health (Publish Only)
| Topic | Data Type | Description |
|-------|-----------|-------------|
| `D1EnvironmentSystemStatus` | string | ONLINE status |
| `D1EnvironmentUptime` | integer | Uptime in seconds |
| `D1EnvironmentFirmwareVersion` | string | Firmware version |

## Automatic Hot Water Control

### AUTO Mode Operation
When `HotWaterSolenoidMode` is set to `AUTO`:
1. **Flow Detected** → Solenoid opens automatically
2. **Flow Stopped** → Solenoid closes automatically

This provides automatic hot water recirculation:
- Detects when water is running (shower, sink, etc.)
- Opens hot water line for instant hot water
- Closes when water stops to save energy

### MANUAL Mode Operation
When set to `MANUAL`:
- Solenoid controlled only via MQTT commands
- Flow detection has no effect on solenoid
- Useful for maintenance or testing

## MQTT Command Examples

### Environmental Monitoring
```bash
# Subscribe to all environmental data
mosquitto_sub -h 192.168.8.1 -t "Environment#" -v

# View temperature only
mosquitto_sub -h 192.168.8.1 -t EnvironmentTempAmbient -v

# View humidity
mosquitto_sub -h 192.168.8.1 -t EnvironmentHumidity -v
```

### Hot Water Control
```bash
# Set to AUTO mode (flow-triggered)
mosquitto_pub -h 192.168.8.1 -t HotWaterSolenoidMode -m "AUTO"

# Manual ON
mosquitto_pub -h 192.168.8.1 -t HotWaterSolenoidCommand -m "ON"

# Manual OFF
mosquitto_pub -h 192.168.8.1 -t HotWaterSolenoidCommand -m "OFF"

# Monitor flow rate
mosquitto_sub -h 192.168.8.1 -t HotWaterFlowRate -v
```

### Cabinet Lock Control
```bash
# Lock the cabinet
mosquitto_pub -h 192.168.8.1 -t CabLockRearPassSideCommand -m "LOCK"

# Unlock the cabinet
mosquitto_pub -h 192.168.8.1 -t CabLockRearPassSideCommand -m "UNLOCK"

# Toggle lock state
mosquitto_pub -h 192.168.8.1 -t CabLockRearPassSideCommand -m "TOGGLE"

# Request status
mosquitto_pub -h 192.168.8.1 -t CabLockRearPassSideCommand -m "STATUS"
```

## Flow Sensor Calibration

### YF-S201 Specifications
- **Pulses per Liter**: ~450 (typical)
- **Flow Range**: 1-30 L/min
- **Operating Voltage**: 5-18V DC
- **Max Current**: 15mA @ 5V

### Calibration Process
1. Measure actual water volume dispensed
2. Count pulses from sensor
3. Calculate: `pulses_per_liter = total_pulses / liters_dispensed`
4. Update calibration factor in code:
   ```cpp
   flowRate = (pulses / YOUR_CALIBRATION_FACTOR) * (60000.0 / interval);
   ```

### Current Calibration
- Default: 450 pulses/liter
- Adjust based on your specific sensor

## OTA Updates

### Hostname
`D1Mini-Environment.local`

### Update via PlatformIO
```bash
cd D1Mini_CabLock_RearPassSide
pio run --target upload
```

### Update via Web Browser
1. Navigate to: `http://D1Mini-Environment.local:8266/update`
2. Upload compiled firmware.bin
3. Password: `Chester2025`

## Sensor Reading Intervals

| Function | Interval | Description |
|----------|----------|-------------|
| BME280 Read | 5 seconds | Temp/humidity/pressure |
| Flow Calculation | 1 second | Flow rate calculation |
| Status Publish | 60 seconds | Lock status update |
| Health Publish | 60 seconds | System health metrics |

## Build Information

### Firmware Version
- **v1.0.0** - Initial multi-function release

### Memory Usage
- **Flash**: 338,396 bytes (35.3% of 958,448)
- **RAM**: 32,156 bytes (39.3% of 81,920)

### Dependencies
- PubSubClient @ 2.8.0
- Servo @ 1.0.2
- Adafruit BME280 Library @ 2.3.0
- Adafruit Unified Sensor @ 1.1.15
- ArduinoOTA @ 1.0
- ESP8266WiFi @ 1.0
- Wire @ 1.0

## Troubleshooting

### BME280 Not Found
**Symptom**: `⚠ BME280 sensor NOT found!`

**Solutions**:
1. Check I2C wiring (SDA=D2, SCL=D1)
2. Verify sensor address (try 0x76 or 0x77)
3. Check 3.3V power supply
4. Scan I2C bus with scanner sketch

### Flow Sensor Not Detecting
**Symptom**: Flow rate always 0

**Solutions**:
1. Check sensor wiring
2. Verify 5V power supply
3. Test sensor manually (blow through it)
4. Check interrupt pin (D8)
5. Verify sensor orientation (arrow shows flow direction)

### Solenoid Not Operating
**Symptom**: Relay clicks but solenoid doesn't move

**Solutions**:
1. Check 12V power supply to solenoid
2. Verify relay wiring (COM, NO)
3. Test solenoid with direct 12V
4. Check current draw (solenoid may be faulty)

### Cabinet Lock Servo Issues
**Symptom**: Servo doesn't move or moves erratically

**Solutions**:
1. Check 5V power supply (servo needs good power)
2. Adjust LOCKED_POSITION and UNLOCKED_POSITION values
3. Test servo separately
4. Verify D4 connection

### WiFi Connection Failed
**Solutions**:
1. Verify SSID: `Chester IOT`
2. Check password: `2025Chester9894`
3. Move closer to router
4. Check router is powered on

## Integration with Home Assistant

### Example Configuration
```yaml
# MQTT Sensors
sensor:
  - platform: mqtt
    name: "RV Environment Temperature"
    state_topic: "EnvironmentTempAmbient"
    unit_of_measurement: "°F"

  - platform: mqtt
    name: "RV Environment Humidity"
    state_topic: "EnvironmentHumidity"
    unit_of_measurement: "%"

  - platform: mqtt
    name: "Hot Water Flow Rate"
    state_topic: "HotWaterFlowRate"
    unit_of_measurement: "L/min"

# Hot Water Control
switch:
  - platform: mqtt
    name: "Hot Water Solenoid"
    command_topic: "HotWaterSolenoidCommand"
    state_topic: "HotWaterSolenoidStatus"
    payload_on: "ON"
    payload_off: "OFF"

# Cabinet Lock
lock:
  - platform: mqtt
    name: "Rear Pass Side Cabinet"
    command_topic: "CabLockRearPassSideCommand"
    state_topic: "CabLockRearPassSideStatus"
    payload_lock: "LOCK"
    payload_unlock: "UNLOCK"
```

## Safety Features

1. **OTA Safety**: Hot water solenoid turns OFF during firmware updates
2. **Power-On State**: Solenoid defaults to OFF on boot
3. **Watchdog**: WiFi reconnection if connection lost
4. **Flow Threshold**: 0.1 L/min minimum to detect flow (prevents false triggers)
5. **Servo Power Management**: Servo detaches after movement to save power and reduce noise

## Future Enhancements

Potential additions:
- EEPROM persistence for hot water mode
- Flow totalizer (track total water usage)
- Temperature-based alerts
- Humidity-based ventilation control
- Multiple cabinet lock support on one board
- Low flow alert (potential leak detection)

## Support

For issues or questions:
1. Check Serial monitor output (115200 baud)
2. Verify MQTT broker connectivity
3. Review wiring diagrams
4. Test components individually
