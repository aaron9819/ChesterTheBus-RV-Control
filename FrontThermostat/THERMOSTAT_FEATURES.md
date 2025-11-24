# Kitchen Pass Side - Thermostat Features

## Overview
This ESP32-S2 board now combines:
- Cabinet lock control (servo)
- Front loop valve control (relay) - NOT controlled by thermostat
- **NEW: Thermostat with BME280 temperature sensor**
- **NEW: Engine loop valve control (relay)**
- **NEW: PWM fan control with smart timing**
- **NEW: Up/Down buttons for temperature adjustment**

## Hardware Pin Assignments

### ESP32-S2 Mini GPIO Pins:
```
GPIO16 - Servo control (cabinet lock)
GPIO39 - Front loop relay (manual control only)
GPIO37 - Engine loop relay (thermostat controlled)
GPIO35 - PWM fan control (0-255 speed)
GPIO33 - Temperature UP button (with internal pullup)
GPIO34 - Temperature DOWN button (with internal pullup)
GPIO38 - BME280 SDA (I2C data)
GPIO36 - BME280 SCL (I2C clock)
```

### Relay Wiring:
- **Active LOW**: Writing LOW turns relay ON, HIGH turns relay OFF
- Both engine loop and front loop relays use this logic

### BME280 Sensor:
- I2C Address: 0x76 or 0x77 (auto-detected)
- Provides: Temperature, Humidity, Pressure
- Currently only temperature is used

### Buttons:
- Use internal pullup resistors
- Active LOW (pressed = LOW, released = HIGH)
- Debounced with 200ms delay

## Thermostat Logic

### Temperature Control:
1. **Set Temperature Range**: 10°C - 30°C (adjustable in 0.5°C increments)
2. **Default Set Point**: 21°C
3. **Hysteresis**: ±0.5°C to prevent oscillation

### Heating Sequence:
1. When `currentTemp < (setTemp - 0.5°C)`:
   - Engine loop valve OPENS
   - System waits **3 minutes** for valve to fully open and water to circulate
   - Fan starts after 3-minute delay

2. **Fan Speed Control**:
   - Proportional to temperature difference
   - 1°C difference = ~25% speed
   - 2°C difference = ~50% speed
   - 3°C difference = ~75% speed
   - 4+°C difference = 100% speed

3. When `currentTemp > (setTemp + 0.5°C)`:
   - Engine loop valve CLOSES
   - Fan continues running for **up to 5 minutes** (cooldown period)
   - Fan then turns OFF

### Front Loop Behavior:
- Front loop operates **independently** of thermostat
- Controlled only via MQTT commands
- No automatic control by temperature

## MQTT Topics

### Published (Status):
```
ThermostatTemp          - Current temperature (°C)
ThermostatSetTemp       - Set temperature (°C)
ThermostatHeating       - "ON" or "OFF"
EngineLoopStatus        - "OPEN" or "CLOSED"
FanSpeed                - PWM value (0-255)
FrontLoopStatus         - "ON" or "OFF"
CabLockKitchenPassSideStatus - "LOCKED" or "UNLOCKED"
```

### Subscribed (Commands):
```
FrontLoopCommand        - "ON" or "OFF"
CabLockKitchenPassSideCommand - "LOCK", "UNLOCK", "TOGGLE"
CabLockAllCommand       - "LOCK", "UNLOCK" (affects all locks)
```

## Button Controls

- **UP Button (GPIO33)**: Increases set temperature by 0.5°C
- **DOWN Button (GPIO34)**: Decreases set temperature by 0.5°C
- Changes are immediately published to MQTT
- Visual feedback via serial monitor

## Timing Constants

```cpp
VALVE_DELAY_MS = 180000     // 3 minutes before fan starts
FAN_COOLDOWN_MS = 300000    // 5 minutes fan cooldown after valve closes
BUTTON_DEBOUNCE_MS = 200    // Button debounce delay
```

## Temperature Reading

- BME280 read every **5 seconds**
- Published to MQTT on each reading
- Thermostat logic evaluated continuously in loop

## Future Enhancements (Not Implemented Yet)

1. **Display Screen**: Small OLED to show current/set temp
2. **Front Loop Integration**: Could add to thermostat logic later
3. **Schedule Support**: Time-based temperature adjustments
4. **Remote Set Temperature**: MQTT command to adjust setpoint

## Wiring Checklist

- [ ] BME280 sensor connected to GPIO38 (SDA) and GPIO36 (SCL)
- [ ] Engine loop relay connected to GPIO37
- [ ] PWM fan controller connected to GPIO35
- [ ] UP button connected to GPIO33 (to GND when pressed)
- [ ] DOWN button connected to GPIO34 (to GND when pressed)
- [ ] Front loop relay connected to GPIO39 (existing)
- [ ] Cabinet lock servo connected to GPIO16 (existing)
- [ ] 3.3V and GND to BME280
- [ ] Power supply for relays and fan

## Testing

1. Monitor serial output at 115200 baud
2. Press UP/DOWN buttons to verify temperature adjustment
3. Check MQTT topics are publishing
4. Verify BME280 reads temperature correctly
5. Test heating cycle:
   - Set temp above current temp
   - Observe valve open
   - Wait 3 minutes
   - Verify fan starts
   - Lower set temp
   - Verify valve closes
   - Verify fan continues for 5 minutes

## Compilation

```bash
cd "D1Mini_CabLock_KitchenPassSide"
pio run
```

**Memory Usage:**
- RAM: 17.3% (56,600 bytes)
- Flash: 60.0% (786,974 bytes)

## Upload

Via OTA (if connected):
```bash
pio run -t upload
```

Via USB:
1. Comment out OTA lines in platformio.ini
2. Connect ESP32-S2 via USB
3. Run: `pio run -t upload`
