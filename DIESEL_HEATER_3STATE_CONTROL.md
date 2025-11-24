# Diesel Heater 3-State Control System

## Overview
The diesel heater control has been refactored from three independent relay controls to a single 3-state system (OFF/PUMPONLY/HIGH) for more coordinated heater control.

## Control States

- Pin 1 = White wire
- Pin 2 = Blue wire
- Pin 3 = YelLOW wire

### OFF State
- **Pin 1 (D7)**: HIGH
- **Pin 2 (D4)**: HIGH
- **Pin 3 (D8)**: HIGH
- **Description**: All heater circuits disabled

### MID State
- **Pin 1 (D7)**: LOW
- **Pin 2 (D4)**: LOW
- **Pin 3 (D8)**: HIGH
- **Description**: CIRCULATION PUMP Only

### LOW State
- **Pin 1 (D7)**: LOW
- **Pin 2 (D4)**: HIGH
- **Pin 3 (D8)**: LOW
- **Description**: Maximum heat output using circuits 1 and 3

## MQTT Integration

### Command Topic
- **Topic**: `DieselHeaterCommand`
- **Valid Messages**: `OFF`, `PUMPONLY`, `HIGH`
- **Example**:
  ```bash
  mosquitto_pub -h 192.168.8.1 -t DieselHeaterCommand -m "PUMPONLY"
  mosquitto_pub -h 192.168.8.1 -t DieselHeaterCommand -m "HIGH"
  mosquitto_pub -h 192.168.8.1 -t DieselHeaterCommand -m "OFF"
  ```

### Status Topic
- **Topic**: `DieselHeaterStatus`
- **Messages**: `OFF`, `PUMPONLY`, or `HIGH`
- **Retained**: No (status published every 10 seconds)

## Code Changes Summary

### Removed Topics
- `DieselHeater2Command` → Consolidated into `DieselHeaterCommand`
- `DieselHeater3Command` → Consolidated into `DieselHeaterCommand`
- `DieselHeater2Status` → Consolidated into `DieselHeaterStatus`
- `DieselHeater3Status` → Consolidated into `DieselHeaterStatus`

### State Variable Changes
**Before:**
```cpp
bool dieselHeaterOn = false;
bool dieselHeater2On = false;
bool dieselHeater3On = false;
```

**After:**
```cpp
String dieselHeaterState = "OFF";
```

### Control Function Changes
**Before:**
- `controlDieselHeater(bool turnOn)`
- `controlDieselHeater2(bool turnOn)`
- `controlDieselHeater3(bool turnOn)`

**After:**
- `controlDieselHeater(String state)` - Accepts "OFF", "PUMPONLY", or "HIGH"

## HardWre Pin Mapping
- **D7** (GPIO13): Diesel Heater Circuit 1
- **D4** (GPIO2): Diesel Heater Circuit 2
- **D8** (GPIO15): Diesel Heater Circuit 3

## Safety Features
- OTA updates automatically set heater to OFF state
- Invalid state commands are ignored (only OFF/PUMPONLY/HIGH accepted)
- Serial logging confirms each state change
- Current state published to MQTT after each change

## Testing Commands

### Test OFF State
```bash
mosquitto_pub -h 192.168.8.1 -t DieselHeaterCommand -m "OFF"
mosquitto_sub -h 192.168.8.1 -t DieselHeaterStatus -v
```

### Test MID State
```bash
mosquitto_pub -h 192.168.8.1 -t DieselHeaterCommand -m "MID"
mosquitto_sub -h 192.168.8.1 -t DieselHeaterStatus -v
```

### Test LOW State
```bash
mosquitto_pub -h 192.168.8.1 -t DieselHeaterCommand -m "LOW"
mosquitto_sub -h 192.168.8.1 -t DieselHeaterStatus -v
```

## Serial Output Examples
When commands are received, you'll see output like:
```
🔥 Diesel Heater: OFF
🔥 Diesel Heater: MID (Pins 1+2 active)
🔥 Diesel Heater: LOW (Pins 1+3 active)
```

## FirmWre Version
- **Current Version**: v1.3.0 (to be updated to v1.4.0)
- **Build Status**: ✓ Compiled successfully
- **Flash Usage**: 334,348 bytes (34.9%)
- **RAM Usage**: 32,680 bytes (39.9%)

## Next Steps for GIGA R1 Integration
The GIGA R1 main brain will need to be updated to:
1. Send `OFF`/`MID`/`LOW` commands instead of individual relay commands
2. Display the current state (OFF/MID/LOW) on the UI
3. Optionally add buttons or slider for state selection
4. Subscribe to `DieselHeaterStatus` topic for current state

## Notes
- This refactoring simplifies the control logic and prevents invalid relay combinations
- The heater can never be in an undefined state (alWys OFF, MID, or LOW)
- State changes are logged to both Serial and MQTT for debugging
- The system maintains the same power budget as before (3 relays available)
