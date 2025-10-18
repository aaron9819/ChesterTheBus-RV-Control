# EEPROM Configuration Persistence Guide

## Overview
The D1 Plumbing system now saves temperature configuration settings to EEPROM (flash memory) so they persist across reboots and power cycles.

## Problem Solved
**Before**: If the D1 Mini rebooted, all temperature settings would reset to hardcoded defaults:
- Fresh water: 40°F
- Grey water: 35°F
- Exhaust fan high: 85°F
- Exhaust fan low: 75°F

**After**: Temperature settings are automatically saved to EEPROM whenever changed via MQTT and restored on boot.

## How It Works

### 1. Configuration Structure
Settings are stored in a structured format with validation:
```cpp
struct ConfigData {
  uint16_t magic;              // 0xA5C3 - validates EEPROM data
  uint8_t version;             // Version 1 - for future compatibility
  float freshWaterTempTarget;
  float greyWaterTempTarget;
  float exhaustFanTempHigh;
  float exhaustFanTempLow;
  uint16_t checksum;           // Data integrity validation
};
```

### 2. Automatic Save
Every time you change a setting via MQTT, it's automatically saved:
```bash
# This command now saves to EEPROM immediately
mosquitto_pub -h 192.168.8.1 -t FreshWaterHeatTempSet -m "42.0"
```

Serial output:
```
Fresh water target temp set to: 42.00 °F
💾 Configuration saved to EEPROM
```

### 3. Automatic Load on Boot
When the D1 Mini boots up, it:
1. Reads EEPROM data
2. Validates magic number (0xA5C3)
3. Checks version compatibility
4. Validates checksum
5. Range-checks all values
6. Loads settings if all checks pass
7. Falls back to defaults if validation fails

Serial output on successful load:
```
✓ Configuration loaded from EEPROM
  Fresh Water Target: 42.00 °F
  Grey Water Target: 37.50 °F
  Exhaust Fan High: 82.00 °F
  Exhaust Fan Low: 72.00 °F
```

Serial output on first boot (no saved data):
```
⚠️  EEPROM: No valid config found (wrong magic number)
   Using default temperature settings
```

## Validation & Safety Features

### Magic Number Validation
- Ensures EEPROM contains valid configuration data
- Prevents reading random/uninitialized memory

### Version Control
- Current version: 1
- Allows future config structure changes
- Old versions rejected for safety

### Checksum Validation
- Detects corrupted data from power loss during write
- Prevents loading invalid temperature values

### Range Validation
- Fresh/Grey water: 32.0°F - 60.0°F
- Exhaust fan: 60.0°F - 120.0°F
- Out-of-range values rejected

## MQTT Topics That Trigger Saves
These topics automatically save to EEPROM when settings change:
- `FreshWaterHeatTempSet` - Fresh water target temperature
- `GreyWaterHeatTempSet` - Grey water target temperature
- `ExhaustFanTempHigh` - Exhaust fan turn-on temperature
- `ExhaustFanTempLow` - Exhaust fan turn-off temperature

## Testing

### Test Configuration Persistence
1. **Set a custom temperature**:
   ```bash
   mosquitto_pub -h 192.168.8.1 -t FreshWaterHeatTempSet -m "45.0"
   ```

2. **Verify it was saved**:
   - Check Serial monitor for: `💾 Configuration saved to EEPROM`

3. **Reboot the D1 Mini**:
   ```bash
   mosquitto_pub -h 192.168.8.1 -t D1PlumbingOTACommand -m "REBOOT"
   ```

4. **Verify settings restored**:
   - Check Serial monitor for: `✓ Configuration loaded from EEPROM`
   - Should show your custom value (45.0°F)

5. **Query current config**:
   ```bash
   mosquitto_pub -h 192.168.8.1 -t D1PlumbingConfigRequest -m "1"
   mosquitto_sub -h 192.168.8.1 -t D1PlumbingConfigResponse -C 1
   ```

### Test Corruption Handling
To verify the system handles corrupted data gracefully:
1. Flash new firmware (EEPROM gets random data)
2. System should detect invalid magic/checksum
3. Falls back to defaults automatically
4. Serial shows: `⚠️ EEPROM: Checksum mismatch (data corrupted)`

## Memory Usage

### EEPROM Allocation
- **Size**: 512 bytes reserved
- **Used**: ~20 bytes for configuration
- **Overhead**: Magic, version, checksum (5 bytes)

### Impact on Build
- **Flash**: +1,256 bytes (335,604 total, 35.0%)
- **RAM**: +400 bytes (33,080 total, 40.4%)
- **Build time**: +0.1 seconds

## Default Values
If EEPROM is empty or corrupted, these defaults are used:
```cpp
float freshWaterTempTarget = 40.0;    // °F
float greyWaterTempTarget = 35.0;     // °F
float exhaustFanTempHigh = 85.0;      // °F
float exhaustFanTempLow = 75.0;       // °F
```

## Implementation Details

### Save Function
```cpp
void saveConfigToEEPROM() {
  // 1. Create config structure
  // 2. Populate with current values
  // 3. Calculate checksum
  // 4. Write to EEPROM
  // 5. Commit (actual write to flash)
}
```

### Load Function
```cpp
void loadConfigFromEEPROM() {
  // 1. Read from EEPROM
  // 2. Validate magic number
  // 3. Check version compatibility
  // 4. Verify checksum
  // 5. Range-check values
  // 6. Apply if all checks pass
  // 7. Use defaults if validation fails
}
```

### Checksum Calculation
Simple additive checksum over all config bytes except the checksum field itself.

## Troubleshooting

### Settings Not Persisting
**Symptom**: Settings reset after reboot

**Possible causes**:
1. MQTT command not reaching device
2. Value out of valid range
3. EEPROM write failed

**Debug steps**:
- Check Serial monitor for `💾 Configuration saved to EEPROM`
- Verify MQTT publish succeeded
- Check value is within valid range

### "EEPROM: Checksum mismatch" on Boot
**Symptom**: Settings not loading, using defaults

**Causes**:
- First boot (no data saved yet)
- Power loss during EEPROM write
- Flash memory corruption

**Solution**:
- Send new settings via MQTT
- Settings will save properly and load on next boot

### "Config values out of range"
**Symptom**: Loaded values rejected

**Causes**:
- Corrupted flash memory
- Wrong data in EEPROM

**Solution**:
- System automatically uses safe defaults
- Send new valid settings via MQTT

## Future Enhancements
Potential additions to consider:
- Save mode states (AUTO/MANUAL)
- Save diesel heater state
- Save valve positions
- Configuration backup/restore via MQTT
- Factory reset command

## Firmware Version
- **Feature Added**: v1.4.0
- **EEPROM Version**: 1
- **Magic Number**: 0xA5C3
