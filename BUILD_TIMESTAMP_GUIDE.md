# Build Timestamp Feature

## Overview
All projects now automatically embed the build date and time into the firmware. This information is displayed at startup and helps you track which version is running on each board.

## What Gets Embedded

Each build includes three pieces of information:
1. **BUILD_DATE** - The date the firmware was compiled (e.g., "Oct 18 2025")
2. **BUILD_TIME** - The time the firmware was compiled (e.g., "14:32:15")
3. **BUILD_TIMESTAMP** - Unix timestamp (for programmatic use)

## Serial Output Example

When a board boots up, you'll see:

```
============================================
  D1 ENVIRONMENTAL & HOT WATER CONTROLLER
  Chester the Bus RV Control System
  Firmware: v1.0.0
  Build: Oct 18 2025 14:32:15
============================================
```

This makes it easy to verify:
- ✅ Which firmware version is running
- ✅ When it was last flashed
- ✅ If the board has the latest updates

## Projects Updated

The following projects now display build timestamps:

1. **D1 Plumbing System** (`D1 Plumbing system/`)
   - Displays at startup in Serial monitor

2. **D1 Mini Environment** (`D1Mini_CabLock_RearPassSide/`)
   - Displays at startup in Serial monitor

3. **GIGA R1 Main Brain** (`GIGA_R1_TheBrain/`)
   - Displays at startup in Serial monitor

## How It Works

### platformio.ini Configuration
Each project's `platformio.ini` includes build flags:

```ini
build_flags =
    -D BUILD_TIMESTAMP=\"$UNIX_TIME\"
    -D BUILD_DATE=\"__DATE__\"
    -D BUILD_TIME=\"__TIME__\"
```

These flags:
- `BUILD_DATE` - Compiler macro that expands to build date
- `BUILD_TIME` - Compiler macro that expands to build time
- `BUILD_TIMESTAMP` - Environment variable with Unix timestamp

### Code Implementation
In each `main.cpp`, the startup code checks for these macros:

```cpp
#ifdef BUILD_DATE
Serial.print("  Build: ");
Serial.print(BUILD_DATE);
Serial.print(" ");
Serial.println(BUILD_TIME);
#endif
```

The `#ifdef` ensures backwards compatibility if building without the flags.

## Benefits

### 1. Version Tracking
Know exactly when each board was last updated:
```
Board A: Built Oct 18 2025 14:32:15
Board B: Built Oct 15 2025 09:45:22  ← Needs update!
```

### 2. Troubleshooting
When diagnosing issues, you can quickly verify:
- "Is this the latest firmware?"
- "Did my OTA update succeed?"
- "Which board has the old code?"

### 3. Deployment History
Keep a log of when each board was flashed:
```
Oct 18 14:32 - Environment board updated with flow sensor fix
Oct 18 14:35 - Plumbing board updated with EEPROM persistence
Oct 18 14:40 - GIGA R1 updated with diesel heater 3-state control
```

### 4. Multi-Board Coordination
When updating multiple boards, ensure they're all synchronized:
```bash
# Flash all boards on Oct 18
pio run --target upload  # Board 1
pio run --target upload  # Board 2
pio run --target upload  # Board 3

# All will show: Build: Oct 18 2025 14:30:XX
```

## Viewing Build Info

### Method 1: Serial Monitor
Connect to the board's serial port at 115200 baud and reboot:
```bash
pio device monitor
# Press reset button on board
# Watch for build timestamp in startup messages
```

### Method 2: Add MQTT Publishing
You can add the build timestamp to MQTT publishing:

```cpp
void publishHealth() {
  // ... existing code ...

  #ifdef BUILD_DATE
  String buildInfo = String(BUILD_DATE) + " " + String(BUILD_TIME);
  mqttClient.publish(Topic_Build_Info, buildInfo.c_str());
  #endif
}
```

### Method 3: Display on Screen (GIGA R1)
For GIGA R1 with display, you could show it on the UI:

```cpp
#ifdef BUILD_DATE
tft.setCursor(10, 450);
tft.setTextSize(1);
tft.print("Build: ");
tft.print(BUILD_DATE);
tft.print(" ");
tft.println(BUILD_TIME);
#endif
```

## Build Date Format

The compiler macros use standard formats:
- **DATE**: `"Mmm dd yyyy"` (e.g., "Oct 18 2025")
- **TIME**: `"hh:mm:ss"` (24-hour format)

## Limitations

### Timestamp Precision
The timestamp reflects when **compilation** happened, not when **uploading** happened.

- If you build once and upload to 3 boards, all 3 will show the same timestamp
- This is usually fine since you typically build right before uploading

### OTA Updates
After an OTA update, the board needs to reboot to show the new timestamp:
```bash
# Upload new firmware via OTA
pio run --target upload

# Board reboots automatically
# New timestamp appears in Serial output
```

## Troubleshooting

### "Build: (null)" or Missing Timestamp
**Cause**: Build flags not properly configured

**Solution**:
1. Check `platformio.ini` has `build_flags` section
2. Clean build directory: `pio run --target clean`
3. Rebuild: `pio run`

### Incorrect Date/Time
**Cause**: System clock incorrect

**Solution**:
- Timestamp comes from your computer's system clock
- Ensure your macOS time is set correctly
- Check timezone settings

### Same Timestamp After Update
**Cause**: Build cache preventing recompilation

**Solution**:
```bash
# Clean and rebuild
pio run --target clean
pio run
```

## Future Enhancements

Potential additions:
- Git commit hash in build info
- Build environment (computer name)
- Firmware size in KB
- Configuration checksums
- Auto-increment build numbers

## Example Output from All Boards

### D1 Plumbing System
```
============================================
  D1 PLUMBING SYSTEM CONTROLLER
  Chester the Bus RV Control System
  Firmware: v1.4.0
  Build: Oct 18 2025 14:32:15
============================================
```

### D1 Mini Environment
```
============================================
  D1 ENVIRONMENTAL & HOT WATER CONTROLLER
  Chester the Bus RV Control System
  Firmware: v1.0.0
  Build: Oct 18 2025 14:35:22
============================================
```

### GIGA R1 Main Brain
```
=================================
  GIGA R1 - MAIN CONTROLLER
  Chester the Bus RV Control
  Firmware: v2.1.0
  Build: Oct 18 2025 14:40:18
=================================
```

## Integration with Deployment Scripts

You can create a deployment script that logs timestamps:

```bash
#!/bin/bash
# deploy-all.sh

echo "=== Chester the Bus Firmware Deployment ==="
echo "Started: $(date)"
echo ""

echo "Building D1 Plumbing System..."
cd "D1 Plumbing system"
pio run --target upload
echo "Deployed at: $(date)"
echo ""

echo "Building D1 Environment..."
cd "../D1Mini_CabLock_RearPassSide"
pio run --target upload
echo "Deployed at: $(date)"
echo ""

echo "Building GIGA R1..."
cd "../GIGA_R1_TheBrain"
pio run --target upload
echo "Deployed at: $(date)"
echo ""

echo "=== Deployment Complete ==="
```

This keeps a log of all deployments with timestamps!
