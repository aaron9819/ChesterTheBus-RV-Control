# Over-The-Air (OTA) Update Guide - ChesterTheBus RV Control

## Overview
All D1 Mini cabinet lock controllers now support OTA (Over-The-Air) firmware updates over WiFi. This means you can update the firmware wirelessly without physically connecting a USB cable to each board.

## OTA Configuration

### Hostnames & Credentials
All boards are configured with the following OTA settings:

| Board | OTA Hostname | Password |
|-------|--------------|----------|
| **Kitchen Driver Side** | `CabLock-KitchenDriverSide.local` | `Chester2025` |
| **Kitchen Pass Side** | `CabLock-KitchenPassSide.local` | `Chester2025` |
| **Rear Driver Side** | `CabLock-RearDriverSide.local` | `Chester2025` |
| **D1 Plumbing** | `D1Mini-Plumbing.local` | `Chester2025` |

### Firmware Version
Current firmware version for all cabinet locks: **v1.1.0**

## How to Update Firmware via OTA

### Method 1: PlatformIO Command Line (Recommended)

1. **Make sure the board is powered on and connected to WiFi**
   - Check serial monitor or MQTT status to confirm connection

2. **Navigate to the project folder** in terminal:
   ```bash
   cd /Users/aaronlapp/Desktop/ChesterTheBus-RV-Control/D1Mini_CabLock_KitchenDriverSide
   ```

3. **Upload firmware via OTA**:
   ```bash
   pio run --target upload --upload-port CabLock-KitchenDriverSide.local
   ```

4. **Monitor progress** (optional):
   ```bash
   pio device monitor
   ```

### Method 2: PlatformIO VS Code Extension

1. Open the project folder in VS Code
2. Click the PlatformIO icon in the left sidebar
3. Under PROJECT TASKS, expand your environment (e.g., `d1_mini_lite`)
4. Click **Platform** → **Upload (OTA)**
5. Select the target device from the list

### Method 3: Manual Upload with esptool.py

```bash
# Build the firmware first
pio run

# Upload via OTA
python -m esptool --chip esp8266 --port CabLock-KitchenDriverSide.local write_flash 0x00000 .pio/build/d1_mini_lite/firmware.bin
```

## OTA Update Behavior

### During OTA Update:
- ✅ Board accepts incoming firmware
- ✅ Servo is automatically detached for safety
- ✅ Progress is displayed in serial monitor (every 10%)
- ✅ LED may blink during update
- ⏸️ MQTT commands are **NOT** processed during update

### After Successful Update:
- ✅ Board automatically reboots
- ✅ Reconnects to WiFi
- ✅ Reconnects to MQTT broker
- ✅ Publishes status messages
- ✅ Ready to receive commands

### If OTA Update Fails:
- ⚠️ Board continues running old firmware
- ⚠️ Error message printed to serial monitor
- ⚠️ No reboot occurs
- ✅ Normal operation resumes immediately

## Troubleshooting

### "Device not found" error
**Problem:** PlatformIO can't find the device on the network

**Solutions:**
1. Verify board is powered on and WiFi connected
2. Check that you're on the same network (Chester IOT)
3. Try using the IP address instead of hostname:
   ```bash
   pio run --target upload --upload-port 192.168.8.XXX
   ```
4. Restart the board and wait 30 seconds for WiFi/mDNS to initialize
5. Ping the hostname to verify network connectivity:
   ```bash
   ping CabLock-KitchenDriverSide.local
   ```

### "Authentication failed" error
**Problem:** OTA password is incorrect

**Solution:**
- Verify the password is set correctly in `platformio.ini`:
  ```ini
  upload_flags =
    --auth=Chester2025
  ```

### "Upload timeout" error
**Problem:** Network connection interrupted during upload

**Solutions:**
1. Move closer to the WiFi router (GL-AR300M16)
2. Check WiFi signal strength in serial monitor
3. Temporarily stop other network-heavy operations
4. Retry the upload

### Board stuck in boot loop after OTA
**Problem:** New firmware has a critical bug

**Solution:**
1. Connect via USB cable
2. Upload working firmware via serial:
   ```bash
   pio run --target upload
   ```
3. Board will recover and run the working firmware

## Safety Features

### Automatic Safeguards During OTA:
- 🔒 Servo motors are detached (locks stay in current position)
- 🔒 No MQTT commands accepted during update
- 🔒 If update fails, board returns to normal operation

### Critical Automation (D1 Plumbing only):
- 🚨 During OTA, **ALL relays are disabled** for safety
- 🚨 Tank heaters, exhaust fan, and other relays are turned OFF
- ✅ After successful OTA, automation resumes automatically

## Best Practices

1. **Test First**: Always test new firmware on one board before updating all boards
2. **Good Signal**: Ensure strong WiFi signal before starting OTA update
3. **Power Stability**: Make sure board has stable power during update
4. **Version Control**: Always update `FIRMWARE_VERSION` define when making changes
5. **Backup**: Keep a working firmware `.bin` file for emergency recovery
6. **Update One at a Time**: Don't try to update multiple boards simultaneously
7. **Monitor Serial**: Watch serial output during first OTA update to verify success

## Finding Board IP Addresses

### Via Serial Monitor:
```bash
pio device monitor
```
Look for: `IP address: 192.168.8.XXX`

### Via MQTT:
Subscribe to status topics and check for connection messages

### Via Router:
1. Log into GL-AR300M16 at http://192.168.8.1
2. Go to **Clients** page
3. Look for devices named "CabLock_*"

## Updating All Cabinet Locks Script

Create a shell script to update all locks sequentially:

```bash
#!/bin/bash
# update-all-locks.sh

echo "🔄 Updating all cabinet locks via OTA..."

cd "/Users/aaronlapp/Desktop/ChesterTheBus-RV-Control"

echo ""
echo "📦 1/3 - Updating Kitchen Driver Side..."
cd "D1Mini_CabLock_KitchenDriverSide"
pio run --target upload --upload-port CabLock-KitchenDriverSide.local
if [ $? -eq 0 ]; then
    echo "✅ Kitchen Driver Side updated successfully"
else
    echo "❌ Kitchen Driver Side update failed"
fi

echo ""
echo "📦 2/3 - Updating Kitchen Pass Side..."
cd "../D1Mini_CabLock_KitchenPassSide"
pio run --target upload --upload-port CabLock-KitchenPassSide.local
if [ $? -eq 0 ]; then
    echo "✅ Kitchen Pass Side updated successfully"
else
    echo "❌ Kitchen Pass Side update failed"
fi

echo ""
echo "📦 3/3 - Updating Rear Driver Side..."
cd "../D1Mini_CabLock_RearDriverSide"
pio run --target upload --upload-port CabLock-RearDriverSide.local
if [ $? -eq 0 ]; then
    echo "✅ Rear Driver Side updated successfully"
else
    echo "❌ Rear Driver Side update failed"
fi

echo ""
echo "🎉 All updates complete!"
```

Make it executable:
```bash
chmod +x update-all-locks.sh
./update-all-locks.sh
```

## Emergency Recovery

If a board becomes unresponsive after OTA:

1. **Connect USB cable** to the board
2. **Hold down the RESET button** (if needed)
3. **Upload via serial**:
   ```bash
   pio run --target upload
   ```
4. **Verify operation** in serial monitor
5. **Try OTA again** once serial upload works

## Questions?

- Check serial monitor for detailed OTA progress messages
- Monitor MQTT topics for status updates
- Review error messages in PlatformIO output
- Verify network connectivity with `ping` command

---

**Last Updated:** October 7, 2025
**Firmware Version:** v1.1.0
**OTA Password:** Chester2025
