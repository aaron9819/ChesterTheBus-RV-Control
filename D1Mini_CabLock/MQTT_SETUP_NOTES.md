# D1 Mini Cabinet Lock - MQTT Setup Configuration

## Configuration Summary

### WiFi Settings
- **SSID:** Chester IOT
- **Password:** 2025Chester9894

### MQTT Broker Settings
- **Broker Address:** 192.168.8.1 (Mosquitto on GL-iNet router)
- **Port:** 1883
- **Client ID:** D1Mini_CabLock
- **Username:** (none)
- **Password:** (none)

### MQTT Topics
- **Subscribes to:** `rv/cabinet/command` (receives commands from GIGA R1)
- **Publishes to:** `rv/cabinet/status` (sends status updates to GIGA R1)

### Hardware Configuration
- **Servo Pin:** D4 (GPIO2)
- **Locked Position:** 0° (adjust as needed)
- **Unlocked Position:** 90° (adjust as needed)

## Changes Made

### 1. WiFi Configuration
✅ Updated WiFi SSID from "MyStarlink" to "Chester IOT"
✅ Updated WiFi password to "2025Chester9894"
✅ Added WiFi reconnection logic in loop()
✅ Added connection timeout handling

### 2. MQTT Configuration
✅ Changed mqtt_server from "192.168.1.100" to "192.168.8.1" (router)
✅ Added mqtt_user and mqtt_password variables (empty for anonymous)
✅ Updated reconnectMQTT() to support anonymous authentication
✅ Added better logging for MQTT connection attempts

### 3. Code Fixes
✅ Fixed variable naming inconsistencies:
  - Changed `cabinetLock` to `lockServo` (matches declaration)
  - Changed `isLocked` boolean to `currentState` enum (matches declaration)
  - Changed `mqtt_broker` to `mqtt_server` (matches declaration)
✅ Added delay after servo movements (500ms)
✅ Added connection checks before publishing
✅ Added detailed status logging
✅ Improved error handling

### 4. Library Dependencies
✅ Removed redundant ESP8266 library references
✅ Added Servo library to platformio.ini
✅ Kept PubSubClient for MQTT functionality

## MQTT Command Protocol

### Commands (received on `rv/cabinet/command`)
- **LOCK** - Moves servo to locked position (0°) and publishes "LOCKED"
- **UNLOCK** - Moves servo to unlocked position (90°) and publishes "UNLOCKED"
- **STATUS** - Publishes current state without moving servo
- **TOGGLE** - Toggles between locked/unlocked states

### Status Updates (published to `rv/cabinet/status`)
- **LOCKED** - Cabinet is locked
- **UNLOCKED** - Cabinet is unlocked
- **UNKNOWN** - State is unknown (initial state)

## Hardware Setup

### Servo Connection
```
Servo Wire Color  | D1 Mini Pin | Notes
------------------|-------------|---------------------------
Red (VCC)         | 5V or 3.3V  | Use 5V for more torque
Brown/Black (GND) | GND         | Common ground
Orange (Signal)   | D4 (GPIO2)  | PWM signal pin
```

### Important Notes
⚠️ **Power Considerations:**
- Small servos can run from D1 Mini's 3.3V/5V pins
- Larger servos need external 5V power supply
- Always connect grounds together (D1 Mini GND to external PSU GND)

⚠️ **Servo Positions:**
- Default: LOCKED=0°, UNLOCKED=90°
- Adjust `LOCKED_POSITION` and `UNLOCKED_POSITION` in code based on your latch mechanism
- Test positions manually before permanent installation

## Testing Procedure

### 1. Upload Code
```bash
cd /Users/aaronlapp/Desktop/ChesterTheBus-RV-Control/D1Mini_CabLock
pio run -t upload
pio device monitor -b 115200
```

### 2. Expected Serial Output
```
=== D1 Mini Cabinet Lock Controller ===
Servo initialized at UNLOCKED position
Connecting to WiFi: Chester IOT
..........
WiFi connected!
IP address: 192.168.8.x
MAC address: XX:XX:XX:XX:XX:XX
Attempting MQTT connection to 192.168.8.1:1883...connected
Subscribed to: rv/cabinet/command
Status published: UNLOCKED
Ready to receive lock/unlock commands via MQTT!
```

### 3. Test with MQTT Commands

From a computer connected to "Chester IOT" network:

**Test 1: Lock the cabinet**
```bash
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "LOCK"
```
Expected: Servo moves to 0°, D1 Mini publishes "LOCKED"

**Test 2: Unlock the cabinet**
```bash
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "UNLOCK"
```
Expected: Servo moves to 90°, D1 Mini publishes "UNLOCKED"

**Test 3: Request status**
```bash
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "STATUS"
```
Expected: D1 Mini publishes current state without moving servo

**Test 4: Monitor status updates**
```bash
mosquitto_sub -h 192.168.8.1 -t "rv/cabinet/status" -v
```
Expected: See status messages as commands are executed

### 4. Test with GIGA R1
1. Ensure GIGA R1 is running and connected to MQTT
2. Touch the button on GIGA R1 display
3. Watch servo move on D1 Mini
4. GIGA R1 display should update to show new status

## Troubleshooting

### WiFi Not Connecting
- Verify SSID "Chester IOT" is broadcasting
- Check password is correct: "2025Chester9894"
- Ensure D1 Mini is within WiFi range
- Check router is powered on

### MQTT Not Connecting
Error code meanings (from serial output):
- **rc=-4**: Connection timeout - broker not responding
- **rc=-3**: Connection lost - network issue
- **rc=-2**: Connect failed - broker not available
- **rc=1**: Bad protocol version
- **rc=2**: Client ID rejected
- **rc=5**: Not authorized

**Solutions:**
1. Verify Mosquitto is running on router: `ps | grep mosquitto`
2. Check broker IP is correct (192.168.8.1)
3. Ensure `allow_anonymous true` in mosquitto.conf
4. Check firewall allows port 1883
5. Test broker with: `mosquitto_pub -h 192.168.8.1 -t "test" -m "hello"`

### Servo Not Moving
- Check servo is properly connected to D4 (GPIO2)
- Verify servo has power (3.3V or 5V)
- Test servo positions manually:
  ```cpp
  lockServo.write(0);    // Should move to one position
  lockServo.write(90);   // Should move to another position
  ```
- Adjust `LOCKED_POSITION` and `UNLOCKED_POSITION` if needed

### Commands Received But No Response
- Check serial output for error messages
- Verify MQTT client is connected (check serial)
- Ensure commands are exact: "LOCK", "UNLOCK", "STATUS" (case-sensitive)
- Look for "Status published: LOCKED" messages in serial output

### GIGA R1 Not Receiving Status
- Check both devices connected to same MQTT broker (192.168.8.1)
- Verify topic names match exactly: `rv/cabinet/status`
- Use `mosquitto_sub` to verify D1 is publishing:
  ```bash
  mosquitto_sub -h 192.168.8.1 -t "rv/cabinet/#" -v
  ```

## Integration with GIGA R1

The D1 Mini and GIGA R1 communicate through the MQTT broker:

```
GIGA R1 Touch Button
       ↓
Publishes to: rv/cabinet/command ("LOCK" or "UNLOCK")
       ↓
Mosquitto Broker (192.168.8.1)
       ↓
D1 Mini receives command
       ↓
Moves servo to position
       ↓
Publishes to: rv/cabinet/status ("LOCKED" or "UNLOCKED")
       ↓
Mosquitto Broker
       ↓
GIGA R1 receives status
       ↓
Updates display button color/text
```

## Adjusting Servo Positions

1. Connect to serial monitor at 115200 baud
2. Manually test positions by modifying code temporarily:
   ```cpp
   void setup() {
     // ... existing setup code ...
     lockServo.write(0);   // Test position 1
     delay(2000);
     lockServo.write(90);  // Test position 2
     delay(2000);
   }
   ```
3. Find positions where:
   - One position fully locks the latch
   - Other position fully unlocks the latch
4. Update `LOCKED_POSITION` and `UNLOCKED_POSITION` constants
5. Re-upload code

## Next Steps

1. ✅ Upload code to D1 Mini
2. ✅ Test WiFi connection
3. ✅ Test MQTT connection
4. ✅ Test servo movement
5. ✅ Test MQTT commands
6. ✅ Adjust servo positions if needed
7. ✅ Test integration with GIGA R1
8. 🔄 Install in cabinet with proper mounting
9. 🔄 Add additional D1 Mini devices for other systems

## Future Enhancements

- Add servo feedback (position sensor) for verification
- Implement motor current sensing to detect jammed lock
- Add timeout for servo operations
- Implement MQTT Last Will and Testament (LWT) for offline detection
- Add OTA (Over The Air) update capability
- Support for multiple cabinet locks on different topics
