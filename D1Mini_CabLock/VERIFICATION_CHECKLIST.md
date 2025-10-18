# D1 Mini Cabinet Lock - Verification Checklist

## ✅ Code Changes Complete

### WiFi Configuration
- [x] SSID set to: "Chester IOT"
- [x] Password set to: "2025Chester9894"
- [x] WiFi reconnection logic added to loop()

### MQTT Configuration
- [x] MQTT server set to: "192.168.8.1" (router IP)
- [x] MQTT port: 1883
- [x] Client ID: "D1Mini_CabLock"
- [x] Anonymous authentication (no username/password)
- [x] Subscribes to: "rv/cabinet/command"
- [x] Publishes to: "rv/cabinet/status"

### Hardware Configuration
- [x] Servo pin: D4 (GPIO2)
- [x] Locked position: 0°
- [x] Unlocked position: 90°

### Code Fixes
- [x] Fixed `cabinetLock` → `lockServo` variable name
- [x] Fixed `isLocked` → `currentState` variable name
- [x] Fixed `mqtt_broker` → `mqtt_server` variable name
- [x] Added 500ms delay after servo movements
- [x] Added MQTT connection checks before publishing
- [x] Enhanced error logging

### Library Dependencies
- [x] PubSubClient for MQTT
- [x] Servo library added
- [x] Removed redundant ESP8266 library references

## 🔧 Hardware Checklist

### Before Uploading
- [ ] D1 Mini connected to computer via USB
- [ ] Servo connected to D4 (GPIO2)
- [ ] Servo powered (3.3V or 5V pin)
- [ ] Servo ground connected to D1 Mini GND

### Servo Wiring
```
Servo Red    → D1 Mini 5V or 3.3V (power)
Servo Brown  → D1 Mini GND (ground)
Servo Orange → D1 Mini D4 (signal)
```

## 📤 Upload Process

### 1. Clean and Build
```bash
cd /Users/aaronlapp/Desktop/ChesterTheBus-RV-Control/D1Mini_CabLock
pio run -t clean
pio run
```

### 2. Upload to Device
```bash
pio run -t upload
```

### 3. Monitor Serial Output
```bash
pio device monitor -b 115200
```

## 🔍 Expected Serial Output

### Successful Boot
```
=== D1 Mini Cabinet Lock Controller ===
Servo initialized at UNLOCKED position
Connecting to WiFi: Chester IOT
..........
WiFi connected!
IP address: 192.168.8.xxx
MAC address: XX:XX:XX:XX:XX:XX
Attempting MQTT connection to 192.168.8.1:1883...connected
Subscribed to: rv/cabinet/command
Status published: UNLOCKED
Ready to receive lock/unlock commands via MQTT!
```

### When Receiving Commands
```
MQTT message received on topic: rv/cabinet/command - Message: LOCK
>>> LOCKING Cabinet
Status published: LOCKED
```

```
MQTT message received on topic: rv/cabinet/command - Message: UNLOCK
>>> UNLOCKING Cabinet
Status published: UNLOCKED
```

## 🧪 Testing Procedure

### Test 1: WiFi Connection
**Expected Behavior:**
- ✅ Connects to "Chester IOT" within 20 seconds
- ✅ Obtains IP address (192.168.8.x)
- ✅ Displays MAC address

**If Fails:**
- Check SSID and password
- Verify router is powered on
- Ensure D1 Mini is in range

### Test 2: MQTT Connection
**Expected Behavior:**
- ✅ Connects to broker at 192.168.8.1:1883
- ✅ Shows "connected" in serial
- ✅ Subscribes to rv/cabinet/command
- ✅ Publishes initial status

**If Fails:**
- Check Mosquitto is running on router
- Verify router IP is 192.168.8.1
- Check allow_anonymous is enabled
- Test with: `mosquitto_pub -h 192.168.8.1 -t "test" -m "hello"`

### Test 3: MQTT Command Reception
From a computer on "Chester IOT" network:

**Test LOCK command:**
```bash
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "LOCK"
```
**Expected:**
- D1 Mini serial shows "MQTT message received"
- Servo moves to 0° position
- D1 Mini publishes "LOCKED" status
- Serial shows "Status published: LOCKED"

**Test UNLOCK command:**
```bash
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "UNLOCK"
```
**Expected:**
- D1 Mini serial shows "MQTT message received"
- Servo moves to 90° position
- D1 Mini publishes "UNLOCKED" status
- Serial shows "Status published: UNLOCKED"

**Test STATUS command:**
```bash
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "STATUS"
```
**Expected:**
- D1 Mini publishes current state
- Servo does NOT move
- Serial shows "Status published: [current state]"

### Test 4: Status Publishing
Monitor status updates:
```bash
mosquitto_sub -h 192.168.8.1 -t "rv/cabinet/status" -v
```
**Expected:**
- Shows "rv/cabinet/status UNLOCKED" on boot
- Shows "rv/cabinet/status LOCKED" when locked
- Shows "rv/cabinet/status UNLOCKED" when unlocked

### Test 5: Integration with GIGA R1
**Prerequisites:**
- GIGA R1 is running and connected to MQTT
- GIGA R1 display shows "MQTT: Connected"

**Test Steps:**
1. Touch "LOCK" button on GIGA R1 display
2. Watch D1 Mini serial output
3. Watch servo move on D1 Mini
4. Watch GIGA R1 display update

**Expected:**
- GIGA R1 button turns yellow "WAIT..."
- D1 Mini receives "LOCK" command
- Servo moves to locked position
- D1 Mini publishes "LOCKED"
- GIGA R1 button turns red "LOCK"

## ⚙️ Servo Position Calibration

### If Servo Positions Need Adjustment

1. **Test Current Positions:**
   - Send LOCK command
   - Observe servo position
   - Send UNLOCK command
   - Observe servo position

2. **Adjust Constants:**
   Edit in main.cpp:
   ```cpp
   #define LOCKED_POSITION 0    // Change this value (0-180)
   #define UNLOCKED_POSITION 90 // Change this value (0-180)
   ```

3. **Common Position Values:**
   - 0° = Full counter-clockwise
   - 90° = Center position
   - 180° = Full clockwise

4. **Find Optimal Positions:**
   ```cpp
   // Temporarily add to setup() for testing
   lockServo.write(0);   // Test
   delay(2000);
   lockServo.write(45);  // Test
   delay(2000);
   lockServo.write(90);  // Test
   delay(2000);
   lockServo.write(135); // Test
   delay(2000);
   lockServo.write(180); // Test
   ```

5. **Re-upload and Test**

## 🚨 Troubleshooting Guide

### WiFi Connection Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| Never connects | Wrong SSID/password | Verify credentials |
| Connects then drops | Poor signal | Move closer to router |
| Timeout after dots | Router not responding | Check router power |

### MQTT Connection Issues

| Error Code | Meaning | Solution |
|------------|---------|----------|
| rc=-4 | Timeout | Check broker IP, verify Mosquitto running |
| rc=-3 | Lost connection | Check network stability |
| rc=-2 | Connect failed | Broker not available or wrong port |
| rc=2 | ID rejected | Client ID conflict (another device with same ID) |
| rc=5 | Not authorized | Anonymous not allowed in broker config |

### Servo Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| Servo doesn't move | Not powered | Check VCC connection |
| Servo jitters | Insufficient power | Use external 5V supply |
| Wrong position | Position values incorrect | Calibrate LOCKED/UNLOCKED positions |
| No response | Wrong pin | Verify D4 connection |

### Command Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| Commands not received | Not subscribed | Check subscription in serial |
| Unknown command error | Wrong format | Commands are case-sensitive: "LOCK" not "lock" |
| No status published | MQTT disconnected | Check MQTT connection status |

## 📊 Success Criteria

You'll know everything is working when:

1. ✅ **WiFi Connected**
   - Serial shows "WiFi connected!"
   - Shows valid IP address

2. ✅ **MQTT Connected**
   - Serial shows "connected"
   - Shows "Subscribed to: rv/cabinet/command"

3. ✅ **Commands Work**
   - mosquitto_pub commands trigger servo movement
   - Serial shows command reception
   - Servo moves to correct positions

4. ✅ **Status Updates Work**
   - mosquitto_sub shows status messages
   - Status matches servo position

5. ✅ **GIGA R1 Integration Works**
   - Touching GIGA button moves servo
   - GIGA display updates with status
   - Round-trip time < 1 second

## 🔄 Next Steps After Verification

1. ✅ Verify WiFi connection
2. ✅ Verify MQTT connection
3. ✅ Test commands via mosquitto_pub
4. ✅ Calibrate servo positions
5. ✅ Test with GIGA R1
6. 🔄 Install in cabinet
7. 🔄 Test with actual latch mechanism
8. 🔄 Fine-tune positions for reliable operation
9. 🔄 Add additional D1 Mini devices for other systems

## 📝 Notes for Installation

- **Mounting:** Secure D1 Mini to prevent movement during operation
- **Wiring:** Use appropriate wire gauge for power (20-22 AWG)
- **Servo:** Use metal gear servo for heavy latches
- **Power:** Consider external 5V supply for high-torque servos
- **Testing:** Test lock/unlock 20+ times before final installation
- **Backup:** Keep USB cable accessible for updates

## 🔐 Security Notes

- Currently using anonymous MQTT (no authentication)
- Suitable for isolated RV network only
- Do not expose MQTT broker to internet without authentication
- Future: Add MQTT username/password authentication
- Future: Enable MQTT over TLS (encrypted)

## 📚 Additional Resources

- **D1 Mini Pinout:** https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
- **Servo Library:** https://www.arduino.cc/reference/en/libraries/servo/
- **PubSubClient:** https://github.com/knolleary/pubsubclient
- **MQTT Testing:** https://mosquitto.org/man/mosquitto_pub-1.html
