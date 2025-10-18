# MQTT Configuration Verification Checklist

## ✅ Code Changes Complete

### WiFi Configuration
- [x] SSID set to: "Chester IOT"
- [x] Password set to: "2025Chester9894"

### MQTT Configuration
- [x] MQTT server set to: "192.168.8.1" (router IP)
- [x] MQTT port: 1883
- [x] Client ID: "GIGA_R1_Brain"
- [x] No username/password (anonymous authentication)

### Code Cleanup
- [x] Removed uMQTTBroker includes and initialization
- [x] Removed local broker code
- [x] Updated mqttClient.setServer() to use external broker
- [x] Updated reconnectMQTT() to support anonymous connection
- [x] Updated display to show external broker IP

### Library Dependencies
- [x] Removed uMQTTBroker from platformio.ini
- [x] Kept PubSubClient for MQTT client functionality

## 🔧 Before Uploading

1. **Verify Router IP Address**
   - Default GL-iNet IP is 192.168.8.1
   - If different, update `mqtt_server` in main.cpp line 10

2. **Verify Mosquitto is Running**
   - SSH into router: `ssh root@192.168.8.1`
   - Check process: `ps | grep mosquitto`
   - Check config: `cat /etc/mosquitto/mosquitto.conf`

3. **Verify Mosquitto Configuration**
   Required settings in mosquitto.conf:
   ```
   listener 1883 0.0.0.0
   allow_anonymous true
   ```

## 📤 Upload Process

1. Connect GIGA R1 to computer via USB
2. Open PlatformIO terminal in VS Code
3. Clean and build:
   ```bash
   pio run -t clean
   pio run
   ```
4. Upload:
   ```bash
   pio run -t upload
   ```
5. Monitor serial output:
   ```bash
   pio device monitor -b 115200
   ```

## 🔍 Testing After Upload

### Serial Output Check
Look for these messages in serial monitor:
```
=== GIGA R1 WiFi - Cabinet Lock Controller ===
Connecting to WiFi: Chester IOT
WiFi connected!
IP address: 192.168.8.x
Configuring MQTT client to connect to broker at 192.168.8.1:1883
Attempting MQTT connection to 192.168.8.1:1883...connected
Subscribed to: rv/cabinet/status
```

### Expected Behavior
- ✅ WiFi connects to "Chester IOT"
- ✅ GIGA gets IP address (192.168.8.x)
- ✅ MQTT connects to broker at 192.168.8.1:1883
- ✅ Subscribes to rv/cabinet/status
- ✅ Display shows MQTT: Connected

### If Connection Fails

**WiFi Not Connecting:**
- Verify SSID and password are correct
- Check WiFi is in range
- Verify router is broadcasting the SSID

**MQTT Not Connecting:**
- Check serial output for error code (rc=X)
- Error codes:
  - rc=-4: Connection timeout
  - rc=-3: Connection lost
  - rc=-2: Connect failed
  - rc=-1: Disconnected
  - rc=1: Bad protocol
  - rc=2: ID rejected
  - rc=3: Server unavailable
  - rc=4: Bad credentials
  - rc=5: Not authorized

**Common Fixes:**
1. Verify router IP is correct (192.168.8.1)
2. Ensure Mosquitto service is running on router
3. Check firewall allows port 1883
4. Verify `allow_anonymous true` in mosquitto.conf

## 🧪 MQTT Testing Commands

From a computer connected to "Chester IOT" network:

### Subscribe to all RV topics:
```bash
mosquitto_sub -h 192.168.8.1 -t "rv/#" -v
```

### Publish test command:
```bash
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "STATUS"
```

### Monitor cabinet status:
```bash
mosquitto_sub -h 192.168.8.1 -t "rv/cabinet/status" -v
```

## 📝 Integration with D1 Mini Devices

Make sure your D1 Mini devices are also configured to:
- Connect to WiFi: "Chester IOT" / "2025Chester9894"
- Connect to MQTT broker: 192.168.8.1:1883
- Use same topics: rv/cabinet/command and rv/cabinet/status

## ✨ Success Criteria

You'll know everything is working when:
1. ✅ GIGA R1 connects to WiFi
2. ✅ GIGA R1 connects to MQTT broker
3. ✅ Display shows "MQTT: Connected"
4. ✅ Serial monitor shows successful subscription
5. ✅ Can send/receive MQTT messages using mosquitto_pub/sub
6. ✅ D1 Mini devices can communicate through the broker
