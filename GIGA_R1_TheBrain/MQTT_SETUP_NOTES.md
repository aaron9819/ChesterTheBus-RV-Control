# MQTT Setup Configuration - GIGA R1

## Configuration Summary

### WiFi Settings
- **SSID:** Chester IOT
- **Password:** 2025Chester9894

### MQTT Broker Settings
- **Broker Address:** 192.168.8.1 (GL-iNet router default IP)
- **Port:** 1883
- **Client ID:** GIGA_R1_Brain
- **Username:** (none)
- **Password:** (none)

### MQTT Topics Used
- **Command Topic:** `rv/cabinet/command`
- **Status Topic:** `rv/cabinet/status`

## Changes Made

### 1. WiFi Configuration
✅ Updated WiFi SSID from "MyStarlink" to "Chester IOT"
✅ Updated WiFi password to "2025Chester9894"

### 2. MQTT Configuration
✅ Changed from local MQTT broker to external Mosquitto broker on router
✅ Added mqtt_server variable pointing to router IP (192.168.8.1)
✅ Added mqtt_user and mqtt_password variables (empty strings for no auth)
✅ Removed uMQTTBroker initialization code
✅ Updated mqttClient.setServer() to use external broker address

### 3. Library Dependencies
✅ Removed uMQTTBroker library dependencies from platformio.ini
✅ Removed #include <uMQTTBroker.h> from main.cpp

### 4. Display Updates
✅ Updated UI to show external broker IP instead of GIGA's own IP

## Important Notes

### Router IP Address
The default IP for GL-iNet routers (GL-AR300M16-ext) is **192.168.8.1**. 
If your router is configured differently, update the `mqtt_server` constant in main.cpp.

### Verifying Router IP
To verify your router's IP address:
1. Connect to the "Chester IOT" WiFi network
2. Open a web browser and go to http://192.168.8.1
3. If the router admin page loads, the IP is correct

### Testing MQTT Connection
You can test if Mosquitto is running on your router by:
```bash
# From a computer connected to Chester IOT network
mosquitto_sub -h 192.168.8.1 -t "rv/#" -v
```

### Troubleshooting

**If GIGA R1 cannot connect to MQTT broker:**
1. Verify Mosquitto is running on the router
2. Check that the router IP is 192.168.8.1
3. Ensure Mosquitto is configured to listen on 0.0.0.0:1883 (all interfaces)
4. Check Mosquitto configuration allows anonymous connections (since no username/password)
5. Monitor serial output at 115200 baud for connection status

**To check Mosquitto config on GL-iNet router:**
```bash
ssh root@192.168.8.1
cat /etc/mosquitto/mosquitto.conf
```

Typical configuration should include:
```
listener 1883 0.0.0.0
allow_anonymous true
```

## Code Architecture

The GIGA R1 now acts as an MQTT **client** only:
- Subscribes to: `rv/cabinet/status`
- Publishes to: `rv/cabinet/command`

It no longer runs its own MQTT broker. All MQTT communication goes through the Mosquitto broker on the router.

## Next Steps

1. Upload the updated code to GIGA R1
2. Monitor serial output to verify WiFi and MQTT connection
3. Test MQTT communication with your D1 Mini devices
4. Consider adding more topics for other RV systems as mentioned in requirements

## Future Enhancements

- Add reconnection logic with exponential backoff
- Implement MQTT QoS levels for critical commands
- Add SSL/TLS support for encrypted MQTT (port 8883)
- Implement retained messages for system state
- Add MQTT username/password when broker security is enabled
