# Chester The Bus - RV Control System Integration Guide

## System Overview

This document describes how the GIGA R1 and D1 Mini devices communicate through the Mosquitto MQTT broker for RV control.

## Network Architecture

```
┌─────────────────────────────────────────────────┐
│          GL-iNet Router (GL-AR300M16)           │
│                                                  │
│  WiFi SSID: Chester IOT                         │
│  Password: 2025Chester9894                      │
│  Router IP: 192.168.8.1                         │
│                                                  │
│  ┌──────────────────────────────────┐          │
│  │   Mosquitto MQTT Broker          │          │
│  │   - Port: 1883                   │          │
│  │   - Anonymous auth enabled        │          │
│  │   - No username/password          │          │
│  └──────────────────────────────────┘          │
└─────────────────────────────────────────────────┘
                    │
        ┌───────────┴───────────┐
        │                       │
┌───────▼─────────┐    ┌────────▼─────────┐
│   GIGA R1 WiFi  │    │  D1 Mini (ESP)   │
│   "The Brain"   │    │  Cabinet Lock    │
│                 │    │                  │
│ IP: 192.168.8.x │    │ IP: 192.168.8.y  │
│                 │    │                  │
│ - Touch Display │    │ - Servo Control  │
│ - MQTT Client   │    │ - MQTT Client    │
│                 │    │                  │
└─────────────────┘    └──────────────────┘
```

## Device Configuration Summary

| Setting          | GIGA R1 Brain        | D1 Mini CabLock      |
|------------------|----------------------|----------------------|
| **WiFi SSID**    | Chester IOT          | Chester IOT          |
| **WiFi Password**| 2025Chester9894      | 2025Chester9894      |
| **MQTT Broker**  | 192.168.8.1:1883     | 192.168.8.1:1883     |
| **Client ID**    | GIGA_R1_Brain        | D1Mini_CabLock       |
| **Auth**         | Anonymous (none)     | Anonymous (none)     |
| **Subscribes**   | rv/cabinet/status    | rv/cabinet/command   |
| **Publishes**    | rv/cabinet/command   | rv/cabinet/status    |

## MQTT Topic Structure

### Current Topics
- `rv/cabinet/command` - Commands from GIGA R1 to D1 Mini
- `rv/cabinet/status` - Status updates from D1 Mini to GIGA R1

### Planned Topics (for future expansion)
```
rv/
├── cabinet/
│   ├── command      (GIGA → D1)
│   └── status       (D1 → GIGA)
├── thermostat/
│   ├── rear/
│   │   ├── command
│   │   ├── status
│   │   └── temperature
│   └── front/
│       ├── command
│       ├── status
│       └── temperature
├── tanks/
│   ├── fresh/
│   │   ├── level
│   │   ├── temperature
│   │   └── heater/command
│   ├── grey/
│   │   ├── level
│   │   ├── temperature
│   │   └── heater/command
│   ├── propane/
│   │   └── level
│   └── diesel/
│       └── level
├── heater/
│   ├── diesel/
│   │   ├── command
│   │   ├── status
│   │   └── temperature
│   └── hydronic/
│       └── temperature
├── engine/
│   ├── block/
│   │   ├── heater/command
│   │   └── temperature
│   └── status
├── leveling/
│   ├── command
│   ├── status
│   └── sensors/
│       ├── front_left
│       ├── front_right
│       ├── rear_left
│       └── rear_right
├── blinds/
│   ├── [location]/
│   │   ├── command
│   │   └── position
├── locks/
│   ├── [location]/
│   │   ├── command
│   │   └── status
└── system/
    ├── status
    └── errors
```

## Communication Flow - Cabinet Lock Example

### Lock Command Flow
```
1. User touches "LOCK" button on GIGA R1 display
   ↓
2. GIGA R1: Sets button to yellow "WAIT..." state
   ↓
3. GIGA R1: Publishes "LOCK" to rv/cabinet/command
   ↓
4. Mosquitto Broker: Routes message to all subscribers
   ↓
5. D1 Mini: Receives "LOCK" command
   ↓
6. D1 Mini: Moves servo to LOCKED_POSITION (0°)
   ↓
7. D1 Mini: Publishes "LOCKED" to rv/cabinet/status
   ↓
8. Mosquitto Broker: Routes message to all subscribers
   ↓
9. GIGA R1: Receives "LOCKED" status
   ↓
10. GIGA R1: Updates button to red "LOCK" state
```

### Status Request Flow
```
1. GIGA R1 boots up or times out
   ↓
2. GIGA R1: Publishes "STATUS" to rv/cabinet/command
   ↓
3. D1 Mini: Receives "STATUS" command
   ↓
4. D1 Mini: Publishes current state to rv/cabinet/status
   ↓
5. GIGA R1: Updates display with current state
```

## Message Protocols

### Cabinet Lock Commands
| Command | Description                          | Response       |
|---------|--------------------------------------|----------------|
| LOCK    | Lock the cabinet                     | LOCKED         |
| UNLOCK  | Unlock the cabinet                   | UNLOCKED       |
| STATUS  | Request current status (no movement) | LOCKED/UNLOCKED|
| TOGGLE  | Toggle between locked/unlocked       | LOCKED/UNLOCKED|

### Cabinet Lock Status
| Status   | Meaning                    |
|----------|----------------------------|
| LOCKED   | Cabinet is locked          |
| UNLOCKED | Cabinet is unlocked        |
| UNKNOWN  | State unknown (boot/error) |

## Setup Checklist

### Router/Broker Setup
- [ ] GL-iNet router powered on
- [ ] WiFi "Chester IOT" is broadcasting
- [ ] Router IP is 192.168.8.1
- [ ] Mosquitto broker is installed
- [ ] Mosquitto is configured:
  ```
  listener 1883 0.0.0.0
  allow_anonymous true
  ```
- [ ] Mosquitto is running: `ps | grep mosquitto`

### GIGA R1 Setup
- [ ] Code updated with correct WiFi credentials
- [ ] Code updated with correct MQTT broker IP
- [ ] Code compiled and uploaded
- [ ] Serial output shows WiFi connected
- [ ] Serial output shows MQTT connected
- [ ] Display shows "MQTT: Connected"

### D1 Mini Setup
- [ ] Code updated with correct WiFi credentials
- [ ] Code updated with correct MQTT broker IP
- [ ] Servo connected to D4 pin
- [ ] Servo positions tested and adjusted
- [ ] Code compiled and uploaded
- [ ] Serial output shows WiFi connected
- [ ] Serial output shows MQTT connected
- [ ] Servo responds to MQTT commands

## Testing Commands

### Monitor All RV Traffic
```bash
mosquitto_sub -h 192.168.8.1 -t "rv/#" -v
```

### Test Cabinet Lock
```bash
# Lock
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "LOCK"

# Unlock
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "UNLOCK"

# Status
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "STATUS"
```

### Monitor Cabinet Status
```bash
mosquitto_sub -h 192.168.8.1 -t "rv/cabinet/status" -v
```

### Test Broker Connectivity
```bash
# From any device on Chester IOT network
mosquitto_pub -h 192.168.8.1 -t "test" -m "hello"
mosquitto_sub -h 192.168.8.1 -t "test" -v
```

## Troubleshooting

### Problem: Devices can't connect to WiFi
**Check:**
- Router is powered on
- SSID "Chester IOT" is broadcasting
- Password "2025Chester9894" is correct
- Devices are in range

### Problem: Devices can't connect to MQTT
**Check:**
- Mosquitto is running: `ssh root@192.168.8.1` then `ps | grep mosquitto`
- Broker allows anonymous: `cat /etc/mosquitto/mosquitto.conf | grep anonymous`
- Firewall allows 1883: `iptables -L | grep 1883`
- Test with: `mosquitto_pub -h 192.168.8.1 -t "test" -m "hello"`

### Problem: GIGA R1 sends commands but D1 doesn't respond
**Check:**
- D1 Mini serial shows "MQTT message received"
- Topic names match exactly (case-sensitive)
- D1 Mini is connected to MQTT (check serial)
- Use mosquitto_sub to verify messages are published

### Problem: D1 Mini publishes status but GIGA R1 doesn't update
**Check:**
- GIGA R1 serial shows "MQTT message received"
- GIGA R1 is subscribed to correct topic
- Status message format matches ("LOCKED" or "UNLOCKED")
- Use mosquitto_sub to verify messages are published

### Problem: Commands work but timeout occurs
**Check:**
- Network latency (should be < 100ms on local network)
- Increase COMMAND_TIMEOUT in GIGA R1 code (currently 4000ms)
- Check for WiFi interference

## System Expansion Plan

### Phase 1 (Current)
- ✅ GIGA R1 WiFi with touch display
- ✅ D1 Mini cabinet lock control
- ✅ MQTT broker on router

### Phase 2 (Next)
- [ ] D1 Mini rear thermostat
- [ ] D1 Mini front thermostat
- [ ] Temperature sensor integration
- [ ] Relay control for heating/cooling

### Phase 3
- [ ] D1 Mini window blind control
- [ ] Multiple blind positions
- [ ] Motor control with H-bridge

### Phase 4
- [ ] Tank level monitoring (Mopeka sensors)
- [ ] Tank heater control
- [ ] Victron Energy integration

### Phase 5
- [ ] Leveling system control
- [ ] Multiple relay control
- [ ] Leveling sensors

### Phase 6
- [ ] Diesel heater control
- [ ] Engine block heater
- [ ] Home Assistant integration

## Security Considerations

### Current Setup (Local Network)
- Anonymous MQTT (no authentication)
- Local network only (no internet exposure)
- Suitable for isolated RV network

### Future Enhancements
- [ ] Add MQTT username/password
- [ ] Enable MQTT over TLS (port 8883)
- [ ] Implement client certificates
- [ ] Add firewall rules to restrict access
- [ ] Enable WPA3 on router if supported

## Performance Expectations

### Typical Latencies
- WiFi connection: 2-5 seconds
- MQTT connection: 0.5-1 second
- Command transmission: 10-50ms
- Servo movement: 200-500ms
- Total response time: < 1 second

### Network Requirements
- WiFi range: Within 50 feet of router
- Bandwidth: < 1 KB/s per device
- Concurrent devices: 50+ supported by GL-iNet router

## Backup and Recovery

### Configuration Backup
All configuration is in code - backup your project folder:
```bash
tar -czf ChesterTheBus-backup-$(date +%Y%m%d).tar.gz ChesterTheBus-RV-Control/
```

### Router Backup
Backup router config through web interface:
- Login to http://192.168.8.1
- Go to System → Backup
- Download backup file

### Recovery Procedure
1. Restore router from backup
2. Verify Mosquitto is running
3. Re-upload Arduino code to devices
4. Test MQTT connectivity

## Support and Documentation

### Project Files
- `/GIGA_R1_TheBrain/` - Main controller code
- `/D1Mini_CabLock/` - Cabinet lock code
- `/D1 Plumbing system/` - Future plumbing controls
- `SETUP_INSTRUCTIONS.md` - General setup
- This file - System integration guide

### Useful Resources
- PubSubClient: https://github.com/knolleary/pubsubclient
- MQTT Protocol: https://mqtt.org/
- GL-iNet Docs: https://docs.gl-inet.com/
- Arduino GIGA R1: https://docs.arduino.cc/hardware/giga-r1-wifi/

### Getting Help
1. Check serial monitor output at 115200 baud
2. Use mosquitto_pub/sub for testing
3. Review this integration guide
4. Check device-specific MQTT_SETUP_NOTES.md files
