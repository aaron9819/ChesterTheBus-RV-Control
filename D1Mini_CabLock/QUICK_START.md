# D1 Mini Cabinet Lock - Quick Start Guide

## 🚀 Quick Setup (5 Minutes)

### 1. Hardware Setup
```
Servo Red    → D1 Mini 5V
Servo Brown  → D1 Mini GND
Servo Orange → D1 Mini D4
```

### 2. Upload Code
```bash
cd /Users/aaronlapp/Desktop/ChesterTheBus-RV-Control/D1Mini_CabLock
pio run -t upload
pio device monitor -b 115200
```

### 3. Verify Connection
Look for in serial monitor:
```
WiFi connected!
Attempting MQTT connection to 192.168.8.1:1883...connected
Ready to receive lock/unlock commands via MQTT!
```

### 4. Test Commands
```bash
# Lock
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "LOCK"

# Unlock
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/command" -m "UNLOCK"
```

## ✅ Configuration Summary

| Setting | Value |
|---------|-------|
| WiFi SSID | Chester IOT |
| WiFi Password | 2025Chester9894 |
| MQTT Broker | 192.168.8.1:1883 |
| Servo Pin | D4 |
| Locked Position | 0° |
| Unlocked Position | 90° |

## 🎯 MQTT Topics

| Topic | Direction | Purpose |
|-------|-----------|---------|
| rv/cabinet/command | Subscribe | Receives commands from GIGA R1 |
| rv/cabinet/status | Publish | Sends status to GIGA R1 |

## 📋 Commands

| Command | Action |
|---------|--------|
| LOCK | Move servo to 0°, publish "LOCKED" |
| UNLOCK | Move servo to 90°, publish "UNLOCKED" |
| STATUS | Publish current state (no movement) |
| TOGGLE | Switch between locked/unlocked |

## 🔧 Troubleshooting

### WiFi won't connect
- Check router is on
- Verify password: "2025Chester9894"

### MQTT won't connect
- Check Mosquitto is running: `ps | grep mosquitto`
- Test broker: `mosquitto_pub -h 192.168.8.1 -t "test" -m "hello"`

### Servo won't move
- Check wiring to D4
- Verify power (red wire to 5V)
- Check ground (brown wire to GND)

### Adjust servo positions
Edit lines 24-25 in main.cpp:
```cpp
#define LOCKED_POSITION 0    // Change 0-180
#define UNLOCKED_POSITION 90 // Change 0-180
```

## 📖 Full Documentation

- **MQTT_SETUP_NOTES.md** - Complete configuration details
- **VERIFICATION_CHECKLIST.md** - Detailed testing procedures
- **../SYSTEM_INTEGRATION_GUIDE.md** - Overall system architecture

## 🎮 Integration with GIGA R1

Once both devices are running:
1. Touch button on GIGA R1 display
2. Servo on D1 Mini moves
3. GIGA R1 display updates with status

**That's it!** The system is fully operational.
