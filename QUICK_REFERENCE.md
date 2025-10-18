# ChesterTheBus RV Control - Quick Reference

## 🚀 Getting Started

### Initial Setup (Run Once)
```bash
./setup-env.sh          # Configure environment variables
source ~/.zshrc         # Reload shell config
```

## 🧹 Cache Management

```bash
./clear-cache.sh        # Clear all caches
./clear-cache.sh soft   # Clear only build artifacts
./clear-cache.sh pio    # Clear only PlatformIO cache
./clear-cache.sh vscode # Clear only VS Code cache
```

## 🔨 Building Projects

### GIGA R1 WiFi (Main Brain)
```bash
cd GIGA_R1_TheBrain
pio run                 # Build
pio run -t upload       # Upload via USB
pio device monitor      # View serial output
```

### D1 Plumbing System
```bash
cd "D1 Plumbing system"
pio run -t upload       # Build and upload
pio device monitor -b 115200
```

### Cabinet Locks
```bash
cd D1Mini_CabLock
pio run -t upload

# Or specific locks:
cd D1Mini_CabLock_KitchenDriverSide
cd D1Mini_CabLock_KitchenPassSide
cd D1Mini_CabLock_RearDriverSide
```

### Mopeka Tank Level Sensors
```bash
cd "Mopeka Testing"
pio run -t upload
pio device monitor
```

## 📡 OTA (Wireless) Updates

```bash
# List available devices
pio device list

# Upload wirelessly to GIGA
cd GIGA_R1_TheBrain
pio run -t upload --upload-port GIGA-R1-TheBrain.local

# Upload to D1 Mini (Plumbing)
cd "D1 Plumbing system"
pio run -t upload --upload-port D1Mini-Plumbing.local
```

## 🔐 Security

### Environment Variables
```bash
# Check API key is set
echo $ANTHROPIC_API_KEY

# Check PlatformIO cache
echo $PLATFORMIO_CACHE_DIR

# Project path
echo $CHESTER_PROJECT_PATH
```

### Never Commit These Files!
- `secrets.h`
- `.env`
- `*.key`
- `.vscode/.cache/`
- `.pio/`

## 🐛 Troubleshooting

### Build fails
```bash
./clear-cache.sh pio
cd <project-folder>
pio lib install
pio run
```

### IntelliSense broken
```bash
./clear-cache.sh vscode
# Reload VS Code window
```

### Out of disk space
```bash
# Check sizes
du -sh .pio .vscode/.cache

# Prune old libraries
pio system prune

# Full clean
./clear-cache.sh
```

### Upload fails
```bash
# Check connected devices
pio device list

# Try different port
pio run -t upload --upload-port /dev/cu.usbmodem*
```

## 📊 System Status

### Check MQTT Broker (GL-AR300M16)
```bash
# Ping router
ping 192.168.8.1

# Test MQTT (if mosquitto clients installed)
mosquitto_sub -h 192.168.8.1 -t "#" -v
```

### Monitor Serial Output
```bash
# GIGA R1 (115200 baud)
pio device monitor -b 115200

# D1 Mini (115200 baud)
pio device monitor -b 115200
```

## 🏠 Home Assistant Integration

### MQTT Topics for ChesterTheBus

**Tank Temperatures:**
- `HydronicTemperature`
- `FreshWaterTemperature`
- `GreyWaterTemperature`
- `PlumbingAmbientTemperature`

**Tank Levels (Mopeka):**
- `rv/tank/fresh/level`
- `rv/tank/grey/level`
- `rv/tank/propane/level`
- `rv/tank/diesel/level`

**Cabinet Locks:**
- `rv/cabinet/kitchen1/command` (LOCK/UNLOCK)
- `rv/cabinet/kitchen1/status` (LOCKED/UNLOCKED)
- `rv/cabinet/all/command` (Control all)

**Relay Controls:**
- `FreshWaterHeatCommand` (ON/OFF)
- `GreyWaterHeatCommand` (ON/OFF)
- `ExhaustFanCommand` (ON/OFF)
- `DieselHeaterCommand` (ON/OFF)

## 🔧 VS Code Commands

```
Ctrl+Shift+P (Cmd+Shift+P on Mac) then:
  - PlatformIO: Build
  - PlatformIO: Upload
  - PlatformIO: Clean
  - PlatformIO: Serial Monitor
  - Reload Window
```

## 📦 Library Updates

```bash
# Update all libraries for a project
cd <project-folder>
pio lib update

# Update PlatformIO core
pio upgrade

# Update platform (ESP8266, ESP32, Arduino)
pio platform update
```

## 🌐 Network Configuration

**WiFi SSID:** `Chester IOT`
**WiFi Password:** `2025Chester9894`
**MQTT Broker:** `192.168.8.1:1883`
**Router:** GL-AR300M16

## 📁 Project Structure

```
ChesterTheBus-RV-Control/
├── GIGA_R1_TheBrain/           # Main controller with display
├── D1 Plumbing system/         # Temperature sensors & safety
├── D1Mini_CabLock/             # Base cabinet lock controller
├── D1Mini_CabLock_*/           # Individual cabinet locks
├── Mopeka Testing/             # Tank level sensor integration
├── clear-cache.sh              # Cache management
├── setup-env.sh                # Environment setup
└── CACHE_OPTIMIZATION_GUIDE.md # Full documentation
```

## 📞 Emergency Commands

### Reboot via MQTT
```bash
mosquitto_pub -h 192.168.8.1 -t "D1PlumbingOTACommand" -m "REBOOT"
```

### Disable All Heaters (Safety)
```bash
mosquitto_pub -h 192.168.8.1 -t "FreshWaterHeatCommand" -m "OFF"
mosquitto_pub -h 192.168.8.1 -t "GreyWaterHeatCommand" -m "OFF"
mosquitto_pub -h 192.168.8.1 -t "DieselHeaterCommand" -m "OFF"
```

### Unlock All Cabinets
```bash
mosquitto_pub -h 192.168.8.1 -t "rv/cabinet/all/command" -m "UNLOCK"
```

## 💡 Tips

- Use `pio device monitor` to watch serial output while debugging
- Cache clears are rarely needed - rebuilds take longer!
- OTA updates only work when devices are on same WiFi network
- Check router (192.168.8.1) if MQTT fails
- D1 Mini projects have critical automation that runs even offline

---

**ChesterTheBus RV Control System v1.0**
*Keeping your RV smart, safe, and comfortable on the road! 🚌*
