# Chester the Bus - RV Control System

<div align="center">

**A comprehensive Arduino-based automation and monitoring system for RV/bus conversions**

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Ready-orange.svg)](https://platformio.org/)
[![Arduino](https://img.shields.io/badge/Arduino-Compatible-blue.svg)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

</div>

---

## 🚐 Overview

Chester the Bus is a modular, distributed control system designed for RV/bus conversions. The system uses multiple ESP8266 D1 Mini boards and an Arduino GIGA R1 WiFi as the main controller, all communicating via MQTT over a local network provided by a GL-AR300M16 router.

### Key Features

- 🌡️ **Environmental Monitoring** - Temperature, humidity, and pressure sensing
- 💧 **Plumbing Automation** - Tank heating, water flow detection, and hot water control
- 🔥 **Heating Control** - Multi-stage diesel heater control
- 🔒 **Security** - Cabinet lock automation via servo motors
- 📊 **Remote Monitoring** - MQTT integration with Home Assistant
- 🔄 **OTA Updates** - Wireless firmware updates for all boards
- 💾 **Configuration Persistence** - Settings saved to EEPROM
- 📱 **Touch Display** - GIGA R1 with GigaDisplay for local control

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    GL-AR300M16 Router                        │
│                 (192.168.8.1 - MQTT Broker)                  │
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
┌───────▼────────┐   ┌────────▼────────┐   ┌───────▼────────┐
│  GIGA R1 WiFi  │   │  D1 Mini        │   │  D1 Mini       │
│  Main Brain    │   │  Plumbing       │   │  Environment   │
│  + Display     │   │  System         │   │  + Hot Water   │
└────────────────┘   └─────────────────┘   └────────────────┘
```

### Hardware Components

#### Main Controller
- **Arduino GIGA R1 WiFi** with GigaDisplay Shield
  - Central control interface
  - Touch screen UI
  - System monitoring and control

#### Plumbing System Board (D1 Mini Lite)
- 6x DS18B20 temperature sensors (OneWire)
- 8-channel relay module
- Controls:
  - Fresh water tank heater
  - Grey water tank heater
  - Exhaust fan (temperature-based)
  - Hydronic system valves (rear/engine loops)
  - 3-stage diesel heater control

#### Environmental & Hot Water Board (D1 Mini Lite)
- BME280 sensor (temp/humidity/pressure)
- YF-S201 Hall Effect flow sensor
- Hot water solenoid valve
- Cabinet lock servo motor
- Automatic hot water circulation

#### Network Infrastructure
- **GL-AR300M16** Smart Router
  - Local WiFi network
  - Mosquitto MQTT broker
  - Acts as system backbone

---

## 📁 Project Structure

```
ChesterTheBus-RV-Control/
├── GIGA_R1_TheBrain/          # Main controller with display
│   ├── src/main.cpp
│   └── platformio.ini
├── D1 Plumbing system/        # Temperature & heating control
│   ├── src/main.cpp
│   └── platformio.ini
├── D1Mini_CabLock_RearPassSide/  # Environment & hot water
│   ├── src/main.cpp
│   └── platformio.ini
├── D1Mini_CabLock_*/          # Additional cabinet lock boards
├── Documentation/
│   ├── BOARD_ARCHITECTURE.md
│   ├── BME280_INTEGRATION_GUIDE.md
│   ├── BUILD_TIMESTAMP_GUIDE.md
│   ├── CACHE_OPTIMIZATION_GUIDE.md
│   ├── DIESEL_HEATER_3STATE_CONTROL.md
│   ├── EEPROM_PERSISTENCE_GUIDE.md
│   ├── FLOW_SENSOR_INTEGRATION.md
│   ├── OTA_UPDATE_GUIDE.md
│   ├── QUICK_REFERENCE.md
│   └── SYSTEM_INTEGRATION_GUIDE.md
└── README.md
```

---

## 🚀 Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (IDE or CLI)
- [Visual Studio Code](https://code.visualstudio.com/) (recommended)
- USB cables for initial programming
- Hardware components (see Bill of Materials)

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/aaron9819/ChesterTheBus-RV-Control.git
   cd ChesterTheBus-RV-Control
   ```

2. **Open in PlatformIO**
   ```bash
   # Open in VS Code with PlatformIO extension
   code .
   ```

3. **Configure WiFi credentials**

   Update WiFi settings in each project's `main.cpp`:
   ```cpp
   const char* ssid = "Chester IOT";
   const char* password = "2025Chester9894";
   ```

4. **Build and upload firmware**
   ```bash
   # For Plumbing System
   cd "D1 Plumbing system"
   pio run --target upload

   # For Environment Board
   cd "../D1Mini_CabLock_RearPassSide"
   pio run --target upload

   # For GIGA R1
   cd "../GIGA_R1_TheBrain"
   pio run --target upload
   ```

---

## 📡 MQTT Topics

### Plumbing System

| Topic | Type | Description |
|-------|------|-------------|
| `HydronicTemperature` | Sensor | Hydronic system temperature |
| `FreshWaterTemperature` | Sensor | Fresh water tank temperature |
| `GreyWaterTemperature` | Sensor | Grey water tank temperature |
| `PlumbingTempAmbient` | Sensor | Ambient temperature |
| `DieselHeaterCommand` | Command | OFF/MID/HIGH |
| `FreshWaterHeatMode` | Command | AUTO/MANUAL |

### Environmental System

| Topic | Type | Description |
|-------|------|-------------|
| `EnvironmentTempAmbient` | Sensor | Temperature (°F) |
| `EnvironmentHumidity` | Sensor | Relative humidity (%) |
| `EnvironmentPressure` | Sensor | Atmospheric pressure (hPa) |
| `HotWaterFlowRate` | Sensor | Flow rate (L/min) |
| `HotWaterSolenoidMode` | Command | AUTO/MANUAL |

---

## 🛠️ Features

### Automatic Tank Heating
- Prevents freezing in winter
- Configurable temperature thresholds
- Hysteresis to prevent rapid cycling
- Works offline (local automation)

### Hot Water Circulation
- Flow-triggered hot water valve
- Instant hot water at faucets
- Energy efficient (only runs when needed)
- Manual override available

### Diesel Heater Control
- 3-stage relay control (OFF/MID/HIGH)
- MQTT-based remote control
- Safety shutoff during OTA updates

### Cabinet Lock System
- Servo-based electronic locks
- Individual or group control
- Status feedback via MQTT
- Low power mode when idle

### Environmental Monitoring
- BME280 sensor integration
- Temperature, humidity, pressure
- 5-second update intervals
- MQTT publishing for Home Assistant

---

## 🔧 Configuration

### EEPROM Settings Persistence

Temperature thresholds are saved to EEPROM and persist across reboots:

```bash
# Set fresh water target temperature
mosquitto_pub -h 192.168.8.1 -t FreshWaterHeatTempSet -m "42.0"

# Set exhaust fan thresholds
mosquitto_pub -h 192.168.8.1 -t ExhaustFanTempHigh -m "85.0"
mosquitto_pub -h 192.168.8.1 -t ExhaustFanTempLow -m "75.0"
```

### OTA Updates

All boards support Over-The-Air updates:

```bash
# Update via PlatformIO
pio run --target upload --upload-port D1Mini-Plumbing.local

# Or use the web interface
# http://D1Mini-Plumbing.local:8266/update
# Password: Chester2025
```

---

## 📊 Home Assistant Integration

Example `configuration.yaml`:

```yaml
mqtt:
  sensor:
    - name: "RV Fresh Water Temperature"
      state_topic: "FreshWaterTemperature"
      unit_of_measurement: "°F"

    - name: "RV Environment Temperature"
      state_topic: "EnvironmentTempAmbient"
      unit_of_measurement: "°F"

    - name: "RV Humidity"
      state_topic: "EnvironmentHumidity"
      unit_of_measurement: "%"

  switch:
    - name: "RV Fresh Water Heater"
      command_topic: "FreshWaterHeatCommand"
      state_topic: "FreshWaterHeatStatus"
      payload_on: "ON"
      payload_off: "OFF"
```

---

## 🐛 Troubleshooting

### Board won't connect to WiFi
1. Verify SSID and password in code
2. Check router is powered on
3. Ensure board is in range
4. Try USB serial monitor to see connection attempts

### MQTT commands not working
1. Verify Mosquitto is running on router
2. Check MQTT broker IP (192.168.8.1)
3. Test with mosquitto_sub to see messages
4. Verify topic names match

### OTA update fails
1. Ensure board is on same network
2. Check hostname resolution (`.local`)
3. Verify password is correct
4. Try `pio device list` to see available boards

### Temperature sensors not reading
1. Check OneWire bus wiring
2. Verify 4.7kΩ pullup resistor
3. Check sensor addresses in code
4. Look for -999.0 readings (disconnected sensor)

---

## 📖 Documentation

Detailed documentation is available in the repository:

- **[BOARD_ARCHITECTURE.md](BOARD_ARCHITECTURE.md)** - Complete system design
- **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Command reference
- **[OTA_UPDATE_GUIDE.md](OTA_UPDATE_GUIDE.md)** - Wireless update procedures
- **[EEPROM_PERSISTENCE_GUIDE.md](EEPROM_PERSISTENCE_GUIDE.md)** - Configuration storage
- **[BUILD_TIMESTAMP_GUIDE.md](BUILD_TIMESTAMP_GUIDE.md)** - Build tracking

---

## 🤝 Contributing

Contributions are welcome! This is a personal RV project but others building similar systems may find it useful.

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **PlatformIO** - Development platform
- **Arduino Community** - Libraries and support
- **Adafruit** - BME280 and sensor libraries
- **GL.iNet** - Compact router hardware
- **Home Assistant** - MQTT integration inspiration

---

## 📞 Contact

**Project Maintainer:** Aaron

**Repository:** [https://github.com/aaron9819/ChesterTheBus-RV-Control](https://github.com/aaron9819/ChesterTheBus-RV-Control)

---

<div align="center">

**Built with ❤️ for mobile living**

🚐 **Chester the Bus** 🚐

</div>
