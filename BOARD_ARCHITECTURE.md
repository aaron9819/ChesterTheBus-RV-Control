# ChesterTheBus RV Control System - Board Architecture

## System Overview

Multi-board ESP8266-based RV automation system with MQTT communication and centralized control via Arduino GIGA R1 WiFi.

---

## Board Layout

### 1. **GIGA R1 WiFi** (Main Brain) 📱
**Hardware**: Arduino GIGA R1 WiFi with 800x480 touch display

**Responsibilities**:
- Main user interface (touch display)
- Cabinet lock control buttons
- Lock All / Unlock All commands
- System status display
- MQTT command publisher

**Firmware**: `GIGA_R1_TheBrain/src/main.cpp` (v2.0.0)

**MQTT Topics**:
- Publishes: `CabLock*Command`, `*Command` (for all systems)
- Subscribes: `CabLock*Status`, `*Status` (for system feedback)

---

### 2. **D1 Mini - Plumbing System** 🚰
**Hardware**: ESP8266 D1 Mini + 8-channel relay + 6x DS18B20 sensors + flow sensor

**Responsibilities**:
- 6 DS18B20 temperature sensors (hydronic, fresh water, grey water, ambient, supply/return manifolds)
- Water flow rate monitoring (Hall Effect sensor)
- Tank heater automation (fresh + grey water anti-freeze)
- Exhaust fan automation (temperature-based)
- Diesel heater control (2 relays)
- Hydronic system valve control (rear loop + engine loop)
- Critical automation (runs offline)
- Freeze/high-temp alerts

**Pin Assignment**:
```
D0 = Fresh Water Heater (relay)
D1 = DS18B20 OneWire (6 sensors)
D2 = Grey Water Heater (relay)
D3 = Exhaust Fan (relay)
D4 = Diesel Heater #2 (relay)
D5 = Rear Loop Valve (relay)
D6 = Engine Loop Valve (relay)
D7 = Diesel Heater #1 (relay)
D8 = Water Flow Sensor (interrupt)
```

**Firmware**: `D1 Plumbing system/src/main.cpp` (v1.2.0)

**OTA**: `D1Mini-Plumbing.local`

**MQTT Topics**:
- Publishes: Temperature readings, flow rate, relay status, alerts
- Subscribes: Relay commands, mode changes, configuration

---

### 3. **D1 Mini - Cabinet Lock (Kitchen Driver Side)** 🔒
**Hardware**: ESP8266 D1 Mini + Servo motor

**Responsibilities**:
- Single cabinet lock servo control
- Lock/unlock commands via MQTT
- Status reporting

**Pin Assignment**:
```
D4 = Servo motor (PWM)
```

**Firmware**: `D1Mini_CabLock_KitchenDriverSide/src/main.cpp` (v1.1.0)

**OTA**: `CabLock-KitchenDriverSide.local`

**MQTT Topics**:
- Publishes: `CabLockKitchenDriverSideStatus`
- Subscribes: `CabLockKitchenDriverSideCommand`, `CabLockAllCommand`

---

### 4. **D1 Mini - Cabinet Lock (Kitchen Pass Side)** 🔒
**Hardware**: ESP8266 D1 Mini + Servo motor

**Responsibilities**:
- Single cabinet lock servo control
- Lock/unlock commands via MQTT
- Status reporting

**Pin Assignment**:
```
D4 = Servo motor (PWM)
```

**Firmware**: `D1Mini_CabLock_KitchenPassSide/src/main.cpp` (v1.1.0)

**OTA**: `CabLock-KitchenPassSide.local`

**MQTT Topics**:
- Publishes: `CabLockKitchenPassSideStatus`
- Subscribes: `CabLockKitchenPassSideCommand`, `CabLockAllCommand`

---

### 5. **D1 Mini - Cabinet Lock (Rear Driver Side)** 🔒
**Hardware**: ESP8266 D1 Mini + Servo motor

**Responsibilities**:
- Single cabinet lock servo control
- Lock/unlock commands via MQTT
- Status reporting

**Pin Assignment**:
```
D4 = Servo motor (PWM)
```

**Firmware**: `D1Mini_CabLock_RearDriverSide/src/main.cpp` (v1.1.0)

**OTA**: `CabLock-RearDriverSide.local`

**MQTT Topics**:
- Publishes: `CabLockRearDriverSideStatus`
- Subscribes: `CabLockRearDriverSideCommand`, `CabLockAllCommand`

---

### 6. **D1 Mini - Environment & Hot Water** 🌡️💧 (PLANNED)
**Hardware**: ESP8266 D1 Mini + BME280 sensor + relay + servo

**Responsibilities**:
- BME280 sensor (ambient temp, humidity, pressure)
- Hot water solenoid control (automated by flow sensor)
- Cabinet lock servo (Rear Passenger Side)
- Flow-triggered hot water automation

**Planned Pin Assignment**:
```
D1 = I2C SCL (BME280)
D2 = I2C SDA (BME280)
D4 = Cabinet Lock Servo (PWM)
D5 = Hot Water Solenoid (relay)
```

**Firmware**: TO BE CREATED

**OTA**: TBD (suggest: `D1Mini-Environment.local`)

**MQTT Topics**:
- Publishes: Ambient temp/humidity/pressure, hot water status, cabinet lock status
- Subscribes: `FlowSensorStatus` (from Plumbing board), hot water override commands, cabinet lock commands

**Automation Logic**:
```cpp
if (flowSensorStatus == "FLOWING") {
    openHotWaterSolenoid();
} else {
    closeHotWaterSolenoid();
}
```

---

## Network Architecture

### WiFi Network
**SSID**: `Chester IOT`
**Password**: `2025Chester9894`

### MQTT Broker
**Location**: GL-AR300M16 Router
**IP Address**: `192.168.8.1`
**Port**: `1883`
**Authentication**: None

### OTA Configuration
**Password**: `Chester2025`
**Port**: `8266`
**Protocol**: ESP OTA (mDNS)

---

## Communication Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    GIGA R1 WiFi (Main Brain)                    │
│                     Touch Display Interface                      │
└────────────┬───────────────────────────────────┬────────────────┘
             │                                   │
             │         MQTT Broker               │
             │      (192.168.8.1:1883)           │
             │                                   │
    ┌────────┴────────┐                ┌────────┴────────────┐
    │                 │                │                     │
┌───▼────┐  ┌────▼────┐  ┌────▼────┐  │  ┌────▼────┐  ┌────▼────┐
│D1 Mini │  │ D1 Mini │  │ D1 Mini │  │  │ D1 Mini │  │ D1 Mini │
│Plumbing│  │CabLock 1│  │CabLock 2│  │  │CabLock 3│  │Environ. │
└────────┘  └─────────┘  └─────────┘  │  └─────────┘  └─────────┘
                                       │
                                       └──► Flow Sensor Data
```

---

## Data Flow Examples

### Example 1: Lock All Cabinets
1. User presses "Lock All" on GIGA display
2. GIGA publishes: `CabLockAllCommand = "LOCK"`
3. All 3 cabinet lock D1 Minis receive command
4. Each servo moves to locked position (90°)
5. Each D1 Mini publishes: `CabLock*Status = "LOCKED"`
6. GIGA display updates button states

### Example 2: Flow-Triggered Hot Water
1. User turns on shower (cold water flows)
2. Plumbing D1 Mini detects flow via Hall sensor
3. Publishes: `FlowSensorStatus = "FLOWING"`
4. Environment D1 Mini receives status
5. Opens hot water solenoid relay
6. Hot water mixes with cold water
7. When flow stops, solenoid closes after delay

### Example 3: Freeze Protection (Offline)
1. Temperature drops below 40°F
2. Plumbing D1 Mini runs AUTO mode (no WiFi needed)
3. Activates fresh water tank heater
4. Publishes alert (if WiFi available): `AlertFreezeWarning = "ACTIVE"`
5. GIGA displays warning (if connected)
6. Heater remains on until temp > 45°F (5° hysteresis)

---

## Power Management

### RV Battery Conservation
- Servo motors detach after movement (eliminates holding current)
- Critical relays only (heaters, exhaust fan)
- Non-critical relays controlled by GIGA (manual override)
- ESP8266 boards: ~80mA each when active
- Relays: ~70mA per active relay
- Total idle: ~0.5A @ 12V = 6W

### OTA Safety
- All relays turn OFF during firmware updates
- Prevents stuck-on relays during update failure
- Critical for heater safety

---

## Failure Modes & Redundancy

### WiFi Loss
- ✅ Plumbing system continues critical automation
- ✅ Tank heaters operate in AUTO mode
- ✅ Exhaust fan operates in AUTO mode
- ❌ GIGA display shows disconnected
- ❌ Manual relay control unavailable

### MQTT Broker Failure
- ✅ Same as WiFi loss (local automation continues)
- ❌ No inter-board communication
- ❌ Flow sensor hot water automation may fail

### Individual Board Failure
- ❌ That subsystem offline
- ✅ Other boards continue operating
- ✅ GIGA shows status unavailable
- ✅ Modular design allows easy replacement

### GIGA Display Failure
- ✅ All D1 boards continue autonomous operation
- ✅ MQTT still functions
- ❌ No touch control interface
- Fallback: Use MQTT client on phone/tablet

---

## Firmware Versions

| Board | Version | Last Updated | Notes |
|-------|---------|--------------|-------|
| GIGA R1 | v2.0.0 | Oct 7, 2025 | Touch fix, double-tap fix |
| D1 Plumbing | v1.2.0 | Oct 18, 2025 | Flow sensor added |
| CabLock Driver | v1.1.0 | Oct 7, 2025 | OTA + deduplication |
| CabLock Pass | v1.1.0 | Oct 7, 2025 | OTA + deduplication |
| CabLock Rear | v1.1.0 | Oct 7, 2025 | OTA + deduplication |
| D1 Environment | TBD | Not created | Planned |

---

## Expansion Capabilities

### Available Expansion Options:
1. **PCF8574 I2C GPIO Expander** - Add 8 more pins to any D1 board
2. **Additional D1 Minis** - Modular system supports unlimited boards
3. **MCP23017 I2C GPIO Expander** - Add 16 more pins (I2C)
4. **Mopeka Tank Sensors** - Bluetooth tank level monitoring
5. **Victron Cerbo GX Integration** - Solar system monitoring
6. **External Control Screens** - Additional touch displays

---

## Development Notes

### Build System
- PlatformIO for all ESP8266/ESP32 boards
- Arduino IDE compatible
- Shared cache enabled for fast compilation

### OTA Updates
- All D1 boards support OTA (ArduinoOTA library)
- GIGA R1 uses USB upload only (no OTA support yet)
- Update command: `pio run --target upload --upload-port <hostname>.local`

### Debugging
- Serial output: 115200 baud
- MQTT topic monitoring: `mosquitto_sub -h 192.168.8.1 -t '#'`
- WiFi signal strength monitoring: Published periodically

---

## Future Enhancements

### Short Term:
- [ ] Create D1 Environment board firmware
- [ ] Add BME280 sensor integration
- [ ] Implement flow-triggered hot water automation
- [ ] Add 4th cabinet lock servo

### Medium Term:
- [ ] Water usage tracking and reporting
- [ ] Leak detection (abnormal flow patterns)
- [ ] Energy consumption monitoring
- [ ] Remote access via VPN/cloud

### Long Term:
- [ ] Machine learning for usage prediction
- [ ] Voice control integration (Alexa/Google)
- [ ] Mobile app (replace GIGA for some functions)
- [ ] Integration with RV park systems

---

## Troubleshooting Quick Reference

### Board Won't Connect to WiFi
1. Check SSID/password in code
2. Verify router is powered on
3. Check WiFi signal strength
4. Try manual power cycle of board

### OTA Update Fails
1. Verify board is online (ping hostname.local)
2. Check OTA password matches
3. Ensure board not in critical operation
4. Try USB upload as fallback

### MQTT Commands Not Working
1. Verify broker running (ping 192.168.8.1)
2. Check topic names match exactly (case-sensitive)
3. Monitor MQTT traffic with mosquitto_sub
4. Verify board subscribed to correct topics

### Sensor Readings Incorrect
1. Check sensor wiring and power
2. Verify pull-up resistors (DS18B20: 4.7kΩ)
3. Calibrate sensors if needed
4. Check for loose connections

---

*Last Updated: October 18, 2025*
*System Version: v2.0*
