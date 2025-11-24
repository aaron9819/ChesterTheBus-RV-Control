# Home Assistant MQTT Discovery

All ChesterTheBus devices now automatically publish MQTT discovery payloads for seamless integration with Home Assistant.

## Overview

When each device connects to MQTT, it automatically publishes discovery configuration messages to Home Assistant. This allows Home Assistant to automatically detect and create entities for all sensors, switches, controls, and alerts without manual configuration.

## Discovery Topics

Home Assistant discovery uses the following topic pattern:
```
homeassistant/<component>/<device_id>/<object_id>/config
```

## Devices and Entities

### 1. Plumbing System (ESP32-S2)
**Device ID:** `chester_plumbing`  
**Model:** ESP32-S2 Mini

#### Temperature Sensors
- Hydronic Temperature (°C)
- Fresh Water Tank Temperature (°C)
- Grey Water Tank Temperature (°C)
- Return Manifold Temperature (°C)
- Ambient Temperature (°C)

#### Environmental Sensors
- Humidity (%)
- Atmospheric Pressure (hPa)

#### Pump Monitoring
- Hot Water Pump Pressure (PSI)
- Hot Water Pump Speed (%)
- Hot Water Pump Status
- Hot Water Flow Rate (L/min)
- Hot Water Flow Status

#### Switches (Valves & Controls)
- Fresh Water Tank Heater (ON/OFF)
- Grey Water Tank Heater (ON/OFF)
- Rear Loop Valve (OPEN/CLOSE)
- Engine Loop Valve (OPEN/CLOSE)
- Front Loop Valve (OPEN/CLOSE)
- Exhaust Fan (ON/OFF)
- Main Water Pump (ON/OFF)
- Domestic Hot Water (OPEN/CLOSE)

#### Select Entity
- Diesel Heater (OFF / PUMP ONLY / HIGH)

#### Binary Sensors (Alerts)
- Freeze Warning
- High Temperature Alert
- System Error
- Pump Error
- High Humidity Alert (Leak Detection)

#### Lock
- Rear Pass Cabinet Lock

---

### 2. Front Thermostat (ESP32-S2)
**Device ID:** `chester_thermostat`  
**Model:** ESP32-S2 Mini

#### Temperature Sensors
- Temperature (°F)
- Temperature (°C)

#### Climate Entity
- **Thermostat** - Full climate control
  - Current temperature (°F)
  - Set temperature (°F, 50-86°F range)
  - Modes: off, heat
  - Temperature step: 0.5°F

#### Binary Sensors
- Heating Status (ON/OFF)

#### Sensors
- Fan Speed (PWM value)

#### Lock
- Cabinet Lock (KitchenPassSide)

---

### 3. Cabinet Locks (D1 Mini - ESP8266)
**Device IDs:** `chester_cablock_<location>`  
**Model:** D1 Mini (ESP8266)

Each cabinet lock creates its own device:
- chester_cablock_kitchendriverside
- chester_cablock_kitchenpassside
- chester_cablock_kitchenupperpassside
- chester_cablock_reardriverside
- chester_cablock_rearpassside

#### Lock
- Cabinet Lock with LOCK/UNLOCK commands

---

## Home Assistant Integration

### Automatic Discovery
1. Ensure Home Assistant MQTT integration is configured
2. Connect to MQTT broker: `192.168.8.1:1883`
3. Power on any ChesterTheBus device
4. Entities will appear automatically in Home Assistant

### Discovery Configuration Location
- **Settings** → **Devices & Services** → **MQTT**
- All entities will be grouped under their respective devices

### Entity Naming Convention
```
<Device Name> <Entity Name>
```
Examples:
- `Chester Plumbing System Hydronic Temperature`
- `Chester Front Thermostat Thermostat`
- `Cabinet Lock - KitchenDriverSide Cabinet Lock`

## MQTT Topics Used

### Plumbing System
```
State Topics (Sensors):
- HydronicTemperature
- FreshWaterTemperature
- GreyWaterTemperature
- PlumbingTempReturnManifold
- EnvironmentTempAmbient
- EnvironmentHumidity
- EnvironmentPressure
- HotWaterPumpPressure
- HotWaterPumpSpeed
- HotWaterPumpStatus
- HotWaterFlowRate
- HotWaterFlowStatus

Command Topics (Switches):
- FreshWaterHeatCommand (ON/OFF)
- GreyWaterHeatCommand (ON/OFF)
- RearLoopCommand (OPEN/CLOSE)
- EngineLoopCommand (OPEN/CLOSE)
- FrontLoopCommand (OPEN/CLOSE)
- ExhaustFanCommand (ON/OFF)
- MainWaterPumpCommand (ON/OFF)
- DomesticHotWaterCommand (OPEN/CLOSE)

Status Topics:
- FreshWaterHeatStatus
- GreyWaterHeatStatus
- RearLoopStatus
- EngineLoopStatus
- FrontLoopStatus
- ExhaustFanStatus
- MainWaterPumpStatus
- DomesticHotWaterStatus

Alerts:
- AlertFreezeWarning
- AlertHighTemp
- AlertSystemError
- AlertPumpError
- AlertHighHumidity

Diesel Heater:
- DieselHeaterStatus (OFF/PUMP ONLY/HIGH)
- DieselHeaterCommand

Lock:
- CabLockRearPassSideStatus
- CabLockRearPassSideCommand
```

### Front Thermostat
```
Temperature:
- ThermostatTemp (Celsius)
- ThermostatTempF (Fahrenheit)
- ThermostatSetTemp (Celsius)
- ThermostatSetTempF (Fahrenheit)
- ThermostatSetTempFCommand (Set temperature in F)

Climate Control:
- ThermostatMode (off/heat)
- ThermostatModeCommand
- ThermostatHeating (ON/OFF)

Fan:
- FanSpeed (PWM)

Lock:
- CabLockKitchenPassSideStatus
- CabLockKitchenPassSideCommand
```

### Cabinet Locks
```
- CabLock<Location>Status (LOCKED/UNLOCKED)
- CabLock<Location>Command (LOCK/UNLOCK)
- CabLockAllCommand (Controls all locks simultaneously)
```

## Device Class Mappings

Home Assistant uses device classes to provide appropriate icons and UI elements:

- **Temperature sensors:** `device_class: temperature`
- **Humidity sensors:** `device_class: humidity`
- **Pressure sensors:** `device_class: pressure`
- **Switches:** Valves, heaters, fans
- **Binary sensors:** Alerts and warnings
- **Lock:** Cabinet locks with lock/unlock control
- **Climate:** Full thermostat control
- **Select:** Multi-state controls (diesel heater)

## Icons

Custom Material Design Icons (MDI) are assigned to enhance visual identification:

- 🔥 Fire icons for heaters and heating
- 💧 Water icons for tanks and flow
- 🌡️ Thermometer for temperatures
- 🔒 Lock icons for cabinet locks
- ⚠️ Alert icons for warnings
- 🔧 Wrench for pumps and valves
- 💨 Fan icon for exhaust fan

## Retained Messages

Discovery payloads are published with the **retained flag** set to `true`, ensuring:
- Persistence across MQTT broker restarts
- Immediate availability when Home Assistant reconnects
- No need to manually trigger rediscovery

## Rediscovery

To force rediscovery:
1. Restart any ChesterTheBus device
2. It will republish discovery payloads on MQTT connection
3. Home Assistant will update entity configurations

Or manually:
```bash
mosquitto_pub -h 192.168.8.1 -t "homeassistant/status" -m "online"
```

## Removing Devices

If you need to remove a device from Home Assistant:
1. Delete the device from **Settings** → **Devices & Services** → **MQTT**
2. Clear retained discovery messages:
```bash
mosquitto_pub -h 192.168.8.1 -t "homeassistant/<component>/<device_id>/<object_id>/config" -n -r
```

## Troubleshooting

### Entities not appearing
1. Check MQTT integration is enabled in Home Assistant
2. Verify MQTT broker connection (192.168.8.1:1883)
3. Check device logs for "✓ Home Assistant Discovery published"
4. Restart Home Assistant Core

### Entities showing as unavailable
1. Verify device is connected to MQTT
2. Check device is publishing state topics
3. Monitor MQTT traffic: `mosquitto_sub -h 192.168.8.1 -t "#" -v`

### Discovery messages not retained
- Ensure MQTT broker (Mosquitto) has persistence enabled
- Check broker logs for any issues

## Future Enhancements

Potential additions:
- GIGA R1 TheBrain display board discovery
- Additional cabinet lock devices
- Window blind controllers
- Leveling system sensors
- Victron Energy integration via Ekrano GX
- Mopeka tank level sensors

---

**Note:** All devices automatically publish discovery on boot/reconnect. No manual configuration required!
