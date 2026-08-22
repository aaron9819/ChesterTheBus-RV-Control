# MQTT Topic Reference

Complete topic contract for every board, transcribed from the firmware. Broker is
Mosquitto on `192.168.8.1:1883`, anonymous access.

## Conventions

- **Topic names are flat PascalCase with no slashes** — `FreshWaterHeatCommand`, not
  `plumbing/fresh_water/heat/set`. The only exceptions are the `homeassistant/...`
  discovery tree and the `Debug/SensorIndex_N` diagnostics.
- **Command and state topics are always separate.** A board subscribes to `…Command` and
  publishes to `…Status` / `…State`. This is deliberate: subscribing to a retained state
  topic that the board itself publishes creates a feedback loop on reconnect.
- **State is retained, telemetry is not.** Valve/lock/mode/setpoint states publish with
  `retain=true` so a restarted broker or a fresh Home Assistant instance sees current
  reality immediately. Fast-moving sensor readings publish unretained.
- Payloads are bare ASCII strings — `ON`, `OPEN`, `LOCKED`, `72.5`. No JSON except in
  discovery configs.

---

## Plumbing (ESP32-S2)

### Published — telemetry (not retained)

| Topic | Payload | Cadence |
|---|---|---|
| `HydronicTemperature` | °F, e.g. `142.3` | 5 s |
| `FreshWaterTemperature` | °F (5-sample average) | 5 s |
| `GreyWaterTemperature` | °F (5-sample average) | 5 s |
| `PlumbingTempReturnManifold` | °F | 5 s |
| `EnvironmentTempAmbient` | °F, from BME280 | 5 s |
| `EnvironmentHumidity` | % RH | 5 s |
| `EnvironmentPressure` | hPa | 5 s |
| `HotWaterFlowRate` | L/min, e.g. `2.34` | 1 s |
| `HotWaterFlowStatus` | `FLOWING` / `STOPPED` | 1 s |
| `PlumbingUptime` | seconds since boot | 60 s |
| `PlumbingFirmwareVersion` | e.g. `v3.1.0` | 60 s |
| `Debug/SensorIndex_0` … `_3` | °F per OneWire index | 30 s |

`HotWaterFlowStatus` publishes the literal string `STOPPED`. (The source comment says
`NOT_FLOWING`; the comment is wrong, the code is right.)

### Published — state (retained)

| Topic | Payload |
|---|---|
| `PlumbingSystemStatus` | `ONLINE` / `OFFLINE` (LWT) / `LEAK_DETECTED_SHUTDOWN` |
| `ManifoldPressure` | PSI, 0–85 |
| `FreshWaterHeatStatus` | `ON` / `OFF` |
| `GreyWaterHeatStatus` | `ON` / `OFF` |
| `ExhaustFanStatus` | `ON` / `OFF` |
| `RearLoopStatus` | `OPEN` / `CLOSE` |
| `EngineLoopStatus` | `OPEN` / `CLOSE` |
| `FrontLoopStatus` | `OPEN` / `CLOSE` |
| `DieselHeaterStatus` | `ON` / `OFF` |
| `ElectricHeaterStatus` | `ON` / `OFF` |
| `MainWaterPumpStatus` | `ON` / `OFF` |
| `DomesticHotWaterStatus` | `OPEN` / `CLOSE` |
| `CabLockRearPassSideStatus` | `LOCKED` / `UNLOCKED` |
| `PlumbingMode` | `AUTO` / `MANUAL` |
| `FreshWaterHeatTempSet` | °F setpoint |
| `GreyWaterHeatTempSet` | °F setpoint |
| `ExhaustFanTempSet` | °F setpoint |
| `PlumbingAlert` | descriptive text, `OKAY` when clear |

### Subscribed

| Topic | Accepted payloads | Effect |
|---|---|---|
| `FreshWaterHeatCommand` | `ON` / anything else = off | Fresh tank heater relay |
| `GreyWaterHeatCommand` | `ON` / anything else = off | Grey tank heater relay |
| `ExhaustFanCommand` | `ON` / anything else = off | Exhaust fan relay |
| `RearLoopCommand` | `ON` or `OPEN` / else close | Rear loop valve |
| `EngineLoopCommand` | `ON` or `OPEN` / else close | Engine loop valve; request persisted to NVS |
| `FrontLoopCommand` | `ON` or `OPEN` / else close | Front loop valve; request persisted to NVS |
| `DieselHeaterCommand` | `ON` / else off | 3-relay diesel heater group |
| `ElectricHeaterCommand` | `ON` / else off | Electric heater (active HIGH) |
| `MainWaterPumpCommand` | `START`/`ON` or `STOP`/`OFF` | Main pump relay |
| `DomesticHotWaterCommand` | `OPEN` / `CLOSE` | DHW solenoid (also driven automatically by flow) |
| `PlumbingModeCommand` | `AUTO` / `MANUAL` | Global automation enable; persisted |
| `FreshWaterHeatTempSetCommand` | float, accepted 32–120 °F | Fresh tank setpoint; persisted |
| `GreyWaterHeatTempSetCommand` | float, accepted 32–120 °F | Grey tank setpoint; persisted |
| `ExhaustFanTempSetCommand` | float, accepted 32–140 °F | Fan setpoint; persisted |
| `ClearHighHumidityCommand` | `clear` / `CLEAR` / `true` / `TRUE` | Clears the leak lockout |
| `CabLockRearPassSideCommand` | `LOCK` / `UNLOCK` / `TOGGLE` / `STATUS` | Rear pass cabinet servo |
| `CabLockAllCommand` | `LOCK` / `UNLOCK` / `TOGGLE` / `STATUS` | Group lock control |

Out-of-range setpoints are silently discarded — the board neither applies them nor
republishes the old value, so a rejected write leaves Home Assistant briefly out of sync
until the next 10 s status publish corrects it.

---

## FrontThermostat (ESP32-S2)

### Published (all retained)

| Topic | Payload |
|---|---|
| `ThermostatTempF` | current °F (BME280 + −6.0 °C offset) |
| `ThermostatHumidity` | % RH |
| `ThermostatPressure` | hPa |
| `ThermostatSetTempF` | setpoint °F |
| `ThermostatMode` | `off` / `heat` |
| `ThermostatHeating` | `ON` / `OFF` |
| `ThermostatFanPercent` | `0`–`100` |
| `ThermostatFanPreset` | `auto` / `low` / `medium` / `high` |
| `CabLockKitchenPassSideStatus` | `LOCKED` / `UNLOCKED` / `UNKNOWN` |

### Published as a command to another board

| Topic | Payload | Consumer |
|---|---|---|
| `EngineLoopCommand` | `OPEN` / `CLOSE`, unretained | Plumbing board |

### Subscribed

| Topic | Accepted payloads | Effect |
|---|---|---|
| `ThermostatSetTempFCommand` | float, accepted 50–86 °F | Setpoint; persisted to EEPROM |
| `ThermostatModeCommand` | `off` / `heat` / `auto` (case-insensitive) | `heat` and `auto` both select HEAT_AUTO |
| `ThermostatFanCommand` | `ON` / `OFF` | Manual fan override; `ON` with no prior speed = 50 % |
| `ThermostatFanSpeedCommand` | integer 0–100 | Manual fan percentage |
| `ThermostatFanPresetCommand` | `auto` / `low` / `medium` / `high` | 0 / 33 / 66 / 100 % |
| `CabLockKitchenPassSideCommand` | `LOCK` / `UNLOCK` | Cabinet servo |
| `CabLockAllCommand` | `LOCK` / `UNLOCK` | Group lock control |

This board has **no Last Will**. `ThermostatMode` staying at its last retained value after a
crash is expected — there is no availability topic to fall back on.

---

## AltInverterCharger (ESP32-S2)

| Topic | Direction | Payload |
|---|---|---|
| `AltInverterStatus` | publish, retained, **LWT** | `ONLINE` / `OFFLINE` |
| `AltInverterBatteryVoltage` | publish, retained | calibrated volts, e.g. `12.53` |
| `AltInverterBatteryRawMV` | publish, retained | averaged mV at the ADC pin |
| `AltInverterCalScale` | publish, retained | current calibration scale |
| `AltInverterRelayState` | publish, retained | `ON` during the 500 ms pulse, `OFF` after |
| `AltInverterTemperature` | publish, retained | DS18B20 °C |
| `AltInverterRSSI` | publish, retained | WiFi signal, dBm |
| `AltInverterPowerCommand` | subscribe | `PRESS` or `TOGGLE` → one relay pulse |
| `AltInverterCalibrateCommand` | subscribe | actual battery volts, accepted 5–40 V |

Voltage publishes on a ≥0.05 V change (sampled every 1 s) or at least every 10 s.
Temperature publishes on a ≥0.2 °C change or at least every 60 s.

Commands received within **3 s of an MQTT (re)connect are ignored**, so a stale retained
`PRESS` can never toggle the inverter at boot.

---

## HX-Bypass (D1 Mini)

| Topic | Direction | Payload |
|---|---|---|
| `HydronicTemperature` | **subscribe** | °F, published by the Plumbing board |
| `HXBypassValveCommand` | subscribe | `ON` / `OFF` — honoured only in manual mode |
| `HXBypassValveStatus` | publish, retained | `ON` / `OFF` |
| `HXBypassAutoModeCommand` | subscribe | `ON` / `OFF` |
| `HXBypassAutoModeStatus` | publish, retained | `ON` / `OFF` |
| `HXBypassTempSetpointCommand` | subscribe | float, accepted 130–200 °F |
| `HXBypassTempSetpointStatus` | publish, retained | e.g. `160.00` |

A valve command arriving while auto mode is on is rejected, and the board republishes its
actual state so the Home Assistant toggle snaps back rather than lying.

No Last Will and no persistence — on reboot this board returns to auto mode with a 160 °F
setpoint and a closed valve.

---

## Cabinet Locks (D1 Mini Lite ×3)

Topics are built from `CABINET_ID` at runtime: `CabLock<ID>Command` / `CabLock<ID>Status`.

| Topic | Direction | Payload |
|---|---|---|
| `CabLockKitchenDriverSideCommand` | subscribe | `LOCK` / `UNLOCK` / `TOGGLE` / `STATUS` |
| `CabLockKitchenDriverSideStatus` | publish, retained | `LOCKED` / `UNLOCKED` / `UNKNOWN` |
| `CabLockKitchenUpperPassSideCommand` / `…Status` | " | " |
| `CabLockRearDriverSideCommand` / `…Status` | " | " |
| `CabLockAllCommand` | subscribe | broadcast to all five locks |

Status also republishes on a 60 s heartbeat. `UNKNOWN` only appears if a status request
arrives before the servo has been initialised, which in practice means never — `setup()`
drives every lock to UNLOCKED at boot.

None of the lock boards set a Last Will.

---

## Full topic index

Alphabetical, with owning board.

| Topic | Board |
|---|---|
| `AltInverterBatteryRawMV` | AltInverterCharger |
| `AltInverterBatteryVoltage` | AltInverterCharger |
| `AltInverterCalScale` | AltInverterCharger |
| `AltInverterCalibrateCommand` | AltInverterCharger |
| `AltInverterPowerCommand` | AltInverterCharger |
| `AltInverterRSSI` | AltInverterCharger |
| `AltInverterRelayState` | AltInverterCharger |
| `AltInverterStatus` | AltInverterCharger |
| `AltInverterTemperature` | AltInverterCharger |
| `CabLockAllCommand` | all five lock nodes |
| `CabLockKitchenDriverSide{Command,Status}` | CabLock KitchenDriverSide |
| `CabLockKitchenPassSide{Command,Status}` | FrontThermostat |
| `CabLockKitchenUpperPassSide{Command,Status}` | CabLock KitchenUpperPassSide |
| `CabLockRearDriverSide{Command,Status}` | CabLock RearDriverSide |
| `CabLockRearPassSide{Command,Status}` | Plumbing |
| `ClearHighHumidityCommand` | Plumbing |
| `Debug/SensorIndex_0..3` | Plumbing |
| `DieselHeater{Command,Status}` | Plumbing |
| `DomesticHotWater{Command,Status}` | Plumbing |
| `ElectricHeater{Command,Status}` | Plumbing |
| `EngineLoopCommand` | Plumbing (sub) ← FrontThermostat (pub) |
| `EngineLoopStatus` | Plumbing |
| `EnvironmentHumidity` | Plumbing |
| `EnvironmentPressure` | Plumbing |
| `EnvironmentTempAmbient` | Plumbing |
| `ExhaustFan{Command,Status}` | Plumbing |
| `ExhaustFanTempSet{,Command}` | Plumbing |
| `FreshWaterHeat{Command,Status}` | Plumbing |
| `FreshWaterHeatTempSet{,Command}` | Plumbing |
| `FreshWaterTemperature` | Plumbing |
| `FrontLoop{Command,Status}` | Plumbing |
| `GreyWaterHeat{Command,Status}` | Plumbing |
| `GreyWaterHeatTempSet{,Command}` | Plumbing |
| `GreyWaterTemperature` | Plumbing |
| `HXBypassAutoMode{Command,Status}` | HX-Bypass |
| `HXBypassTempSetpoint{Command,Status}` | HX-Bypass |
| `HXBypassValve{Command,Status}` | HX-Bypass |
| `HotWaterFlowRate` | Plumbing |
| `HotWaterFlowStatus` | Plumbing |
| `HydronicTemperature` | Plumbing (pub) → HX-Bypass (sub) |
| `MainWaterPump{Command,Status}` | Plumbing |
| `ManifoldPressure` | Plumbing |
| `PlumbingAlert` | Plumbing |
| `PlumbingFirmwareVersion` | Plumbing |
| `PlumbingMode{,Command}` | Plumbing |
| `PlumbingSystemStatus` | Plumbing |
| `PlumbingTempReturnManifold` | Plumbing |
| `PlumbingUptime` | Plumbing |
| `RearLoop{Command,Status}` | Plumbing |
| `Thermostat{Heating,Humidity,Pressure,TempF}` | FrontThermostat |
| `ThermostatFan{Command,Percent,Preset,PresetCommand,SpeedCommand}` | FrontThermostat |
| `ThermostatMode{,Command}` | FrontThermostat |
| `ThermostatSetTempF{,Command}` | FrontThermostat |

---

## Working with the broker

```bash
mosquitto_sub -h 192.168.8.1 -t '#' -v
```

```bash
mosquitto_pub -h 192.168.8.1 -t CabLockAllCommand -m LOCK
```

Clearing a stale retained value (for example after renaming a topic) means publishing an
empty retained message to it:

```bash
mosquitto_pub -h 192.168.8.1 -t SomeOldTopic -r -n
```
