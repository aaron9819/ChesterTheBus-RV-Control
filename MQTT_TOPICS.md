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
| `HydronicTemperature` | °F, e.g. `142.3` — the `buffer_tank` position | 5 s |
| `FreshWaterTemperature` | °F (5-sample average) | 5 s |
| `GreyWaterTemperature` | °F (5-sample average) | 5 s |
| `PlumbingTempReturnManifold` | °F | 5 s |
| `PlumbingTempSupplyManifold` | °F | 5 s |
| `EnvironmentTempAmbient` | °F, from BME280 | 5 s |
| `EnvironmentHumidity` | % RH | 5 s |
| `EnvironmentPressure` | hPa | 5 s |
| `HotWaterFlowRate` | L/min, e.g. `2.34` | on change, else 30 s |
| `HotWaterFlowRateGPM` | US gal/min | on change, else 30 s |
| `HotWaterFlowStatus` | `FLOWING` / `STOPPED` | on change, else 30 s |
| `HotWaterFlowTotal` | litres since boot | on change, else 30 s |
| `HotWaterFlowTotalGal` | US gallons since boot | on change, else 30 s |
| `HotWaterFlowPulses` | raw pulse count since boot | on change, else 30 s |
| `PlumbingUptime` | seconds since boot | 60 s |
| `PlumbingFirmwareVersion` | e.g. `v3.5.0` | 60 s |

A temperature publishes only while its harness position is bound to a probe that is
answering. `Debug/SensorIndex_0`…`_3` are gone — index-based mapping was replaced by
ROM-address binding in `v3.2.0`.

`HotWaterFlowStatus` publishes the literal string `STOPPED`. (The source comment says
`NOT_FLOWING`; the comment is wrong, the code is right.)

### DS18B20 harness (five probes, bound by ROM address)

Position keys: `supply_manifold`, `return_manifold`, `buffer_tank`, `grey_tank`,
`fresh_tank`.

| Topic | Direction | Payload |
|---|---|---|
| `PlumbingTempSensorsFound` | publish, retained | devices answering on the bus |
| `PlumbingTempSensorsLive` | publish, retained | positions bound and reading |
| `PlumbingTempSensorScan` | publish, retained | JSON: every device, its position, reading, plus missing positions |
| `PlumbingTempPresence/<key>` | publish, retained | `ONLINE` / `OFFLINE` — the HA availability topic for that position's sensor |
| `PlumbingTempIdentifyState` | publish, retained | `IDLE` / `WAITING: <key>` / `DONE: <key>=<addr>` / `TIMEOUT` / `NO_SENSORS` |
| `PlumbingTempSensorCommand` | subscribe | `SCAN` / `REPORT` / `CLEAR` |
| `PlumbingTempAssignCommand` | subscribe | `<key>=<16-char ROM hex>` or `<key>=<bus index>`; empty value clears |
| `PlumbingTempIdentifyCommand` | subscribe | `<key>` to start identify-by-warming, `CANCEL` to abort |
| `HotWaterFlowKFactor` | publish, retained | flow meter calibration, pulses per litre |
| `HotWaterFlowCalibrateCommand` | subscribe | `START`, then the measured volume — `1G` (US gallons), `2.5L`, or a bare number (litres) |

Identify snapshots every probe, then binds whichever rises ≥4 °F within 60 s and beats the
runner-up by ≥2 °F. Bindings persist in NVS.

### Published — state (retained)

| Topic | Payload |
|---|---|
| `PlumbingSystemStatus` | `ONLINE` / `OFFLINE` (LWT) / `LEAK_DETECTED_SHUTDOWN` |
| `FreshWaterHeatStatus` | `ON` / `OFF` |
| `GreyWaterHeatStatus` | `ON` / `OFF` |
| `ExhaustFanStatus` | `ON` / `OFF` |
| `RearLoopStatus` | `OPEN` / `CLOSE` |
| `EngineLoopStatus` | `OPEN` / `CLOSE` (relay — which way the valve is driven) |
| `EngineLoopValveState` | `OPEN` / `OPENING` / `CLOSING` / `CLOSED` — modelled travel, 10.5 min each way, reversible mid-travel (retained) |
| `EngineLoopValvePosition` | `0`–`100` % open, modelled (retained; every 30 s while moving) |
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
| `ThermostatAvailability` | `online` / `offline` (Last Will); used by every HA entity |
| `ThermostatTempF` | current °F (BME280 + −6.0 °C offset), on change + every 60 s |
| `ThermostatHumidity` | % RH, corrected to room temperature |
| `ThermostatPressure` | hPa |
| `ThermostatSetTempF` | setpoint °F |
| `ThermostatMode` | `off` / `heat` |
| `ThermostatAction` | `off` / `idle` / `preheating` / `heating` / `fan` (HA climate action) |
| `ThermostatHeating` | `ON` / `OFF` |
| `ThermostatFanPercent` | `0`–`100`, exactly as requested |
| `ThermostatFanPreset` | `auto` / `low` / `medium` / `high` / `None` (custom manual speed) |
| `ThermostatSensorFault` | `ON` / `OFF` |
| `ThermostatValveFault` | `ON` / `OFF` — engine loop reported closed while heat requested |
| `CabLockKitchenPassSideStatus` | `LOCKED` / `UNLOCKED` |

State changes publish within ~1 s; everything is also republished on a 60 s heartbeat.

### Published as a command to another board

| Topic | Payload | Consumer |
|---|---|---|
| `EngineLoopCommand` | `OPEN` / `CLOSE`, unretained | Plumbing board |

The wanted state is held and re-sent until the broker accepts it (e.g. on reconnect), and
`CLOSE` is asserted once after boot. It is never re-sent automatically after Plumbing
reports the valve closed — that raises `ThermostatValveFault` instead.

### Subscribed

| Topic | Accepted payloads | Effect |
|---|---|---|
| `ThermostatSetTempFCommand` | float, accepted 50–86 °F | Setpoint; persisted to EEPROM |
| `ThermostatModeCommand` | `off` / `heat` / `auto` (case-insensitive) | `heat` and `auto` both select HEAT_AUTO |
| `ThermostatFanCommand` | `ON` / `OFF` | `ON` = manual at current speed (50 % if stopped); `OFF` = held off for this heat cycle, else auto |
| `ThermostatFanSpeedCommand` | integer 0–100 | Manual fan percentage; `0` = same as `OFF` |
| `ThermostatFanPresetCommand` | `auto` / `low` / `medium` / `high` | auto / 33 / 66 / 100 % |
| `EngineLoopStatus` | `OPEN` / `CLOSE` | Valve feedback from Plumbing |
| `EngineLoopValveState` | `OPEN` / `OPENING` / `CLOSING` / `CLOSED` | Coil counts as hot only at `OPEN` (Plumbing's travel model) |
| `PlumbingSystemStatus` | `ONLINE` / `OFFLINE` | While `OFFLINE`, the local coil model is used instead |
| `CabLockKitchenPassSideCommand` | `LOCK` / `UNLOCK` / `TOGGLE` / `STATUS` | Cabinet servo |
| `CabLockAllCommand` | `LOCK` / `UNLOCK` / `TOGGLE` / `STATUS` | Group lock control |

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

## TempSensorBus (ESP32-S2)

Five DS18B20 sensors on one OneWire bus, bound to harness positions by **ROM address**
(persisted in NVS), not by discovery index.

### Published — temperatures (retained)

| Topic | Payload | Rate |
|---|---|---|
| `TempBusSupplyManifold` | °F | on ≥0.2 °F change, or 30 s |
| `TempBusReturnManifold` | °F | " |
| `TempBusBufferTank` | °F | " |
| `TempBusGreyTank` | °F | " |
| `TempBusFreshTank` | °F | " |

Each has a companion availability topic `TempBus/Presence/<role key>` (`ONLINE` / `OFFLINE`,
retained), so an unwired position shows as unavailable in Home Assistant instead of holding
its last reading.

### Published — diagnostics

| Topic | Payload | Notes |
|---|---|---|
| `TempBusStatus` | `ONLINE` / `OFFLINE` | retained, LWT |
| `TempBus/Raw/<ROM hex>` | °F, every 5 s | **not retained** — one per device on the bus, assigned or not |
| `TempBusScan` | JSON, retained | every device + role + reading + read/error counts, plus missing roles |
| `TempBusLog` | text | mirror of the serial log |
| `TempBusSensorsFound` | integer, retained | devices answering on the bus |
| `TempBusSensorsAssigned` | integer, retained | roles bound to a device that is present |
| `TempBusFault` | `ON` / `OFF`, retained | `ON` while fewer than five roles are live |
| `TempBusParasitePower` | `ON` / `OFF`, retained | `ON` means a sensor's VDD is unwired |
| `TempBusIdentifyState` | text, retained | `IDLE` / `WAITING: <role>` / `DONE: <role>=<addr>` / `TIMEOUT` |
| `TempBusRSSI` | dBm, retained | |
| `TempBusUptime` | seconds, retained | |

### Subscribed

| Topic | Payload |
|---|---|
| `TempBusCommand` | `SCAN` / `REPORT` / `CLEAR` / `SAVE` / `RESET` |
| `TempBusAssignCommand` | `<role>=<16-char ROM hex>` or `<role>=<bus index>`; empty value clears |
| `TempBusIdentifyCommand` | `<role>` to start identify-by-warming, `CANCEL` to abort |

Role keys: `supply_manifold`, `return_manifold`, `buffer_tank`, `grey_tank`,
`fresh_tank`.

Identify snapshots every sensor, then binds whichever one rises ≥4 °F within 60 s and beats
the runner-up by ≥2 °F. Commands received within **3 s of an MQTT (re)connect are ignored**.

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
| `EngineLoopStatus` | Plumbing (pub) → FrontThermostat (sub) |
| `EngineLoopValveState` | Plumbing (pub) → FrontThermostat (sub) |
| `EngineLoopValvePosition` | Plumbing |
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
| `HotWaterFlowCalibrateCommand` | Plumbing |
| `HotWaterFlowKFactor` | Plumbing |
| `HotWaterFlowPulses` | Plumbing |
| `HotWaterFlowRate{,GPM}` | Plumbing |
| `HotWaterFlowStatus` | Plumbing |
| `HotWaterFlowTotal{,Gal}` | Plumbing |
| `HydronicTemperature` | Plumbing (pub) → HX-Bypass (sub) |
| `MainWaterPump{Command,Status}` | Plumbing |
| `PlumbingAlert` | Plumbing |
| `PlumbingFirmwareVersion` | Plumbing |
| `PlumbingMode{,Command}` | Plumbing |
| `PlumbingSystemStatus` | Plumbing |
| `PlumbingTempAssignCommand` | Plumbing |
| `PlumbingTempIdentify{Command,State}` | Plumbing |
| `PlumbingTempPresence/<key>` | Plumbing |
| `PlumbingTempReturnManifold` | Plumbing |
| `PlumbingTempSensor{Command,Scan}` | Plumbing |
| `PlumbingTempSensors{Found,Live}` | Plumbing |
| `PlumbingTempSupplyManifold` | Plumbing |
| `PlumbingUptime` | Plumbing |
| `RearLoop{Command,Status}` | Plumbing |
| `TempBus/Presence/<role>` | TempSensorBus |
| `TempBus/Raw/<ROM hex>` | TempSensorBus |
| `TempBusAssignCommand` | TempSensorBus |
| `TempBusBufferTank` | TempSensorBus |
| `TempBusCommand` | TempSensorBus |
| `TempBusFault` | TempSensorBus |
| `TempBusFreshTank` | TempSensorBus |
| `TempBusGreyTank` | TempSensorBus |
| `TempBusIdentify{Command,State}` | TempSensorBus |
| `TempBusLog` | TempSensorBus |
| `TempBusParasitePower` | TempSensorBus |
| `TempBusRSSI` | TempSensorBus |
| `TempBusReturnManifold` | TempSensorBus |
| `TempBusScan` | TempSensorBus |
| `TempBusSensors{Found,Assigned}` | TempSensorBus |
| `TempBusStatus` | TempSensorBus |
| `TempBusSupplyManifold` | TempSensorBus |
| `TempBusUptime` | TempSensorBus |
| `Thermostat{Action,Availability,Heating,Humidity,Pressure,SensorFault,TempF,ValveFault}` | FrontThermostat |
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
