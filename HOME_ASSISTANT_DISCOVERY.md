# Home Assistant MQTT Discovery

Every board publishes its own discovery configs on each MQTT connect, so entities appear
without any YAML in Home Assistant. This document lists exactly what each board creates.

## How it works

On successful MQTT connect, and again on every reconnect, each board publishes retained
JSON configs to `homeassistant/<component>/<object>/config`. Home Assistant's MQTT
integration (with discovery enabled, which is the default) creates or updates the entity.

Discovery topic shapes differ slightly between boards because they were written at
different times — both forms are valid:

```
homeassistant/<component>/<device_id>/<object_id>/config   # Plumbing, HX-Bypass, cab locks, AltInverter
homeassistant/<component>/<device_id>_<object>/config      # FrontThermostat
```

Configs are retained, so entities survive a Home Assistant restart even if the board is
offline. The corollary: **renaming an entity leaves the old retained config on the broker**
and the stale entity keeps appearing until you clear it. Boards handle this explicitly
where a rename has already happened — Plumbing publishes an empty retained message to
`homeassistant/select/chester_plumbing/diesel_heater/config` (the old 3-state select), and
FrontThermostat clears `homeassistant/sensor/thermostat_KitchenPassSide_fan/config` (the
old read-only fan sensor). To clear any other leftover yourself:

```bash
mosquitto_pub -h 192.168.8.1 -t 'homeassistant/sensor/old_thing/config' -r -n
```

---

## Device: Chester Plumbing System (`chester_plumbing`)

Model `ESP32-S2 Mini`, manufacturer `ChesterTheBus`, `sw_version` tracks `FIRMWARE_VERSION`.
This device alone accounts for 26 entities: 11 sensors, 10 switches, 3 numbers,
a select, and a lock.

### Sensors

| Entity | Device class | Unit | State topic |
|---|---|---|---|
| Hydronic Temperature | temperature | °F | `HydronicTemperature` |
| Fresh Tank Temperature | temperature | °F | `FreshWaterTemperature` |
| Grey Tank Temperature | temperature | °F | `GreyWaterTemperature` |
| Return Manifold Temperature | temperature | °F | `PlumbingTempReturnManifold` |
| Ambient Temperature | temperature | °F | `EnvironmentTempAmbient` |
| Humidity | humidity | % | `EnvironmentHumidity` |
| Atmospheric Pressure | pressure | hPa | `EnvironmentPressure` |
| Hot Water Flow Rate | — | L/min | `HotWaterFlowRate` |
| Hot Water Flow Status | — | — | `HotWaterFlowStatus` |
| Plumbing Alerts | — | — | `PlumbingAlert` |

### Switches

| Entity | On / Off payload | Command topic |
|---|---|---|
| Fresh Water Tank Heater | `ON` / `OFF` | `FreshWaterHeatCommand` |
| Grey Water Tank Heater | `ON` / `OFF` | `GreyWaterHeatCommand` |
| Rear Loop Valve | `OPEN` / `CLOSE` | `RearLoopCommand` |
| Engine Loop Valve | `OPEN` / `CLOSE` | `EngineLoopCommand` |
| Front Loop Valve | `OPEN` / `CLOSE` | `FrontLoopCommand` |
| Exhaust Fan | `ON` / `OFF` | `ExhaustFanCommand` |
| Main Water Pump | `ON` / `OFF` | `MainWaterPumpCommand` |
| Domestic Hot Water | `OPEN` / `CLOSE` | `DomesticHotWaterCommand` |
| Diesel Heater | `ON` / `OFF` | `DieselHeaterCommand` |
| Electric Heater | `ON` / `OFF` | `ElectricHeaterCommand` |

### Numbers

| Entity | Range | Step | Command topic |
|---|---|---|---|
| Fresh Water Heater Setpoint | 32–120 °F | 1 | `FreshWaterHeatTempSetCommand` |
| Grey Water Heater Setpoint | 32–120 °F | 1 | `GreyWaterHeatTempSetCommand` |
| Exhaust Fan Temperature Setpoint | 32–140 °F | 1 | `ExhaustFanTempSetCommand` |

### Select and lock

| Entity | Type | Detail |
|---|---|---|
| Plumbing Mode | select | Options `AUTO` / `MANUAL` → `PlumbingModeCommand` |
| Rear Pass Cabinet Lock | lock | `LOCK` / `UNLOCK` → `CabLockRearPassSideCommand` |

The Plumbing device does **not** declare an `availability_topic`, even though it sets a
Last Will on `PlumbingSystemStatus`. Entities therefore show their last retained value
rather than *unavailable* when the board drops off. Adding `availability_topic` to the
discovery helper would fix that if you want it.

---

## Device: Front Thermostat (`thermostat_KitchenPassSide`)

Model `ESP32-S2 Thermostat`, manufacturer `Chester RV Control`.

| Entity | Component | Detail |
|---|---|---|
| Front Thermostat | `climate` | Modes `off` / `heat`; 50–86 °F, 0.5° step, 0.1 precision, unit F |
| Front Humidity | `sensor` | % RH, device_class humidity |
| Front Pressure | `sensor` | hPa, device_class pressure |
| Thermostat Fan | `fan` | On/off + 1–100 % + presets `auto`/`low`/`medium`/`high` |
| Thermostat Heating | `binary_sensor` | device_class heat, `ON`/`OFF` |
| Cabinet Lock | `lock` | `LOCK`/`UNLOCK` ↔ `LOCKED`/`UNLOCKED` |

The climate entity reads current temperature from `ThermostatTempF` and writes the setpoint
to `ThermostatSetTempFCommand`. The fan entity derives its on/off state from the percentage
with a value template (`'ON' if value | int > 0 else 'OFF'`), so there is no separate fan
state topic.

Note this device declares no availability topic and the board sets no Last Will.

---

## Device: Alt Inverter Charger (`chester_alt_inverter`)

Model `ESP32-S2 Mini`, manufacturer `Chester RV Control`. This is the only device that
wires availability through properly — **every** entity carries
`availability_topic: AltInverterStatus` with `ONLINE`/`OFFLINE`, so all of them grey out in
Home Assistant when the board drops.

| Entity | Component | Category | Detail |
|---|---|---|---|
| Inverter Power | `button` | — | `payload_press: PRESS` → one 500 ms relay pulse |
| Inverter Battery Voltage | `sensor` | — | V, measurement, 2 dp |
| Inverter Temperature | `sensor` | — | °C, measurement, 1 dp |
| Inverter Relay | `binary_sensor` | diagnostic | Shows the pulse |
| Battery ADC Raw | `sensor` | diagnostic | mV at the ADC pin |
| Voltage Cal Scale | `sensor` | diagnostic | Current calibration factor |
| WiFi RSSI | `sensor` | diagnostic | dBm, signal_strength |

---

## Device: Heat Exchanger Bypass (`chester_hxbypass`)

Model `D1 Mini (ESP8266)`, manufacturer `ChesterTheBus`.

| Entity | Component | Detail |
|---|---|---|
| HX Bypass Valve | `switch` | `HXBypassValveCommand` / `…Status`; ignored while auto mode is on |
| HX Bypass Auto Mode | `switch` | `HXBypassAutoModeCommand` / `…Status` |
| HX Bypass Temperature Setpoint | `number` | 130–200 °F, step 1 |

The switch uses default payloads (`ON`/`OFF`), which is what the firmware sends and expects.

---

## Devices: Cabinet Locks

Each lock node registers as its own Home Assistant device. The device id is
`chester_cablock_<cabinet_id>` **lowercased** — for example `chester_cablock_kitchendriverside`.

| Device | Entity | Component |
|---|---|---|
| Cabinet Lock - KitchenDriverSide | Cabinet Lock - KitchenDriverSide | `lock` |
| Cabinet Lock - KitchenUpperPassSide | Cabinet Lock - KitchenUpperPassSide | `lock` |
| Cabinet Lock - RearDriverSide | Cabinet Lock - RearDriverSide | `lock` |

Each is configured `optimistic: false`, so Home Assistant waits for the retained status
publish rather than assuming the command took effect. The remaining two locks
(KitchenPassSide, RearPassSide) are entities on the FrontThermostat and Plumbing devices
respectively, not standalone devices.

These lock configs carry no `sw_version` — the discovery payload omits it.

---

## Verifying discovery

Watch the configs land as a board boots:

```bash
mosquitto_sub -h 192.168.8.1 -t 'homeassistant/#' -v
```

If entities do not appear, check in this order:

1. The board actually connected — look for `ONLINE` on its status topic.
2. MQTT discovery is enabled in Home Assistant's MQTT integration settings and the prefix
   is `homeassistant`.
3. The PubSubClient buffer is large enough. Discovery payloads are long; Plumbing,
   HX-Bypass, and the three lock nodes set `setBufferSize(1024)`, while FrontThermostat
   and AltInverterCharger use 2048. A payload exceeding the buffer is silently dropped by
   PubSubClient — no error, the entity just never shows up.
