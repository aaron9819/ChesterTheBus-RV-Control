# Board Architecture

Hardware inventory, pin assignments, and per-board responsibilities. Every value here is
read directly from the firmware sources listed under each board.

---

## System Overview

Eight independently-flashed boards on a private WiFi network, coordinating exclusively
through a Mosquitto MQTT broker. There is no central controller: each board owns its own
hardware, persists its own state, and publishes retained status so the broker (and Home
Assistant) can be resynchronised at any time.

```
                     ┌──────────────────────────────────────┐
                     │  GL-iNet router — 192.168.8.1        │
                     │  SSID "Chester IOT"                  │
                     │  Mosquitto broker :1883 (anonymous)  │
                     │  Home Assistant (MQTT discovery)     │
                     └──────────────┬───────────────────────┘
                                    │
   ┌──────────────┬─────────────────┼──────────────┬────────────────┬────────────────┐
   │              │                 │              │                │                │
┌──┴────────┐ ┌───┴──────────┐ ┌────┴────────┐ ┌───┴──────────┐ ┌───┴──────────┐ ┌───┴──────────┐
│ Plumbing  │ │FrontThermo-  │ │AltInverter- │ │ HX-Bypass    │ │ 3 × CabLock  │ │TempSensorBus │
│ ESP32-S2  │ │stat ESP32-S2 │ │Charger S2   │ │ D1 Mini      │ │ D1 Mini Lite │ │ ESP32-S2     │
│ v3.6.1    │ │ v5.3.1       │ │ v1.2.1      │ │ v1.1.1       │ │ v3.2.1       │ │ v1.2.1       │
└───────────┘ └──────────────┘ └─────────────┘ └──────────────┘ └──────────────┘ └──────────────┘
```

Common to every board:

| Setting | Value |
|---|---|
| WiFi SSID | `Chester IOT` |
| WiFi password | `secrets.ini` → `wifi_password` |
| MQTT broker | `192.168.8.1:1883` |
| MQTT auth | none (empty user/password) |
| OTA password | `secrets.ini` → `ota_password` |
| Serial monitor | 115200 baud |

WiFi and OTA credentials are not in the code. They live in `secrets.ini` at the repo root (gitignored); copy `secrets.example.ini` to create it. Every `platformio.ini` loads it with `extra_configs` and passes the values in as `SECRET_*` build flags.

---

## 1. Plumbing — ESP32-S2 Mini

**Source:** `Plumbing/src/main_esp32s2.cpp` · **Env:** `lolin_s2_mini` · **Firmware:** `v3.6.1`
**OTA host:** `Plumbing.local` · **MQTT client id:** `ESP32S2_Plumbing`

The largest board and the authoritative source of truth for every hydronic valve and
heater state. It also carries the *RearPassSide* cabinet lock servo (`CABINET_ID
"RearPassSide"`), so this board doubles as the fifth cabinet lock node.

### Pin assignments

| Pin | Signal | Notes |
|---|---|---|
| GPIO14 | DS18B20 OneWire bus | Shared bus, 4 sensors, 12-bit resolution |
| GPIO9 | Fresh water tank heater relay | Active LOW |
| GPIO2 | Grey water tank heater relay | Active LOW |
| GPIO12 | Exhaust fan relay | Active LOW |
| GPIO11 | Main water pump relay | **Normally closed** — HIGH = pump ON, LOW = pump OFF |
| GPIO10 | Domestic hot water solenoid | Active LOW |
| GPIO5 | Rear loop valve relay | Active LOW |
| GPIO3 | Engine loop valve relay | Active LOW |
| GPIO4 | Front loop valve relay | Active LOW |
| GPIO8 | Diesel heater relay 1 (white wire) | Active LOW |
| GPIO7 | Diesel heater relay 2 (blue wire) | Active LOW |
| GPIO6 | Diesel heater relay 3 (yellow wire) | Active LOW |
| GPIO37 | Electric heater relay | **Active HIGH** — HIGH = ON |
| GPIO16 | Cabinet lock servo (RearPassSide) | 50 Hz, 500–2400 µs, detached when idle |
| GPIO17 | Flow sensor pulse input | `INPUT_PULLUP`, FALLING-edge interrupt |
| GPIO33 | BME280 SDA | I²C |
| GPIO35 | BME280 SCL | I²C |

The main water pump relay is the one inverted output on the board — it is wired normally
closed so that a dead board leaves the pump running rather than dead.

### DS18B20 harness — five probes, addressed by ROM

Five sensors share the GPIO14 bus. Each harness position is bound to a sensor's 64-bit ROM
address, stored in NVS (`plumbing` namespace, keys `ts_supply`, `ts_return`, `ts_buffer`,
`ts_grey`, `ts_fresh` — keyed to the position, so removing a position never reshuffles the
rest). **Discovery index is not used.** Index order is ROM-address order, so it shifts whenever a probe is added, replaced,
or drops off the bus — on a bus that drives tank heaters, silently swapping two readings is
not an acceptable failure mode.

| Position key | Reading | Topic | Filtered? |
|---|---|---|---|
| `supply_manifold` | Supply manifold | `PlumbingTempSupplyManifold` | no |
| `return_manifold` | Return manifold | `PlumbingTempReturnManifold` | no |
| `buffer_tank` | Hydronic buffer tank | `HydronicTemperature` | no |
| `grey_tank` | Grey water tank | `GreyWaterTemperature` | 5-sample moving average |
| `fresh_tank` | Fresh water tank | `FreshWaterTemperature` | 5-sample moving average |

Positions are labelled by hand: press a position's Identify button in Home Assistant, warm
that probe, and the sensor that rises ≥4 °F — beating the runner-up by ≥2 °F — takes the
position. Conversions are non-blocking; the bus is re-enumerated only while a position is
missing, so a healthy harness generates no search traffic at all.

### Sensor conversions

- **Temperatures** — read in °C, converted to °F for all state and MQTT. `-999.0` is the
  sentinel for a disconnected sensor and suppresses publication. Readings outside
  −67…257 °F (the DS18B20's rated range) are treated as failed reads.
- **Flow rate** — 450 pulses per litre (YF-S201 class sensor), recomputed every 1000 ms.
- **BME280** — auto-probed at `0x76`, falling back to `0x77`. Temperature published in °F,
  pressure in hPa.

### Timing

| Task | Interval |
|---|---|
| Pressure read | 1 s |
| Flow calculation + publish | 1 s |
| Temperature collect + next conversion started, BME280, automation, alerts | 5 s |
| OneWire re-enumeration | 60 s, **only while a position is missing** |
| Status publish (pressure, pump, mode, setpoints) | 10 s |
| Health publish (uptime, ONLINE, firmware) | 60 s |
| WiFi liveness check | 10 s |
| MQTT reconnect attempt | 5 s min interval |
| EEPROM flush debounce | 2 s after last change |

### Reliability

- **Task watchdog** — `esp_task_wdt` at 60 s, fed each loop, panic-reset on expiry.
  Detached during OTA and reattached on completion or error.
- **Network** — after 8 consecutive WiFi/MQTT failures the board calls `ESP.restart()`.
- **Last Will** — broker publishes `OFFLINE` to `PlumbingSystemStatus` on unexpected drop.
- **Probe-loss fail-safe** — in AUTO, a tank heater whose own probe has been missing for
  60 s is switched OFF. Without it the heater has no control input and would stay energised
  indefinitely. Governed by `TEMP_PROBE_LOSS_FAILSAFE` in the source.

Behaviour, automation rules, and safety interlocks: **[Plumbing/README.md](Plumbing/README.md)**

---

## 2. FrontThermostat — ESP32-S2 Mini

**Source:** `FrontThermostat/src/main.cpp` · **Env:** `lolin_s2_mini` · **Firmware:** `v5.3.1`
**OTA host:** `Thermostat-KitchenPassSide.local` · **Cabinet ID:** `KitchenPassSide`

Front-zone climate control with a PI-controlled hydronic fan coil. It does not switch any
heat source directly — it publishes `EngineLoopCommand` and the Plumbing board opens or
closes the actual valve. Also carries the *KitchenPassSide* cabinet lock servo.

### Pin assignments

| Pin | Signal | Notes |
|---|---|---|
| GPIO8 | Cabinet lock servo | Attached only while moving |
| GPIO10 | Fan PWM output | 25 kHz, 8-bit, `ledcAttach` |
| GPIO21 | Temperature UP button | `INPUT_PULLUP`, active LOW |
| GPIO17 | Temperature DOWN button | `INPUT_PULLUP`, active LOW |
| GPIO33 | BME280 / OLED SDA | Shared I²C bus |
| GPIO35 | BME280 / OLED SCL | Shared I²C bus |

### Peripherals

- **BME280** at `0x76` only (no `0x77` fallback). A fixed `TEMP_CALIBRATION_OFFSET` of
  **−6.0 °C** is applied to every reading to compensate for enclosure self-heating.
  `NaN` readings are discarded rather than published.
- **SSD1306 OLED**, 64×48, address `0x3C`. If absent at boot the board retries every 10 s
  and runs headless in the meantime.

Behaviour, PI tuning, and button semantics: **[FrontThermostat/README.md](FrontThermostat/README.md)**

---

## 3. AltInverterCharger — ESP32-S2 Mini

**Source:** `AltInverterCharger/src/main.cpp` · **Env:** `lolin_s2_mini` · **Firmware:** `v1.2.1`
**OTA host:** `AltInverterCharger.local`

Pulses the inverter's momentary power button and monitors its supply battery.

| Pin | Signal | Notes |
|---|---|---|
| GPIO3 | Battery voltage sense | **ADC1_CH2 — must stay on GPIO1–10** |
| GPIO16 | Relay control | `RELAY_ACTIVE_HIGH` is `false` → active LOW |
| GPIO15 | Status LED | Onboard Lolin S2 Mini LED, lit while relay closed |
| GPIO5 | DS18B20 data | Needs a 4.7 kΩ pullup to 3.3 V |

ADC2 (GPIO11–GPIO20) is unusable while WiFi is active on the ESP32-S2 — the WiFi driver
owns it. Keep the battery sense wire on ADC1.

Calibration, safety rate-limits, and topic list: **[AltInverterCharger/README.md](AltInverterCharger/README.md)**

---

## 4. HX-Bypass — Wemos D1 Mini (ESP8266)

**Source:** `HX-Bypass/src/main.cpp` · **Env:** `d1_mini` · **Firmware:** `v1.1.1`
**OTA host:** `HX-Bypass.local` · **Static IP:** `192.168.8.140`

Drives the heat-exchanger bypass valve for engine coolant. The only board with a static IP
configured in firmware (`WiFi.config`, reapplied on every reconnect); everything else uses
DHCP reservations on the router.

| Pin | Signal | Notes |
|---|---|---|
| D4 (GPIO2) | Bypass valve relay | Active LOW; idles HIGH (valve closed) |

It has no temperature sensor of its own — it subscribes to `HydronicTemperature` published
by the Plumbing board. This is the one hard cross-board data dependency in the system.

Control logic: **[HX-Bypass/README.md](HX-Bypass/README.md)**

---

## 5–7. Cabinet Locks — D1 Mini Lite (ESP8266) ×3

**Envs:** `d1_mini_lite` · **Firmware:** `v3.2.1`

| Directory | `CABINET_ID` | OTA hostname |
|---|---|---|
| `D1Mini_CabLock_KitchenDriverSide` | `KitchenDriverSide` | `CabLock-KitchenDriverSide.local` |
| `D1Mini_CabLock_KitchenUpperPassSide` | `KitchenUpperPassSide` | `CabLock-KitchenUpperPassSide.local` |
| `D1Mini_CabLock_RearDriverSide` | `RearDriverSide` | `CabLock-RearDriverSide.local` |

| Pin | Signal | Notes |
|---|---|---|
| D4 (GPIO2) | Lock servo signal | Attached only while moving, then detached |

Servo positions are `LOCKED_POSITION 90` and `UNLOCKED_POSITION 5` on every lock node in
the system, including the two servos hosted on the ESP32-S2 boards.

Full behaviour and the per-board source differences: **[CABINET_LOCKS.md](CABINET_LOCKS.md)**

---

## 8. TempSensorBus — ESP32-S2 Mini

**Source:** `TempSensorBus/src/main.cpp` · **Env:** `lolin_s2_mini` · **Firmware:** `v1.2.1`
**OTA host:** `TempSensorBus.local`

Bring-up rig for the five-sensor hydronic temperature harness. Not a control board — it
reads, reports, and helps label sensors while the harness is being built.

| Pin | Signal | Notes |
|---|---|---|
| GPIO14 | DS18B20 OneWire bus | Shared bus, 6 sensors, 12-bit; one 4.7 kΩ pullup to 3.3 V at the controller end |
| GPIO15 | Status LED | Onboard LED — solid when all roles are live, blinking otherwise |

Same bus pin as the Plumbing board, so a harness can be moved between the two unchanged.

### Role map — by ROM address, not index

| Role key | Sensor |
|---|---|
| `supply_manifold` | Supply manifold |
| `return_manifold` | Return manifold |
| `buffer_tank` | Buffer tank |
| `grey_tank` | Grey tank |
| `fresh_tank` | Fresh tank |

Each role stores the sensor's 64-bit ROM address in NVS (`tempbus` namespace, keys `a_supply`,
`a_return`, `a_buffer`, `a_grey`, `a_fresh` — keyed to the role, not to its position in the
table). Discovery index is never used: it shifts whenever a sensor is added, removed, or
replaced, and on a shared bus that silently swaps readings.
Addresses do not shift.

The board re-enumerates the bus every 30 s, so a sensor spliced in mid-build appears without
a reboot. Roles are bound either by publishing the address directly or by identify-by-warming
— press the role's button, warm that sensor by hand, and the sensor that rises fastest takes
the role.

### No exhaust channel on this bus

The boiler exhaust position was removed. A DS18B20 is rated −55 to +125 °C and diesel heater
exhaust at the outlet runs well past that, so measuring it needs a K-type thermocouple
(MAX31855/31856) on its own channel rather than a probe on this bus. Readings outside
−67…257 °F are still counted as errors and logged rather than published.

Wiring, debug topics, and the symptom table: **[TempSensorBus/README.md](TempSensorBus/README.md)**

---

## Cabinet lock inventory

Five servo locks exist, but only three have dedicated boards — the other two ride along on
larger controllers:

| Cabinet | Host board | Servo pin |
|---|---|---|
| KitchenDriverSide | `D1Mini_CabLock_KitchenDriverSide` | D4 |
| KitchenUpperPassSide | `D1Mini_CabLock_KitchenUpperPassSide` | D4 |
| RearDriverSide | `D1Mini_CabLock_RearDriverSide` | D4 |
| KitchenPassSide | `FrontThermostat` | GPIO8 |
| RearPassSide | `Plumbing` | GPIO16 |

All five subscribe to `CabLockAllCommand` for group lock/unlock.

---

## Cross-board dependencies

Only two message flows cross board boundaries. Everything else is board ↔ Home Assistant.

| Publisher | Topic | Subscriber | Purpose |
|---|---|---|---|
| Plumbing | `HydronicTemperature` | HX-Bypass | Feeds the bypass valve's auto-mode comparison |
| FrontThermostat | `EngineLoopCommand` | Plumbing | Thermostat calls for heat; Plumbing actuates the valve |

Note that `EngineLoopCommand` is a shared write path: Home Assistant can also publish to it.
The Plumbing board persists whichever request arrived last and treats itself as authoritative
for the resulting valve state.

---

## Relay polarity summary

Getting this wrong energises hardware at boot, so it is worth stating plainly.

| Board | Output | Idle (OFF) level | Active (ON) level |
|---|---|---|---|
| Plumbing | All valves, heaters, exhaust fan, DHW solenoid | HIGH | LOW |
| Plumbing | Main water pump (normally closed) | LOW | HIGH |
| Plumbing | Electric heater (GPIO37) | LOW | HIGH |
| AltInverterCharger | Inverter button relay | HIGH | LOW |
| HX-Bypass | Bypass valve relay | HIGH | LOW |
