# Board Architecture

Hardware inventory, pin assignments, and per-board responsibilities. Every value here is
read directly from the firmware sources listed under each board.

---

## System Overview

Seven independently-flashed boards on a private WiFi network, coordinating exclusively
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
   ┌──────────────┬─────────────────┼──────────────┬────────────────┐
   │              │                 │              │                │
┌──┴────────┐ ┌───┴──────────┐ ┌────┴────────┐ ┌───┴──────────┐ ┌───┴──────────┐
│ Plumbing  │ │FrontThermo-  │ │AltInverter- │ │ HX-Bypass    │ │ 3 × CabLock  │
│ ESP32-S2  │ │stat ESP32-S2 │ │Charger S2   │ │ D1 Mini      │ │ D1 Mini Lite │
│ v3.1.0    │ │ v4.1.0       │ │ v1.1.0      │ │ v1.0.0       │ │ v3.0.0       │
└───────────┘ └──────────────┘ └─────────────┘ └──────────────┘ └──────────────┘
```

Common to every board:

| Setting | Value |
|---|---|
| WiFi SSID | `Chester IOT` |
| WiFi password | `2025Chester9894` |
| MQTT broker | `192.168.8.1:1883` |
| MQTT auth | none (empty user/password) |
| OTA password | `Chester2025` |
| Serial monitor | 115200 baud |

Credentials are hardcoded in each `src/main.cpp`. There is no `secrets.h` indirection.

---

## 1. Plumbing — ESP32-S2 Mini

**Source:** `Plumbing/src/main_esp32s2.cpp` · **Env:** `lolin_s2_mini` · **Firmware:** `v3.1.0`
**OTA host:** `Plumbing.local` · **MQTT client id:** `ESP32S2_Plumbing`

The largest board and the authoritative source of truth for every hydronic valve and
heater state. It also carries the *RearPassSide* cabinet lock servo (`CABINET_ID
"RearPassSide"`), so this board doubles as the fifth cabinet lock node.

### Pin assignments

| Pin | Signal | Notes |
|---|---|---|
| GPIO13 | DS18B20 OneWire bus | Shared bus, 4 sensors, 12-bit resolution |
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
| GPIO1 | Manifold pressure sensor | ADC1_CH0, 12-bit |
| GPIO17 | Flow sensor pulse input | `INPUT_PULLUP`, FALLING-edge interrupt |
| GPIO33 | BME280 SDA | I²C |
| GPIO35 | BME280 SCL | I²C |

The main water pump relay is the one inverted output on the board — it is wired normally
closed so that a dead board leaves the pump running rather than dead.

### DS18B20 index map

All four sensors share the GPIO13 bus and are addressed by discovery index, not by ROM
address. **Physical bus order determines which reading is which.**

| Index | Sensor | Filtered? |
|---|---|---|
| 0 | Return manifold | no |
| 1 | Hydronic tank | no |
| 2 | Grey water tank | 5-sample moving average |
| 3 | Fresh water tank | 5-sample moving average |

Every 30 s the board publishes all four raw indices to `Debug/SensorIndex_0` … `_3` so the
mapping can be verified against physical reality after any bus rewire.

### Sensor conversions

- **Temperatures** — read in °C, converted to °F for all state and MQTT. `-999.0` is the
  sentinel for a disconnected sensor and suppresses publication.
- **Manifold pressure** — `analogRead` (12-bit), subtract an ADC zero offset of **580
  counts**, scale to voltage over 3.3 V, then `PSI = (V / 3.3) × 80`. Clamped to 0–85 PSI.
- **Flow rate** — 450 pulses per litre (YF-S201 class sensor), recomputed every 1000 ms.
- **BME280** — auto-probed at `0x76`, falling back to `0x77`. Temperature published in °F,
  pressure in hPa.

### Timing

| Task | Interval |
|---|---|
| Pressure read | 1 s |
| Flow calculation + publish | 1 s |
| Temperature read, BME280, automation, alert check | 5 s |
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

Behaviour, automation rules, and safety interlocks: **[Plumbing/README.md](Plumbing/README.md)**

---

## 2. FrontThermostat — ESP32-S2 Mini

**Source:** `FrontThermostat/src/main.cpp` · **Env:** `lolin_s2_mini` · **Firmware:** `v4.1.0`
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

**Source:** `AltInverterCharger/src/main.cpp` · **Env:** `lolin_s2_mini` · **Firmware:** `v1.1.0`
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

**Source:** `HX-Bypass/src/main.cpp` · **Env:** `d1_mini` · **Firmware:** `v1.0.0`
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

**Envs:** `d1_mini_lite` · **Firmware:** `v3.0.0`

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
