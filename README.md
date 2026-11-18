# Chester the Bus — RV Control System

Firmware for the automation and monitoring system in a bus conversion. Eight independently
flashed microcontrollers handle hydronic heating, water, climate, power, and cabinet
security, coordinating entirely through MQTT and surfacing in Home Assistant.

There is no central controller. Each board owns its hardware, persists its own state where
it matters, and publishes retained status so the rest of the system can resynchronise at any
time.

---

## Boards

| Project | Hardware | Firmware | Responsibility |
|---|---|---|---|
| [`Plumbing`](Plumbing/) | ESP32-S2 Mini | `v3.5.0` | Hydronic valves, tank heaters, diesel + electric heat, water pump, exhaust fan, DHW solenoid, 5× DS18B20 (ROM-addressed), BME280, pressure, flow, RearPassSide lock |
| [`FrontThermostat`](FrontThermostat/) | ESP32-S2 Mini | `v5.2.0` | Front zone PI climate control, fan coil, OLED, buttons, KitchenPassSide lock |
| [`AltInverterCharger`](AltInverterCharger/) | ESP32-S2 Mini | `v1.1.1` | Inverter power button pulse, battery voltage, DS18B20 |
| [`HX-Bypass`](HX-Bypass/) | Wemos D1 Mini | `v1.0.1` | Engine-coolant heat exchanger bypass valve |
| [`TempSensorBus`](TempSensorBus/) | ESP32-S2 Mini | `v1.1.1` | Harness bring-up rig — 5× DS18B20 on one OneWire bus, address-based role assignment, identify-by-warming |
| [`D1Mini_CabLock_KitchenDriverSide`](D1Mini_CabLock_KitchenDriverSide/) | D1 Mini Lite | `v3.1.0` | Cabinet lock servo |
| [`D1Mini_CabLock_KitchenUpperPassSide`](D1Mini_CabLock_KitchenUpperPassSide/) | D1 Mini Lite | `v3.1.0` | Cabinet lock servo |
| [`D1Mini_CabLock_RearDriverSide`](D1Mini_CabLock_RearDriverSide/) | D1 Mini Lite | `v3.1.0` | Cabinet lock servo |

---

## System map

```
                  ┌────────────────────────────────────────┐
                  │  GL-iNet router — 192.168.8.1          │
                  │  SSID "Chester IOT"                    │
                  │  Mosquitto :1883  ·  Home Assistant    │
                  └───────────────────┬────────────────────┘
                                      │  MQTT
      ┌─────────────┬─────────────────┼──────────────┬─────────────────┐
      │             │                 │              │                 │
 ┌────┴─────┐ ┌─────┴────────┐ ┌──────┴──────┐ ┌─────┴──────┐ ┌────────┴────────┐
 │ Plumbing │ │FrontThermostat│ │AltInverter │ │ HX-Bypass  │ │ CabLock × 3     │
 │  ESP32-S2│ │   ESP32-S2    │ │  ESP32-S2  │ │  D1 Mini   │ │  D1 Mini Lite   │
 └────┬─────┘ └─────┬─────────┘ └────────────┘ └─────┬──────┘ └─────────────────┘
      │             │                                 │
      │  EngineLoopCommand                            │
      │◄────────────┘                                 │
      │                                               │
      └──────────  HydronicTemperature  ──────────────►
```

Only two messages cross board boundaries. The thermostat asks for heat by publishing
`EngineLoopCommand`, which the Plumbing board acts on; the Plumbing board publishes
`HydronicTemperature`, which the bypass valve reacts to. Everything else is a board talking
to Home Assistant.

---

## Quick start

```bash
pio run -t upload
```

Run that from inside a project folder. Most projects are configured for OTA — see
[OTA_UPDATE_GUIDE.md](OTA_UPDATE_GUIDE.md) for hostnames, ports, and when USB is required.

```bash
pio device monitor
```

115200 baud on every board. The boot banner reports firmware version, detected sensors, and
restored output state.

Watch the whole system:

```bash
mosquitto_sub -h 192.168.8.1 -t '#' -v
```

---

## Documentation

| Document | Contents |
|---|---|
| [BOARD_ARCHITECTURE.md](BOARD_ARCHITECTURE.md) | Every pin, sensor, and relay polarity, per board |
| [MQTT_TOPICS.md](MQTT_TOPICS.md) | Complete topic contract with payloads and retain flags |
| [HOME_ASSISTANT_DISCOVERY.md](HOME_ASSISTANT_DISCOVERY.md) | Every auto-discovered entity |
| [OTA_UPDATE_GUIDE.md](OTA_UPDATE_GUIDE.md) | Flashing, OTA targets, state across a reflash |
| [CABINET_LOCKS.md](CABINET_LOCKS.md) | All five locks, commands, and their quirks |
| [DEVELOPMENT.md](DEVELOPMENT.md) | Toolchain, layout, firmware conventions |
| [Plumbing/README.md](Plumbing/README.md) | Automation rules, leak detection, persistence, calibration |
| [FrontThermostat/README.md](FrontThermostat/README.md) | Heating cycle, PI tuning, fan control, buttons |
| [AltInverterCharger/README.md](AltInverterCharger/README.md) | Relay pulse safety, voltage calibration |
| [HX-Bypass/README.md](HX-Bypass/README.md) | Bypass valve control logic |

---

## Network

| Setting | Value |
|---|---|
| SSID | `Chester IOT` |
| MQTT broker | `192.168.8.1:1883`, anonymous |
| WiFi / OTA passwords | `secrets.ini` (gitignored) |
| Serial | 115200 baud |

WiFi and OTA credentials are not in the code. They live in `secrets.ini` at the repo root (gitignored); copy `secrets.example.ini` to create it. Every `platformio.ini` loads it with `extra_configs` and passes the values in as `SECRET_*` build flags. Changing the
network means editing that one file and reflashing every board.

Static DHCP reservations on the router are recommended for every board. HX-Bypass is the
only one that sets its own static IP in firmware (`192.168.8.140`).

---

## Design principles

**The board is authoritative.** Hardware state lives on the board that owns the hardware,
not in the broker or in Home Assistant. The Plumbing board restores every relay from NVS
before the network is even up, then overwrites the broker's retained values with its own.

**Commands and state never share a topic.** Boards subscribe to `…Command` and publish to
`…Status`. A board that subscribed to its own retained state topic would replay its last
state as a command on every reconnect.

**Retain state, not telemetry.** Valve positions, modes, and setpoints are retained.
Temperature and flow readings are not.

**Fail toward safe.** The main water pump relay is wired normally closed, so a dead board
leaves water pressure available rather than cutting it. The inverter relay is forced open
before an OTA update so it cannot stick closed. A confirmed water leak latches a shutdown
that only a human can clear.
