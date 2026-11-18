# Cabinet Locks

Five servo-driven cabinet locks. Three have dedicated D1 Mini Lite boards; the other two
are hosted on larger controllers that had a spare GPIO.

| Cabinet | Host | Servo pin | Firmware |
|---|---|---|---|
| KitchenDriverSide | `D1Mini_CabLock_KitchenDriverSide` | D4 (GPIO2) | `v3.1.0` |
| KitchenUpperPassSide | `D1Mini_CabLock_KitchenUpperPassSide` | D4 (GPIO2) | `v3.1.0` |
| RearDriverSide | `D1Mini_CabLock_RearDriverSide` | D4 (GPIO2) | `v3.1.0` |
| KitchenPassSide | `FrontThermostat` (ESP32-S2) | GPIO8 | `v5.2.0` |
| RearPassSide | `Plumbing` (ESP32-S2) | GPIO16 | `v3.5.0` |

---

## Servo behaviour

Identical across all five:

| Constant | Value |
|---|---|
| `LOCKED_POSITION` | 90° |
| `UNLOCKED_POSITION` | 5° |

The servo is **attached only while moving**. Each lock/unlock attaches the servo, writes
the position, waits for travel, then detaches. This stops the servo from buzzing and
holding torque against the latch, which would both draw current continuously and wear the
gearbox.

Travel delay is 500 ms on the D1 Mini nodes and the Plumbing board, 1000 ms on
FrontThermostat.

---

## Commands

Each board subscribes to its own `CabLock<ID>Command` plus the shared `CabLockAllCommand`.

| Payload | D1 Mini nodes | Plumbing | FrontThermostat |
|---|---|---|---|
| `LOCK` | yes | yes | yes |
| `UNLOCK` | yes | yes | yes |
| `TOGGLE` | yes | yes | yes |
| `STATUS` | yes | yes | yes |

The D1 Mini nodes and FrontThermostat ignore a repeat of the same command within 1 s.

Lock everything:

```bash
mosquitto_pub -h 192.168.8.1 -t CabLockAllCommand -m LOCK
```

---

## Status

`CabLock<ID>Status` carries `LOCKED`, `UNLOCKED`, or `UNKNOWN`, always retained. Every
board also republishes its status on a 60 s heartbeat.

`UNKNOWN` is the initial value of the state enum but is unreachable in practice — every
board restores its saved state during `setup()` (see below).

---

## Lock state survives a reboot

All five save their lock state and drive the servo back to it on boot, so a brownout while
cranking the engine (or a reflash) does not unlock a cabinet:

| Host | Stored in |
|---|---|
| D1 Mini nodes (v3.1.0+) | EEPROM emulation — 2 bytes, written only when the state changes |
| Plumbing | NVS key `cabLock`, restored before WiFi starts (v3.4.0+) |
| FrontThermostat | EEPROM struct v2, alongside heat mode and setpoint |

Before v3.1.0 the D1 Mini nodes attached and detached the servo in `setup()` with no delay,
so it never actually moved — yet they reported `UNLOCKED`. The first boot on v3.1.0 has
nothing saved and comes up `UNLOCKED`.

**Networking.** The D1 Mini nodes configure MQTT and OTA whether or not WiFi is up at
boot, and retry the broker every 5 s without blocking. Before v3.1.0, a node that powered up
before the router never reached MQTT until it was power-cycled.

---

## Home Assistant

Each of the three dedicated nodes registers as its own device, `chester_cablock_<id>` with
the id lowercased (`chester_cablock_kitchendriverside`). Each exposes one `lock` entity
configured `optimistic: false`, so Home Assistant waits for the retained status publish
instead of assuming the command landed.

The other two appear as `lock` entities on the Front Thermostat and Chester Plumbing System
devices.

---

## The three D1 Mini sketches

Since v3.1.0 the three `src/main.cpp` files are identical apart from `CABINET_ID` and
`otaHostname`. To add a fourth lock node, copy any of them and change those two lines plus
`upload_port` in `platformio.ini`.

---

## Flashing

```bash
pio run -t upload
```

All three are configured for OTA (`CabLock-<ID>.local`, port 8266, password `Chester2025`).
Comment out the `upload_*` block in `platformio.ini` for USB.

Static DHCP reservations on the router are strongly recommended — mDNS resolution of the
`.local` names is unreliable, and an IP you can count on makes OTA far less frustrating:

```bash
pio run -t upload --upload-port 192.168.8.171
```
