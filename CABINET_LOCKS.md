# Cabinet Locks

Five servo-driven cabinet locks. Three have dedicated D1 Mini Lite boards; the other two
are hosted on larger controllers that had a spare GPIO.

| Cabinet | Host | Servo pin | Firmware |
|---|---|---|---|
| KitchenDriverSide | `D1Mini_CabLock_KitchenDriverSide` | D4 (GPIO2) | `v3.0.0` |
| KitchenUpperPassSide | `D1Mini_CabLock_KitchenUpperPassSide` | D4 (GPIO2) | `v3.0.0` |
| RearDriverSide | `D1Mini_CabLock_RearDriverSide` | D4 (GPIO2) | `v3.0.0` |
| KitchenPassSide | `FrontThermostat` (ESP32-S2) | GPIO8 | `v4.1.0` |
| RearPassSide | `Plumbing` (ESP32-S2) | GPIO16 | `v3.1.0` |

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
| `TOGGLE` | yes | yes | — |
| `STATUS` | yes | yes | — |

FrontThermostat implements only `LOCK` and `UNLOCK`; anything else is ignored. This matters
if you script against `CabLockAllCommand` with `TOGGLE` — four locks would flip and the
KitchenPassSide one would not.

Lock everything:

```bash
mosquitto_pub -h 192.168.8.1 -t CabLockAllCommand -m LOCK
```

---

## Status

`CabLock<ID>Status` carries `LOCKED`, `UNLOCKED`, or `UNKNOWN`, always retained. Every
board also republishes its status on a 60 s heartbeat.

`UNKNOWN` is the initial value of the state enum but is unreachable in practice — every
board drives its servo to UNLOCKED during `setup()` and sets the state accordingly.

---

## No persistence — locks open on reboot

None of the D1 Mini lock nodes save state. `setup()` unconditionally drives the servo to
`UNLOCKED_POSITION`, so **a power cut or a reflash unlocks the cabinet**.

The Plumbing board is the exception: it persists `cabLock` to NVS and restores the locked
position on boot.

FrontThermostat does not persist its lock state either — its EEPROM struct holds only the
heat mode and setpoint.

This is worth knowing before flashing a lock node while the vehicle is moving.

---

## Home Assistant

Each of the three dedicated nodes registers as its own device, `chester_cablock_<id>` with
the id lowercased (`chester_cablock_kitchendriverside`). Each exposes one `lock` entity
configured `optimistic: false`, so Home Assistant waits for the retained status publish
instead of assuming the command landed.

The other two appear as `lock` entities on the Front Thermostat and Chester Plumbing System
devices.

---

## Source differences between the three D1 Mini nodes

The three sketches are near-identical. Only `CABINET_ID` and `otaHostname` are meant to
differ, but two nodes have drifted:

| | KitchenDriverSide | RearDriverSide | KitchenUpperPassSide |
|---|---|---|---|
| 1 s duplicate-command cooldown | yes | yes | **no** |
| `ArduinoOTA.handle()` position in `loop()` | after WiFi check | **first** | after WiFi check |

The missing cooldown on KitchenUpperPassSide means a repeated `LOCK` will re-drive the
servo rather than being ignored. Harmless, but it is a real difference, not a
documentation gap. Both other differences are cosmetic.

If you are adding a fourth lock node, copy `D1Mini_CabLock_KitchenDriverSide` — it has the
cooldown — and change `CABINET_ID`, `otaHostname`, and `upload_port`.

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
