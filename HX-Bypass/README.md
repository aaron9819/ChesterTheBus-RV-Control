# Heat Exchanger Bypass

Wemos D1 Mini (ESP8266) · `src/main.cpp` · firmware `v1.1.1` · env `d1_mini`
OTA `HX-Bypass.local` · static IP `192.168.8.140`

Drives the solenoid valve that bypasses the engine-coolant heat exchanger. When the
hydronic tank is hot enough, the valve opens and coolant is routed around the exchanger
instead of dumping more heat into the tank.

This board has **no temperature sensor of its own**. It subscribes to `HydronicTemperature`
published by the Plumbing board — if that board is offline, this one keeps acting on the
last value it received.

---

## Hardware

| Pin | Signal | Notes |
|---|---|---|
| D4 (GPIO2) | Valve relay | Active LOW — LOW energises the relay, HIGH releases it |

`setup()` drives the pin HIGH before anything else, so the valve starts closed.

Note that D4/GPIO2 is also the ESP8266's onboard LED pin and a boot-mode strapping pin. It
must be high at boot, which the idle-HIGH relay wiring satisfies.

### Static IP

Unlike every other board, this one configures its own address in firmware:

```
IP       192.168.8.140
Gateway  192.168.8.1
Subnet   255.255.255.0
DNS      8.8.8.8
```

`WiFi.config()` is called again on every reconnect in `loop()`, so the address survives an
AP restart. If you move the board to a different network, that block in `setup()` and
`loop()` both need changing.

---

## Control logic

### Auto mode (default)

```
valve OPEN   when  HydronicTemperature >  setpoint
valve CLOSED when  HydronicTemperature <  setpoint − 2 °F   (HYSTERESIS_F)
in between         no change
```

Setpoint defaults to **160 °F** and is settable over the range 130–200 °F. The 2 °F band
keeps a reading that hovers at the setpoint (a new one every 5 s) from chattering the relay.

A `reverseLogic` flag exists in the source (default `false`) that inverts the comparison if
your valve is plumbed the other way round (open below the setpoint, close 2 °F above it).
It is a compile-time change, not an MQTT one.

**Stale input.** The valve follows Plumbing's `HydronicTemperature`, which is only published
while the buffer-tank probe is reading. If it stops, the valve holds its last state — there
is no timeout yet, because the safe direction depends on how the bypass is plumbed.

### Manual mode

With `HXBypassAutoModeCommand` set to `OFF`, the valve holds whatever state it was in and
responds only to `HXBypassValveCommand`.

A valve command arriving while auto mode is **on** is rejected, and the board immediately
republishes its real state — so a Home Assistant toggle pressed in auto mode visibly snaps
back rather than showing a state the hardware is not in.

Switching auto mode back on re-evaluates the temperature comparison at once.

### Refresh

`updateValve()` re-asserts the relay and republishes status whenever the target state
changes *or* 60 s have elapsed, so the physical relay is periodically driven back to its
intended state even if it was disturbed.

---

## MQTT

| Topic | Direction | Payload |
|---|---|---|
| `HydronicTemperature` | subscribe | °F, from the Plumbing board |
| `HXBypassValveCommand` | subscribe | `ON` / `OFF` — manual mode only |
| `HXBypassValveStatus` | publish, retained | `ON` / `OFF` |
| `HXBypassAutoModeCommand` | subscribe | `ON` / `OFF` |
| `HXBypassAutoModeStatus` | publish, retained | `ON` / `OFF` |
| `HXBypassTempSetpointCommand` | subscribe | float, accepted 130–200 |
| `HXBypassTempSetpointStatus` | publish, retained | e.g. `160.00` |

Setpoint values outside 130–200 are silently ignored — no change, no republish.

---

## Home Assistant

Device **Heat Exchanger Bypass** (`chester_hxbypass`), auto-discovered on every MQTT
connect:

| Entity | Type |
|---|---|
| HX Bypass Valve | switch |
| HX Bypass Auto Mode | switch |
| HX Bypass Temperature Setpoint | number, 130–200 °F, step 1 |

No availability topic and no Last Will, so entities keep showing their last retained value
when the board drops off the network.

---

## No persistence

Nothing is saved. After a reboot or a reflash the board comes up with:

- auto mode **on**
- setpoint **160 °F**
- valve **closed**

A custom setpoint has to be re-sent after every restart. If that matters, a Home Assistant
automation triggered on the board's discovery or first status publish is the simplest fix.

---

## Building

```bash
pio run -t upload
```

The OTA block in `platformio.ini` is **commented out**, so this uploads over USB by default.
The running firmware does enable `ArduinoOTA` regardless — uncomment the
`upload_protocol` / `upload_port` / `upload_flags` lines to push over the air to
`HX-Bypass.local` (password from `secrets.ini`, port 8266), or target `192.168.8.140` directly
if mDNS is being unhelpful.

Serial monitor is 115200 baud. The firmware logs each relay write along with a read-back of
the pin state, which makes wiring problems obvious from the monitor alone.
