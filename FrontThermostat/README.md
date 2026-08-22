# Front Thermostat

ESP32-S2 Mini · `src/main.cpp` · firmware `v4.1.0` · env `lolin_s2_mini`
Cabinet ID `KitchenPassSide` · OTA `Thermostat-KitchenPassSide.local`

Front-zone climate control for a hydronic fan coil. It measures zone temperature with a
BME280, runs a PI controller to set fan speed, and requests heat by publishing
`EngineLoopCommand` — the Plumbing board owns the actual valve. It also hosts the
KitchenPassSide cabinet lock servo and drives a small OLED.

Pins: **[../BOARD_ARCHITECTURE.md](../BOARD_ARCHITECTURE.md)** · Topics: **[../MQTT_TOPICS.md](../MQTT_TOPICS.md)**

---

## Modes

Two modes only, stored as `HeatMode`:

| Mode | `ThermostatMode` | Behaviour |
|---|---|---|
| `HEAT_OFF` | `off` | Closes the engine loop, stops auto fan control, zeroes the PI state |
| `HEAT_AUTO` | `heat` | Full PI heating loop |

`ThermostatModeCommand` accepts `off`, `heat`, or `auto` case-insensitively; `heat` and
`auto` both select `HEAT_AUTO`. Home Assistant's climate entity advertises `off` and `heat`.

Manual fan control keeps working in `HEAT_OFF` — turning heat off does not silence a fan
you deliberately switched on from Home Assistant.

---

## Heating cycle

Evaluated once per second. Error is `setTemp − currentTemp`, in °C internally.

```
error > 0.5 °C  and not already heating
    ├─ publish EngineLoopCommand = OPEN
    ├─ start the fan-delay clock
    └─ heatingActive = true, integral reset

while heating
    ├─ engine loop open < 10.5 min  →  fan held at 0
    └─ engine loop open ≥ 10.5 min  →  fan speed = PI output

error ≤ 0 °C  and heating
    ├─ publish EngineLoopCommand = CLOSE
    ├─ fan to 0
    └─ heatingActive = false, integral reset
```

The asymmetry is intentional: heat is called for at +0.5 °C of error but only cut at 0,
which gives roughly half a degree of hysteresis and stops the loop short-cycling.

### The fan delay

`FAN_DELAY` is **630000 ms — 10.5 minutes**. Running the fan before the coil is hot just
blows cold air into the cabin, so the fan waits for the loop to come up to temperature.

There is credit for a recently-closed loop. If the engine loop was closed less than
10.5 minutes ago, the coil is still warm, so `engineLoopOpenTime` is backdated by however
long it has been closed. Reheating after a short cycle starts the fan immediately rather
than making you wait the full delay again.

---

## PI controller

Output is fan speed as a percentage, 0–100, mapped to 8-bit PWM at 25 kHz on GPIO10.

| Parameter | Value | Purpose |
|---|---|---|
| `U_BIAS` | 30.0 | Baseline output — the fan idles at 30 % rather than from zero |
| `KP` | 11.0 | Proportional gain |
| `KI` | 0.09 | Integral gain |
| `DEADBAND` | 0.5 °C | Errors smaller than this are treated as zero |
| `BOOST_THRESHOLD` | 3.0 °C | Above this error, skip the controller entirely |
| `BOOST_OUTPUT` | 90.0 % | Fixed output while boosting |
| `RATE_LIMIT` | 10.0 %/s | Maximum change in output per second |

Three behaviours worth knowing:

- **Boost.** An error over 3 °C means the zone is cold enough that proportional control is
  pointless — the controller jumps straight to 90 %, resets the integral, and skips the
  rate limiter.
- **Anti-windup.** The integral only accumulates when the output is not already pinned at
  0 or 100 in the direction the error is pushing, and is clamped to `±100/KI`.
- **Rate limiting.** Output changes no faster than 10 % per second, so the fan ramps
  audibly rather than stepping.

---

## Fan control from Home Assistant

The fan is exposed as a full HA `fan` entity, and manual control takes priority over the PI
controller through a `manualFanOverride` flag.

| Command topic | Payload | Effect |
|---|---|---|
| `ThermostatFanCommand` | `ON` | Override on; 50 % if the fan was stopped |
| `ThermostatFanCommand` | `OFF` | Override off, fan to 0 |
| `ThermostatFanSpeedCommand` | 0–100 | Override on when > 0; sets speed directly |
| `ThermostatFanPresetCommand` | `auto` | Override off — PI resumes next cycle |
| `ThermostatFanPresetCommand` | `low` / `medium` / `high` | Override on at 33 / 66 / 100 % |

While the override is set, the PI controller still runs and still opens and closes the
engine loop — it just stops writing to the fan. Setting the preset back to `auto` returns
control without disturbing the heating cycle.

---

## Buttons

Two momentary buttons to ground, `INPUT_PULLUP`, 50 ms debounce.

| Action | Effect |
|---|---|
| UP short press | Setpoint +0.5 °C, capped at 30 °C |
| DOWN short press | Setpoint −0.5 °C, floored at 10 °C |
| UP held ≥ 1 s | Mode → `HEAT_AUTO` |
| DOWN held ≥ 1 s | Mode → `HEAT_OFF` |

Every change saves to EEPROM and republishes state immediately. Note the button range
(10–30 °C, i.e. 50–86 °F) matches the MQTT setpoint limits, so both paths land in the same
range. There is no press-and-hold repeat — each short press is one 0.5 ° step.

---

## Display

64×48 SSD1306 at `0x3C`, sharing the I²C bus with the BME280. Three display states:

| State | Shows | Duration |
|---|---|---|
| `DISPLAY_CURRENT_TEMP` | Current temperature in °F, or `NO SENSOR` | default |
| `DISPLAY_SET_TEMP` | Setpoint in °F | 3 s after a change |
| `DISPLAY_MODE_CHANGE` | `HEAT AUTO` / `HEAT OFF` | 2 s after a change |

If the display is missing at boot the board runs headless and retries `display.begin()`
every 10 s, so a loose I²C connector recovers on its own.

---

## Temperature calibration

`TEMP_CALIBRATION_OFFSET` is **−6.0 °C**, applied to every BME280 reading before it is used
or published. That is a large correction and it is compensating for enclosure self-heating —
the BME280 sits near the ESP32-S2 and the OLED. If you rehouse the board or move the sensor
onto a lead, this constant needs remeasuring against a reference thermometer.

`NaN` readings from a failed I²C transaction are discarded rather than published, so a
glitching sensor freezes the last good value instead of poisoning the control loop.

Internally the setpoint is °C. Everything on MQTT and the display is °F.

---

## Persistence

A 64-byte EEPROM region holds a `ThermostatState` struct guarded by magic `0xA5B3`, a
version byte, and an additive checksum. Only two values persist:

- `heatMode`
- `setTemp` (°C)

A failed magic, version, or checksum check falls back to defaults: mode `OFF`, setpoint
21 °C (69.8 °F). Fan state, override flag, and preset are **not** persisted — the fan comes
up off after any reboot.

---

## Recovery

| Condition | Action |
|---|---|
| WiFi down | Reconnect attempts each loop; `ESP.restart()` after 120 s |
| MQTT down | Reconnect every 5 s; `ESP.restart()` after 120 s |
| Display missing | Retry `begin()` every 10 s |
| BME280 missing at boot | Runs without it — `updateThermostat()` returns immediately, no heating |

There is no Last Will on this board, so Home Assistant shows the last retained values
rather than *unavailable* when it drops off. A stale `ThermostatMode` of `heat` on a board
that is actually offline is the failure mode to watch for.

---

## Building

```bash
pio run -t upload
```

Configured for OTA by default. Comment out the `upload_*` block in `platformio.ini` for a
USB flash. Serial monitor is 115200 baud.
