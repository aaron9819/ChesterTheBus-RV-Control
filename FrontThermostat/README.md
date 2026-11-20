# Front Thermostat

ESP32-S2 Mini · `src/main.cpp` · firmware `v5.3.0` · env `lolin_s2_mini`
Cabinet ID `KitchenPassSide` · OTA `Thermostat-KitchenPassSide.local`

Front-zone climate control for a hydronic fan coil. It measures zone temperature with a
BME280, runs a PI controller to set fan speed, and requests heat by publishing
`EngineLoopCommand` — the Plumbing board owns the actual valve. It also hosts the
KitchenPassSide cabinet lock servo and drives a small OLED.

Pins: **[../BOARD_ARCHITECTURE.md](../BOARD_ARCHITECTURE.md)** · Topics: **[../MQTT_TOPICS.md](../MQTT_TOPICS.md)**

Requires Arduino-ESP32 core **3.x** (last built against 3.2.0). WiFi and OTA
credentials come from `secrets.ini` at the repo root (gitignored; template `secrets.example.ini`).

---

## Modes

| Mode | `ThermostatMode` | Behaviour |
|---|---|---|
| `HEAT_OFF` | `off` | Requests the engine loop closed, stops automatic fan control |
| `HEAT_AUTO` | `heat` | Engine loop held open at/below setpoint; PI fan control |

`ThermostatModeCommand` accepts `off`, `heat`, or `auto` case-insensitively. Manual fan
control from Home Assistant keeps working in `HEAT_OFF`.

---

## Heating cycle

Evaluated once per second. Error is `setpoint − room`, in °C internally; the setpoint is
stored in °F.

The engine loop and the fan are decided separately.

**Engine loop** — held open whenever heat is on and the room is at or below the setpoint:

```
room ≤ setpoint                   → request engine loop OPEN
room > setpoint + 0.5 °F          → request engine loop CLOSE
in between                        → no change (keeps the valve from cycling on sensor noise)
```

The margin is `LOOP_CLOSE_ABOVE_F`. Keeping the loop open below the setpoint means the coil
is already hot when the fan is needed, so there is no 10.5-minute warm-up between cycles.

**Fan (PI)** — runs on its own hysteresis while the loop stays open:

```
error > 0.5 °C and not heating   → heatingActive = true (calling for heat)
heating, coil still warming       → fan held at 0 ("WARM" on the display)
heating, coil warm                → fan = PI output ("HEAT")
error ≤ 0 °C and heating          → heatingActive = false, fan ramps down; loop stays open
```

Heat mode `off` or a temperature-sensor fault stops the fan and closes the loop.

### Engine-loop requests are held, not fired once

The wanted valve state is kept and re-sent until the broker accepts it, so a request made
while MQTT is down is delivered on reconnect. After every boot the thermostat asserts
`CLOSE` once (unless it immediately needs heat), so a valve left open by a crash or OTA
is closed.

The thermostat subscribes to Plumbing's `EngineLoopStatus`. If the valve is reported
**closed** more than 20 s after the thermostat asked for heat (Plumbing's leak shutdown,
or someone using HA's *Engine Loop Valve* switch), it does **not** re-open it — Plumbing
does not block re-opening during a leak alarm, so that would defeat the shutdown. Instead
it stops the fan, shows `VALVE!`, and raises *Heat Valve Fault* in HA. A deliberate user
action (button press, mode or setpoint change) retries; the valve reported open again
clears the fault on its own.

### The fan delay (coil warm-up)

The fan waits until the coil is hot, which is when the engine-loop valve is fully open.
That is decided by the **Plumbing board's valve-travel model**: it publishes
`EngineLoopValveState`, and the coil counts as hot only while that reads `OPEN`. Plumbing
owns the valve and keeps the model running (and saved) through a thermostat reboot, so an
OTA or power blip on the thermostat no longer costs a 10.5-minute fan pause while the loop
has been open all along.

The valve takes 10.5 minutes to travel fully either way, starting from wherever it is: closed
for 3 minutes and then reopened, it is 3 minutes from fully open again.

**Fallback.** While Plumbing is `OFFLINE`, or running firmware without the topic, the
thermostat uses its own copy of the same model: `FAN_DELAY` (10.5 min) of warmth built while
its OPEN request stands, drained at the same rate after it is withdrawn. That copy starts cold
at boot and its clock starts when the OPEN request is actually delivered.

---

## PI controller

Output is fan speed as a percentage, 0–100, mapped to 8-bit PWM at 25 kHz on GPIO10.

| Parameter | Value | Purpose |
|---|---|---|
| `U_BIAS` | 30.0 | Baseline output |
| `KP` | 11.0 | Proportional gain (%/°C) |
| `KI` | 0.09 | Integral gain |
| `DEADBAND` | 0.5 °C | Inside this, no proportional term and the integral bleeds off (120 s) |
| `BOOST_ENTER` / `BOOST_EXIT` | 3.0 / 2.5 °C | Boost at 90 % with hysteresis |
| `RATE_LIMIT` | 10 %/s | Ramp limit for the automatic fan, both up and down |

The controller keeps running through the hysteresis band, so the fan winds down toward
30 % as the room approaches the setpoint instead of sitting at 100 % and cutting out. The
integral is clamped so bias + integral never exceeds 100 %, stops integrating while
saturated, and the time step is capped at 5 s so long gaps can't wind it up.

---

## Fan control from Home Assistant

| Command | Effect |
|---|---|
| `ThermostatFanCommand ON` | Manual, at the current speed (50 % if stopped) |
| `ThermostatFanCommand OFF` | While heating: fan held off for the rest of this heat cycle. Otherwise: back to auto |
| `ThermostatFanSpeedCommand 1–100` | Manual at that speed (preset shows as none) |
| `ThermostatFanSpeedCommand 0` | Same as OFF |
| `ThermostatFanPresetCommand auto` | Back to automatic control, ramping from the current speed |
| `ThermostatFanPresetCommand low/medium/high` | Manual at 33 / 66 / 100 % |

The published percentage is exactly what was requested (no rounding drift), and the
preset state shows `None` whenever a custom manual speed is in effect. Manual control is
shown on the display with an `M`.

---

## Buttons

| Action | Effect |
|---|---|
| Press UP or DOWN | First press shows the setpoint; further presses change it by 1 °F |
| Hold UP or DOWN | Repeats after 0.6 s, faster after 2.5 s |
| Hold **both** for 1.5 s | Toggles heat on/off (screen shows `MODE` / `HEAT` or `OFF`) |
| Any press while the screen is off | Only wakes the screen |

Presses act on press, not release. Setpoint range is 50–86 °F, the same as Home Assistant.
Setpoint changes are saved 2 s after the last press.

---

## Display

64×48 SSD1306 at `0x3C`, sharing the I²C bus with the BME280. Status line in rows 0–7; the
big value (size 3 when it fits, else 2) is centred below it.

The panel is wired for the SSD1306's *alternative* COM-pin layout (`0x12`), but
Adafruit_SSD1306 has no 64×48 case and sends the sequential layout (`0x02`). That
interleaves the picture — buffer rows 16–23 never appear and the rest is stretched — which
is why older firmware drew everything from row 20 down and lost the top of the status line.
`beginDisplay()` sends `0x12` after `begin()`, so all 48 rows now map 1:1.

| Top line | Meaning |
|---|---|
| `OFF` | Heat mode off |
| `AUTO 70` | Heat on, fan not running, setpoint 70 °F (engine loop open if room ≤ setpoint) |
| `WARM>70` | Calling for heat, coil warming up, fan waiting |
| `HEAT>70` | Calling for heat, fan running |
| `SENSOR!` | Temperature sensor fault — heating disabled |
| `VALVE!` | Engine loop closed by something else while heat was requested |
| `SET` / `SET (OFF)` | Setpoint view (heat mode off) |

Flags on the right: `M` = fan under manual control, `!` = not connected to MQTT. The large
number is room temperature rounded to the nearest °F (`--F` when unknown).

The screen dims after 30 s without a press and turns off after 10 minutes (it stays on,
dimmed, while a fault is shown). The panel is probed every 2 s, so one that drops off the
bus or is plugged in later is re-initialised automatically.

---

## Temperature sensor

`TEMP_CALIBRATION_OFFSET` is **−6.0 °C**, compensating for enclosure self-heating. If you
rehouse the board, remeasure it against a reference thermometer. Humidity is corrected to
room temperature (the sensor reads RH at its own warmer temperature, which under-reports).

The sensor is health-checked on every read: it must answer on I²C, must not have reset into
sleep mode (it is re-initialised if it has), and readings must be plausible. With no valid
reading for 60 s the thermostat raises *Sensor Fault*, stops heating and closes the loop. A
sensor missing at boot is retried every 10 s.

---

## Persistence

A 64-byte EEPROM region holds a `ThermostatState` struct (version 2) guarded by a magic
number and a Fletcher-16 checksum. Persisted: heat mode, setpoint (°F), and cabinet lock
state. Invalid data falls back to OFF, 70 °F, unlocked.

> Versions before v5.0.0 had a checksum bug that meant saved settings never restored, so
> every reboot came up OFF.

---

## Cabinet lock

`CabLockKitchenPassSideCommand` and `CabLockAllCommand` accept `LOCK`, `UNLOCK`,
`TOGGLE`, `STATUS` (case-insensitive). The lock state is saved, and the servo is returned
to the saved position at boot — it no longer unlocks on every reboot. Servo moves are
non-blocking. The servo library is pinned to LEDC timer 0 and the fan to channel 4
(timer 2) so they cannot share PWM hardware.

---

## Recovery

| Condition | Action |
|---|---|
| WiFi down | Core auto-reconnect; connection attempt restarted every 30 s; reboot after 15 min |
| MQTT down | Reconnect every 5 s while WiFi is up; reboot after 15 min |
| Display missing | Probed every 2 s, re-initialised when found |
| BME280 missing/failed | Retried every 10 s; heating disabled while faulted |

Networking never blocks the loop for long, so buttons and heating keep working while
offline. The board publishes `ThermostatAvailability` (`online`, with an `offline` Last
Will), and every HA entity uses it, so HA shows *unavailable* when the board drops off.

---

## Building

```bash
pio run -t upload
```

Configured for OTA by default. Comment out the `upload_*` block in `platformio.ini` for a
USB flash. Serial monitor is 115200 baud.
