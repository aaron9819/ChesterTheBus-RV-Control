# Plumbing & Environment Controller

ESP32-S2 Mini · `src/main_esp32s2.cpp` · firmware `v3.1.0` · env `lolin_s2_mini`

The central board of the RV system. It owns every hydronic valve, both tank heaters, the
diesel and electric heat sources, the main water pump, the exhaust fan, the domestic hot
water solenoid, and the RearPassSide cabinet lock. It reads four DS18B20 temperature
sensors, a BME280, a manifold pressure transducer, and a turbine flow sensor.

Pin assignments and sensor wiring: **[../BOARD_ARCHITECTURE.md](../BOARD_ARCHITECTURE.md)**
Topic contract: **[../MQTT_TOPICS.md](../MQTT_TOPICS.md)**

---

## The board is authoritative

This is the design principle the rest of the firmware follows. The board — not Home
Assistant, not the broker — holds the true state of every output.

1. Every `control*()` function writes the GPIO, updates the in-memory state, marks NVS
   dirty, and publishes the new state **retained**.
2. On boot, `restoreValveStatesFromEEPROM()` reads every output back out of NVS and drives
   the hardware to match *before* WiFi or MQTT come up.
3. On every MQTT connect, `publishAllStatesToBroker()` overwrites the broker's retained
   values with the board's own.

The board never subscribes to a topic it publishes. Commands arrive on `…Command`, states
leave on `…Status`. That separation is what prevents a retained state message from being
replayed to the board as a command on reconnect.

---

## Automation

All automation is gated on a single global mode published to `PlumbingMode` and set via
`PlumbingModeCommand`. `MANUAL` disables every rule below; direct MQTT commands still work
in both modes.

### Tank freeze protection

| Rule | Condition |
|---|---|
| Fresh water heater ON | `FreshWaterTemperature < FreshWaterHeatTempSet` (default 40 °F) |
| Fresh water heater OFF | `FreshWaterTemperature > setpoint + 2 °F` |
| Grey water heater ON | `GreyWaterTemperature < GreyWaterHeatTempSet` (default 35 °F) |
| Grey water heater OFF | `GreyWaterTemperature > setpoint + 2 °F` |

Both tank readings are 5-sample moving averages, which damps the DS18B20 quantisation
noise that would otherwise chatter the relays around the setpoint.

### Exhaust fan

| Rule | Condition |
|---|---|
| Fan ON | `EnvironmentTempAmbient >= ExhaustFanTempSet` (default 85 °F) |
| Fan OFF | `EnvironmentTempAmbient < setpoint − 3 °F` |

3 °F of hysteresis. Requires a working BME280; with no sensor the rule never fires.

### Domestic hot water on demand

Independent of `PlumbingMode` — this runs unconditionally, driven from the flow sensor
interrupt, because it has to react at tap-open speed.

```
flow detected  → open DHW solenoid immediately, cancel any pressure relief
flow stops     → close DHW solenoid (unless a pressure-relief window is open)
```

Flow is considered real only when both conditions hold: a pulse arrived within the last
3 s, **and** at least 10 pulses were counted in the last 1 s window. The pulse-count floor
is what rejects drips and electrical noise on the sensor line.

There is a 2 s `PRESSURE_RELIEF_DURATION` window during which the solenoid is held open
after a stop; when it expires the valve closes only if flow is still absent.

---

## Safety

### Humidity leak detection

The BME280 doubles as a leak detector. A sharp humidity rise in a sealed bay means water
where it should not be. But the sensor also produces transient spikes, so detection is
two-stage — a spike must *persist* to count.

| Constant | Value |
|---|---|
| `HUMIDITY_SPIKE_THRESHOLD` | +10 % RH |
| Detection window | rise must occur in under 60 s |
| `SPIKE_RECOVERY_TIMEOUT` | 30 s |
| `HUMIDITY_SPIKE_TOLERANCE` | 3 % RH |

```
rise > 10% within 60s   →  record baseline, start 30s timer, state = PENDING
   ├─ humidity falls to within 3% of baseline before 30s  →  transient, discard
   └─ still elevated after 30s                            →  CONFIRMED LEAK
```

On a confirmed leak the board:

1. Closes the rear, engine, and front loop valves (clearing the persisted engine/front
   loop MQTT requests so they do not reopen on reboot).
2. Turns off the grey water heater.
3. Turns off the main water pump.
4. Publishes `High Humidity - possible leak` to `PlumbingAlert` (retained).
5. Publishes `LEAK_DETECTED_SHUTDOWN` to `PlumbingSystemStatus` (retained).

The alarm latches. Clear it only after finding the water:

```bash
mosquitto_pub -h 192.168.8.1 -t ClearHighHumidityCommand -m clear
```

Clearing resets the alarm flag, discards any pending spike, and rebaselines humidity to the
current reading. It does **not** reopen the valves or restart the pump — that is a
deliberate manual step.

### Temperature alerts

| Alert | Condition | Message |
|---|---|---|
| Freeze warning | fresh or grey tank < 35 °F | `Tank Freeze Warning ACTIVE` |
| High temperature | hydronic tank > 180 °F | `High Hydronic Tank Temp ACTIVE` |

Both are advisory — they publish to `PlumbingAlert` and log to serial, but take no action.
Freeze protection is handled by the tank heater automation above.

### Alert priority

`PlumbingAlert` is a single retained topic carrying the most severe active alert:

```
High Humidity  >  High Hydronic Tank Temp  >  Tank Freeze Warning  >  OKAY
```

When a condition clears, `refreshAlertPublish()` recomputes and republishes, dropping back
to `OKAY` when nothing is active.

### Watchdog

An `esp_task_wdt` at 60 s is fed once per `loop()`. If the loop stalls — a hung I²C
transaction, a blocking network call — the board panics and resets. The watchdog is
detached for the duration of an OTA upload and reattached on both the success and error
paths.

---

## Diesel heater

Three relays drive the heater's control harness. Despite the wiring, the firmware exposes
a plain on/off switch — the older three-state (OFF / PUMPONLY / HIGH) select was removed,
and the board actively deletes its stale Home Assistant discovery config on connect.

| Relay | GPIO | Wire | ON state | OFF state |
|---|---|---|---|---|
| 1 | GPIO8 | white | LOW (energised) | HIGH |
| 2 | GPIO7 | blue | HIGH (de-energised) | HIGH |
| 3 | GPIO6 | yellow | LOW (energised) | HIGH |

All three relays are active LOW. "ON" energises circuits 1 and 3 while leaving circuit 2
open; "OFF" releases all three. This combination is restored verbatim from NVS on boot.

The **electric heater** on GPIO37 is the one active-HIGH output on the board — `HIGH` is on.

---

## Persistence

Uses the ESP32 `Preferences` API (NVS flash), namespace `plumbing`. Two groups of keys:

**Configuration** — written immediately on change via `saveConfigToPreferences()`:

| Key | Contents | Default |
|---|---|---|
| `fwTarget` | Fresh water setpoint °F | 40.0 |
| `gwTarget` | Grey water setpoint °F | 35.0 |
| `fanSet` | Exhaust fan setpoint °F | 85.0 |
| `plumbingMode` | `AUTO` / `MANUAL` | `AUTO` |
| `engLoopMqtt` | Last engine loop request | false |
| `frtLoopMqtt` | Last front loop request | false |

**Output state** — written through a debounce to spare the flash:

| Key | Output | Restore default |
|---|---|---|
| `freshHeat` | Fresh water heater | off |
| `greyHeat` | Grey water heater | off |
| `exhaustFan` | Exhaust fan | off |
| `rearLoop` | Rear loop valve | closed |
| `engineLoop` | Engine loop valve | closed |
| `frontLoop` | Front loop valve | closed |
| `dieselHtr` | Diesel heater | off |
| `electricHtr` | Electric heater | off |
| `mainPump` | Main water pump | **on** |
| `dhwSolenoid` | DHW solenoid | closed |
| `cabLock` | Cabinet lock | unlocked |

`saveValveStatesToEEPROM()` only sets a dirty flag; `flushEEPROM()` performs the actual
write from `loop()` once 2 s have passed with no further change. A burst of valve changes
therefore costs one flash write, not ten.

The main pump defaults to **on** because its relay is normally closed — the safe failure
mode is water pressure available, not a dead pump.

---

## Sensor calibration notes

**Manifold pressure.** The transducer's ADC reading at 0 PSI is 580 counts, subtracted as
`ADC_ZERO_OFFSET` before conversion. If a replacement sensor reads non-zero at rest, that
constant is what to adjust. Full scale assumes 0–80 PSI across the 3.3 V range; readings
clamp at 85 PSI.

**Flow rate.** `450.0` pulses per litre, correct for a YF-S201. A different turbine means a
different K-factor — it is the only constant in `calculateFlowRate()`.

**Temperature sensor order.** The four DS18B20s are addressed by discovery index on a
shared bus, not by ROM address, so **replacing or rewiring a sensor can silently reshuffle
which reading is which**. The board publishes all four raw indices to
`Debug/SensorIndex_0`…`_3` every 30 s for exactly this reason. After any bus work, warm one
sensor by hand and watch which index moves:

```bash
mosquitto_sub -h 192.168.8.1 -t 'Debug/SensorIndex_+' -v
```

Expected order is `0` return manifold, `1` hydronic, `2` grey tank, `3` fresh tank.

---

## Building

```bash
pio run
```

```bash
pio run -t upload
```

`platformio.ini` is configured for OTA to `Plumbing.local`. Comment out the `upload_*`
block for a USB flash. Serial monitor is 115200 baud.

The boot banner reports firmware version, every DS18B20 address found with its index,
BME280 presence, and the full restored relay state — read it after any flash to confirm
the sensor bus and outputs came back the way you expect.
