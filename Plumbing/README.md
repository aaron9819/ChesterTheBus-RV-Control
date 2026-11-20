# Plumbing & Environment Controller

ESP32-S2 Mini · `src/main_esp32s2.cpp` · firmware `v3.6.1` · env `lolin_s2_mini`

The central board of the RV system. It owns every hydronic valve, both tank heaters, the
diesel and electric heat sources, the main water pump, the exhaust fan, the domestic hot
water solenoid, and the RearPassSide cabinet lock. It reads five DS18B20 temperature
probes on one OneWire bus, a BME280, and a turbine flow sensor.

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

**Probe-loss fail-safe.** A heater whose own probe has died has no control input — neither
rule above can fire, so it would stay energised indefinitely. In AUTO, a heater whose probe
has been missing for 60 s is switched OFF and `PlumbingAlert` names the position. MANUAL
leaves the user in charge. Set `TEMP_PROBE_LOSS_FAILSAFE` to `false` in the source to
restore the old behaviour (heater holds its last state, alert only).

### Exhaust fan

| Rule | Condition |
|---|---|
| Fan ON | `EnvironmentTempAmbient >= ExhaustFanTempSet` (default 85 °F) |
| Fan OFF | `EnvironmentTempAmbient < setpoint − 3 °F` |

3 °F of hysteresis. Requires a working BME280; with no sensor the rule never fires.

### Domestic hot water on demand

Independent of `PlumbingMode` — this runs unconditionally, off the flow sensor interrupt.

A four-state machine evaluated every **50 ms**:

```
IDLE ──flow present──▶ STARTING ──held 600 ms──▶ ACTIVE ──flow gone──▶ STOPPING ──quiet 1.5 s──▶ IDLE
         ▲                  │                                              │
         └──dropped out─────┘                    └──flow returns───────────┘
```

The solenoid is touched only on the two committed transitions: `STARTING → ACTIVE` opens
it, `STOPPING → IDLE` closes it.

**Presence is deliberately hair-trigger.** A single pulse counts, and keeps counting until
the line has been quiet for `FLOW_PULSE_IDLE_MS` (500 ms). That is what makes a trickle
register. Noise is rejected by **duration**, not by a pulse-count floor — a burst has to
sustain for `FLOW_ON_DELAY_MS` and land at least 5 pulses to open the valve.

**Why it works that way.** The old test was "≥10 pulses in the last 1 s window". Closing
the solenoid kicks the turbine hard enough to clear that outright, so a tap shutting off
produced *close → reopen for a second → close*. Three things now prevent it: the burst
cannot outlast the 600 ms on-delay, the 50 ms evaluation means a blip can no longer be
rounded up to a full second, and a 1 s `FLOW_CLOSE_LOCKOUT_MS` refuses to even start the
on-delay for pulses arriving right after the board itself closed the valve.

The 1.5 s off-delay also serves the purpose the old `PRESSURE_RELIEF_DURATION` was meant
to — holding the valve open briefly after flow stops. That code was dead (the flag was
never set `true`) and has been removed.

Tuning knobs are all at the top of `src/main_esp32s2.cpp`: `FLOW_ON_DELAY_MS` is what to
raise if a reopen ever survives, `FLOW_PULSE_IDLE_MS` is the sensitivity to a trickle.

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

The alarm latches. Clear it only after finding the water — with the **Clear Leak Alarm**
button in Home Assistant, or:

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
| High temperature | buffer tank > 180 °F | `High Hydronic Tank Temp ACTIVE` |
| Probe missing | a harness position is unbound or off the bus | `Temp probe missing: <positions>` |

All are advisory — they publish to `PlumbingAlert` and log to serial. The only alert
condition that takes an action is a missing tank probe, via the fail-safe above.

### Alert priority

`PlumbingAlert` is a single retained topic carrying the most severe active alert:

```
High Humidity  >  High Hydronic Tank Temp  >  Tank Freeze Warning
               >  Boiler fault  >  Temp probe missing  >  OKAY
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

## Engine loop valve travel

`EngineLoopStatus` only says which way the relay drives the valve. The board also models
where the valve physically is: it takes **10.5 minutes** (`ENGINE_LOOP_TRAVEL_MS`) to travel
fully either way, starting from wherever it is when the command changes. Closed for 3 minutes
and then reopened, it is 3 minutes from fully open again, not 10.5.

| Topic | Payload |
|---|---|
| `EngineLoopValveState` | `OPEN` / `OPENING` / `CLOSING` / `CLOSED` (retained, on every change) |
| `EngineLoopValvePosition` | `0`–`100` % open (retained; every 30 s while moving). Rounds toward the end it is leaving, so `100` only ever means `OPEN` and `0` only `CLOSED` |

The FrontThermostat holds its fan until `EngineLoopValveState` reads `OPEN`.

The position is saved to NVS (`elTravel`) on every state change and once a minute while
moving. On boot:

| Reset | Assumed position |
|---|---|
| Warm (OTA, restart, crash, watchdog) | The saved position — the relay only dropped for a second or two |
| Power-on or brownout | Closed — the valve may have moved for an unknown time; it travels from there |
| Warm, nothing saved yet (first boot on v3.5.0) | Fully in whichever direction the relay is set |

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

**Flow rate.** The L/min conversion is `(pulses / K) x (60000 / interval_ms)`, which is
dimensionally correct — so a reading that is "way off" means `K`, the pulses-per-litre
figure, does not match the turbine actually fitted. 450 is the YF-S201 value
(F = 7.5 x Q L/min); other turbines differ by an order of magnitude (a YF-S402 is nearer
5880), which is exactly the kind of error that looks like a unit problem.

`K` is no longer hardcoded. Measure it with a bucket test:

```bash
mosquitto_pub -h 192.168.8.1 -t HotWaterFlowCalibrateCommand -m START
# run water into a measured container, then tell the board what came out
mosquitto_pub -h 192.168.8.1 -t HotWaterFlowCalibrateCommand -m 1G
```

A trailing `G` is US gallons, `L` or a bare number is litres. The board computes
`K = pulses / litres`, saves it to NVS, and republishes `HotWaterFlowKFactor`. It rejects
runs under 100 pulses and any result outside 50–20000 pulses/L. *Start Flow Calibration*
in Home Assistant sends the `START` step.

The larger the measured volume, the better the result — a gallon is far more accurate than
a cup. `HotWaterFlowPulses` (raw count since boot) is published as a diagnostic if you
would rather compute `K` by hand.

Rate is published in both `HotWaterFlowRate` (L/min) and `HotWaterFlowRateGPM`, with
running totals on `HotWaterFlowTotal` and `HotWaterFlowTotalGal`. Totals are per-boot and
are derived from the pulse count at publish time, so recalibrating `K` corrects them
retroactively.

**Temperature harness.** The five DS18B20s are bound to harness positions by 64-bit ROM
address, held in NVS — never by discovery index. Replacing or rewiring a probe cannot
reshuffle which reading is which; a replaced probe simply reports as missing until it is
bound.

| Position key | Reading | Topic |
|---|---|---|
| `supply_manifold` | Supply manifold | `PlumbingTempSupplyManifold` |
| `return_manifold` | Return manifold | `PlumbingTempReturnManifold` |
| `buffer_tank` | Hydronic buffer tank | `HydronicTemperature` |
| `grey_tank` | Grey water tank | `GreyWaterTemperature` |
| `fresh_tank` | Fresh water tank | `FreshWaterTemperature` |

**Binding a probe by hand.** Press *Identify \<position\>* in Home Assistant, then hold that
probe in your hand. The board snapshots every sensor and watches for 60 s; the one that
rises ≥4 °F, beating the runner-up by ≥2 °F, takes the position and is saved to NVS. The
margin is what stops a bundle of probes in the same warm bay producing a coin-flip. Watch
it happen on serial, or on the *Temp Identify State* entity.

```bash
mosquitto_pub -h 192.168.8.1 -t PlumbingTempIdentifyCommand -m grey_tank
```

**Binding by address**, when the ROM is already known from a bus report:

```bash
mosquitto_pub -h 192.168.8.1 -t PlumbingTempAssignCommand -m 'grey_tank=28FF641E1234ABCD'
```

A bus index from the report works too (`grey_tank=2`), and an empty value clears the
position. `PlumbingTempSensorCommand` takes `SCAN`, `REPORT`, and `CLEAR`.

**Checking the harness.** `PlumbingTempSensorsFound` counts devices on the bus,
`PlumbingTempSensorsLive` counts positions bound and reading, and `PlumbingTempSensorScan`
carries the full map as JSON. Each position also drives `PlumbingTempPresence/<key>`, which
is a Home Assistant availability topic — a probe that drops off reads *unavailable* rather
than freezing on its last value.

**Why a probe is failing.** Each sensor in `PlumbingTempSensorScan` carries `reads` and
`errors` counts, and the JSON carries `parasite` (true if any probe reports parasite
power — a missing VDD wire). While a read is failing, the sensor also gets `err` and `sp`
(the raw 9-byte scratchpad that failed). The scan is republished every 60 s while anything
is failing, and on demand with *Print Temp Bus Report*. Each read is retried once.

| `err` | Meaning | Usual cause |
|---|---|---|
| `no_presence` | No presence pulse on reset | Bus shorted or open, or a device holding the line low |
| `no_reply` | Scratchpad all `FF` — nothing answered its address | Probe lost power mid-bus, marginal signalling |
| `all_zero` | Scratchpad all `00` | Data line held low during the read |
| `crc` | Scratchpad CRC mismatch | Pullup value, run length, star wiring, a bad splice, two devices answering |
| `power_on_85C` | 85 °C power-on value — conversion never ran | VDD missing or too weak (parasite power) |
| `range` | Outside −67…257 °F | Probe past its rating |

**Range limit.** A DS18B20 is rated −55…+125 °C. Readings outside −67…257 °F are counted as
failed reads, not temperatures. The boiler exhaust position was removed for this reason — an
exhaust outlet runs well past the part's rating, and measuring it wants a K-type thermocouple
and a MAX31855/31856 on its own channel.

---

## Retired entities

Removing a feature from the firmware does not remove it from Home Assistant. Discovery
configs and state values are published **retained**, so the broker keeps serving them to HA
forever unless something overwrites them with an empty retained message.

`publishHomeAssistantDiscovery()` ends with a block that does exactly that, for:

| Entity | Why it went |
|---|---|
| Hot Water Pump Speed / Status / Pressure, Pump Error | PWM pump controller no longer fitted |
| Manifold Pressure | transducer removed |
| Diesel Heater (`select`) | replaced by a plain switch |

It runs on every MQTT connect and is idempotent, so it can stay indefinitely. To retire
something else later, add it to `retiredDiscovery` (the HA entity) and `retiredTopics` (its
retained state), or clear one by hand:

```bash
mosquitto_pub -h 192.168.8.1 -t 'homeassistant/sensor/chester_plumbing/<object_id>/config' -r -n
mosquitto_pub -h 192.168.8.1 -t '<StateTopic>' -r -n
```

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

The boot banner reports firmware version, the restored harness positions, a full OneWire
bus table, BME280 presence, and the restored relay state — read it after any flash to
confirm the sensor bus and outputs came back the way you expect.

On the first boot after upgrading from `v3.1.0` no positions are bound, so every
temperature reads unavailable, `PlumbingAlert` names all five, and freeze protection is
inactive until you bind them. Identify each probe once and it persists.
