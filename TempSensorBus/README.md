# TempSensorBus

ESP32-S2 Mini · `src/main.cpp` · firmware `v1.1.1` · env `lolin_s2_mini`

A bring-up rig for the five DS18B20 sensors that will live on one shared OneWire bus. It
exists to answer three questions while the harness is being built, without a laptop under
the bus:

1. **Is every sensor on the bus?** — the board re-enumerates every 30 s, so a sensor
   spliced in mid-build appears on its own.
2. **Which physical sensor is which?** — press an Identify button in Home Assistant, warm
   that sensor with your hand, and the board binds it to that harness position by ROM
   address.
3. **Is the bus healthy?** — CRC failures, parasite power, per-sensor error counts, and
   raw per-address readings are all published.

Assignments are stored by **64-bit ROM address in NVS**, not by discovery index. Index
order changes whenever a sensor is added, removed, or replaced; addresses do not. Adding a
seventh sensor to this bus will not silently swap two readings, which is the failure mode
the Plumbing board's index-based mapping has.

---

## Hardware

| Function | Pin | Notes |
|---|---|---|
| DS18B20 OneWire bus | GPIO13 | One 4.7 kΩ pullup, data → 3.3 V, at the controller end only |
| Status LED | GPIO15 | Onboard LED — solid when all roles are live, blinking otherwise |

Same bus pin as the Plumbing board, so harness wiring is interchangeable between the two.

### Wiring

All five sensors are wired in parallel on three conductors:

```
3.3V ──┬──────┬──────┬──────┬──────┬──  VDD (red)
       │      │      │      │      │
      [S1]   [S2]   [S3]   [S4]   [S5]
       │      │      │      │      │
DATA ──┴──────┴──────┴──────┴──────┴──  (yellow/white)
GND  ───────────────────────────────────  (black)

        4.7kΩ from DATA to 3.3V, once, at the ESP32 end
```

Notes that matter on a long RV harness:

- **VDD must be wired.** Leaving it off puts the bus in parasite mode, which is unreliable
  at length and with 12-bit conversions. The board publishes `TempBusParasitePower` = `ON`
  if it detects this — that entity going ON means a sensor is missing its red wire.
- **3.3 V, not 5 V.** The ESP32-S2 data pin is not 5 V tolerant.
- **One pullup only.** A second 4.7 kΩ somewhere down the run will look like intermittent
  CRC failures.
- Star topology (long spurs off a trunk) is the usual cause of CRC failures on a bus this
  size. Daisy-chain where you can.

### No exhaust probe on this bus

The diesel boiler exhaust position was removed. A DS18B20 is rated **−55 to +125 °C
(−67 to 257 °F)** and the die sits in a plastic or crimped-steel package; a Webasto/Espar
exhaust pipe runs 200–300 °C under load, so a DS18B20 clamped there reads its rail, drifts,
and then fails. Measuring the outlet wants a **K-type thermocouple with a MAX31855/31856**
on its own channel, not a probe on this bus.

The firmware still treats anything outside −67…257 °F as a read error, increments that
sensor's error counter, and logs it, so an out-of-range probe anywhere on the bus shows up
clearly rather than silently.

---

## Harness positions

| Role key | Home Assistant entity | State topic |
|---|---|---|
| `supply_manifold` | Supply Manifold Temp | `TempBusSupplyManifold` |
| `return_manifold` | Return Manifold Temp | `TempBusReturnManifold` |
| `buffer_tank` | Buffer Tank Temp | `TempBusBufferTank` |
| `grey_tank` | Grey Tank Temp | `TempBusGreyTank` |
| `fresh_tank` | Fresh Tank Temp | `TempBusFreshTank` |

All temperatures are **°F**, retained, matching the rest of the system. Published on a
0.2 °F change or every 30 s, whichever comes first.

---

## Labelling the harness

### Identify by warming (the intended workflow)

1. In Home Assistant, press **Identify Supply Manifold Temp** (or publish `supply_manifold`
   to `TempBusIdentifyCommand`).
2. Hold that sensor in your hand, or breathe on it.
3. The board snapshots every sensor's temperature, then watches for 60 s. The sensor that
   rises **≥ 4 °F** and beats the runner-up by **≥ 2 °F** wins the role and is saved to NVS.

The margin requirement is what keeps a bundle of sensors sitting in the same warm engine bay
from producing a coin-flip. If nothing clears it, the board reports `TIMEOUT` on
`TempBusIdentifyState` and the leader/runner-up rise appears in the serial log — usually the
fix is to warm it harder or for longer. **Cancel Identify** aborts a run.

### Assign directly

If the ROM address is already known — printed on the sensor, or read from a bus report:

```bash
mosquitto_pub -h 192.168.8.1 -t TempBusAssignCommand -m 'buffer_tank=28FF641E1234ABCD'
```

Or by current bus index, from the `#` column of a report:

```bash
mosquitto_pub -h 192.168.8.1 -t TempBusAssignCommand -m 'buffer_tank=2'
```

An empty value clears the role: `-m 'buffer_tank='`. Assigning an address that another role
already holds moves it — one address can only own one position.

---

## Debugging

### Serial

115200 baud. A full bus table prints every 15 s:

```
--- OneWire bus report --- GPIO13  up 00:12:34  parasite=no  found=6
  #  ROM Address        Role                 Temp F   Temp C   Reads  Errors
  0  28FF641E1234ABCD   supply_manifold        118.4     48.0     148       0
  1  28AA107C1B2C3D4E   return_manifold        104.9     40.5     148       0
  2  2891B4220E1F0055   buffer_tank            133.2     56.2     148       0
  3  28C40D9911223344   grey_tank               58.1     14.5     148       2
  4  2871E5FF00AA1234   (unassigned)            71.6     22.0     148       0
  5  283C0018AABBCCDD   fresh_tank              55.4     13.0     148       0
```

Sensor 4 is a stray on the bus with no harness position — every device answering is listed
whether or not it is bound.

With nothing on the bus it prints the wiring checklist instead of an empty table.

### Over MQTT

Every log line also goes to `TempBusLog`, so the board can be watched from a phone while it
is physically out of reach:

```bash
mosquitto_sub -h 192.168.8.1 -t 'TempBus/#' -t 'TempBus*' -v
```

| Topic | Contents |
|---|---|
| `TempBus/Raw/<ROM hex>` | °F per address every 5 s, **whether or not it has a role** |
| `TempBusScan` | JSON: every device, its role, reading, read count, error count, plus missing roles |
| `TempBusLog` | Mirror of the serial log |
| `TempBusSensorsFound` | Devices answering on the bus |
| `TempBusSensorsAssigned` | Roles bound to a device that is actually present |
| `TempBusFault` | `ON` while fewer than five roles are live |
| `TempBusParasitePower` | `ON` if the bus is running parasitic — check VDD wiring |
| `TempBusIdentifyState` | `IDLE` / `WAITING: <role>` / `DONE: <role>=<addr>` / `TIMEOUT` |

The raw feed is the useful one during a build: a sensor shows up on `TempBus/Raw/…` the
moment it is spliced in, before anyone decides what it is.

### Commands

| Topic | Payload | Effect |
|---|---|---|
| `TempBusCommand` | `SCAN` | Re-enumerate the bus now |
| `TempBusCommand` | `REPORT` | Print/publish the full bus table |
| `TempBusCommand` | `CLEAR` | Forget every role assignment |
| `TempBusCommand` | `SAVE` | Force an NVS write |
| `TempBusCommand` | `RESET` | Restart the board |
| `TempBusAssignCommand` | `<role>=<addr\|index>` | Bind a role; empty value clears it |
| `TempBusIdentifyCommand` | `<role>` or `CANCEL` | Start / abort identify-by-warming |

Retained commands replayed on reconnect are ignored for the first 3 s after an MQTT connect.

### Symptom table

| What you see | Usual cause |
|---|---|
| `found=0` | No pullup, data on the wrong pin, or no common ground |
| Count flickers between rescans | Marginal pullup, star topology, or a bad splice |
| `Search returned N repeat address(es)` | The bus search kept re-finding the same sensor — marginal signalling; the distinct count is still correct |
| `CRC failure(s) during search` | Same as above, or a run that is too long for 12-bit |
| `parasite=YES` | A sensor's VDD is unconnected |
| One sensor's `Errors` climbing | That sensor or its splice specifically — the address tells you which |
| `NOT A DS18B20` in the table | A device answered with an unexpected family code |
| A channel erroring only when hot | That DS18B20 is past its 125 °C rating — see above |

---

## Home Assistant

Discovery publishes under device **Temp Sensor Bus** (`chester_temp_bus`):

- 6 temperature sensors, each with its own availability topic — an unwired position reads
  **unavailable** instead of freezing on its last value
- 6 Identify buttons, plus Rescan Bus / Print Bus Report / Clear Assignments / Cancel Identify
- Diagnostics: Sensors Found, Sensors Assigned, Identify State, Bus Scan JSON, Last Log Line,
  WiFi RSSI, Uptime, Bus Fault, Parasite Power

---

## Flashing

First flash over USB — comment out the `upload_*` block in `platformio.ini`, hold **0**
while tapping **RST** on the S2 Mini, then:

```bash
pio run -t upload --upload-port /dev/cu.usbmodem101
```

After that, OTA at `TempSensorBus.local:3232`, password `Chester2025`:

```bash
pio run -t upload
```

---

## Moving this into production

Once the harness is labelled, the five ROM addresses are in NVS and visible in `TempBusScan`.
Copy them into whichever board ends up owning these sensors and assign by address there too
— that is the whole point of not using discovery indices.
