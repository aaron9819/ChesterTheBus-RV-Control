# AltInverterCharger

ESP32-S2 Mini board that controls the inverter's power button via a momentary
relay pulse and reports battery voltage to Home Assistant over MQTT.
HA handles the automations; this board does three things:

1. **Inverter power toggle** — closes a relay for 500ms (like pressing the
   inverter's power button) when commanded over MQTT.
2. **Battery voltage** — reads the inverter's supply battery voltage on an
   ADC pin and publishes it (calibrated) to MQTT.
3. **Temperature** — DS18B20 sensor, published to MQTT/HA.

## Hardware

| Function | Pin | Notes |
|---|---|---|
| Relay control | GPIO16 | Plain GPIO, no boot/strapping function on the S2 |
| Battery voltage ADC | GPIO3 | **ADC1_CH2 — must be ADC1!** See below |
| Status LED | GPIO15 | Onboard LED, lights while relay is pulsing |
| DS18B20 data | GPIO5 | Needs 4.7k pullup to 3.3V (VCC → 3.3V, GND → GND) |

### Why GPIO3 for the ADC (the WiFi conflict)

On the ESP32-S2, **ADC2 (GPIO11–GPIO20) cannot be used while WiFi is
active** — the WiFi driver takes ownership of ADC2 and reads will fail or
return garbage. ADC1 covers **GPIO1–GPIO10** and works fine alongside WiFi.
GPIO3 = ADC1_CH2, so it's safe. If you ever move the sense wire, keep it on
GPIO1–GPIO10.

### Relay wiring

- Relay signal → GPIO16, relay VCC/GND per your module.
- Firmware is configured **active-LOW** (`RELAY_ACTIVE_HIGH` is `false` in
  `src/main.cpp`), which suits most opto-isolated relay boards. Flip that
  constant to `true` for an active-HIGH module.
- The relay contacts wire across the inverter's momentary power button.
- The pin is driven to its idle state first thing in `setup()`, and OTA
  updates force the relay open so it can never stick closed mid-update.

### Voltage divider

The existing divider reads **~500mV at 12.5V battery**, i.e. a ~25:1 ratio.
The firmware uses `analogReadMilliVolts()` (factory eFuse-calibrated) with
11dB attenuation (~0–2500mV usable), averages 32 samples, then multiplies by
a stored scale factor (default **25.0**).

## Calibration

1. Measure the actual battery voltage with a multimeter (e.g. 12.64V).
2. Publish that value to `AltInverterCalibrateCommand`:
   ```
   mosquitto_pub -h 192.168.8.1 -t AltInverterCalibrateCommand -m "12.64"
   ```
3. The board takes a fresh averaged ADC reading, computes
   `scale = actual_volts / raw_volts`, saves it to EEPROM (survives reboot
   and reflash), and immediately re-publishes the corrected voltage.

The current scale and raw mV are visible in HA as diagnostic entities
(`Voltage Cal Scale`, `Battery ADC Raw`) to sanity-check the math.

## MQTT Topics

| Topic | Direction | Payload |
|---|---|---|
| `AltInverterStatus` | publish (retained, LWT) | `ONLINE` / `OFFLINE` |
| `AltInverterPowerCommand` | subscribe | `PRESS` or `TOGGLE` — pulse relay 500ms |
| `AltInverterRelayState` | publish (retained) | `ON` during pulse, `OFF` after |
| `AltInverterBatteryVoltage` | publish (retained) | volts, e.g. `12.53` |
| `AltInverterBatteryRawMV` | publish (retained) | raw averaged mV at the pin |
| `AltInverterCalibrateCommand` | subscribe | actual battery volts, accepted 5–40V |
| `AltInverterCalScale` | publish (retained) | current calibration scale |
| `AltInverterRSSI` | publish (retained) | WiFi signal dBm |
| `AltInverterTemperature` | publish (retained) | DS18B20 °C, e.g. `23.4` |

Voltage publishes on a ≥0.05V change (checked every 1s) and at least every
10s regardless. Temperature reads every 10s (non-blocking 750ms conversion)
and publishes on a ≥0.2°C change or at least every 60s. If the DS18B20 is
missing or unplugged, the board keeps re-probing every 30s and simply stops
publishing until it returns — boot order doesn't matter.

### Safety behavior

- Presses are rate-limited: ignored while a pulse is active and for 2s after
  the last press (protects against duplicate/bounced commands).
- Commands arriving within 3s of an MQTT (re)connect are ignored, so a stale
  or retained command can never toggle the inverter on reboot.

## Home Assistant Entities

Device: **Alt Inverter Charger** (`chester_alt_inverter`), auto-discovered.

| Entity | Type | Notes |
|---|---|---|
| Inverter Power | button | One press = one 500ms relay pulse |
| Inverter Battery Voltage | sensor | V, device_class voltage |
| Inverter Temperature | sensor | °C, device_class temperature |
| Inverter Relay | binary_sensor | diagnostic, shows the pulse |
| Battery ADC Raw | sensor | diagnostic, mV |
| Voltage Cal Scale | sensor | diagnostic |
| WiFi RSSI | sensor | diagnostic |

All entities track availability via `AltInverterStatus` (LWT), so they show
*unavailable* in HA if the board drops off the network.

## Flashing

`platformio.ini` is currently set up for **OTA** upload to
`AltInverterCharger.local` (password `Chester2025`, port 3232), so a plain
`pio run -t upload` goes over the air.

A board that has never been flashed has no OTA listener, so its first upload
must be over USB — comment out the `upload_protocol` / `upload_port` /
`upload_flags` block at the bottom of `platformio.ini`, or override the port
on the command line:

```bash
pio run -t upload --upload-port /dev/cu.usbmodem101
```

```bash
pio device monitor         # 115200 baud
```
