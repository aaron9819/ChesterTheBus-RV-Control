# Flashing and OTA Updates

Every board in the system runs `ArduinoOTA` and can be updated over WiFi. All boards share
the OTA password `Chester2025`.

## OTA target table

| Project | Board | OTA hostname | Port | Configured in `platformio.ini`? |
|---|---|---|---|---|
| `Plumbing` | ESP32-S2 Mini | `Plumbing.local` | 3232 | yes |
| `FrontThermostat` | ESP32-S2 Mini | `Thermostat-KitchenPassSide.local` | 3232 | yes |
| `AltInverterCharger` | ESP32-S2 Mini | `AltInverterCharger.local` | 3232 | yes |
| `HX-Bypass` | D1 Mini | `HX-Bypass.local` | 8266 | **no — commented out** |
| `D1Mini_CabLock_KitchenDriverSide` | D1 Mini Lite | `CabLock-KitchenDriverSide.local` | 8266 | yes |
| `D1Mini_CabLock_KitchenUpperPassSide` | D1 Mini Lite | `CabLock-KitchenUpperPassSide.local` | 8266 | yes |
| `D1Mini_CabLock_RearDriverSide` | D1 Mini Lite | `CabLock-RearDriverSide.local` | 8266 | yes |

ESP8266 boards use OTA port 8266; ESP32 boards use 3232. Both are the platform defaults.

The firmware on every board enables OTA at runtime regardless of what `platformio.ini` says
— the `upload_protocol` lines only control which transport `pio run -t upload` uses.
HX-Bypass ships with those lines commented, so its default upload path is USB even though
the running firmware is listening for OTA.

## Uploading

For a project whose `platformio.ini` already has the OTA block active, an upload is just:

```bash
pio run -t upload
```

To force USB on such a project, comment out the three `upload_*` blocks at the bottom of
its `platformio.ini` first, or override on the command line:

```bash
pio run -t upload --upload-port /dev/cu.usbmodem101
```

For HX-Bypass, uncomment its `upload_protocol` / `upload_port` / `upload_flags` block to
switch it to OTA.

## First flash must be USB

A board with no firmware has no OTA listener. Any newly built or reflashed-from-blank board
gets its first upload over USB, after which OTA works.

On the Lolin S2 Mini, entering the bootloader means holding **0** while tapping **RST**,
then releasing 0. The board reappears as a new serial device.

## Serial monitor

```bash
pio device monitor
```

All boards are 115200 baud (set via `monitor_speed`). Boot output identifies the firmware
version, the discovered sensors, and the restored relay states, which is the fastest way to
confirm a flash took.

## What each board does during an update

- **Plumbing** — sets `otaInProgress`, which suspends the entire MQTT/sensor/automation
  section of `loop()`, and detaches the 60 s task watchdog so a slow upload cannot trigger
  a panic reset. The watchdog is reattached on completion *and* on the error path.
- **AltInverterCharger** — forces the relay open and clears `relayPulsing` before the
  update starts, so an update landing mid-pulse cannot leave the inverter's power button
  held down. `loop()` returns early while `otaInProgress` is set.
- **Cabinet locks** — detach the servo if attached, so it is not driven during the update.
- **FrontThermostat, HX-Bypass** — no special handling; they simply serve the update.

## Relay and valve state across a reflash

Only the Plumbing board restores its outputs after a restart. It reads every relay state
back out of NVS in `setup()` and reapplies it to the hardware before MQTT even connects, so
an OTA update does not disturb an active heating loop.

Everything else comes up in a defined default state:

| Board | State after reboot |
|---|---|
| Plumbing | All relays restored from NVS; setpoints and mode restored |
| FrontThermostat | Mode and setpoint restored from EEPROM; fan off, engine loop assumed closed |
| AltInverterCharger | Voltage calibration restored from EEPROM; relay open |
| HX-Bypass | **Nothing persisted** — auto mode on, 160 °F setpoint, valve closed |
| Cabinet locks | **Nothing persisted** — servo driven to UNLOCKED at boot |

Reflashing a cabinet lock therefore unlocks it. Reflashing HX-Bypass discards a custom
setpoint. Both are worth knowing before an update while the vehicle is in use.

## Troubleshooting

**`.local` hostname does not resolve.** mDNS on the GL-iNet router is unreliable when
clients roam. Upload to the IP instead:

```bash
pio run -t upload --upload-port 192.168.8.140
```

Reserve a DHCP lease per board on the router and give it a recognisable name — that makes
both OTA and debugging far easier. HX-Bypass is the only board that sets its own static IP
in firmware (`192.168.8.140`).

**Authentication failed.** The password is `Chester2025` on every board, set both by
`ArduinoOTA.setPassword()` in firmware and by `--auth=` in `platformio.ini`. They have to
match; if you change one, change both.

**Upload starts then stalls.** Usually WiFi signal. The ESP8266 boards in particular will
drop mid-transfer on a weak link. Move closer to the router or fall back to USB.

**Board is unreachable and unresponsive.** The Plumbing board self-restarts after 8
consecutive network failures and FrontThermostat/AltInverterCharger restart after 120 s
without WiFi or MQTT, so give an apparently dead board two minutes before pulling it.
