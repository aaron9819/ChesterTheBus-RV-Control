# Flashing and OTA Updates

Every ESP board can be updated three ways:

1. **From a phone, tablet, or any browser** — the board's own update page. No software needed.
2. **From a computer with PlatformIO** — `pio run -t upload` over WiFi (ArduinoOTA).
3. **Over USB** — needed only for a blank board.

All of them use the board's **device password**. It starts as `ota_password` from
`secrets.ini` and can be changed on the board's settings page. The GIGA R1 boards are not
covered by any of this and still update over USB only.

## Updating from a phone or tablet

1. Download the board's file from the repo's
   [Releases page](https://github.com/aaron9819/ChesterTheBus-RV-Control/releases/latest).
   Each file is named after the board, for example `Plumbing-v3.6.0.bin`.
2. On the bus WiFi, open `http://<hostname>.local/` (hostnames in the table below), or
   the board's IP address if `.local` names do not resolve on your device. Some Android
   phones need the IP.
3. Log in with user **`admin`** and the device password.
4. Tap **Update firmware**, choose the `.bin` file, and tap **Upload and install**. The
   upload takes 10–60 s, then the board restarts on the new firmware.

The page refuses a file whose name does not contain the board's hostname, which stops the
thermostat's firmware from going onto a cabinet lock. If you renamed the file on purpose,
tick the box and try again. Stored settings (WiFi, MQTT, password) and each board's own
state are kept across an update.

## Settings page and changing networks

The same page (`http://<hostname>.local/`) shows the WiFi connection, IP, MQTT broker and
firmware version. It also lets you change:

- the WiFi network name and password
- the MQTT broker address and port
- the device password
- a static IP and gateway (HX-Bypass only; leave the IP blank for DHCP)

Blank password fields keep the current value. Saving restarts the board.

**Setup hotspot.** If a board cannot reach its WiFi for 30 seconds (new router, changed
password), it opens its own hotspot named **`<hostname>-Setup`**, for example
`Plumbing-Setup`. The hotspot password is the device password. Join it from a phone and
the settings page opens on its own; if it doesn't, browse to `http://192.168.4.1/`. Enter
the new WiFi details and save. The board keeps controlling its hardware the whole time, and
the hotspot closes as soon as it is back on WiFi.

While the hotspot is up, the boards' "network gone too long, restart" timers are paused. If
nobody joins it for 15 minutes the board restarts once and tries again.

**Reset network settings** (on the settings page) puts back the settings built into the
firmware. A Releases download has none built in, so on those the board goes straight to
its setup hotspot.

**Where the settings live.** ESP32 boards keep them in NVS (namespace `chesternet`). ESP8266
boards keep them in the top half of the EEPROM sector (bytes 256–511). The code is in
`lib_shared/ChesterNet`. On first boot a board stores the defaults from `secrets.ini`.
After that the stored values win, with one exception: a board whose settings were never
edited on the web page follows `secrets.ini` again if the defaults built into its
firmware change.

## Publishing release firmware

```bash
tools/build-release.sh
```

This builds every ESP board with `-D CHESTER_RELEASE_BUILD`, so no WiFi or device
password is compiled in. It writes `release/firmware/<hostname>-<version>.bin` plus
`SHA256SUMS.txt`, and refuses to finish if any value from `secrets.ini` turns up inside a
binary. Upload the folder's contents as a GitHub release:

```bash
gh release create firmware-YYYY-MM-DD release/firmware/* --title "Firmware YYYY-MM-DD" --notes "..."
```

Bump a board's `FIRMWARE_VERSION` before publishing a changed build, so the file name and
the settings page both show which version is installed.

## OTA target table

| Project | Board | OTA hostname | Port | Configured in `platformio.ini`? |
|---|---|---|---|---|
| `Plumbing` | ESP32-S2 Mini | `Plumbing.local` | 3232 | yes |
| `FrontThermostat` | ESP32-S2 Mini | `Thermostat-KitchenPassSide.local` | 3232 | yes |
| `AltInverterCharger` | ESP32-S2 Mini | `AltInverterCharger.local` | 3232 | yes |
| `HX-Bypass` | D1 Mini | `HX-Bypass.local` | 8266 | **no — commented out** |
| `TempSensorBus` | ESP32-S2 Mini | `TempSensorBus.local` | 3232 | yes |
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
- **FrontThermostat** — shows `UPDATING` on the OLED and saves pending state before restarting.
- **HX-Bypass** — no special handling; it simply serves the update.

A browser upload runs the same handling through `ChesterNet::onUpdateStart()` /
`onUpdateEnd()`. It blocks `loop()` until the upload finishes, just like ArduinoOTA does.

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
| Cabinet locks | Lock state restored from EEPROM; servo driven back to it at boot |

Reflashing HX-Bypass discards a custom setpoint, which is worth knowing before an update
while the vehicle is in use. Network settings are kept on every board.

## Troubleshooting

**`.local` hostname does not resolve.** mDNS on the GL-iNet router is unreliable when
clients roam. Upload to the IP instead:

```bash
pio run -t upload --upload-port 192.168.8.140
```

Reserve a DHCP lease per board on the router and give it a recognisable name — that makes
both OTA and debugging far easier. HX-Bypass is the only board that sets its own static IP
(default `192.168.8.140`, editable or clearable for DHCP on its settings page).

**Authentication failed.** `secrets.ini` has two OTA keys: `ota_password` is compiled into
the firmware, and `ota_upload_password` is what `pio run -t upload` sends. The upload has to
match the password the board is *currently* running. When rotating the password, set
`ota_password` to the new one, flash every board while `ota_upload_password` still holds the
old one, then change `ota_upload_password` to match.

**Upload starts then stalls.** Usually WiFi signal. The ESP8266 boards in particular will
drop mid-transfer on a weak link. Move closer to the router or fall back to USB.

**Board is unreachable and unresponsive.** The Plumbing board self-restarts after 8
consecutive network failures, AltInverterCharger/TempSensorBus after 120 s and
FrontThermostat after 15 min without WiFi or MQTT, so give an apparently dead board two
minutes before pulling it. If its WiFi is gone, look for its `<hostname>-Setup` hotspot
instead.

**Forgot the device password.** Only a USB reflash recovers it, because stored settings
survive an ordinary upload. Connect the board over USB and run `pio run -t erase`, then
`pio run -t upload` with the USB upload settings. The board comes back with the
`secrets.ini` defaults. Erasing also clears the board's own saved state (relay states,
calibration, lock position).
