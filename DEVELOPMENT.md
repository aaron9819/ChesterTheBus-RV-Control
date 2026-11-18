# Development Guide

Toolchain, repository layout, and the conventions the firmware follows.

## Prerequisites

- **PlatformIO** — either the VS Code extension or the CLI (`pip install platformio`).
- Board platforms are pulled automatically on first build:
  `espressif32` for the ESP32-S2 boards, `espressif8266` for the D1 Minis.

Each project is a self-contained PlatformIO project with its own `platformio.ini`. There is
no shared library directory and no monorepo build — open a project folder, or run `pio` from
inside it.

## Repository layout

```
ChesterTheBus-RV-Control/
├── Plumbing/                             ESP32-S2 · main plumbing controller
│   └── src/main_esp32s2.cpp
├── FrontThermostat/                      ESP32-S2 · front zone climate
├── AltInverterCharger/                   ESP32-S2 · inverter button + battery
├── HX-Bypass/                            D1 Mini  · heat exchanger bypass valve
├── D1Mini_CabLock_KitchenDriverSide/     D1 Mini Lite · cabinet lock
├── D1Mini_CabLock_KitchenUpperPassSide/  D1 Mini Lite · cabinet lock
├── D1Mini_CabLock_RearDriverSide/        D1 Mini Lite · cabinet lock
├── build-all-projects.sh                 Build every project, warm the caches
├── clear-cache.sh                        Clear PlatformIO / IntelliSense caches
└── setup-env.sh                          Local env vars (gitignored)
```

The Plumbing project's source file is `src/main_esp32s2.cpp`, not `src/main.cpp`. A
`src/main.cpp.bak` sits alongside it — PlatformIO ignores `.bak`, so only the one file
compiles.

## Building

```bash
pio run
```

```bash
pio run -t upload
```

```bash
pio device monitor
```

All boards are 115200 baud. Upload targets and the USB-vs-OTA question are covered in
**[OTA_UPDATE_GUIDE.md](OTA_UPDATE_GUIDE.md)**.

To build everything in one pass — useful after a toolchain upgrade, or to warm the caches
before a long session:

```bash
./build-all-projects.sh
```

## Caching

First builds are slow because PlatformIO downloads platforms, toolchains, and libraries.
After that, rebuilds are seconds.

| Cache | Location | Cleared by |
|---|---|---|
| Build artifacts and project libraries | `<project>/.pio.nosync/` | `./clear-cache.sh` |
| Shared packages and libraries | `~/.platformio/` | `./clear-cache.sh` (optional prompt) |
| VS Code IntelliSense | `.vscode/.cache/` | `./clear-cache.sh` |

All three are gitignored. Clear them when a build fails in a way that makes no sense —
stale object files after switching platform versions are the usual cause.

**Why `.pio.nosync` and not `.pio`.** Every `platformio.ini` sets
`[platformio] workspace_dir = .pio.nosync`. The repo lives on an iCloud-synced Desktop, and
iCloud was dropping conflict copies (`ESP32PWM 2.cpp`, …) and cloud-only stubs into the
build and library folders, which broke builds. iCloud skips anything ending in `.nosync`.
Keep that line in any new project.

## Configuration conventions

**Credentials live in `secrets.ini`.** One gitignored file at the repo root holds the SSID,
WiFi password, and OTA passwords for every project (template: `secrets.example.ini`). Each
`platformio.ini` loads it with `extra_configs = ../secrets.ini`, compiles the values in as
`SECRET_WIFI_SSID`, `SECRET_WIFI_PASSWORD` and `SECRET_OTA_PASSWORD`, and uses
`ota_upload_password` for `--auth`. A missing file stops the build with an `#error`. **This
repository is public** — never put a credential back into a tracked file. Passwords that
were committed before 2026-11-17 are still in the git history, so they must never be reused.
New projects: copy the `extra_configs` line and the three `SECRET_*` build flags.

**Build flags.** Every `platformio.ini` defines `BUILD_TIMESTAMP`, `BUILD_DATE`, and
`BUILD_TIME`. **No current firmware references them** — they are compiled in and unused.
Firmware identity comes from the `FIRMWARE_VERSION` string each sketch defines, printed in
the boot banner and, where applicable, published as `sw_version` in the Home Assistant
discovery payload.

**Firmware versions** are per board and bumped by hand:

| Project | Version |
|---|---|
| FrontThermostat | `v4.1.0` |
| Plumbing | `v3.1.0` |
| Cabinet locks (×3) | `v3.0.0` |
| AltInverterCharger | `v1.1.0` |
| HX-Bypass | `v1.0.0` |

## Firmware conventions

These patterns recur across the codebase and are worth matching in new work.

**Separate command and state topics.** A board subscribes to `…Command` and publishes to
`…Status`. Never subscribe to a topic the board itself publishes — a retained state message
would be replayed as a command on every reconnect.

**Retain state, not telemetry.** Valve positions, modes, setpoints, and lock states publish
with `retain=true`. Temperature and flow readings do not.

**Publish state from the setter.** Each `control*()` / `lock*()` function writes the GPIO,
updates the in-memory flag, persists if applicable, and publishes — in that order, in one
place. State never diverges from hardware because there is only one path to both.

**Validate before applying.** Setpoint handlers range-check and silently discard
out-of-range values. Be aware this leaves the sender's UI briefly stale until the next
periodic status publish corrects it.

**Detach servos when idle.** Attach, write, delay for travel, detach. Holding position
draws current and wears the gearbox.

**Debounce flash writes.** The Plumbing board marks NVS dirty and flushes 2 s later from
`loop()`, so a burst of valve changes costs one write.

## Where to look

| Question | Document |
|---|---|
| Which pin does what | [BOARD_ARCHITECTURE.md](BOARD_ARCHITECTURE.md) |
| What topic carries what | [MQTT_TOPICS.md](MQTT_TOPICS.md) |
| What entities exist in HA | [HOME_ASSISTANT_DISCOVERY.md](HOME_ASSISTANT_DISCOVERY.md) |
| How to flash a board | [OTA_UPDATE_GUIDE.md](OTA_UPDATE_GUIDE.md) |
| How the plumbing automation behaves | [Plumbing/README.md](Plumbing/README.md) |
| How the thermostat's PI loop works | [FrontThermostat/README.md](FrontThermostat/README.md) |
| Cabinet lock behaviour | [CABINET_LOCKS.md](CABINET_LOCKS.md) |
