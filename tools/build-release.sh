#!/usr/bin/env bash
# Builds credential-free firmware for every ESP board into release/firmware/,
# named <hostname>-<version>.bin, plus SHA256SUMS.txt.
#
# These are the files published on GitHub Releases for installing from a
# browser (http://<hostname>.local/update). They are built with
# -D CHESTER_RELEASE_BUILD, so no WiFi or device password is compiled in: a
# board that is already set up keeps its stored settings, and a blank board
# starts its setup hotspot. The script refuses to finish if any value from
# secrets.ini turns up inside a binary.
#
# Usage:  tools/build-release.sh            (from anywhere in the repo)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/release/firmware"

# project folder : hostname (the hostname is what the update page expects in the file name)
BOARDS=(
  "Plumbing:Plumbing"
  "FrontThermostat:Thermostat-KitchenPassSide"
  "AltInverterCharger:AltInverterCharger"
  "TempSensorBus:TempSensorBus"
  "HX-Bypass:HX-Bypass"
  "D1Mini_CabLock_KitchenDriverSide:CabLock-KitchenDriverSide"
  "D1Mini_CabLock_KitchenUpperPassSide:CabLock-KitchenUpperPassSide"
  "D1Mini_CabLock_RearDriverSide:CabLock-RearDriverSide"
)

rm -rf "$OUT"
mkdir -p "$OUT"

for entry in "${BOARDS[@]}"; do
  project="${entry%%:*}"
  host="${entry##*:}"
  dir="$ROOT/$project"
  version="$(grep -h -m1 -E '^#define FIRMWARE_VERSION' "$dir"/src/*.cpp | sed -E 's/.*"([^"]+)".*/\1/')"
  echo "==> $project ($host $version)"
  (cd "$dir" && PLATFORMIO_BUILD_FLAGS="-D CHESTER_RELEASE_BUILD" pio run -s)
  bin="$(ls "$dir"/.pio.nosync/build/*/firmware.bin | head -1)"
  cp "$bin" "$OUT/${host}-${version}.bin"
done

# No secret may appear in a published binary. Values are read from secrets.ini
# and only ever passed to grep, never printed.
if [[ -f "$ROOT/secrets.ini" ]]; then
  leaked=0
  while IFS= read -r value; do
    [[ -z "$value" ]] && continue
    if grep -l -a -F -- "$value" "$OUT"/*.bin >/dev/null 2>&1; then
      leaked=1
    fi
  done < <(awk -F= '/^(wifi_ssid|wifi_password|ota_password|ota_upload_password)[ \t]*=/ {
              v=$0; sub(/^[^=]*=/, "", v); gsub(/^[ \t]+|[ \t]+$/, "", v); print v }' "$ROOT/secrets.ini")
  if [[ $leaked -ne 0 ]]; then
    echo "ERROR: a value from secrets.ini was found inside a release binary - deleting the output." >&2
    rm -rf "$OUT"
    exit 1
  fi
  echo "Checked: no secrets.ini value appears in any binary."
else
  echo "WARNING: no secrets.ini - skipped the leaked-credential check." >&2
fi

(cd "$OUT" && shasum -a 256 *.bin > SHA256SUMS.txt)
echo
echo "Release firmware in $OUT:"
ls -l "$OUT"
echo
echo "Note: the build folders now hold release builds. The next normal 'pio run'"
echo "rebuilds with your secrets.ini defaults automatically."
