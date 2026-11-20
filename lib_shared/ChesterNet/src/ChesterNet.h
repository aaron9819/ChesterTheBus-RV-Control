// ChesterNet - network settings, setup hotspot, and browser firmware updates
// shared by every ESP32 / ESP8266 board in the bus.
//
// What it gives a board:
//   * WiFi SSID/password, MQTT broker, and the device password are stored on the
//     board (ESP32 NVS / ESP8266 EEPROM) instead of being fixed at compile time.
//     The values in secrets.ini are only the defaults a board starts with.
//   * A web page at http://<hostname>.local/ to view and change those settings
//     and to upload new firmware (.bin) from any phone, tablet, or laptop browser.
//   * A setup hotspot "<hostname>-Setup" that appears when the board has been off
//     WiFi for PORTAL_DELAY_MS, so a new router can be configured without a cable.
//
// Board integration (see any board's setup()/loop()):
//   setup():  ChesterNet::begin(hostname, FIRMWARE_VERSION);  -- before WiFi.begin
//             then read ssid/password/broker/password from the getters below.
//   loop():   ChesterNet::loop();                              -- every pass
//   WiFi reconnect code:  only call WiFi.begin()/disconnect() when ChesterNet::wifiRetryAllowed()
//   "restart if the network is gone" code: skip while ChesterNet::portalActive()
//
// Release builds (-D CHESTER_RELEASE_BUILD) compile in NO credentials: a board
// flashed with one keeps the settings it already has stored, and a blank board
// comes up straight into the setup hotspot.
#pragma once

#include <Arduino.h>

// ESP8266 boards share the EEPROM sector with ChesterNet: the board's own data
// lives below CHESTERNET_EEPROM_OFFSET, ChesterNet's above it. Every
// EEPROM.begin() on an ESP8266 board MUST use CHESTERNET_EEPROM_SIZE - commit()
// rewrites the whole sector with only the begun size, so a smaller size would
// erase the stored network settings.
#define CHESTERNET_EEPROM_SIZE   512
#define CHESTERNET_EEPROM_OFFSET 256

namespace ChesterNet {

// Call once, early in setup(), before any WiFi.begin().
// defaultStaticIp/defaultGateway: only for a board that needs a fixed address
// (HX-Bypass). They are offered as editable fields on the settings page; leaving
// them blank there means DHCP. Ignored in release builds.
void begin(const char* hostname, const char* firmwareVersion,
           const char* defaultStaticIp = nullptr, const char* defaultGateway = nullptr);

// Call every loop() pass. Serves the web page and runs the setup hotspot.
void loop();

// Stored settings. Pointers stay valid for the life of the program, so they can
// be handed straight to WiFi.begin() / PubSubClient::setServer().
const char* wifiSsid();
const char* wifiPassword();
const char* mqttHost();
uint16_t    mqttPort();
const char* devicePassword();   // ArduinoOTA password + web page password

// Applies the stored static IP (if any) - call right before WiFi.begin().
// Does nothing when the board uses DHCP.
void applyStaticIp();

// True while the setup hotspot is up.
bool portalActive();

// False while the setup hotspot is up. ChesterNet then retries the saved
// network itself, once a minute: every connection attempt makes the radio scan
// all channels, and a hotspot whose radio keeps leaving its channel is invisible.
bool wifiRetryAllowed();

// Called when a browser firmware upload starts / finishes. Use them to put
// outputs in a safe state (the upload blocks loop() until it is done) and, on
// the ESP32-S2 Plumbing board, to detach the task watchdog. On success the
// board restarts right after onUpdateEnd(true).
void onUpdateStart(void (*fn)());
void onUpdateEnd(void (*fn)(bool ok));

}  // namespace ChesterNet
