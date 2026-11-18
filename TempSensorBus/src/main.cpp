#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <stdarg.h>

// ============================================
// CONFIGURATION
// ============================================
#define DEVICE_NAME "TempSensorBus"
#define FIRMWARE_VERSION "v1.1.1"

// WiFi and OTA credentials come from secrets.ini at the repo root (gitignored),
// passed in as build flags by platformio.ini. Template: secrets.example.ini.
#if !defined(SECRET_WIFI_SSID) || !defined(SECRET_WIFI_PASSWORD) || !defined(SECRET_OTA_PASSWORD)
#error "WiFi/OTA credentials missing - copy secrets.example.ini to secrets.ini at the repo root"
#endif

// WiFi Configuration
const char* ssid = SECRET_WIFI_SSID;
const char* password = SECRET_WIFI_PASSWORD;

// MQTT Configuration
const char* mqtt_server = "192.168.8.1";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* mqtt_client_id = "TempSensorBus";

// OTA Configuration
const char* otaHostname = DEVICE_NAME;
const char* otaPassword = SECRET_OTA_PASSWORD;

// Hardware Configuration
// GPIO13 matches the Plumbing board's OneWire bus so harness wiring is identical.
// The bus needs a single 4.7k pullup from data to 3.3V, at the controller end.
#define ONEWIRE_PIN 13
#define STATUS_LED_PIN 15   // Onboard LED on Lolin S2 Mini (active HIGH)

#define ROLE_COUNT 5        // Named sensor positions in the harness
#define MAX_BUS_SENSORS 12  // Room to spot strays / doubled-up sensors during a build

// ============================================
// MQTT TOPICS
// ============================================
// State topics are PascalCase, matching the rest of the system.
// Debug namespaces use slashes so they group in MQTT Explorer.
const char* topic_status         = "TempBusStatus";          // ONLINE/OFFLINE (retained, LWT)
const char* topic_command        = "TempBusCommand";         // SCAN | REPORT | CLEAR | SAVE | RESET
const char* topic_assign_cmd     = "TempBusAssignCommand";   // "<role>=<ROM hex>" or "<role>=<bus index>"
const char* topic_identify_cmd   = "TempBusIdentifyCommand"; // "<role>" to start, "CANCEL" to abort
const char* topic_identify_state = "TempBusIdentifyState";   // IDLE | WAITING… | DONE… | TIMEOUT
const char* topic_found          = "TempBusSensorsFound";    // Count of devices on the bus
const char* topic_assigned       = "TempBusSensorsAssigned"; // Count of roles bound to a live sensor
const char* topic_fault          = "TempBusFault";           // ON when a role is missing/unassigned
const char* topic_parasite       = "TempBusParasitePower";   // ON/OFF - parasite-powered bus detected
const char* topic_scan           = "TempBusScan";            // JSON array of everything on the bus
const char* topic_log            = "TempBusLog";             // Mirror of the serial log
const char* topic_rssi           = "TempBusRSSI";
const char* topic_uptime         = "TempBusUptime";

const char* topic_raw_prefix      = "TempBus/Raw/";      // + ROM hex -> °F, every read cycle
const char* topic_presence_prefix = "TempBus/Presence/"; // + role key -> ONLINE/OFFLINE

// ============================================
// TIMING
// ============================================
const unsigned long READ_INTERVAL         = 5000;   // Start a conversion every 5s
const unsigned long CONVERSION_MS         = 800;    // 12-bit conversion time + margin
const unsigned long BUS_RESCAN_INTERVAL   = 30000;  // Re-enumerate the bus while wiring
const unsigned long REPORT_INTERVAL       = 15000;  // Serial bus table
const unsigned long SCAN_PUBLISH_INTERVAL = 30000;  // TempBusScan JSON
const unsigned long PUBLISH_INTERVAL      = 30000;  // Force a temp publish at least this often
const float         PUBLISH_DELTA_F       = 0.2;    // ...or sooner on this much change
const unsigned long STATUS_INTERVAL       = 60000;
const unsigned long MQTT_CONNECT_GRACE_MS = 3000;   // Ignore retained commands right after connect

// Identify-by-warming
const unsigned long IDENTIFY_WINDOW_MS = 60000; // How long to watch after the button is pressed
const float IDENTIFY_DELTA_F  = 4.0;            // Rise that counts as "you touched this one"
const float IDENTIFY_MARGIN_F = 2.0;            // Winner must beat runner-up by this much

// Recovery / Watchdog
const unsigned long WIFI_RECOVERY_TIMEOUT = 120000;
const unsigned long MQTT_RECOVERY_TIMEOUT = 120000;

// DS18B20 plausibility window (-55…+125 °C is the part's rated range)
const float TEMP_MIN_F = -67.0;
const float TEMP_MAX_F = 257.0;

// ============================================
// ROLE TABLE
// ============================================
// key         -> MQTT/identify command payload
// nvsKey      -> NVS key holding this position's ROM address (<=15 chars)
// legacyIndex -> old "a<n>" NVS key this position used, for one-time migration
// name        -> Home Assistant entity name
// stateTopic  -> retained °F topic
struct RoleDef {
  const char* key;
  const char* nvsKey;
  int         legacyIndex;
  const char* name;
  const char* stateTopic;
  const char* icon;
};

// nvsKey is tied to the position, not to its index in this table, so a position
// can be added or dropped without shuffling everyone else's stored ROM address.
// legacyIndex points back at the original index-keyed layout (supply 0,
// return 1, buffer 2, grey 3, boiler_exhaust 4, fresh 5); loadAssignments()
// migrates from it once, then it is never read again.
const RoleDef ROLES[ROLE_COUNT] = {
  { "supply_manifold", "a_supply", 0, "Supply Manifold Temp", "TempBusSupplyManifold", "mdi:pipe"         },
  { "return_manifold", "a_return", 1, "Return Manifold Temp", "TempBusReturnManifold", "mdi:pipe"         },
  { "buffer_tank",     "a_buffer", 2, "Buffer Tank Temp",     "TempBusBufferTank",     "mdi:storage-tank" },
  { "grey_tank",       "a_grey",   3, "Grey Tank Temp",       "TempBusGreyTank",       "mdi:storage-tank" },
  { "fresh_tank",      "a_fresh",  5, "Fresh Tank Temp",      "TempBusFreshTank",      "mdi:storage-tank" }
};

// ============================================
// STATE
// ============================================
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Preferences preferences;

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature tempSensors(&oneWire);

// Everything currently answering on the bus, in search order
struct BusSensor {
  DeviceAddress addr;
  char hex[17];
  bool  isDS18B20;   // Family code recognised as a temperature sensor
  bool  valid;       // Last read produced a plausible number
  float tempF;
  float baselineF;   // Snapshot taken when identify mode starts
  uint32_t reads;
  uint32_t errors;
  int  role;         // Index into ROLES, or -1 if unassigned
};

BusSensor bus[MAX_BUS_SENSORS];
int busCount = 0;
bool parasitePower = false;

// One entry per named harness position
struct RoleState {
  DeviceAddress addr;
  bool  assigned;      // An address is stored for this role
  bool  present;       // ...and that address answered on the last scan
  bool  valid;         // ...and produced a plausible reading
  float tempF;
  float lastPublished;
  unsigned long lastPublish;
  bool  lastPresencePublished;
  bool  presencePublishedOnce;
};

RoleState roles[ROLE_COUNT];

bool conversionPending = false;
unsigned long conversionStart = 0;
unsigned long lastReadStart = 0;
unsigned long lastRescan = 0;
unsigned long lastReport = 0;
unsigned long lastScanPublish = 0;
unsigned long lastStatusUpdate = 0;
unsigned long mqttConnectedAt = 0;
unsigned long wifiLostSince = 0;
unsigned long mqttLostSince = 0;
bool otaInitialized = false;
bool otaInProgress = false;
bool lastFaultPublished = true;
bool faultPublishedOnce = false;

// Identify mode
bool identifyActive = false;
int  identifyRole = -1;
unsigned long identifyStart = 0;
unsigned long lastIdentifyProgress = 0;

// ============================================
// FORWARD DECLARATIONS
// ============================================
void logf(const char* fmt, ...);
void addrToHex(const DeviceAddress addr, char* out);
bool hexToAddr(const String& hex, DeviceAddress out);
void rescanBus();
void mapRolesToBus();
void startConversion();
void collectConversion();
void publishRoleTemp(int r, bool force);
void publishPresence(int r, bool force);
void publishCounts();
void publishScanJson();
void printBusReport();
void assignRole(int r, const DeviceAddress addr, bool save);
void clearRole(int r, bool save);
void startIdentify(int r);
void handleIdentify();
void loadAssignments();
void saveAssignments();
int  roleIndexFromKey(const String& key);
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool reconnectMQTT();
void publishHomeAssistantDiscovery();
void sendStatus();

// ============================================
// LOGGING
// ============================================
// Every line goes to both the serial monitor and TempBusLog, so the board can be
// watched from a phone while it is physically out of reach.
void logf(const char* fmt, ...) {
  char buf[200];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  Serial.println(buf);
  if (mqttClient.connected()) {
    mqttClient.publish(topic_log, buf);
  }
}

// ============================================
// ADDRESS HELPERS
// ============================================
void addrToHex(const DeviceAddress addr, char* out) {
  for (uint8_t i = 0; i < 8; i++) {
    sprintf(out + (i * 2), "%02X", addr[i]);
  }
  out[16] = '\0';
}

// Accepts "28FF641E1234ABCD", with or without a 0x prefix, any case.
bool hexToAddr(const String& hex, DeviceAddress out) {
  String s = hex;
  s.trim();
  s.toUpperCase();
  if (s.startsWith("0X")) s = s.substring(2);
  s.replace(":", "");
  s.replace("-", "");
  if (s.length() != 16) return false;

  for (uint8_t i = 0; i < 8; i++) {
    char hi = s.charAt(i * 2);
    char lo = s.charAt(i * 2 + 1);
    if (!isHexadecimalDigit(hi) || !isHexadecimalDigit(lo)) return false;
    out[i] = (uint8_t)strtoul(s.substring(i * 2, i * 2 + 2).c_str(), NULL, 16);
  }
  return true;
}

bool addrEqual(const DeviceAddress a, const DeviceAddress b) {
  return memcmp(a, b, 8) == 0;
}

bool addrIsZero(const DeviceAddress a) {
  for (uint8_t i = 0; i < 8; i++) if (a[i] != 0) return false;
  return true;
}

// ============================================
// BUS ENUMERATION
// ============================================
// Searched directly rather than through DallasTemperature so CRC failures and
// non-temperature devices are visible instead of being silently dropped.
void rescanBus() {
  // Carry per-address counters and baselines across a rescan
  BusSensor previous[MAX_BUS_SENSORS];
  int previousCount = busCount;
  memcpy(previous, bus, sizeof(bus));

  busCount = 0;
  memset(bus, 0, sizeof(bus));

  // Let DallasTemperature do its own self-contained enumeration first - it
  // refreshes the device count and the parasite-power flag.
  tempSensors.begin();
  parasitePower = tempSensors.isParasitePowerMode();

  DeviceAddress addr;
  int crcFailures = 0;
  int duplicates = 0;
  int iterations = 0;

  // NOTHING may touch the bus between search() calls. The search walks a binary
  // tree using state held inside the OneWire object, and any call that issues
  // reset_search() sends it back to the top - it then re-finds the same devices
  // forever. DallasTemperature::setResolution() does exactly that internally
  // whenever more than one device is present, so resolution is configured in a
  // second pass below rather than inside this loop.
  oneWire.reset_search();
  while (oneWire.search(addr)) {
    // A search that cannot terminate would otherwise fill the table with
    // phantom copies of a real sensor and look like a device-count explosion.
    if (++iterations > MAX_BUS_SENSORS * 4) {
      logf("WARN  Search did not terminate after %d hits - aborting scan", iterations);
      break;
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
      crcFailures++;
      continue;
    }

    // Reject an address the search has already handed us this pass
    bool duplicate = false;
    for (int i = 0; i < busCount; i++) {
      if (addrEqual(bus[i].addr, addr)) { duplicate = true; break; }
    }
    if (duplicate) {
      duplicates++;
      continue;
    }

    if (busCount >= MAX_BUS_SENSORS) {
      logf("WARN  More than %d distinct devices on the bus - extras ignored", MAX_BUS_SENSORS);
      break;
    }

    BusSensor* s = &bus[busCount];
    memcpy(s->addr, addr, 8);
    addrToHex(s->addr, s->hex);
    // 0x28 DS18B20, 0x22 DS1822, 0x10 DS18S20
    s->isDS18B20 = (addr[0] == 0x28 || addr[0] == 0x22 || addr[0] == 0x10);
    s->role = -1;
    s->valid = false;
    s->tempF = -999.0;
    s->baselineF = -999.0;   // Never let a freshly seen sensor win an identify

    // Restore counters if we have seen this address before
    for (int p = 0; p < previousCount; p++) {
      if (addrEqual(previous[p].addr, s->addr)) {
        s->reads     = previous[p].reads;
        s->errors    = previous[p].errors;
        s->tempF     = previous[p].tempF;
        s->valid     = previous[p].valid;
        s->baselineF = previous[p].baselineF;
        break;
      }
    }
    busCount++;
  }

  // Second pass - safe now that the search has finished. The `true` skips
  // DallasTemperature's global bit-resolution recalculation, which is the part
  // that calls reset_search().
  for (int i = 0; i < busCount; i++) {
    if (bus[i].isDS18B20) {
      tempSensors.setResolution(bus[i].addr, 12, true);
    } else {
      logf("WARN  Non-temperature device on bus: %s (family 0x%02X)",
           bus[i].hex, bus[i].addr[0]);
    }
  }

  if (crcFailures > 0) {
    logf("WARN  %d CRC failure(s) during search - check pullup, wire length, splices",
         crcFailures);
  }
  if (duplicates > 0) {
    logf("WARN  Search returned %d repeat address(es) - marginal bus, %d distinct device(s) kept",
         duplicates, busCount);
  }

  if (busCount != previousCount) {
    logf("BUS   Device count changed: %d -> %d", previousCount, busCount);
  }

  mapRolesToBus();
}

// Bind each stored role address to whatever is on the bus right now.
void mapRolesToBus() {
  // Drop every existing tag first. This also runs after assign/clear, where
  // bus[] still carries the previous mapping - a stale tag would let the old
  // sensor keep feeding a role that was moved to a different sensor.
  for (int i = 0; i < busCount; i++) bus[i].role = -1;

  for (int r = 0; r < ROLE_COUNT; r++) {
    roles[r].present = false;
    if (!roles[r].assigned) continue;

    for (int i = 0; i < busCount; i++) {
      if (addrEqual(bus[i].addr, roles[r].addr)) {
        bus[i].role = r;
        roles[r].present = true;
        break;
      }
    }
    if (!roles[r].present) {
      roles[r].valid = false;
    }
    publishPresence(r, false);
  }
}

// ============================================
// TEMPERATURE READING (non-blocking)
// ============================================
void startConversion() {
  if (busCount == 0) return;
  tempSensors.requestTemperatures();  // Skip-ROM convert, all devices at once
  conversionPending = true;
  conversionStart = millis();
}

void collectConversion() {
  conversionPending = false;

  for (int i = 0; i < busCount; i++) {
    BusSensor* s = &bus[i];
    if (!s->isDS18B20) continue;

    s->reads++;
    float c = tempSensors.getTempC(s->addr);
    float f = (c * 9.0 / 5.0) + 32.0;

    if (c == DEVICE_DISCONNECTED_C || f < TEMP_MIN_F || f > TEMP_MAX_F) {
      s->errors++;
      s->valid = false;
      logf("ERR   %s read failed (raw %.2fC) - errors %lu/%lu",
           s->hex, c, (unsigned long)s->errors, (unsigned long)s->reads);
    } else {
      s->valid = true;
      s->tempF = f;
    }

    // Raw per-address feed, published whether or not the sensor has a role yet.
    // Not retained - stale addresses should disappear when a sensor is unplugged.
    char rawTopic[64];
    snprintf(rawTopic, sizeof(rawTopic), "%s%s", topic_raw_prefix, s->hex);
    char valStr[12];
    dtostrf(s->valid ? s->tempF : -999.0, 5, 1, valStr);
    if (mqttClient.connected()) {
      mqttClient.publish(rawTopic, valStr);
    }

    // Push through to the role this sensor is bound to
    if (s->role >= 0) {
      roles[s->role].valid = s->valid;
      if (s->valid) roles[s->role].tempF = s->tempF;
      publishRoleTemp(s->role, false);
      publishPresence(s->role, false);
    }
  }

  publishCounts();
}

// ============================================
// PUBLISHING
// ============================================
void publishRoleTemp(int r, bool force) {
  if (!mqttClient.connected()) return;
  if (!roles[r].assigned || !roles[r].valid) return;

  unsigned long now = millis();
  bool changed = fabs(roles[r].tempF - roles[r].lastPublished) >= PUBLISH_DELTA_F;
  bool due = (now - roles[r].lastPublish) >= PUBLISH_INTERVAL;

  if (force || changed || due) {
    char valStr[12];
    dtostrf(roles[r].tempF, 5, 1, valStr);
    mqttClient.publish(ROLES[r].stateTopic, valStr, true);
    roles[r].lastPublished = roles[r].tempF;
    roles[r].lastPublish = now;
  }
}

// Drives the per-entity availability topic so an unwired sensor shows as
// "unavailable" in Home Assistant instead of holding its last reading.
void publishPresence(int r, bool force) {
  if (!mqttClient.connected()) return;

  bool live = roles[r].assigned && roles[r].present && roles[r].valid;
  if (!force && roles[r].presencePublishedOnce && live == roles[r].lastPresencePublished) return;

  char topic[64];
  snprintf(topic, sizeof(topic), "%s%s", topic_presence_prefix, ROLES[r].key);
  mqttClient.publish(topic, live ? "ONLINE" : "OFFLINE", true);
  roles[r].lastPresencePublished = live;
  roles[r].presencePublishedOnce = true;
}

void publishCounts() {
  if (!mqttClient.connected()) return;

  int assignedLive = 0;
  for (int r = 0; r < ROLE_COUNT; r++) {
    if (roles[r].assigned && roles[r].present) assignedLive++;
  }

  char buf[8];
  itoa(busCount, buf, 10);
  mqttClient.publish(topic_found, buf, true);
  itoa(assignedLive, buf, 10);
  mqttClient.publish(topic_assigned, buf, true);
  mqttClient.publish(topic_parasite, parasitePower ? "ON" : "OFF", true);

  bool fault = (assignedLive < ROLE_COUNT);
  if (!faultPublishedOnce || fault != lastFaultPublished) {
    mqttClient.publish(topic_fault, fault ? "ON" : "OFF", true);
    lastFaultPublished = fault;
    faultPublishedOnce = true;
  }
}

// Full picture of the bus in one message - the fastest way to see what is
// plugged in, what it reads, and which role owns it.
void publishScanJson() {
  if (!mqttClient.connected()) return;

  String json = "{\"count\":" + String(busCount);
  json += ",\"parasite\":" + String(parasitePower ? "true" : "false");
  json += ",\"sensors\":[";
  for (int i = 0; i < busCount; i++) {
    if (i > 0) json += ",";
    json += "{\"i\":" + String(i);
    json += ",\"addr\":\"" + String(bus[i].hex) + "\"";
    json += ",\"role\":\"" + String(bus[i].role >= 0 ? ROLES[bus[i].role].key : "") + "\"";
    json += ",\"f\":" + String(bus[i].valid ? bus[i].tempF : -999.0, 1);
    json += ",\"reads\":" + String(bus[i].reads);
    json += ",\"errors\":" + String(bus[i].errors);
    json += "}";
  }
  json += "],\"missing\":[";
  bool first = true;
  for (int r = 0; r < ROLE_COUNT; r++) {
    if (roles[r].assigned && roles[r].present) continue;
    if (!first) json += ",";
    json += "\"" + String(ROLES[r].key) + "\"";
    first = false;
  }
  json += "]}";

  mqttClient.publish(topic_scan, json.c_str(), true);
}

// ============================================
// SERIAL BUS REPORT
// ============================================
void printBusReport() {
  unsigned long secs = millis() / 1000;

  Serial.println();
  Serial.printf("--- OneWire bus report --- GPIO%d  up %02lu:%02lu:%02lu  parasite=%s  found=%d\n",
                ONEWIRE_PIN, secs / 3600, (secs % 3600) / 60, secs % 60,
                parasitePower ? "YES" : "no", busCount);

  if (busCount == 0) {
    Serial.println("  NO DEVICES. Check: 4.7k pullup to 3.3V, data on GPIO13, GND common,");
    Serial.println("  VDD on 3.3V (not 5V), and that no two sensors share a shorted splice.");
  }

  Serial.println("  #  ROM Address        Role                 Temp F   Temp C   Reads  Errors");
  for (int i = 0; i < busCount; i++) {
    BusSensor* s = &bus[i];
    char tf[10], tc[10];
    if (s->valid) {
      dtostrf(s->tempF, 7, 1, tf);
      dtostrf((s->tempF - 32.0) * 5.0 / 9.0, 7, 1, tc);
    } else {
      strcpy(tf, "   ----");
      strcpy(tc, "   ----");
    }
    Serial.printf("  %d  %-16s   %-20s %s  %s  %6lu  %6lu%s\n",
                  i, s->hex,
                  s->role >= 0 ? ROLES[s->role].key : "(unassigned)",
                  tf, tc,
                  (unsigned long)s->reads, (unsigned long)s->errors,
                  s->isDS18B20 ? "" : "  <- NOT A DS18B20");
  }

  // Roles with a stored address that is not currently answering
  bool anyMissing = false;
  for (int r = 0; r < ROLE_COUNT; r++) {
    if (roles[r].assigned && !roles[r].present) {
      if (!anyMissing) { Serial.println("  MISSING (assigned but not on bus):"); anyMissing = true; }
      char hex[17];
      addrToHex(roles[r].addr, hex);
      Serial.printf("    %-20s %s\n", ROLES[r].key, hex);
    }
  }
  bool anyUnassigned = false;
  for (int r = 0; r < ROLE_COUNT; r++) {
    if (!roles[r].assigned) {
      if (!anyUnassigned) { Serial.println("  UNASSIGNED roles:"); anyUnassigned = true; }
      Serial.printf("    %s\n", ROLES[r].key);
    }
  }
  if (!anyMissing && !anyUnassigned && busCount >= ROLE_COUNT) {
    Serial.println("  All roles assigned and live.");
  }
  Serial.println();
}

// ============================================
// ROLE ASSIGNMENT
// ============================================
void assignRole(int r, const DeviceAddress addr, bool save) {
  char hex[17];
  addrToHex(addr, hex);

  // One address can only own one role - steal it from whoever had it
  for (int other = 0; other < ROLE_COUNT; other++) {
    if (other != r && roles[other].assigned && addrEqual(roles[other].addr, addr)) {
      logf("ASSIGN %s was held by %s - reassigning", hex, ROLES[other].key);
      clearRole(other, false);
    }
  }

  memcpy(roles[r].addr, addr, 8);
  roles[r].assigned = true;
  roles[r].valid = false;
  roles[r].lastPublished = -999.0;

  if (save) saveAssignments();
  mapRolesToBus();
  publishPresence(r, true);
  publishCounts();
  publishScanJson();

  logf("ASSIGN %s -> %s", ROLES[r].key, hex);
}

void clearRole(int r, bool save) {
  memset(roles[r].addr, 0, 8);
  roles[r].assigned = false;
  roles[r].present = false;
  roles[r].valid = false;

  if (save) saveAssignments();
  publishPresence(r, true);
  logf("CLEAR  %s unassigned", ROLES[r].key);
}

int roleIndexFromKey(const String& key) {
  String k = key;
  k.trim();
  k.toLowerCase();
  for (int r = 0; r < ROLE_COUNT; r++) {
    if (k == ROLES[r].key) return r;
  }
  return -1;
}

// ============================================
// IDENTIFY BY WARMING
// ============================================
// Press the role's Identify button in Home Assistant (or publish the role key to
// TempBusIdentifyCommand), then hold that sensor in your hand. The sensor whose
// temperature climbs fastest wins the role. This is how the harness gets labelled
// without tracing wires.
void startIdentify(int r) {
  if (busCount == 0) {
    logf("IDENT  Cannot identify - no devices on the bus");
    return;
  }

  identifyActive = true;
  identifyRole = r;
  identifyStart = millis();
  lastIdentifyProgress = 0;

  for (int i = 0; i < busCount; i++) {
    bus[i].baselineF = bus[i].valid ? bus[i].tempF : -999.0;
  }

  logf("IDENT  Warm the %s sensor now - watching %d device(s) for %lus",
       ROLES[r].key, busCount, IDENTIFY_WINDOW_MS / 1000);

  if (mqttClient.connected()) {
    String s = "WAITING: " + String(ROLES[r].key);
    mqttClient.publish(topic_identify_state, s.c_str(), true);
  }
}

void handleIdentify() {
  if (!identifyActive) return;

  unsigned long elapsed = millis() - identifyStart;

  int bestIdx = -1, secondIdx = -1;
  float bestRise = -1000.0, secondRise = -1000.0;

  for (int i = 0; i < busCount; i++) {
    if (!bus[i].valid || bus[i].baselineF < TEMP_MIN_F) continue;
    float rise = bus[i].tempF - bus[i].baselineF;
    if (rise > bestRise) {
      secondRise = bestRise; secondIdx = bestIdx;
      bestRise = rise;       bestIdx = i;
    } else if (rise > secondRise) {
      secondRise = rise;     secondIdx = i;
    }
  }

  // Progress line once a second so you can see the rise happening
  if (millis() - lastIdentifyProgress >= 1000) {
    lastIdentifyProgress = millis();
    if (bestIdx >= 0) {
      Serial.printf("IDENT  %2lus left | leader %s +%.1fF | runner-up +%.1fF\n",
                    (IDENTIFY_WINDOW_MS - elapsed) / 1000, bus[bestIdx].hex, bestRise,
                    secondIdx >= 0 ? secondRise : 0.0);
    }
  }

  bool decided = (bestIdx >= 0 &&
                  bestRise >= IDENTIFY_DELTA_F &&
                  (secondIdx < 0 || (bestRise - secondRise) >= IDENTIFY_MARGIN_F));

  if (decided) {
    identifyActive = false;
    int r = identifyRole;
    identifyRole = -1;
    logf("IDENT  %s rose %.1fF - assigning to %s", bus[bestIdx].hex, bestRise, ROLES[r].key);
    assignRole(r, bus[bestIdx].addr, true);

    if (mqttClient.connected()) {
      String s = "DONE: " + String(ROLES[r].key) + "=" + String(bus[bestIdx].hex);
      mqttClient.publish(topic_identify_state, s.c_str(), true);
    }
    return;
  }

  if (elapsed >= IDENTIFY_WINDOW_MS) {
    identifyActive = false;
    logf("IDENT  Timeout for %s - best rise %.1fF (need %.1fF, and %.1fF clear of runner-up)",
         ROLES[identifyRole].key, bestRise > -999.0 ? bestRise : 0.0,
         IDENTIFY_DELTA_F, IDENTIFY_MARGIN_F);
    identifyRole = -1;
    if (mqttClient.connected()) {
      mqttClient.publish(topic_identify_state, "TIMEOUT", true);
    }
  }
}

// ============================================
// PERSISTENCE (NVS)
// ============================================
void saveAssignments() {
  for (int r = 0; r < ROLE_COUNT; r++) {
    if (roles[r].assigned) {
      preferences.putBytes(ROLES[r].nvsKey, roles[r].addr, 8);
    } else {
      preferences.remove(ROLES[r].nvsKey);
    }
  }
  Serial.println("Assignments saved to NVS");
}

void loadAssignments() {
  char legacyKey[8];
  char hex[17];
  int loaded = 0;

  for (int r = 0; r < ROLE_COUNT; r++) {
    DeviceAddress addr;
    size_t n = preferences.getBytes(ROLES[r].nvsKey, addr, 8);
    bool migrated = false;

    // Nothing under the position-keyed name means this board was last flashed
    // with the index-keyed layout - read the old slot and carry the assignment
    // across, so a harness labelled before the exhaust position was dropped
    // does not have to be re-identified.
    if (n != 8 && ROLES[r].legacyIndex >= 0) {
      snprintf(legacyKey, sizeof(legacyKey), "a%d", ROLES[r].legacyIndex);
      n = preferences.getBytes(legacyKey, addr, 8);
      migrated = (n == 8);
    }

    if (n == 8 && !addrIsZero(addr) && OneWire::crc8(addr, 7) == addr[7]) {
      memcpy(roles[r].addr, addr, 8);
      roles[r].assigned = true;
      addrToHex(addr, hex);
      if (migrated) {
        if (preferences.putBytes(ROLES[r].nvsKey, addr, 8) == 8) {
          preferences.remove(legacyKey);
        }
        Serial.printf("  %-20s <- %s (migrated from %s)\n", ROLES[r].key, hex, legacyKey);
      } else {
        Serial.printf("  %-20s <- %s\n", ROLES[r].key, hex);
      }
      loaded++;
    } else {
      roles[r].assigned = false;
      Serial.printf("  %-20s <- (unassigned)\n", ROLES[r].key);
    }
    roles[r].lastPublished = -999.0;
    roles[r].tempF = -999.0;
  }
  Serial.printf("%d of %d roles restored from NVS\n", loaded, ROLE_COUNT);

  // The boiler_exhaust position is gone; drop its old slot rather than leave a
  // stale address sitting in NVS.
  preferences.remove("a4");
}

// ============================================
// MQTT CALLBACK
// ============================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  message.trim();

  String t = String(topic);
  Serial.printf("MQTT %s = %s\n", topic, message.c_str());

  // Retained commands get replayed on connect - ignore them briefly
  if (millis() - mqttConnectedAt < MQTT_CONNECT_GRACE_MS) {
    Serial.println("  ignored - connect grace period");
    return;
  }

  if (t == topic_command) {
    String cmd = message;
    cmd.toUpperCase();

    if (cmd == "SCAN") {
      logf("CMD    Rescanning bus");
      rescanBus();
      publishScanJson();
      publishCounts();
      printBusReport();
    } else if (cmd == "REPORT") {
      printBusReport();
      publishScanJson();
      // Push the whole table out over MQTT too
      for (int i = 0; i < busCount; i++) {
        logf("  [%d] %s %-16s %.1fF err=%lu", i, bus[i].hex,
             bus[i].role >= 0 ? ROLES[bus[i].role].key : "(unassigned)",
             bus[i].valid ? bus[i].tempF : -999.0, (unsigned long)bus[i].errors);
      }
    } else if (cmd == "CLEAR") {
      for (int r = 0; r < ROLE_COUNT; r++) clearRole(r, false);
      saveAssignments();
      mapRolesToBus();
      publishCounts();
      publishScanJson();
      logf("CMD    All role assignments cleared");
    } else if (cmd == "SAVE") {
      saveAssignments();
      logf("CMD    Assignments saved");
    } else if (cmd == "RESET") {
      logf("CMD    Restarting");
      delay(200);
      ESP.restart();
    } else {
      logf("CMD    Unknown command '%s' (SCAN|REPORT|CLEAR|SAVE|RESET)", message.c_str());
    }
    return;
  }

  // "<role>=<ROM hex>" binds by address; "<role>=<n>" binds by bus index.
  // "<role>=" (empty) clears the role.
  if (t == topic_assign_cmd) {
    int eq = message.indexOf('=');
    if (eq < 0) {
      logf("ASSIGN Bad payload '%s' - expected <role>=<addr|index>", message.c_str());
      return;
    }
    String roleKey = message.substring(0, eq);
    String value = message.substring(eq + 1);
    value.trim();

    int r = roleIndexFromKey(roleKey);
    if (r < 0) {
      logf("ASSIGN Unknown role '%s'", roleKey.c_str());
      return;
    }

    if (value.length() == 0) {
      clearRole(r, true);
      mapRolesToBus();
      publishCounts();
      publishScanJson();
      return;
    }

    DeviceAddress addr;
    if (hexToAddr(value, addr)) {
      assignRole(r, addr, true);
      return;
    }

    // Fall back to a bus index
    bool numeric = value.length() > 0;
    for (unsigned int i = 0; i < value.length(); i++) {
      if (!isDigit(value.charAt(i))) { numeric = false; break; }
    }
    int idx = value.toInt();
    if (numeric && idx >= 0 && idx < busCount) {
      assignRole(r, bus[idx].addr, true);
    } else {
      logf("ASSIGN '%s' is neither a 16-char ROM address nor a bus index 0-%d",
           value.c_str(), busCount - 1);
    }
    return;
  }

  if (t == topic_identify_cmd) {
    if (message.equalsIgnoreCase("CANCEL")) {
      if (identifyActive) {
        identifyActive = false;
        identifyRole = -1;
        logf("IDENT  Cancelled");
        mqttClient.publish(topic_identify_state, "IDLE", true);
      }
      return;
    }
    int r = roleIndexFromKey(message);
    if (r < 0) {
      logf("IDENT  Unknown role '%s'", message.c_str());
      return;
    }
    startIdentify(r);
    return;
  }
}

// ============================================
// MQTT CONNECTION
// ============================================
bool reconnectMQTT() {
  if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password,
                         topic_status, 1, true, "OFFLINE")) {
    Serial.println("MQTT connected");
    mqttConnectedAt = millis();

    mqttClient.subscribe(topic_command);
    mqttClient.subscribe(topic_assign_cmd);
    mqttClient.subscribe(topic_identify_cmd);

    mqttClient.publish(topic_status, "ONLINE", true);
    mqttClient.publish(topic_identify_state, identifyActive ? "WAITING" : "IDLE", true);

    publishHomeAssistantDiscovery();

    for (int r = 0; r < ROLE_COUNT; r++) {
      publishPresence(r, true);
      publishRoleTemp(r, true);
    }
    publishCounts();
    publishScanJson();
    sendStatus();
    return true;
  }
  return false;
}

// ============================================
// HOME ASSISTANT DISCOVERY
// ============================================
void publishHomeAssistantDiscovery() {
  if (!mqttClient.connected()) return;

  String deviceId = "chester_temp_bus";

  String device = "\"device\":{";
  device += "\"identifiers\":[\"" + deviceId + "\"],";
  device += "\"name\":\"Temp Sensor Bus\",";
  device += "\"model\":\"ESP32-S2 Mini\",";
  device += "\"manufacturer\":\"Chester RV Control\",";
  device += "\"sw_version\":\"" + String(FIRMWARE_VERSION) + "\"";
  device += "}";

  // Board-level availability, shared by every entity
  String avail = "\"availability_topic\":\"" + String(topic_status) + "\",";
  avail += "\"payload_available\":\"ONLINE\",";
  avail += "\"payload_not_available\":\"OFFLINE\",";

  // The boiler_exhaust position is gone. Its discovery configs and state topics
  // are retained on the broker, so Home Assistant keeps showing the entities
  // until an empty retained message lands on top of each. Idempotent - it can
  // stay here indefinitely.
  const char* retired[] = {
    "homeassistant/sensor/", "boiler_exhaust",
    "homeassistant/button/", "id_boiler_exhaust"
  };
  for (size_t i = 0; i < sizeof(retired) / sizeof(retired[0]); i += 2) {
    String t = String(retired[i]) + deviceId + "/" + retired[i + 1] + "/config";
    mqttClient.publish(t.c_str(), "", true);
    delay(50);
  }
  mqttClient.publish("TempBusBoilerExhaust", "", true);
  mqttClient.publish((String(topic_presence_prefix) + "boiler_exhaust").c_str(), "", true);

  // One temperature sensor per harness position. Each carries a second
  // availability topic so an unwired position reads "unavailable" rather than
  // freezing on its last value.
  for (int r = 0; r < ROLE_COUNT; r++) {
    String presenceTopic = String(topic_presence_prefix) + ROLES[r].key;

    String cfgTopic = "homeassistant/sensor/" + deviceId + "/" + ROLES[r].key + "/config";
    String cfg = "{";
    cfg += "\"name\":\"" + String(ROLES[r].name) + "\",";
    cfg += "\"unique_id\":\"" + deviceId + "_" + ROLES[r].key + "\",";
    cfg += "\"state_topic\":\"" + String(ROLES[r].stateTopic) + "\",";
    cfg += "\"unit_of_measurement\":\"\\u00b0F\",";
    cfg += "\"device_class\":\"temperature\",";
    cfg += "\"state_class\":\"measurement\",";
    cfg += "\"suggested_display_precision\":1,";
    cfg += "\"icon\":\"" + String(ROLES[r].icon) + "\",";
    cfg += "\"availability_mode\":\"all\",";
    cfg += "\"availability\":[";
    cfg += "{\"topic\":\"" + String(topic_status) + "\",\"payload_available\":\"ONLINE\",\"payload_not_available\":\"OFFLINE\"},";
    cfg += "{\"topic\":\"" + presenceTopic + "\",\"payload_available\":\"ONLINE\",\"payload_not_available\":\"OFFLINE\"}";
    cfg += "],";
    cfg += device;
    cfg += "}";
    mqttClient.publish(cfgTopic.c_str(), cfg.c_str(), true);

    // Identify button - press, then warm that sensor with your hand
    String btnTopic = "homeassistant/button/" + deviceId + "/id_" + ROLES[r].key + "/config";
    String btn = "{";
    btn += "\"name\":\"Identify " + String(ROLES[r].name) + "\",";
    btn += "\"unique_id\":\"" + deviceId + "_identify_" + ROLES[r].key + "\",";
    btn += "\"command_topic\":\"" + String(topic_identify_cmd) + "\",";
    btn += "\"payload_press\":\"" + String(ROLES[r].key) + "\",";
    btn += "\"icon\":\"mdi:hand-back-right\",";
    btn += "\"entity_category\":\"config\",";
    btn += avail;
    btn += device;
    btn += "}";
    mqttClient.publish(btnTopic.c_str(), btn.c_str(), true);
  }

  // Maintenance buttons
  struct { const char* id; const char* name; const char* topic; const char* payload; const char* icon; }
  buttons[] = {
    { "rescan", "Rescan Bus",         topic_command,      "SCAN",   "mdi:magnify-scan" },
    { "report", "Print Bus Report",   topic_command,      "REPORT", "mdi:table-search" },
    { "clear",  "Clear Assignments",  topic_command,      "CLEAR",  "mdi:eraser"       },
    { "cancel", "Cancel Identify",    topic_identify_cmd, "CANCEL", "mdi:cancel"       }
  };
  for (auto& b : buttons) {
    String topic = "homeassistant/button/" + deviceId + "/" + b.id + "/config";
    String cfg = "{";
    cfg += "\"name\":\"" + String(b.name) + "\",";
    cfg += "\"unique_id\":\"" + deviceId + "_" + b.id + "\",";
    cfg += "\"command_topic\":\"" + String(b.topic) + "\",";
    cfg += "\"payload_press\":\"" + String(b.payload) + "\",";
    cfg += "\"icon\":\"" + String(b.icon) + "\",";
    cfg += "\"entity_category\":\"config\",";
    cfg += avail;
    cfg += device;
    cfg += "}";
    mqttClient.publish(topic.c_str(), cfg.c_str(), true);
  }

  // Diagnostic sensors
  struct { const char* id; const char* name; const char* topic; const char* unit; const char* icon; }
  diags[] = {
    { "sensors_found",    "Sensors Found",    topic_found,          "", "mdi:counter"        },
    { "sensors_assigned", "Sensors Assigned", topic_assigned,       "", "mdi:check-network"  },
    { "identify_state",   "Identify State",   topic_identify_state, "", "mdi:hand-back-right"},
    { "scan",             "Bus Scan JSON",    topic_scan,           "", "mdi:code-json"      },
    { "log",              "Last Log Line",    topic_log,            "", "mdi:text-box"       }
  };
  for (auto& d : diags) {
    String topic = "homeassistant/sensor/" + deviceId + "/" + d.id + "/config";
    String cfg = "{";
    cfg += "\"name\":\"" + String(d.name) + "\",";
    cfg += "\"unique_id\":\"" + deviceId + "_" + d.id + "\",";
    cfg += "\"state_topic\":\"" + String(d.topic) + "\",";
    cfg += "\"icon\":\"" + String(d.icon) + "\",";
    cfg += "\"entity_category\":\"diagnostic\",";
    cfg += avail;
    cfg += device;
    cfg += "}";
    mqttClient.publish(topic.c_str(), cfg.c_str(), true);
  }

  // RSSI
  String rssiTopic = "homeassistant/sensor/" + deviceId + "/rssi/config";
  String rssiCfg = "{";
  rssiCfg += "\"name\":\"WiFi RSSI\",";
  rssiCfg += "\"unique_id\":\"" + deviceId + "_rssi\",";
  rssiCfg += "\"state_topic\":\"" + String(topic_rssi) + "\",";
  rssiCfg += "\"unit_of_measurement\":\"dBm\",";
  rssiCfg += "\"device_class\":\"signal_strength\",";
  rssiCfg += "\"state_class\":\"measurement\",";
  rssiCfg += "\"entity_category\":\"diagnostic\",";
  rssiCfg += avail;
  rssiCfg += device;
  rssiCfg += "}";
  mqttClient.publish(rssiTopic.c_str(), rssiCfg.c_str(), true);

  // Uptime
  String upTopic = "homeassistant/sensor/" + deviceId + "/uptime/config";
  String upCfg = "{";
  upCfg += "\"name\":\"Uptime\",";
  upCfg += "\"unique_id\":\"" + deviceId + "_uptime\",";
  upCfg += "\"state_topic\":\"" + String(topic_uptime) + "\",";
  upCfg += "\"unit_of_measurement\":\"s\",";
  upCfg += "\"device_class\":\"duration\",";
  upCfg += "\"state_class\":\"total_increasing\",";
  upCfg += "\"entity_category\":\"diagnostic\",";
  upCfg += avail;
  upCfg += device;
  upCfg += "}";
  mqttClient.publish(upTopic.c_str(), upCfg.c_str(), true);

  // Bus fault - ON whenever a harness position is missing
  String faultTopic = "homeassistant/binary_sensor/" + deviceId + "/fault/config";
  String faultCfg = "{";
  faultCfg += "\"name\":\"Bus Fault\",";
  faultCfg += "\"unique_id\":\"" + deviceId + "_fault\",";
  faultCfg += "\"state_topic\":\"" + String(topic_fault) + "\",";
  faultCfg += "\"payload_on\":\"ON\",";
  faultCfg += "\"payload_off\":\"OFF\",";
  faultCfg += "\"device_class\":\"problem\",";
  faultCfg += "\"entity_category\":\"diagnostic\",";
  faultCfg += avail;
  faultCfg += device;
  faultCfg += "}";
  mqttClient.publish(faultTopic.c_str(), faultCfg.c_str(), true);

  // Parasite power - should be OFF; ON means a sensor is missing its VDD wire
  String paraTopic = "homeassistant/binary_sensor/" + deviceId + "/parasite/config";
  String paraCfg = "{";
  paraCfg += "\"name\":\"Parasite Power\",";
  paraCfg += "\"unique_id\":\"" + deviceId + "_parasite\",";
  paraCfg += "\"state_topic\":\"" + String(topic_parasite) + "\",";
  paraCfg += "\"payload_on\":\"ON\",";
  paraCfg += "\"payload_off\":\"OFF\",";
  paraCfg += "\"icon\":\"mdi:flash-alert\",";
  paraCfg += "\"entity_category\":\"diagnostic\",";
  paraCfg += avail;
  paraCfg += device;
  paraCfg += "}";
  mqttClient.publish(paraTopic.c_str(), paraCfg.c_str(), true);

  Serial.println("Home Assistant discovery published");
}

// ============================================
// STATUS
// ============================================
void sendStatus() {
  if (!mqttClient.connected()) return;

  mqttClient.publish(topic_status, "ONLINE", true);

  char buf[16];
  itoa(WiFi.RSSI(), buf, 10);
  mqttClient.publish(topic_rssi, buf, true);

  snprintf(buf, sizeof(buf), "%lu", millis() / 1000);
  mqttClient.publish(topic_uptime, buf, true);
}

// ============================================
// SETUP
// ============================================
void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n================================");
  Serial.println("TempSensorBus " + String(FIRMWARE_VERSION));
  Serial.printf("OneWire bus on GPIO%d, %d harness positions\n", ONEWIRE_PIN, ROLE_COUNT);
  Serial.println("================================\n");

  memset(roles, 0, sizeof(roles));

  preferences.begin("tempbus", false);
  Serial.println("Restoring role assignments:");
  loadAssignments();

  Serial.println("\nScanning OneWire bus...");
  tempSensors.begin();
  tempSensors.setWaitForConversion(false);  // Conversions run non-blocking
  rescanBus();
  printBusReport();

  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed - sensors still read to serial");
  }

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);

  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    conversionPending = false;
    Serial.println("OTA Update Starting...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Complete!");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.begin();
    otaInitialized = true;
    Serial.println("OTA initialized");
  }

  Serial.println("\nSetup complete\n");
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // WiFi reconnection
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiLostSince == 0) wifiLostSince = millis();

    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 6) {
      delay(250);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconnected");
      wifiLostSince = 0;
      if (!otaInitialized) {
        ArduinoOTA.begin();
        otaInitialized = true;
      }
    } else if (millis() - wifiLostSince > WIFI_RECOVERY_TIMEOUT) {
      Serial.println("\nWiFi timeout - restarting...");
      ESP.restart();
    }
  } else {
    wifiLostSince = 0;
  }

  // MQTT connection
  if (!mqttClient.connected()) {
    if (mqttLostSince == 0) mqttLostSince = millis();

    if (millis() - mqttLostSince > 5000) {
      Serial.println("MQTT reconnecting...");
      if (reconnectMQTT()) {
        mqttLostSince = 0;
      } else if (millis() - mqttLostSince > MQTT_RECOVERY_TIMEOUT) {
        Serial.println("MQTT timeout - restarting...");
        ESP.restart();
      }
    }
  } else {
    mqttLostSince = 0;
  }

  mqttClient.loop();

  if (otaInitialized) ArduinoOTA.handle();
  if (otaInProgress) return;

  // LED mirrors bus health: solid when all roles are live, blinking otherwise
  bool allLive = true;
  for (int r = 0; r < ROLE_COUNT; r++) {
    if (!roles[r].assigned || !roles[r].present) { allLive = false; break; }
  }
  digitalWrite(STATUS_LED_PIN, allLive ? HIGH : ((millis() / 500) % 2));

  // Non-blocking conversion cycle
  if (!conversionPending && (millis() - lastReadStart >= READ_INTERVAL)) {
    lastReadStart = millis();
    startConversion();
  }
  if (conversionPending && (millis() - conversionStart >= CONVERSION_MS)) {
    collectConversion();
  }

  // Ticks every pass so the countdown and timeout run between conversions
  handleIdentify();

  // Re-enumerate periodically so sensors added mid-build show up on their own.
  // Skipped while a conversion is in flight - a search would corrupt it.
  if (!conversionPending && (millis() - lastRescan >= BUS_RESCAN_INTERVAL)) {
    lastRescan = millis();
    rescanBus();
  }

  if (millis() - lastReport >= REPORT_INTERVAL) {
    lastReport = millis();
    printBusReport();
  }

  if (millis() - lastScanPublish >= SCAN_PUBLISH_INTERVAL) {
    lastScanPublish = millis();
    publishScanJson();
  }

  if (millis() - lastStatusUpdate >= STATUS_INTERVAL) {
    lastStatusUpdate = millis();
    sendStatus();
  }

  delay(10);
}
