#include "ChesterNet.h"

#include <DNSServer.h>

#if defined(ESP32)
  #include <WiFi.h>
  #include <WebServer.h>
  #include <Update.h>
  #include <Preferences.h>
  typedef WebServer CnWebServer;
  #define CN_UPDATE_ERROR() String(Update.errorString())
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  #include <Updater.h>
  #include <EEPROM.h>
  typedef ESP8266WebServer CnWebServer;
  #define CN_UPDATE_ERROR() Update.getErrorString()
#else
  #error "ChesterNet supports ESP32 and ESP8266 only"
#endif

// ----------------------------------------------------------------------------
// Compiled-in defaults
// ----------------------------------------------------------------------------
// Development builds take them from secrets.ini (SECRET_* build flags). Release
// builds - the .bin files published for a future owner - carry none, so nothing
// private ends up in a public download.
#ifdef CHESTER_RELEASE_BUILD
  #define CN_DEF_SSID     ""
  #define CN_DEF_WIFIPASS ""
  #define CN_DEF_DEVPASS  ""
#else
  #if !defined(SECRET_WIFI_SSID) || !defined(SECRET_WIFI_PASSWORD) || !defined(SECRET_OTA_PASSWORD)
    #error "WiFi/OTA credentials missing - copy secrets.example.ini to secrets.ini at the repo root"
  #endif
  #define CN_DEF_SSID     SECRET_WIFI_SSID
  #define CN_DEF_WIFIPASS SECRET_WIFI_PASSWORD
  #define CN_DEF_DEVPASS  SECRET_OTA_PASSWORD
#endif
#define CN_DEF_MQTT_HOST "192.168.8.1"
#define CN_DEF_MQTT_PORT 1883

namespace ChesterNet {

// Off WiFi this long before the setup hotspot appears. Shorter than every
// board's own "network gone - restart" timer, so the hotspot always gets a chance.
static const unsigned long PORTAL_DELAY_MS = 30000UL;
// Hotspot up this long with nobody using it -> restart, as the boards' own
// network-recovery restarts did before (they are suppressed while it is up).
static const unsigned long PORTAL_IDLE_RESTART_MS = 15UL * 60UL * 1000UL;
// While the hotspot is up, ChesterNet alone retries the saved network: one
// attempt of PORTAL_STA_TRY_MS every PORTAL_STA_RETRY_MS. A connection attempt
// makes the radio scan every channel, and the hotspot can only beacon on its own
// - the boards' own back-to-back retries (Plumbing's ran about once a second)
// kept the hotspot effectively invisible.
static const unsigned long PORTAL_STA_RETRY_MS = 60000UL;
static const unsigned long PORTAL_STA_TRY_MS = 12000UL;

static const uint32_t STORE_MAGIC = 0x54454E43;   // "CNET"
static const uint8_t  STORE_LAYOUT = 1;
static const uint8_t  SRC_DEFAULTS = 0;   // seeded from the firmware's defaults
static const uint8_t  SRC_USER = 1;       // saved from the web page

struct Stored {
  uint32_t magic;
  uint8_t  layout;
  uint8_t  source;
  uint16_t mqttPort;
  uint32_t defaultsHash;   // which firmware defaults a SRC_DEFAULTS record came from
  char ssid[33];
  char wifiPass[64];
  char mqttHost[64];
  char devPass[33];
  char staticIp[16];
  char gateway[16];
  uint32_t crc;
};
static_assert(sizeof(Stored) <= CHESTERNET_EEPROM_SIZE - CHESTERNET_EEPROM_OFFSET,
              "ChesterNet settings do not fit their EEPROM region");

static Stored cfg;
static const char* host = "chester";
static const char* fwVersion = "";
static bool staticSupported = false;
static const char* defStaticIp = "";
static const char* defGateway = "";
static bool started = false;

static CnWebServer server(80);
static DNSServer dns;
static bool serverUp = false;

static bool portal = false;
static bool apSecured = false;
static bool staTrying = false;
static unsigned long staTryStart = 0;
static unsigned long lastStaTry = 0;
static char apSsid[33];
static unsigned long wifiDownSince = 0;
static unsigned long lastStationSeen = 0;
static unsigned long restartAt = 0;

static void (*updateStartCb)() = nullptr;
static void (*updateEndCb)(bool) = nullptr;
static bool updateStarted = false;
static String updateError;

// ----------------------------------------------------------------------------
// Storage
// ----------------------------------------------------------------------------
static uint32_t fnv1a(const uint8_t* p, size_t n, uint32_t h = 2166136261UL) {
  while (n--) { h ^= *p++; h *= 16777619UL; }
  return h;
}

static uint32_t storedCrc(const Stored& s) {
  return fnv1a(reinterpret_cast<const uint8_t*>(&s), offsetof(Stored, crc));
}

static void copyField(char* dst, size_t cap, const char* src) {
  strncpy(dst, src ? src : "", cap - 1);
  dst[cap - 1] = '\0';
}

static void fillDefaults(Stored& s) {
  memset(&s, 0, sizeof(s));
  s.magic = STORE_MAGIC;
  s.layout = STORE_LAYOUT;
  s.source = SRC_DEFAULTS;
  s.mqttPort = CN_DEF_MQTT_PORT;
  copyField(s.ssid, sizeof(s.ssid), CN_DEF_SSID);
  copyField(s.wifiPass, sizeof(s.wifiPass), CN_DEF_WIFIPASS);
  copyField(s.mqttHost, sizeof(s.mqttHost), CN_DEF_MQTT_HOST);
  copyField(s.devPass, sizeof(s.devPass), CN_DEF_DEVPASS);
  copyField(s.staticIp, sizeof(s.staticIp), defStaticIp);
  copyField(s.gateway, sizeof(s.gateway), defGateway);
  s.defaultsHash = fnv1a(reinterpret_cast<const uint8_t*>(&s), offsetof(Stored, crc));
}

#if defined(ESP32)
static bool loadStored(Stored& s) {
  Preferences p;
  if (!p.begin("chesternet", true)) return false;
  size_t n = p.getBytes("cfg", &s, sizeof(s));
  p.end();
  return n == sizeof(s);
}
static void saveStored(Stored& s) {
  s.crc = storedCrc(s);
  Preferences p;
  p.begin("chesternet", false);
  p.putBytes("cfg", &s, sizeof(s));
  p.end();
}
#else
static void ensureEeprom() {
  if (EEPROM.length() != CHESTERNET_EEPROM_SIZE) EEPROM.begin(CHESTERNET_EEPROM_SIZE);
}
static bool loadStored(Stored& s) {
  ensureEeprom();
  EEPROM.get(CHESTERNET_EEPROM_OFFSET, s);
  return true;
}
static void saveStored(Stored& s) {
  s.crc = storedCrc(s);
  ensureEeprom();
  EEPROM.put(CHESTERNET_EEPROM_OFFSET, s);
  EEPROM.commit();
}
#endif

static bool storedValid(const Stored& s) {
  return s.magic == STORE_MAGIC && s.layout == STORE_LAYOUT && s.crc == storedCrc(s) &&
         s.ssid[sizeof(s.ssid) - 1] == '\0' && s.wifiPass[sizeof(s.wifiPass) - 1] == '\0' &&
         s.mqttHost[sizeof(s.mqttHost) - 1] == '\0' && s.devPass[sizeof(s.devPass) - 1] == '\0' &&
         s.staticIp[sizeof(s.staticIp) - 1] == '\0' && s.gateway[sizeof(s.gateway) - 1] == '\0';
}

// ----------------------------------------------------------------------------
// Getters
// ----------------------------------------------------------------------------
const char* wifiSsid() { return cfg.ssid; }
const char* wifiPassword() { return cfg.wifiPass; }
const char* mqttHost() { return cfg.mqttHost; }
uint16_t mqttPort() { return cfg.mqttPort; }
const char* devicePassword() { return cfg.devPass; }
bool portalActive() { return portal; }
bool wifiRetryAllowed() { return !portal; }
void onUpdateStart(void (*fn)()) { updateStartCb = fn; }
void onUpdateEnd(void (*fn)(bool)) { updateEndCb = fn; }

void applyStaticIp() {
  if (!staticSupported || !cfg.staticIp[0]) return;
  IPAddress ip, gw;
  if (!ip.fromString(cfg.staticIp)) return;
  if (!gw.fromString(cfg.gateway)) { gw = ip; gw[3] = 1; }
  WiFi.config(ip, gw, IPAddress(255, 255, 255, 0), gw, IPAddress(8, 8, 8, 8));
}

// ----------------------------------------------------------------------------
// Web pages
// ----------------------------------------------------------------------------
static String esc(const char* s) {
  String o;
  for (; *s; s++) {
    switch (*s) {
      case '&': o += F("&amp;"); break;
      case '<': o += F("&lt;"); break;
      case '>': o += F("&gt;"); break;
      case '"': o += F("&quot;"); break;
      case '\'': o += F("&#39;"); break;
      default: o += *s;
    }
  }
  return o;
}

static String head(const char* title) {
  String h = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
               "<meta name='viewport' content='width=device-width,initial-scale=1'><title>");
  h += esc(host);
  h += F(" - ");
  h += title;
  h += F("</title><style>"
         "body{font-family:-apple-system,system-ui,sans-serif;margin:0;padding:16px;background:#f4f5f7;color:#1c1e21}"
         "main{max-width:560px;margin:auto}h1{font-size:1.4em;margin:.2em 0}h2{font-size:1.1em;margin:1.4em 0 .5em}"
         ".card{background:#fff;border-radius:10px;padding:14px 16px;box-shadow:0 1px 3px #0002;margin-bottom:12px}"
         "table{width:100%;border-collapse:collapse}td{padding:4px 0;vertical-align:top}td:first-child{color:#666;width:42%}"
         "label{display:block;margin:10px 0 4px;font-weight:600}input[type=text],input[type=password],input[type=number]{"
         "width:100%;box-sizing:border-box;padding:10px;font-size:16px;border:1px solid #bbb;border-radius:6px}"
         "button,.btn{display:inline-block;margin-top:14px;padding:11px 18px;font-size:16px;border:0;border-radius:6px;"
         "background:#1565c0;color:#fff;text-decoration:none}.warn{background:#b71c1c}.muted{color:#666;font-size:.9em}"
         ".ok{color:#2e7d32}.bad{color:#b71c1c}"
         "@media(prefers-color-scheme:dark){body{background:#121417;color:#e4e6eb}.card{background:#1e2126}"
         "td:first-child,.muted{color:#9aa0a6}input{background:#121417;color:#e4e6eb}}"
         "</style></head><body><main>");
  return h;
}

static const char FOOT[] PROGMEM = "</main></body></html>";

static void sendPage(int code, const String& body) {
  server.send(code, F("text/html"), body + FPSTR(FOOT));
}

static void sendMessage(const char* title, const String& msg, bool backLink = true) {
  String p = head(title);
  p += F("<h1>");
  p += title;
  p += F("</h1><div class='card'><p>");
  p += msg;
  p += F("</p>");
  if (backLink) p += F("<a class='btn' href='/'>Back</a>");
  p += F("</div>");
  sendPage(200, p);
}

static bool clientOnAp() {
  return portal && server.client().localIP() == WiFi.softAPIP();
}

// The setup hotspot is WPA2-protected with the device password, so a client on it
// has already proven it knows the password. Everyone else gets HTTP Basic auth
// (user "admin", password = device password). No device password = open.
static bool authorized() {
  if (!cfg.devPass[0]) return true;
  if (apSecured && clientOnAp()) return true;
  return server.authenticate("admin", cfg.devPass);
}

static bool requireAuth() {
  if (authorized()) return true;
  server.requestAuthentication(BASIC_AUTH, host);
  return false;
}

static String uptimeText() {
  unsigned long s = millis() / 1000;
  char b[32];
  snprintf(b, sizeof(b), "%lud %luh %lum", s / 86400, (s / 3600) % 24, (s / 60) % 60);
  return b;
}

static void handleRoot() {
  if (!requireAuth()) return;
  bool sta = WiFi.status() == WL_CONNECTED;

  String p = head("Settings");
  p += F("<h1>");
  p += esc(host);
  p += F("</h1><p class='muted'>Chester the Bus &middot; firmware ");
  p += esc(fwVersion);
  p += F("</p><div class='card'><table>");
  p += F("<tr><td>WiFi</td><td>");
  if (sta) {
    p += F("<span class='ok'>Connected</span> to ");
    p += esc(WiFi.SSID().c_str());
    p += F(" (");
    p += String(WiFi.RSSI());
    p += F(" dBm)</td></tr><tr><td>IP address</td><td>");
    p += WiFi.localIP().toString();
  } else {
    p += F("<span class='bad'>Not connected</span> (trying &quot;");
    p += esc(cfg.ssid);
    p += F("&quot;)");
  }
  p += F("</td></tr><tr><td>MQTT broker</td><td>");
  p += esc(cfg.mqttHost);
  p += ':';
  p += String(cfg.mqttPort);
  if (portal) {
    p += F("</td></tr><tr><td>Setup hotspot</td><td>On: ");
    p += esc(apSsid);
  }
  p += F("</td></tr><tr><td>Uptime</td><td>");
  p += uptimeText();
  p += F("</td></tr></table></div>");

  p += F("<h2>Network settings</h2><form class='card' method='POST' action='/save'>"
         "<label>WiFi network name</label><input type='text' name='ssid' maxlength='32' required value='");
  p += esc(cfg.ssid);
  p += F("'><label>WiFi password</label><input type='password' name='wifipass' maxlength='63' "
         "placeholder='unchanged'><label>MQTT broker (IP or hostname)</label>"
         "<input type='text' name='mqtthost' maxlength='63' required value='");
  p += esc(cfg.mqttHost);
  p += F("'><label>MQTT port</label><input type='number' name='mqttport' min='1' max='65535' value='");
  p += String(cfg.mqttPort);
  p += F("'>");
  if (staticSupported) {
    p += F("<label>Static IP (blank = automatic / DHCP)</label><input type='text' name='staticip' maxlength='15' value='");
    p += esc(cfg.staticIp);
    p += F("'><label>Gateway (router IP)</label><input type='text' name='gateway' maxlength='15' value='");
    p += esc(cfg.gateway);
    p += F("'>");
  }
  p += F("<label>New device password</label><input type='password' name='devpass' maxlength='32' "
         "placeholder='unchanged'><label>Repeat new device password</label>"
         "<input type='password' name='devpass2' maxlength='32' placeholder='unchanged'>"
         "<p class='muted'>The device password protects this page, the setup hotspot, and "
         "firmware uploads. Blank fields keep their current value. Saving restarts the board.</p>"
         "<button type='submit'>Save and restart</button></form>");

  p += F("<h2>Firmware</h2><div class='card'><p class='muted'>Install a new .bin file from this "
         "phone, tablet, or computer.</p><a class='btn' href='/update'>Update firmware</a></div>");

  p += F("<h2>Maintenance</h2><div class='card'>"
         "<form method='POST' action='/restart' style='display:inline'><button type='submit'>Restart</button></form> "
         "<form method='POST' action='/defaults' style='display:inline' "
         "onsubmit=\"return confirm('Replace the WiFi, MQTT and password settings with the ones built into this firmware?')\">"
         "<button class='warn' type='submit'>Reset network settings</button></form>"
         "<p class='muted'>Reset puts back the network settings built into the firmware. "
         "A downloaded release firmware has none, so the board starts the setup hotspot.</p></div>");
  sendPage(200, p);
}

static void handleSave() {
  if (!requireAuth()) return;

  Stored n = cfg;
  String ssid = server.arg("ssid");
  String wifiPass = server.arg("wifipass");
  String mqttHostArg = server.arg("mqtthost");
  long port = server.arg("mqttport").toInt();
  String devPass = server.arg("devpass");
  String devPass2 = server.arg("devpass2");
  ssid.trim();
  mqttHostArg.trim();

  if (ssid.length() < 1 || ssid.length() > 32) { sendMessage("Not saved", F("WiFi network name must be 1-32 characters.")); return; }
  if (wifiPass.length() > 0 && (wifiPass.length() < 8 || wifiPass.length() > 63)) {
    sendMessage("Not saved", F("A WiFi password must be 8-63 characters.")); return;
  }
  if (mqttHostArg.length() < 1 || mqttHostArg.length() > 63) { sendMessage("Not saved", F("Enter the MQTT broker address.")); return; }
  if (port < 1 || port > 65535) { sendMessage("Not saved", F("MQTT port must be 1-65535.")); return; }
  if (devPass != devPass2) { sendMessage("Not saved", F("The two device passwords do not match.")); return; }
  if (devPass.length() > 0 && (devPass.length() < 8 || devPass.length() > 32)) {
    sendMessage("Not saved", F("The device password must be 8-32 characters.")); return;
  }

  if (staticSupported) {
    String sip = server.arg("staticip");
    String gw = server.arg("gateway");
    sip.trim();
    gw.trim();
    IPAddress a, b;
    if (sip.length() && !a.fromString(sip)) { sendMessage("Not saved", F("Static IP is not a valid address.")); return; }
    if (gw.length() && !b.fromString(gw)) { sendMessage("Not saved", F("Gateway is not a valid address.")); return; }
    copyField(n.staticIp, sizeof(n.staticIp), sip.c_str());
    copyField(n.gateway, sizeof(n.gateway), gw.c_str());
  }

  copyField(n.ssid, sizeof(n.ssid), ssid.c_str());
  if (wifiPass.length()) copyField(n.wifiPass, sizeof(n.wifiPass), wifiPass.c_str());
  copyField(n.mqttHost, sizeof(n.mqttHost), mqttHostArg.c_str());
  n.mqttPort = (uint16_t)port;
  if (devPass.length()) copyField(n.devPass, sizeof(n.devPass), devPass.c_str());
  n.source = SRC_USER;

  saveStored(n);
  cfg = n;
  Serial.println("ChesterNet: settings saved from web page - restarting");
  sendMessage("Saved", F("Settings saved. The board is restarting and will join the WiFi network "
                         "in a few seconds. If it cannot, the setup hotspot comes back after 30 seconds."), false);
  restartAt = millis() + 1500;
}

static void handleDefaults() {
  if (!requireAuth()) return;
  Stored d;
  fillDefaults(d);
  saveStored(d);
  Serial.println("ChesterNet: network settings reset to firmware defaults - restarting");
  sendMessage("Reset", F("Network settings reset to the firmware defaults. Restarting."), false);
  restartAt = millis() + 1500;
}

static void handleRestart() {
  if (!requireAuth()) return;
  sendMessage("Restarting", F("The board is restarting."), false);
  restartAt = millis() + 1000;
}

static void handleUpdatePage() {
  if (!requireAuth()) return;
  String p = head("Firmware update");
  p += F("<h1>Firmware update</h1><div class='card'><p>Current firmware: <b>");
  p += esc(fwVersion);
  p += F("</b></p><p class='muted'>Choose the <b>.bin</b> file for this board. Its name should "
         "contain <b>");
  p += esc(host);
  p += F("</b>. The board keeps working until the upload finishes, then restarts on the new "
         "firmware; settings are kept. Don't close this page during the upload.</p>"
         "<form id='f' method='POST' action='/update' enctype='multipart/form-data'>"
         "<input type='file' name='firmware' accept='.bin' required>"
         "<label style='font-weight:normal'><input type='checkbox' id='force'> The file name is "
         "different, install it anyway</label>"
         "<button type='submit' id='go'>Upload and install</button></form>"
         "<p id='st' class='muted'></p></div><a href='/'>Back</a>"
         "<script>document.getElementById('f').onsubmit=function(){"
         "this.action='/update'+(document.getElementById('force').checked?'?force=1':'');"
         "document.getElementById('go').disabled=true;"
         "document.getElementById('st').textContent='Uploading... this takes up to a minute.';};</script>");
  sendPage(200, p);
}

static void finishUpdate(bool ok) {
  if (updateStarted && updateEndCb) updateEndCb(ok);
  updateStarted = false;
}

static void handleUpdateUpload() {
  HTTPUpload& up = server.upload();

  if (up.status == UPLOAD_FILE_START) {
    updateError = "";
    updateStarted = false;
    if (!authorized()) { updateError = F("Not authorized."); return; }

    String name = up.filename;
    String want = host;
    name.toLowerCase();
    want.toLowerCase();
    if (name.indexOf(want) < 0 && server.arg("force") != "1") {
      updateError = String(F("The file name \"")) + esc(up.filename.c_str()) +
                    F("\" does not contain \"") + esc(host) +
                    F("\". Nothing was installed. If you are sure it is for this board, tick the box and try again.");
      return;
    }

    Serial.printf("ChesterNet: firmware upload started: %s\n", up.filename.c_str());
    if (updateStartCb) updateStartCb();
    updateStarted = true;
#if defined(ESP32)
    bool ok = Update.begin(UPDATE_SIZE_UNKNOWN);
#else
    uint32_t maxSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    bool ok = Update.begin(maxSpace);
#endif
    if (!ok) updateError = CN_UPDATE_ERROR();
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (updateError.length() || !updateStarted) return;
    if (Update.write(up.buf, up.currentSize) != up.currentSize) updateError = CN_UPDATE_ERROR();
  } else if (up.status == UPLOAD_FILE_END) {
    if (updateError.length() || !updateStarted) return;
    if (!Update.end(true)) updateError = CN_UPDATE_ERROR();
    else Serial.printf("ChesterNet: firmware upload complete, %u bytes\n", (unsigned)up.totalSize);
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (updateStarted) Update.end();
    updateError = F("Upload aborted.");
  }
  delay(0);
}

static void handleUpdateDone() {
  if (!authorized()) {
    finishUpdate(false);
    server.requestAuthentication(BASIC_AUTH, host);
    return;
  }
  if (updateError.length() || !updateStarted) {
    if (!updateError.length()) updateError = F("No firmware file was received.");
    Serial.printf("ChesterNet: firmware update failed: %s\n", updateError.c_str());
    finishUpdate(false);
    sendMessage("Update failed", updateError);
    return;
  }
  finishUpdate(true);
  sendMessage("Update installed", F("The new firmware is installed. The board is restarting; this "
                                    "page will not reload by itself. Go back in about 20 seconds."), false);
  restartAt = millis() + 1500;
}

static void handleNotFound() {
  // In setup-hotspot mode every unknown URL (including the phone's captive-portal
  // probe) goes to the settings page, which makes the phone pop it up by itself.
  if (portal) {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    server.send(302, "text/plain", "");
    return;
  }
  server.send(404, "text/plain", "Not found");
}

static void ensureServer() {
  if (serverUp) return;
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/defaults", HTTP_POST, handleDefaults);
  server.on("/restart", HTTP_POST, handleRestart);
  server.on("/update", HTTP_GET, handleUpdatePage);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.onNotFound(handleNotFound);
  server.begin();
  serverUp = true;
  Serial.printf("ChesterNet: web page at http://%s.local/\n", host);
}

// ----------------------------------------------------------------------------
// Setup hotspot
// ----------------------------------------------------------------------------
static void startPortal() {
  // Stop the STA side hunting for the lost network first; loop() retries it on
  // a slow schedule from here on.
  WiFi.setAutoReconnect(false);
  WiFi.disconnect();
  WiFi.mode(WIFI_AP_STA);
  apSecured = strlen(cfg.devPass) >= 8;
  WiFi.softAP(apSsid, apSecured ? cfg.devPass : nullptr);
  dns.start(53, "*", WiFi.softAPIP());
  portal = true;
  staTrying = false;
  lastStaTry = millis();
  lastStationSeen = millis();
  ensureServer();
  Serial.printf("ChesterNet: WiFi unavailable - setup hotspot \"%s\" up at http://%s/\n",
                apSsid, WiFi.softAPIP().toString().c_str());
}

static void stopPortal() {
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  portal = false;
  staTrying = false;
  Serial.println("ChesterNet: WiFi is back - setup hotspot off");
}

// ----------------------------------------------------------------------------
// Public entry points
// ----------------------------------------------------------------------------
void begin(const char* hostname, const char* firmwareVersion,
           const char* defaultStaticIp, const char* defaultGateway) {
  host = hostname;
  fwVersion = firmwareVersion;
  staticSupported = (defaultStaticIp != nullptr);
#ifndef CHESTER_RELEASE_BUILD
  if (defaultStaticIp) defStaticIp = defaultStaticIp;
  if (defaultGateway) defGateway = defaultGateway;
#endif

  // Hostname + "-Setup", cut to the 32-character SSID limit
  snprintf(apSsid, sizeof(apSsid), "%.26s-Setup", host);

  // Nothing here should be written to the SDK's own flash WiFi store; the
  // settings live in ChesterNet's record, and the hotspot must not come back on
  // its own at the next boot.
  WiFi.persistent(false);

  Stored defaults;
  fillDefaults(defaults);
  Stored s;
  bool valid = loadStored(s) && storedValid(s);

  if (!valid) {
    cfg = defaults;
    saveStored(cfg);
    Serial.println("ChesterNet: no stored network settings - saved the firmware defaults");
  } else if (s.source == SRC_DEFAULTS && defaults.ssid[0] && s.defaultsHash != defaults.defaultsHash) {
    // Never edited from the web page, and this firmware carries different
    // defaults (e.g. secrets.ini changed): follow the firmware.
    cfg = defaults;
    saveStored(cfg);
    Serial.println("ChesterNet: firmware defaults changed - stored settings updated");
  } else {
    cfg = s;
  }

  Serial.printf("ChesterNet: WiFi \"%s\", MQTT %s:%u, settings from %s\n", cfg.ssid, cfg.mqttHost,
                cfg.mqttPort, cfg.source == SRC_USER ? "web page" : "firmware defaults");
  started = true;
}

void loop() {
  if (!started) return;
  unsigned long now = millis();
  bool sta = WiFi.status() == WL_CONNECTED;

  if (sta && !serverUp) ensureServer();
  if (serverUp) server.handleClient();
  if (portal) dns.processNextRequest();

  if (sta) {
    wifiDownSince = 0;
    if (portal && WiFi.softAPgetStationNum() == 0) stopPortal();
  } else {
    if (wifiDownSince == 0) wifiDownSince = now ? now : 1;
    if (!portal && (!cfg.ssid[0] || now - wifiDownSince >= PORTAL_DELAY_MS)) startPortal();
  }

  if (portal) {
    int stations = WiFi.softAPgetStationNum();
    if (stations > 0) lastStationSeen = now;

    // Slow retry of the saved network. Never while a phone is on the hotspot -
    // the channel scan would knock it off.
    if (stations > 0) {
      if (staTrying && !sta) WiFi.disconnect();
      staTrying = false;
      lastStaTry = now;
    } else if (!staTrying && cfg.ssid[0] && now - lastStaTry >= PORTAL_STA_RETRY_MS) {
      staTrying = true;
      staTryStart = now;
      lastStaTry = now;
      applyStaticIp();
      WiFi.begin(cfg.ssid, cfg.wifiPass);
    } else if (staTrying && !sta && now - staTryStart >= PORTAL_STA_TRY_MS) {
      staTrying = false;
      WiFi.disconnect();
    }

    if (stations == 0 && now - lastStationSeen >= PORTAL_IDLE_RESTART_MS) {
      Serial.println("ChesterNet: no WiFi for a long time and nobody on the hotspot - restarting");
      delay(100);
      ESP.restart();
    }
  }

  if (restartAt && (long)(now - restartAt) >= 0) {
    delay(100);
    ESP.restart();
  }
}

}  // namespace ChesterNet
