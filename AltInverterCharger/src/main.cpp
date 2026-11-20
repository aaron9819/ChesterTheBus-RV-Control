#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ChesterNet.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <stddef.h>

// ============================================
// CONFIGURATION
// ============================================
#define DEVICE_NAME "AltInverterCharger"
#define FIRMWARE_VERSION "v1.2.1"

// WiFi, MQTT broker and the device (OTA) password are stored on the board by
// ChesterNet (lib_shared/ChesterNet) and editable at http://<hostname>.local/.
// secrets.ini at the repo root only supplies the defaults a board starts with.
// These are filled in from ChesterNet early in setup().
const char* ssid = nullptr;
const char* password = nullptr;

// MQTT Configuration
const char* mqtt_server = nullptr;
int mqtt_port = 1883;
const unsigned long WIFI_KICK_MS = 10000;   // WiFi reconnect attempt interval while down
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* mqtt_client_id = "AltInverterCharger";

// OTA Configuration
const char* otaHostname = DEVICE_NAME;
const char* otaPassword = nullptr;

// MQTT Topics
const char* topic_status = "AltInverterStatus";                  // ONLINE/OFFLINE (retained, LWT)
const char* topic_power_command = "AltInverterPowerCommand";     // "PRESS" -> pulse relay
const char* topic_relay_state = "AltInverterRelayState";         // ON while relay pulsing, OFF otherwise
const char* topic_battery_voltage = "AltInverterBatteryVoltage"; // Calibrated volts
const char* topic_battery_raw_mv = "AltInverterBatteryRawMV";    // Raw averaged millivolts at the ADC pin
const char* topic_calibrate_command = "AltInverterCalibrateCommand"; // payload = actual battery volts
const char* topic_cal_scale = "AltInverterCalScale";             // Current calibration scale (retained)
const char* topic_rssi = "AltInverterRSSI";                      // WiFi signal strength
const char* topic_temperature = "AltInverterTemperature";        // DS18B20 °C

// Hardware Configuration
// BATTERY_ADC_PIN must be an ADC1 pin (GPIO1-GPIO10 on ESP32-S2).
// ADC2 (GPIO11-GPIO20) is unusable while WiFi is active - the WiFi driver owns it.
#define BATTERY_ADC_PIN 3   // ADC1_CH2 - safe with WiFi
#define RELAY_PIN 16        // Plain GPIO, no boot/strapping function on ESP32-S2
#define STATUS_LED_PIN 15   // Onboard LED on Lolin S2 Mini (active HIGH)
#define ONEWIRE_PIN 5       // DS18B20 data line (4.7k pullup to 3.3V)

// Set to false if your relay module is active-LOW (most opto-isolated boards)
#define RELAY_ACTIVE_HIGH false

// Relay pulse timing
const unsigned long RELAY_PULSE_MS = 500;       // Momentary press duration
const unsigned long RELAY_COOLDOWN_MS = 2000;   // Min time between presses
const unsigned long MQTT_CONNECT_GRACE_MS = 3000; // Ignore commands right after connect (stale/retained guard)

// Voltage measurement
// Hardware divider currently reads ~500mV at 12.5V battery -> default scale 25.0.
// Fine-tune via MQTT: publish the actual battery voltage (from a multimeter)
// to AltInverterCalibrateCommand and the scale is recomputed and saved.
const float DEFAULT_VOLTAGE_SCALE = 25.0;
const int ADC_SAMPLES = 32;                          // Averaged per reading
const unsigned long VOLTAGE_SAMPLE_INTERVAL = 1000;  // Read ADC every 1s
const unsigned long VOLTAGE_PUBLISH_INTERVAL = 10000; // Publish at least every 10s
const float VOLTAGE_PUBLISH_DELTA = 0.05;            // ...or sooner on a 50mV change

// Temperature measurement (DS18B20)
// Conversion runs non-blocking so the relay pulse handler is never starved
const unsigned long TEMP_REQUEST_INTERVAL = 10000;   // Start a reading every 10s
const unsigned long TEMP_CONVERSION_MS = 750;        // 12-bit conversion time
const float TEMP_PUBLISH_DELTA = 0.2;                // Publish on a 0.2°C change...
const unsigned long TEMP_PUBLISH_INTERVAL = 60000;   // ...or at least every 60s
const unsigned long TEMP_SENSOR_RETRY_MS = 30000;    // Re-probe if sensor missing

// EEPROM Configuration
#define EEPROM_SIZE 64
#define EEPROM_MAGIC 0xC4E1
#define EEPROM_VERSION 2   // v2: checksum no longer covers itself or padding (v1 never verified)

struct InverterConfig {
  uint16_t magic;
  uint8_t version;
  float voltageScale;
  uint16_t checksum;   // covers every byte before this field
};

// ============================================
// STATE
// ============================================
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature tempSensor(&oneWire);
bool tempSensorFound = false;
bool tempValid = false;
float temperatureC = 0.0;
float lastPublishedTemp = -1000.0;
bool tempConversionPending = false;
unsigned long tempRequestTime = 0;
unsigned long lastTempRequest = 0;
unsigned long lastTempPublish = 0;
unsigned long lastTempSensorProbe = 0;

float voltageScale = DEFAULT_VOLTAGE_SCALE;
float batteryVoltage = 0.0;
float lastPublishedVoltage = -100.0;
uint32_t batteryRawMV = 0;

bool relayPulsing = false;
unsigned long relayPulseStart = 0;
unsigned long lastRelayPress = 0;
unsigned long mqttConnectedAt = 0;

unsigned long lastVoltageSample = 0;
unsigned long lastVoltagePublish = 0;
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_INTERVAL = 60000;

// Recovery / Watchdog
const unsigned long WIFI_RECOVERY_TIMEOUT = 120000;
const unsigned long MQTT_RECOVERY_TIMEOUT = 120000;
unsigned long wifiLostSince = 0;
unsigned long mqttLostSince = 0;
bool otaInitialized = false;
bool otaInProgress = false;

// ============================================
// FORWARD DECLARATIONS
// ============================================
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool reconnectMQTT();
void publishHomeAssistantDiscovery();
void pressInverterButton();
void handleRelayPulse();
void relayWrite(bool closed);
void sampleVoltage();
void publishVoltage(bool force);
void handleTemperature();
void publishTemperature(bool force);
void handleCalibration(float actualVolts);
void sendStatus();
void saveConfigToEEPROM();
void loadConfigFromEEPROM();
uint16_t calculateChecksum(InverterConfig* cfg);

// ============================================
// EEPROM CONFIG MANAGEMENT
// ============================================
// The old version summed sizeof(struct) - 2 bytes. With padding that range
// included the checksum field itself (and an uninitialised pad byte), so a
// saved config never verified on the next boot and the calibration was lost.
uint16_t calculateChecksum(InverterConfig* cfg) {
  uint16_t sum = 0;
  uint8_t* ptr = (uint8_t*)cfg;
  for (size_t i = 0; i < offsetof(InverterConfig, checksum); i++) {
    sum += ptr[i];
  }
  return sum;
}

void saveConfigToEEPROM() {
  InverterConfig cfg = {};   // zeroed: padding bytes are deterministic
  cfg.magic = EEPROM_MAGIC;
  cfg.version = EEPROM_VERSION;
  cfg.voltageScale = voltageScale;
  cfg.checksum = calculateChecksum(&cfg);

  EEPROM.put(0, cfg);
  EEPROM.commit();

  Serial.print("💾 Config saved - voltage scale: ");
  Serial.println(voltageScale, 4);
}

void loadConfigFromEEPROM() {
  InverterConfig cfg;
  EEPROM.get(0, cfg);

  if (cfg.magic == EEPROM_MAGIC &&
      cfg.version == EEPROM_VERSION &&
      cfg.checksum == calculateChecksum(&cfg) &&
      cfg.voltageScale > 1.0 && cfg.voltageScale < 100.0) {
    voltageScale = cfg.voltageScale;
    Serial.print("💾 Config loaded - voltage scale: ");
    Serial.println(voltageScale, 4);
  } else {
    voltageScale = DEFAULT_VOLTAGE_SCALE;
    Serial.print("⚠️  No valid config in EEPROM, default scale: ");
    Serial.println(voltageScale, 4);
  }
}

// ============================================
// RELAY CONTROL
// ============================================
void relayWrite(bool closed) {
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(RELAY_PIN, closed ? HIGH : LOW);
  } else {
    digitalWrite(RELAY_PIN, closed ? LOW : HIGH);
  }
  digitalWrite(STATUS_LED_PIN, closed ? HIGH : LOW);
}

void pressInverterButton() {
  unsigned long now = millis();

  if (relayPulsing) {
    Serial.println("⚠️ Press ignored - relay already pulsing");
    return;
  }
  if (lastRelayPress != 0 && (now - lastRelayPress) < RELAY_COOLDOWN_MS) {
    Serial.println("⚠️ Press ignored - cooldown active");
    return;
  }

  relayPulsing = true;
  relayPulseStart = now;
  lastRelayPress = now;
  relayWrite(true);

  if (mqttClient.connected()) {
    mqttClient.publish(topic_relay_state, "ON", true);
  }
  Serial.println("⚡ Inverter power button PRESSED (relay closed)");
}

void handleRelayPulse() {
  if (relayPulsing && (millis() - relayPulseStart >= RELAY_PULSE_MS)) {
    relayPulsing = false;
    relayWrite(false);

    if (mqttClient.connected()) {
      mqttClient.publish(topic_relay_state, "OFF", true);
    }
    Serial.println("⚡ Inverter power button RELEASED (relay open)");
  }
}

// ============================================
// VOLTAGE MEASUREMENT
// ============================================
void sampleVoltage() {
  uint32_t sumMV = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sumMV += analogReadMilliVolts(BATTERY_ADC_PIN);
  }
  batteryRawMV = sumMV / ADC_SAMPLES;
  batteryVoltage = (batteryRawMV / 1000.0) * voltageScale;
}

void publishVoltage(bool force) {
  if (!mqttClient.connected()) return;

  unsigned long now = millis();
  bool changed = fabs(batteryVoltage - lastPublishedVoltage) >= VOLTAGE_PUBLISH_DELTA;
  bool due = (now - lastVoltagePublish) >= VOLTAGE_PUBLISH_INTERVAL;

  if (force || changed || due) {
    char voltStr[12];
    dtostrf(batteryVoltage, 5, 2, voltStr);
    mqttClient.publish(topic_battery_voltage, voltStr, true);

    char rawStr[12];
    itoa(batteryRawMV, rawStr, 10);
    mqttClient.publish(topic_battery_raw_mv, rawStr, true);

    lastPublishedVoltage = batteryVoltage;
    lastVoltagePublish = now;
  }
}

void handleCalibration(float actualVolts) {
  if (actualVolts < 5.0 || actualVolts > 40.0) {
    Serial.println("⚠️ Calibration rejected - voltage out of range (5-40V)");
    return;
  }

  // Take a fresh averaged reading so calibration isn't based on stale data
  sampleVoltage();

  if (batteryRawMV < 50) {
    Serial.println("⚠️ Calibration rejected - ADC reading too low (<50mV), check wiring");
    return;
  }

  voltageScale = actualVolts / (batteryRawMV / 1000.0);
  saveConfigToEEPROM();

  Serial.print("🔧 Calibrated: ");
  Serial.print(batteryRawMV);
  Serial.print("mV = ");
  Serial.print(actualVolts, 2);
  Serial.print("V -> scale ");
  Serial.println(voltageScale, 4);

  if (mqttClient.connected()) {
    char scaleStr[12];
    dtostrf(voltageScale, 7, 4, scaleStr);
    mqttClient.publish(topic_cal_scale, scaleStr, true);
  }

  // Re-publish with the new scale immediately
  sampleVoltage();
  publishVoltage(true);
}

// ============================================
// TEMPERATURE MEASUREMENT (DS18B20)
// ============================================
void handleTemperature() {
  unsigned long now = millis();

  // Re-probe periodically if the sensor was never found (or got unplugged)
  if (!tempSensorFound) {
    if (now - lastTempSensorProbe >= TEMP_SENSOR_RETRY_MS) {
      lastTempSensorProbe = now;
      tempSensor.begin();
      if (tempSensor.getDeviceCount() > 0) {
        tempSensorFound = true;
        tempSensor.setResolution(12);
        tempSensor.setWaitForConversion(false);
        Serial.println("✓ DS18B20 sensor found");
      }
    }
    return;
  }

  // Start a new conversion every TEMP_REQUEST_INTERVAL
  if (!tempConversionPending && (now - lastTempRequest >= TEMP_REQUEST_INTERVAL)) {
    lastTempRequest = now;
    tempSensor.requestTemperatures();
    tempConversionPending = true;
    tempRequestTime = now;
  }

  // Collect the result once conversion has had time to finish
  if (tempConversionPending && (now - tempRequestTime >= TEMP_CONVERSION_MS)) {
    tempConversionPending = false;
    float t = tempSensor.getTempCByIndex(0);

    if (t == DEVICE_DISCONNECTED_C || t < -55.0 || t > 125.0) {
      Serial.println("⚠️ DS18B20 read failed - will re-probe");
      tempSensorFound = false;
      lastTempSensorProbe = now;
      return;
    }

    temperatureC = t;
    tempValid = true;
    publishTemperature(false);
  }
}

void publishTemperature(bool force) {
  if (!mqttClient.connected() || !tempValid) return;

  unsigned long now = millis();
  bool changed = fabs(temperatureC - lastPublishedTemp) >= TEMP_PUBLISH_DELTA;
  bool due = (now - lastTempPublish) >= TEMP_PUBLISH_INTERVAL;

  if (force || changed || due) {
    char tempStr[12];
    dtostrf(temperatureC, 5, 1, tempStr);
    mqttClient.publish(topic_temperature, tempStr, true);

    lastPublishedTemp = temperatureC;
    lastTempPublish = now;
  }
}

// ============================================
// MQTT CALLBACK
// ============================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();

  Serial.printf("MQTT message on %s: ", topic);
  Serial.println(message);

  // Guard against stale/retained commands replayed right after (re)connect
  if (millis() - mqttConnectedAt < MQTT_CONNECT_GRACE_MS) {
    Serial.println("⚠️ Command ignored - connect grace period");
    return;
  }

  if (String(topic) == topic_power_command) {
    if (message.equalsIgnoreCase("PRESS") || message.equalsIgnoreCase("TOGGLE")) {
      pressInverterButton();
    }
  }
  else if (String(topic) == topic_calibrate_command) {
    float actualVolts = message.toFloat();
    if (actualVolts > 0.0) {
      handleCalibration(actualVolts);
    }
  }
}

// ============================================
// MQTT CONNECTION
// ============================================
bool reconnectMQTT() {
  // Last Will so the broker marks us OFFLINE on unexpected disconnect
  if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password,
                         topic_status, 1, true, "OFFLINE")) {
    Serial.println("✓ MQTT connected");
    mqttConnectedAt = millis();

    mqttClient.subscribe(topic_power_command);
    mqttClient.subscribe(topic_calibrate_command);

    mqttClient.publish(topic_status, "ONLINE", true);
    mqttClient.publish(topic_relay_state, relayPulsing ? "ON" : "OFF", true);

    char scaleStr[12];
    dtostrf(voltageScale, 7, 4, scaleStr);
    mqttClient.publish(topic_cal_scale, scaleStr, true);

    publishHomeAssistantDiscovery();
    publishVoltage(true);
    publishTemperature(true);
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

  String deviceId = "chester_alt_inverter";

  // Device information (shared by all entities)
  String device = "\"device\":{";
  device += "\"identifiers\":[\"" + deviceId + "\"],";
  device += "\"name\":\"Alt Inverter Charger\",";
  device += "\"model\":\"ESP32-S2 Mini\",";
  device += "\"manufacturer\":\"Chester RV Control\",";
  device += "\"sw_version\":\"" + String(FIRMWARE_VERSION) + "\"";
  device += "}";

  // Availability (shared by all entities)
  String availability = "\"availability_topic\":\"" + String(topic_status) + "\",";
  availability += "\"payload_available\":\"ONLINE\",";
  availability += "\"payload_not_available\":\"OFFLINE\",";

  // Inverter power button (momentary toggle)
  String buttonTopic = "homeassistant/button/" + deviceId + "/power/config";
  String buttonConfig = "{";
  buttonConfig += "\"name\":\"Inverter Power\",";
  buttonConfig += "\"unique_id\":\"" + deviceId + "_power\",";
  buttonConfig += "\"command_topic\":\"" + String(topic_power_command) + "\",";
  buttonConfig += "\"payload_press\":\"PRESS\",";
  buttonConfig += "\"icon\":\"mdi:power\",";
  buttonConfig += availability;
  buttonConfig += device;
  buttonConfig += "}";
  mqttClient.publish(buttonTopic.c_str(), buttonConfig.c_str(), true);

  // Battery voltage sensor
  String voltTopic = "homeassistant/sensor/" + deviceId + "/battery_voltage/config";
  String voltConfig = "{";
  voltConfig += "\"name\":\"Inverter Battery Voltage\",";
  voltConfig += "\"unique_id\":\"" + deviceId + "_battery_voltage\",";
  voltConfig += "\"state_topic\":\"" + String(topic_battery_voltage) + "\",";
  voltConfig += "\"unit_of_measurement\":\"V\",";
  voltConfig += "\"device_class\":\"voltage\",";
  voltConfig += "\"state_class\":\"measurement\",";
  voltConfig += "\"suggested_display_precision\":2,";
  voltConfig += availability;
  voltConfig += device;
  voltConfig += "}";
  mqttClient.publish(voltTopic.c_str(), voltConfig.c_str(), true);

  // Relay state binary sensor (diagnostic - shows the 500ms pulse)
  String relayTopic = "homeassistant/binary_sensor/" + deviceId + "/relay/config";
  String relayConfig = "{";
  relayConfig += "\"name\":\"Inverter Relay\",";
  relayConfig += "\"unique_id\":\"" + deviceId + "_relay\",";
  relayConfig += "\"state_topic\":\"" + String(topic_relay_state) + "\",";
  relayConfig += "\"payload_on\":\"ON\",";
  relayConfig += "\"payload_off\":\"OFF\",";
  relayConfig += "\"icon\":\"mdi:electric-switch\",";
  relayConfig += "\"entity_category\":\"diagnostic\",";
  relayConfig += availability;
  relayConfig += device;
  relayConfig += "}";
  mqttClient.publish(relayTopic.c_str(), relayConfig.c_str(), true);

  // Raw ADC millivolts (diagnostic - useful for calibration)
  String rawTopic = "homeassistant/sensor/" + deviceId + "/battery_raw_mv/config";
  String rawConfig = "{";
  rawConfig += "\"name\":\"Battery ADC Raw\",";
  rawConfig += "\"unique_id\":\"" + deviceId + "_battery_raw_mv\",";
  rawConfig += "\"state_topic\":\"" + String(topic_battery_raw_mv) + "\",";
  rawConfig += "\"unit_of_measurement\":\"mV\",";
  rawConfig += "\"state_class\":\"measurement\",";
  rawConfig += "\"icon\":\"mdi:sine-wave\",";
  rawConfig += "\"entity_category\":\"diagnostic\",";
  rawConfig += availability;
  rawConfig += device;
  rawConfig += "}";
  mqttClient.publish(rawTopic.c_str(), rawConfig.c_str(), true);

  // Calibration scale (diagnostic)
  String scaleTopic = "homeassistant/sensor/" + deviceId + "/cal_scale/config";
  String scaleConfig = "{";
  scaleConfig += "\"name\":\"Voltage Cal Scale\",";
  scaleConfig += "\"unique_id\":\"" + deviceId + "_cal_scale\",";
  scaleConfig += "\"state_topic\":\"" + String(topic_cal_scale) + "\",";
  scaleConfig += "\"icon\":\"mdi:tune\",";
  scaleConfig += "\"entity_category\":\"diagnostic\",";
  scaleConfig += availability;
  scaleConfig += device;
  scaleConfig += "}";
  mqttClient.publish(scaleTopic.c_str(), scaleConfig.c_str(), true);

  // DS18B20 temperature sensor
  String tempTopic = "homeassistant/sensor/" + deviceId + "/temperature/config";
  String tempConfig = "{";
  tempConfig += "\"name\":\"Inverter Temperature\",";
  tempConfig += "\"unique_id\":\"" + deviceId + "_temperature\",";
  tempConfig += "\"state_topic\":\"" + String(topic_temperature) + "\",";
  tempConfig += "\"unit_of_measurement\":\"\\u00b0C\",";
  tempConfig += "\"device_class\":\"temperature\",";
  tempConfig += "\"state_class\":\"measurement\",";
  tempConfig += "\"suggested_display_precision\":1,";
  tempConfig += availability;
  tempConfig += device;
  tempConfig += "}";
  mqttClient.publish(tempTopic.c_str(), tempConfig.c_str(), true);

  // WiFi RSSI (diagnostic)
  String rssiTopic = "homeassistant/sensor/" + deviceId + "/rssi/config";
  String rssiConfig = "{";
  rssiConfig += "\"name\":\"WiFi RSSI\",";
  rssiConfig += "\"unique_id\":\"" + deviceId + "_rssi\",";
  rssiConfig += "\"state_topic\":\"" + String(topic_rssi) + "\",";
  rssiConfig += "\"unit_of_measurement\":\"dBm\",";
  rssiConfig += "\"device_class\":\"signal_strength\",";
  rssiConfig += "\"state_class\":\"measurement\",";
  rssiConfig += "\"entity_category\":\"diagnostic\",";
  rssiConfig += availability;
  rssiConfig += device;
  rssiConfig += "}";
  mqttClient.publish(rssiTopic.c_str(), rssiConfig.c_str(), true);

  Serial.println("✓ Home Assistant discovery published");
}

// ============================================
// STATUS
// ============================================
void sendStatus() {
  if (!mqttClient.connected()) return;

  mqttClient.publish(topic_status, "ONLINE", true);

  char rssiStr[8];
  itoa(WiFi.RSSI(), rssiStr, 10);
  mqttClient.publish(topic_rssi, rssiStr, true);
}

// ============================================
// SETUP
// ============================================
void setup() {
  // Drive the relay to its idle (open) state as early as possible
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  relayWrite(false);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n================================");
  Serial.println("AltInverterCharger " + String(FIRMWARE_VERSION));
  Serial.println("================================\n");

  // Initialize EEPROM and load calibration
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("✓ EEPROM initialized");
  loadConfigFromEEPROM();

  // ADC setup - 11dB attenuation gives ~0-2500mV usable range on the S2.
  // analogReadMilliVolts() applies the factory eFuse calibration.
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
  sampleVoltage();
  Serial.print("✓ ADC initialized - battery: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V (raw ");
  Serial.print(batteryRawMV);
  Serial.println("mV)");

  // Initialize DS18B20
  tempSensor.begin();
  if (tempSensor.getDeviceCount() > 0) {
    tempSensorFound = true;
    tempSensor.setResolution(12);
    tempSensor.setWaitForConversion(false); // Non-blocking conversions
    Serial.println("✓ DS18B20 sensor found");
  } else {
    Serial.println("⚠️ DS18B20 not found - will keep probing");
  }

  // Stored network settings (defaults from secrets.ini on first boot)
  ChesterNet::begin(otaHostname, FIRMWARE_VERSION);
  ssid = ChesterNet::wifiSsid();
  password = ChesterNet::wifiPassword();
  mqtt_server = ChesterNet::mqttHost();
  mqtt_port = ChesterNet::mqttPort();
  otaPassword = ChesterNet::devicePassword();
  ChesterNet::onUpdateStart([]() {
    // A browser upload blocks loop() - the relay pulse could not be ended, and a
    // stuck relay holds the inverter's power button down
    otaInProgress = true;
    relayPulsing = false;
    relayWrite(false);
  });
  ChesterNet::onUpdateEnd([](bool ok) { if (!ok) otaInProgress = false; });

  // Connect to WiFi
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 40) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n⚠️ WiFi connection failed");
  }

  // Initialize MQTT
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);

  // Initialize OTA
  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    // Make sure the relay can't get stuck closed during an update
    otaInProgress = true;
    relayPulsing = false;
    relayWrite(false);
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
    Serial.println("✓ OTA initialized");
  }

  Serial.println("\n✓ Setup complete\n");
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Always service the relay pulse first - a stuck-closed relay holds the
  // inverter's power button down, so this must never be starved
  handleRelayPulse();

  // Settings web page, browser firmware updates, setup hotspot
  ChesterNet::loop();

  // WiFi reconnection. One attempt every WIFI_KICK_MS - restarting it on every
  // pass never gave an association time to finish.
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiKick = 0;
    if (wifiLostSince == 0) wifiLostSince = millis();

    if (ChesterNet::wifiRetryAllowed() && (lastWifiKick == 0 || millis() - lastWifiKick >= WIFI_KICK_MS)) {
      lastWifiKick = millis();
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.begin(ssid, password);
      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 6) {
        delay(250);
        handleRelayPulse();
        Serial.print(".");
        attempts++;
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✓ WiFi reconnected");
      wifiLostSince = 0;
      if (!otaInitialized) {
        ArduinoOTA.begin();
        otaInitialized = true;
      }
    } else if (millis() - wifiLostSince > WIFI_RECOVERY_TIMEOUT && !ChesterNet::portalActive()) {
      // Not while the setup hotspot is up - it is how a new network gets entered
      Serial.println("\n⚠️ WiFi timeout - restarting...");
      relayWrite(false);
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
      } else if (millis() - mqttLostSince > MQTT_RECOVERY_TIMEOUT && !ChesterNet::portalActive()) {
        Serial.println("⚠️ MQTT timeout - restarting...");
        relayWrite(false);
        ESP.restart();
      }
    }
  } else {
    mqttLostSince = 0;
  }

  mqttClient.loop();

  if (otaInitialized) {
    ArduinoOTA.handle();
  }
  if (otaInProgress) return; // Skip everything else while updating

  // Sample battery voltage every second, publish on change or every 10s
  if (millis() - lastVoltageSample > VOLTAGE_SAMPLE_INTERVAL) {
    lastVoltageSample = millis();
    sampleVoltage();
    publishVoltage(false);
  }

  // DS18B20 temperature (non-blocking conversion)
  handleTemperature();

  // Periodic status update
  if (millis() - lastStatusUpdate > STATUS_INTERVAL) {
    lastStatusUpdate = millis();
    sendStatus();
  }

  delay(10);
}
