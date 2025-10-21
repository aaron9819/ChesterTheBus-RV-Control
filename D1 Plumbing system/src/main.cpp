#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

// ============================================================================
// D1 PLUMBING SYSTEM CONTROLLER - Chester the Bus RV
// ============================================================================
// Board Responsibilities:
// - Read temperature sensors (hydronic, fresh tank, grey tank, plumbing ambient)
// - Control critical safety systems (exhaust fan, tank heaters, cabinet lock)
// - Execute commands from GIGA R1 for non-critical systems
// - Maintain critical automation even if WiFi/GIGA is down
// - Report all sensor readings and relay states to MQTT
// ============================================================================

#define FIRMWARE_VERSION "v1.1.0"

// ----------------- WiFi Config -----------------
const char* ssid = "Chester IOT";
const char* wifiPassword = "2025Chester9894";

// ----------------- MQTT Config -----------------
const char* mqttServer = "192.168.8.1"; // GL-AR300M16 Router IP
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const char* mqttClientId = "D1Mini_Plumbing";

// ----------------- OTA Config -----------------
const char* otaHostname = "D1Mini-Plumbing";
const char* otaPassword = "Chester2025";  // Password for OTA updates

// ----------------- SENSOR TOPICS (Publish Only) -----------------
const char* Topic_Hydronic_Temp = "HydronicTemperature";
const char* Topic_FreshWtr_Temp = "FreshWaterTemperature";
const char* Topic_GreyWtr_Temp = "GreyWaterTemperature";
const char* Topic_Temp_Ambient = "PlumbingTempAmbient";
const char* Topic_Temp_Supply = "PlumbingTempSupplyManifold";
const char* Topic_Temp_Return = "PlumbingTempReturnManifold";

// ----------------- STATUS TOPICS (Publish Only) -----------------
const char* Topic_FreshWtrHeat_Status = "FreshWaterHeatStatus";
const char* Topic_GreyWtrHeat_Status = "GreyWaterHeatStatus";
const char* Topic_RearLoop_Status = "RearLoopStatus";
const char* Topic_EngineLoop_Status = "EngineLoopStatus";
const char* Topic_ExhaustFan_Status = "ExhaustFanStatus";
const char* Topic_DieselHtr_Status = "DieselHeaterStatus";

// ----------------- COMMAND TOPICS (Subscribe) -----------------
const char* Topic_FreshWtrHeat_Command = "FreshWaterHeatCommand";
const char* Topic_GreyWtrHeat_Command = "GreyWaterHeatCommand";
const char* Topic_RearLoop_Command = "RearLoopCommand";
const char* Topic_EngineLoop_Command = "EngineLoopCommand";
const char* Topic_ExhaustFan_Command = "ExhaustFanCommand";
const char* Topic_DieselHtr_Command = "DieselHeaterCommand";

// ----------------- MODE TOPICS (Subscribe) -----------------
const char* Topic_FreshWtrHeat_Mode = "FreshWaterHeatMode";       // AUTO/MANUAL
const char* Topic_GreyWtrHeat_Mode = "GreyWaterHeatMode";         // AUTO/MANUAL
const char* Topic_ExhaustFan_Mode = "ExhaustFanMode";             // AUTO/MANUAL

// ----------------- CONFIGURATION TOPICS (Subscribe & Publish) -----------------
const char* Topic_FreshWtrHeat_TempSet = "FreshWaterHeatTempSet";
const char* Topic_GreyWtrHeat_TempSet = "GreyWaterHeatTempSet";
const char* Topic_ExhaustFan_TempHigh = "ExhaustFanTempHigh";
const char* Topic_ExhaustFan_TempLow = "ExhaustFanTempLow";
const char* Topic_Config_Request = "D1PlumbingConfigRequest";
const char* Topic_Config_Response = "D1PlumbingConfigResponse";

// ----------------- DEVICE HEALTH TOPICS (Publish Only) -----------------
const char* Topic_D1Plumbing_Status = "D1PlumbingSystemStatus";
const char* Topic_D1Plumbing_Uptime = "D1PlumbingUptime";
const char* Topic_D1Plumbing_WiFi_RSSI = "D1PlumbingWiFiSignal";
const char* Topic_Firmware_Version = "D1PlumbingFirmwareVersion";

// ----------------- ALERT TOPICS (Publish Only) -----------------
const char* Topic_Alert_Freeze_Warning = "AlertFreezeWarning";
const char* Topic_Alert_High_Temp = "AlertHighTemp";
const char* Topic_Alert_System_Error = "AlertSystemError";

// ----------------- OTA TOPICS (Subscribe & Publish) -----------------
const char* Topic_OTA_Command = "D1PlumbingOTACommand";
const char* Topic_OTA_Status = "D1PlumbingOTAStatus";
const char* Topic_OTA_Progress = "D1PlumbingOTAProgress";

// ----------------- HARDWARE PIN DEFINITIONS -----------------
#define ONE_WIRE_BUS D1              // DS18B20 temperature sensors data pin
#define FRESH_WATER_HEATER_PIN D0    // Relay for fresh water tank heater
#define GREY_WATER_HEATER_PIN D2     // Relay for grey water tank heater
#define EXHAUST_FAN_PIN D3           // Relay for exhaust fan
#define REAR_LOOP_VALVE_PIN D5       // Relay for rear loop valve
#define ENGINE_LOOP_VALVE_PIN D6     // Relay for engine loop valve
#define DIESEL_HEATER_PIN D7         // Relay for diesel heater control (relay 1) white wire
#define DIESEL_HEATER_2_PIN D4       // Relay for diesel heater control (relay 2) blue wire
#define DIESEL_HEATER_3_PIN D8       // Relay for diesel heater control (relay 3) yellow wire
// NOTE: Hot water solenoid & flow sensor moved to separate board
// NOTE: BME280 sensor moved to separate dedicated board

// ----------------- SENSOR SETUP -----------------
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Sensor indices (adjust based on your physical sensor addresses)
#define SENSOR_HYDRONIC 0
#define SENSOR_FRESH_WATER 1
#define SENSOR_GREY_WATER 2
#define SENSOR_AMBIENT 3
#define SENSOR_SUPPLY_MANIFOLD 4
#define SENSOR_RETURN_MANIFOLD 5

// ----------------- MQTT CLIENT -----------------
WiFiClient espClient;
PubSubClient client(espClient);

// ----------------- ADJUSTABLE CONFIGURATION (can be changed via MQTT) -----------------
float freshWaterTempTarget = 40.0;    // °F - Fresh water anti-freeze temp
float greyWaterTempTarget = 35.0;     // °F - Grey water anti-freeze temp
float exhaustFanTempHigh = 85.0;      // °F - Fan turns ON above this
float exhaustFanTempLow = 75.0;       // °F - Fan turns OFF below this (hysteresis)

// Temperature thresholds for alerts
#define FREEZE_WARNING_TEMP 35.0      // °F
#define HIGH_TEMP_WARNING 180.0       // °F

// ----------------- EEPROM CONFIGURATION -----------------
#define EEPROM_SIZE 512               // Bytes to allocate for EEPROM emulation
#define EEPROM_MAGIC 0xA5C3           // Magic number to validate EEPROM data
#define EEPROM_VERSION 1              // Version number for config structure

// Configuration structure stored in EEPROM
struct ConfigData {
  uint16_t magic;                     // Magic number for validation
  uint8_t version;                    // Version number
  float freshWaterTempTarget;
  float greyWaterTempTarget;
  float exhaustFanTempHigh;
  float exhaustFanTempLow;
  uint16_t checksum;                  // Simple checksum for data integrity
};

// ----------------- STATE VARIABLES -----------------
// Fresh water heater
bool freshWaterHeaterOn = false;
String freshWaterMode = "AUTO";       // AUTO or MANUAL

// Grey water heater
bool greyWaterHeaterOn = false;
String greyWaterMode = "AUTO";        // AUTO or MANUAL

// Exhaust fan
bool exhaustFanOn = false;
String exhaustFanMode = "AUTO";       // AUTO or MANUAL

// Rear loop valve (GIGA controlled only)
bool rearLoopOpen = false;

// Engine loop valve (GIGA controlled only)
bool engineLoopOpen = false;

// Diesel heater (GIGA controlled only) - OFF, MID, or HIGH
String dieselHeaterState = "OFF";

// Timing variables
unsigned long lastSensorRead = 0;
unsigned long lastStatusPublish = 0;
unsigned long lastHealthPublish = 0;
const unsigned long SENSOR_READ_INTERVAL = 5000;      // 5 seconds
const unsigned long STATUS_PUBLISH_INTERVAL = 10000;  // 10 seconds
const unsigned long HEALTH_PUBLISH_INTERVAL = 60000;  // 60 seconds

// Sensor readings
float hydronicTemp = -127.0;
float freshWaterTemp = -127.0;
float greyWaterTemp = -127.0;
float ambientTemp = -127.0;
float supplyManifoldTemp = -127.0;
float returnManifoldTemp = -127.0;

// Alert states
bool freezeWarningActive = false;
bool highTempWarningActive = false;

// OTA update flag
bool otaInProgress = false;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void setupWifi();
void setupOTA();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void subscribeToTopics();
void readTemperatures();
void publishTemperatures();
void publishStatus();
void publishHealth();
void publishConfiguration();
void handleCriticalAutomation();
void checkAlerts();
void controlFreshWaterHeater(bool turnOn);
void controlGreyWaterHeater(bool turnOn);
void controlExhaustFan(bool turnOn);
void controlRearLoopValve(bool open);
void controlEngineLoopValve(bool open);
void controlDieselHeater(String state);
void saveConfigToEEPROM();
void loadConfigFromEEPROM();
uint16_t calculateChecksum(ConfigData* config);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n");
  Serial.println("============================================");
  Serial.println("  D1 PLUMBING SYSTEM CONTROLLER");
  Serial.println("  Chester the Bus RV Control System");
  Serial.println("  Firmware: " + String(FIRMWARE_VERSION));
  #ifdef BUILD_DATE
  Serial.print("  Build: ");
  Serial.print(BUILD_DATE);
  Serial.print(" ");
  Serial.println(BUILD_TIME);
  #endif
  Serial.println("============================================\n");

  // Initialize EEPROM and load saved configuration
  EEPROM.begin(EEPROM_SIZE);
  loadConfigFromEEPROM();

  // Configure relay pins as outputs (active HIGH for most relay modules)
  pinMode(FRESH_WATER_HEATER_PIN, OUTPUT);
  pinMode(GREY_WATER_HEATER_PIN, OUTPUT);
  pinMode(EXHAUST_FAN_PIN, OUTPUT);
  pinMode(REAR_LOOP_VALVE_PIN, OUTPUT);
  pinMode(ENGINE_LOOP_VALVE_PIN, OUTPUT);
  pinMode(DIESEL_HEATER_PIN, OUTPUT);
  pinMode(DIESEL_HEATER_2_PIN, OUTPUT);
  pinMode(DIESEL_HEATER_3_PIN, OUTPUT);

  // Initialize all relays to OFF (active LOW relays, so HIGH = OFF)
  digitalWrite(FRESH_WATER_HEATER_PIN, HIGH);
  digitalWrite(GREY_WATER_HEATER_PIN, HIGH);
  digitalWrite(EXHAUST_FAN_PIN, HIGH);
  digitalWrite(REAR_LOOP_VALVE_PIN, HIGH);
  digitalWrite(ENGINE_LOOP_VALVE_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_3_PIN, HIGH);

  Serial.println("✓ Relay pins configured");
  Serial.println("  Note: Cabinet lock moved to separate dedicated board");
  Serial.println("  Note: BME280 sensor moved to separate dedicated board");
  Serial.println("  Note: Hot water solenoid & flow sensor moved to separate board");
  Serial.println("  Note: Diesel heater uses 3-state control (OFF/MID/HIGH)");
  Serial.println("    OFF = All pins HIGH");
  Serial.println("    MID = Pins 1+2 HIGH, Pin 3 HIGH");
  Serial.println("    HIGH = Pins 1+3 HIGH, Pin 2 HIGH");

  // Start WiFi connection
  setupWifi();

  // Setup OTA (Over-The-Air updates)
  setupOTA();

  // Configure MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);
  client.setBufferSize(512); // Increase buffer for larger messages

  // Initialize DS18B20 temperature sensors
  sensors.begin();
  int sensorCount = sensors.getDeviceCount();
  Serial.print("✓ Found ");
  Serial.print(sensorCount);
  Serial.println(" DS18B20 temperature sensor(s)");

  if (sensorCount == 0) {
    Serial.println("⚠ WARNING: No temperature sensors detected!");
  }

  // Set sensor resolution (9-12 bits, higher = more accurate but slower)
  sensors.setResolution(12);

  Serial.println("\n✓ Initialization complete");
  Serial.println("============================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Handle OTA updates (must be called frequently)
  ArduinoOTA.handle();

  unsigned long currentMillis = millis();

  // Maintain MQTT connection (skip if OTA is in progress)
  if (!otaInProgress) {
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();

    // Read temperature sensors periodically
    if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
      lastSensorRead = currentMillis;
      readTemperatures();
      publishTemperatures();
      handleCriticalAutomation();
      checkAlerts();
    }    // Publish status periodically
    if (currentMillis - lastStatusPublish >= STATUS_PUBLISH_INTERVAL) {
      lastStatusPublish = currentMillis;
      publishStatus();
    }

    // Publish health metrics periodically
    if (currentMillis - lastHealthPublish >= HEALTH_PUBLISH_INTERVAL) {
      lastHealthPublish = currentMillis;
      publishHealth();
    }
  }
}

// ============================================================================
// WiFi SETUP
// ============================================================================
void setupWifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  Serial.print("SSID: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifiPassword);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected");
    Serial.print("  IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("  Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\n✗ WiFi connection failed!");
    Serial.println("  System will continue with local automation only");
  }
}

// ============================================================================
// OTA SETUP
// ============================================================================
void setupOTA() {
  // Set OTA hostname
  ArduinoOTA.setHostname(otaHostname);

  // Set OTA password (optional but recommended)
  ArduinoOTA.setPassword(otaPassword);

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Callback when OTA starts
  ArduinoOTA.onStart([]() {
    otaInProgress = true;

    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    Serial.println("\n🔄 OTA Update Starting...");
    Serial.println("   Type: " + type);

    // Publish OTA status via MQTT
    client.publish(Topic_OTA_Status, "UPDATING");
    client.publish(Topic_OTA_Progress, "0");

    // Turn off all relays for safety during update (HIGH = OFF for active LOW relays)
    digitalWrite(FRESH_WATER_HEATER_PIN, HIGH);
    digitalWrite(GREY_WATER_HEATER_PIN, HIGH);
    digitalWrite(EXHAUST_FAN_PIN, HIGH);
    controlDieselHeater("OFF");

    Serial.println("   Safety: All relays disabled during update");
  });

  // Callback when OTA ends
  ArduinoOTA.onEnd([]() {
    Serial.println("\n✓ OTA Update Complete!");
    client.publish(Topic_OTA_Status, "SUCCESS");
    client.publish(Topic_OTA_Progress, "100");
    Serial.println("   Rebooting...");
  });

  // Callback for update progress
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percentComplete = (progress / (total / 100));

    // Only print every 10% to avoid spam
    static int lastPercent = -1;
    if (percentComplete != lastPercent && percentComplete % 10 == 0) {
      Serial.printf("   Progress: %u%%\n", percentComplete);

      // Publish progress to MQTT
      char progressStr[8];
      snprintf(progressStr, sizeof(progressStr), "%u", percentComplete);
      client.publish(Topic_OTA_Progress, progressStr);

      lastPercent = percentComplete;
    }
  });

  // Callback when OTA error occurs
  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;

    Serial.printf("\n✗ OTA Error[%u]: ", error);

    String errorMsg;
    if (error == OTA_AUTH_ERROR) {
      errorMsg = "AUTH_FAILED";
      Serial.println("Authentication Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      errorMsg = "BEGIN_FAILED";
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      errorMsg = "CONNECT_FAILED";
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      errorMsg = "RECEIVE_FAILED";
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      errorMsg = "END_FAILED";
      Serial.println("End Failed");
    }

    client.publish(Topic_OTA_Status, errorMsg.c_str());

    // Re-enable safety automation after failed update
    Serial.println("   Resuming normal operation after OTA failure");
  });

  // Start OTA service
  ArduinoOTA.begin();

  Serial.println("✓ OTA (Over-The-Air) update enabled");
  Serial.print("  Hostname: ");
  Serial.print(otaHostname);
  Serial.println(".local");
  Serial.print("  Password: ");
  Serial.println(otaPassword);
  Serial.println("  Ready for wireless firmware updates!");
}

// ============================================================================
// MQTT CONNECTION
// ============================================================================
void reconnectMQTT() {
  static unsigned long lastAttempt = 0;
  unsigned long currentMillis = millis();

  // Only attempt reconnection every 5 seconds to avoid blocking
  if (currentMillis - lastAttempt < 5000) {
    return;
  }
  lastAttempt = currentMillis;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping MQTT reconnect");
    return;
  }

  Serial.print("Connecting to MQTT broker at ");
  Serial.print(mqttServer);
  Serial.print(":");
  Serial.print(mqttPort);
  Serial.print("...");

  // Attempt MQTT connection
  if (client.connect(mqttClientId, mqttUser, mqttPassword)) {
    Serial.println(" connected!");

    // Publish online status
    client.publish(Topic_D1Plumbing_Status, "ONLINE", true);
    client.publish(Topic_Firmware_Version, FIRMWARE_VERSION, true);

    // Subscribe to all command and configuration topics
    subscribeToTopics();

    // Publish initial configuration and status
    publishConfiguration();
    publishStatus();
    publishHealth();

    Serial.println("✓ MQTT setup complete");
  } else {
    Serial.print(" failed, rc=");
    Serial.println(client.state());
    Serial.println("  System will continue with local automation only");
  }
}

// ============================================================================
// SUBSCRIBE TO MQTT TOPICS
// ============================================================================
void subscribeToTopics() {
  // Command topics
  client.subscribe(Topic_FreshWtrHeat_Command);
  client.subscribe(Topic_GreyWtrHeat_Command);
  client.subscribe(Topic_ExhaustFan_Command);
  client.subscribe(Topic_RearLoop_Command);
  client.subscribe(Topic_EngineLoop_Command);
  client.subscribe(Topic_DieselHtr_Command);

  // Mode topics
  client.subscribe(Topic_FreshWtrHeat_Mode);
  client.subscribe(Topic_GreyWtrHeat_Mode);
  client.subscribe(Topic_ExhaustFan_Mode);

  // Configuration topics
  client.subscribe(Topic_FreshWtrHeat_TempSet);
  client.subscribe(Topic_GreyWtrHeat_TempSet);
  client.subscribe(Topic_ExhaustFan_TempHigh);
  client.subscribe(Topic_ExhaustFan_TempLow);
  client.subscribe(Topic_Config_Request);

  // OTA topics
  client.subscribe(Topic_OTA_Command);

  Serial.println("✓ Subscribed to command topics");
}

// ============================================================================
// MQTT CALLBACK (Handle incoming messages)
// ============================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();

  Serial.print("MQTT RX [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  // ========== FRESH WATER HEATER COMMANDS ==========
  if (String(topic) == Topic_FreshWtrHeat_Command) {
    if (message == "ON") {
      controlFreshWaterHeater(true);
    } else if (message == "OFF") {
      controlFreshWaterHeater(false);
    }
  }

  if (String(topic) == Topic_FreshWtrHeat_Mode) {
    if (message == "AUTO" || message == "MANUAL") {
      freshWaterMode = message;
      Serial.print("Fresh water heater mode: ");
      Serial.println(freshWaterMode);
    }
  }

  if (String(topic) == Topic_FreshWtrHeat_TempSet) {
    float newTemp = message.toFloat();
    if (newTemp >= 32.0 && newTemp <= 60.0) {
      freshWaterTempTarget = newTemp;
      Serial.print("Fresh water target temp set to: ");
      Serial.print(freshWaterTempTarget);
      Serial.println(" °F");
      saveConfigToEEPROM();
      publishConfiguration();
    }
  }

  // ========== GREY WATER HEATER COMMANDS ==========
  if (String(topic) == Topic_GreyWtrHeat_Command) {
    if (message == "ON") {
      controlGreyWaterHeater(true);
    } else if (message == "OFF") {
      controlGreyWaterHeater(false);
    }
  }

  if (String(topic) == Topic_GreyWtrHeat_Mode) {
    if (message == "AUTO" || message == "MANUAL") {
      greyWaterMode = message;
      Serial.print("Grey water heater mode: ");
      Serial.println(greyWaterMode);
    }
  }

  if (String(topic) == Topic_GreyWtrHeat_TempSet) {
    float newTemp = message.toFloat();
    if (newTemp >= 32.0 && newTemp <= 60.0) {
      greyWaterTempTarget = newTemp;
      Serial.print("Grey water target temp set to: ");
      Serial.print(greyWaterTempTarget);
      Serial.println(" °F");
      saveConfigToEEPROM();
      publishConfiguration();
    }
  }

  // ========== EXHAUST FAN COMMANDS ==========
  if (String(topic) == Topic_ExhaustFan_Command) {
    if (message == "ON") {
      controlExhaustFan(true);
    } else if (message == "OFF") {
      controlExhaustFan(false);
    }
  }

  if (String(topic) == Topic_ExhaustFan_Mode) {
    if (message == "AUTO" || message == "MANUAL") {
      exhaustFanMode = message;
      Serial.print("Exhaust fan mode: ");
      Serial.println(exhaustFanMode);
    }
  }

  if (String(topic) == Topic_ExhaustFan_TempHigh) {
    float newTemp = message.toFloat();
    if (newTemp >= 60.0 && newTemp <= 120.0) {
      exhaustFanTempHigh = newTemp;
      Serial.print("Exhaust fan HIGH temp set to: ");
      Serial.print(exhaustFanTempHigh);
      Serial.println(" °F");
      saveConfigToEEPROM();
      publishConfiguration();
    }
  }

  if (String(topic) == Topic_ExhaustFan_TempLow) {
    float newTemp = message.toFloat();
    if (newTemp >= 60.0 && newTemp <= 120.0) {
      exhaustFanTempLow = newTemp;
      Serial.print("Exhaust fan LOW temp set to: ");
      Serial.print(exhaustFanTempLow);
      Serial.println(" °F");
      saveConfigToEEPROM();
      publishConfiguration();
    }
  }

  // ========== REAR LOOP VALVE COMMANDS ==========
  if (String(topic) == Topic_RearLoop_Command) {
    if (message == "OPEN") {
      controlRearLoopValve(true);
    } else if (message == "CLOSE") {
      controlRearLoopValve(false);
    }
  }

  // ========== ENGINE LOOP VALVE COMMANDS ==========
  if (String(topic) == Topic_EngineLoop_Command) {
    if (message == "OPEN") {
      controlEngineLoopValve(true);
    } else if (message == "CLOSE") {
      controlEngineLoopValve(false);
    }
  }

  // ========== DIESEL HEATER COMMANDS ==========
  if (String(topic) == Topic_DieselHtr_Command) {
    if (message == "OFF" || message == "MID" || message == "HIGH") {
      controlDieselHeater(message);
    }
  }

  // ========== HOT WATER SOLENOID COMMANDS ==========
  if (String(topic) == Topic_Config_Request) {
    publishConfiguration();
  }

  // ========== OTA UPDATE COMMANDS ==========
  if (String(topic) == Topic_OTA_Command) {
    if (message == "REBOOT") {
      Serial.println("🔄 Reboot requested via MQTT");
      client.publish(Topic_OTA_Status, "REBOOTING");
      delay(1000);
      ESP.restart();
    } else {
      client.publish(Topic_OTA_Status, "USE_PLATFORMIO_OTA");
      Serial.println("ℹ️  OTA updates via MQTT not implemented.");
      Serial.println("   Use PlatformIO OTA upload instead:");
      Serial.println("   pio run --target upload --upload-port " + String(otaHostname) + ".local");
    }
  }
}

// ============================================================================
// READ TEMPERATURE SENSORS
// ============================================================================
void readTemperatures() {
  sensors.requestTemperatures();

  // Read each sensor and convert to Fahrenheit
  float tempC;

  tempC = sensors.getTempCByIndex(SENSOR_HYDRONIC);
  hydronicTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  tempC = sensors.getTempCByIndex(SENSOR_FRESH_WATER);
  freshWaterTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  tempC = sensors.getTempCByIndex(SENSOR_GREY_WATER);
  greyWaterTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  tempC = sensors.getTempCByIndex(SENSOR_AMBIENT);
  ambientTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  tempC = sensors.getTempCByIndex(SENSOR_SUPPLY_MANIFOLD);
  supplyManifoldTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  tempC = sensors.getTempCByIndex(SENSOR_RETURN_MANIFOLD);
  returnManifoldTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;
}// ============================================================================
// PUBLISH TEMPERATURE READINGS
// ============================================================================
void publishTemperatures() {
  char tempStr[8];

  if (hydronicTemp > -999.0) {
    dtostrf(hydronicTemp, 4, 2, tempStr);
    client.publish(Topic_Hydronic_Temp, tempStr);
    Serial.print("Hydronic Temp: ");
    Serial.print(tempStr);
    Serial.println(" °F");
  }

  if (freshWaterTemp > -999.0) {
    dtostrf(freshWaterTemp, 4, 2, tempStr);
    client.publish(Topic_FreshWtr_Temp, tempStr);
    Serial.print("Fresh Water Temp: ");
    Serial.print(tempStr);
    Serial.println(" °F");
  }

  if (greyWaterTemp > -999.0) {
    dtostrf(greyWaterTemp, 4, 2, tempStr);
    client.publish(Topic_GreyWtr_Temp, tempStr);
    Serial.print("Grey Water Temp: ");
    Serial.print(tempStr);
    Serial.println(" °F");
  }

  if (ambientTemp > -999.0) {
    dtostrf(ambientTemp, 4, 2, tempStr);
    client.publish(Topic_Temp_Ambient, tempStr);
    Serial.print("Ambient Temp: ");
    Serial.print(tempStr);
    Serial.println(" °F");
  }

  if (supplyManifoldTemp > -999.0) {
    dtostrf(supplyManifoldTemp, 4, 2, tempStr);
    client.publish(Topic_Temp_Supply, tempStr);
    Serial.print("Supply Manifold Temp: ");
    Serial.print(tempStr);
    Serial.println(" °F");
  }

  if (returnManifoldTemp > -999.0) {
    dtostrf(returnManifoldTemp, 4, 2, tempStr);
    client.publish(Topic_Temp_Return, tempStr);
    Serial.print("Return Manifold Temp: ");
    Serial.print(tempStr);
    Serial.println(" °F");
  }
}// ============================================================================
// CRITICAL AUTOMATION (Runs even if GIGA is offline)
// ============================================================================
void handleCriticalAutomation() {

  // ========== FRESH WATER TANK HEATER (AUTO MODE) ==========
  if (freshWaterMode == "AUTO" && freshWaterTemp > -999.0) {
    if (!freshWaterHeaterOn && freshWaterTemp < freshWaterTempTarget) {
      controlFreshWaterHeater(true);
      Serial.print("🔥 AUTO: Fresh water heater ON (");
      Serial.print(freshWaterTemp);
      Serial.println(" °F)");
    }
    else if (freshWaterHeaterOn && freshWaterTemp > (freshWaterTempTarget + 5.0)) {
      // 5°F hysteresis to prevent rapid cycling
      controlFreshWaterHeater(false);
      Serial.print("❄️ AUTO: Fresh water heater OFF (");
      Serial.print(freshWaterTemp);
      Serial.println(" °F)");
    }
  }

  // ========== GREY WATER TANK HEATER (AUTO MODE) ==========
  if (greyWaterMode == "AUTO" && greyWaterTemp > -999.0) {
    if (!greyWaterHeaterOn && greyWaterTemp < greyWaterTempTarget) {
      controlGreyWaterHeater(true);
      Serial.print("🔥 AUTO: Grey water heater ON (");
      Serial.print(greyWaterTemp);
      Serial.println(" °F)");
    }
    else if (greyWaterHeaterOn && greyWaterTemp > (greyWaterTempTarget + 5.0)) {
      // 5°F hysteresis to prevent rapid cycling
      controlGreyWaterHeater(false);
      Serial.print("❄️ AUTO: Grey water heater OFF (");
      Serial.print(greyWaterTemp);
      Serial.println(" °F)");
    }
  }

  // ========== EXHAUST FAN (AUTO MODE) ==========
  if (exhaustFanMode == "AUTO" && ambientTemp > -999.0) {
    if (!exhaustFanOn && ambientTemp > exhaustFanTempHigh) {
      controlExhaustFan(true);
      Serial.print("💨 AUTO: Exhaust fan ON (");
      Serial.print(ambientTemp);
      Serial.println(" °F)");
    }
    else if (exhaustFanOn && ambientTemp < exhaustFanTempLow) {
      controlExhaustFan(false);
      Serial.print("💨 AUTO: Exhaust fan OFF (");
      Serial.print(ambientTemp);
      Serial.println(" °F)");
    }
  }
}

// ============================================================================
// CHECK FOR ALERT CONDITIONS
// ============================================================================
void checkAlerts() {
  // Check for freeze warning (any tank below 35°F)
  bool freezeCondition = false;
  if (freshWaterTemp > -999.0 && freshWaterTemp < FREEZE_WARNING_TEMP) freezeCondition = true;
  if (greyWaterTemp > -999.0 && greyWaterTemp < FREEZE_WARNING_TEMP) freezeCondition = true;
  if (ambientTemp > -999.0 && ambientTemp < FREEZE_WARNING_TEMP) freezeCondition = true;
  if (supplyManifoldTemp > -999.0 && supplyManifoldTemp < FREEZE_WARNING_TEMP) freezeCondition = true;
  if (returnManifoldTemp > -999.0 && returnManifoldTemp < FREEZE_WARNING_TEMP) freezeCondition = true;

  if (freezeCondition && !freezeWarningActive) {
    client.publish(Topic_Alert_Freeze_Warning, "true");
    freezeWarningActive = true;
    Serial.println("⚠️ ALERT: Freeze warning active!");
  } else if (!freezeCondition && freezeWarningActive) {
    client.publish(Topic_Alert_Freeze_Warning, "false");
    freezeWarningActive = false;
    Serial.println("✓ Freeze warning cleared");
  }

  // Check for high temperature warning (hydronic system overheating)
  if (hydronicTemp > -999.0 && hydronicTemp > HIGH_TEMP_WARNING && !highTempWarningActive) {
    client.publish(Topic_Alert_High_Temp, "true");
    highTempWarningActive = true;
    Serial.println("⚠️ ALERT: High temperature warning!");
  } else if (hydronicTemp < (HIGH_TEMP_WARNING - 10.0) && highTempWarningActive) {
    client.publish(Topic_Alert_High_Temp, "false");
    highTempWarningActive = false;
    Serial.println("✓ High temperature warning cleared");
  }
}

// ============================================================================
// PUBLISH STATUS
// ============================================================================
void publishStatus() {
  client.publish(Topic_FreshWtrHeat_Status, freshWaterHeaterOn ? "ON" : "OFF");
  client.publish(Topic_GreyWtrHeat_Status, greyWaterHeaterOn ? "ON" : "OFF");
  client.publish(Topic_ExhaustFan_Status, exhaustFanOn ? "ON" : "OFF");
  client.publish(Topic_RearLoop_Status, rearLoopOpen ? "OPEN" : "CLOSED");
  client.publish(Topic_EngineLoop_Status, engineLoopOpen ? "OPEN" : "CLOSED");
  client.publish(Topic_DieselHtr_Status, dieselHeaterState.c_str());
}

// ============================================================================
// PUBLISH HEALTH METRICS
// ============================================================================
void publishHealth() {
  // Uptime in seconds
  char uptimeStr[16];
  unsigned long uptime = millis() / 1000;
  snprintf(uptimeStr, sizeof(uptimeStr), "%lu", uptime);
  client.publish(Topic_D1Plumbing_Uptime, uptimeStr);

  // WiFi signal strength
  char rssiStr[8];
  snprintf(rssiStr, sizeof(rssiStr), "%d", WiFi.RSSI());
  client.publish(Topic_D1Plumbing_WiFi_RSSI, rssiStr);

  // System status
  client.publish(Topic_D1Plumbing_Status, "ONLINE");

  Serial.print("📊 Health: Uptime=");
  Serial.print(uptime);
  Serial.print("s, RSSI=");
  Serial.print(rssiStr);
  Serial.println(" dBm");
}

// ============================================================================
// PUBLISH CONFIGURATION
// ============================================================================
void publishConfiguration() {
  char configMsg[256];
  snprintf(configMsg, sizeof(configMsg),
           "{\"freshTarget\":%.1f,\"greyTarget\":%.1f,\"fanHigh\":%.1f,\"fanLow\":%.1f}",
           freshWaterTempTarget, greyWaterTempTarget, exhaustFanTempHigh, exhaustFanTempLow);
  client.publish(Topic_Config_Response, configMsg);
  Serial.print("📝 Config published: ");
  Serial.println(configMsg);
}

// ============================================================================
// CONTROL FUNCTIONS
// ============================================================================

void controlFreshWaterHeater(bool turnOn) {
  digitalWrite(FRESH_WATER_HEATER_PIN, turnOn ? LOW : HIGH); // Active LOW
  freshWaterHeaterOn = turnOn;
  client.publish(Topic_FreshWtrHeat_Status, turnOn ? "ON" : "OFF");
}

void controlGreyWaterHeater(bool turnOn) {
  digitalWrite(GREY_WATER_HEATER_PIN, turnOn ? LOW : HIGH); // Active LOW
  greyWaterHeaterOn = turnOn;
  client.publish(Topic_GreyWtrHeat_Status, turnOn ? "ON" : "OFF");
}

void controlExhaustFan(bool turnOn) {
  digitalWrite(EXHAUST_FAN_PIN, turnOn ? LOW : HIGH); // Active LOW
  exhaustFanOn = turnOn;
  client.publish(Topic_ExhaustFan_Status, turnOn ? "ON" : "OFF");
}

void controlRearLoopValve(bool open) {
  digitalWrite(REAR_LOOP_VALVE_PIN, open ? LOW : HIGH); // Active LOW
  rearLoopOpen = open;
  client.publish(Topic_RearLoop_Status, open ? "OPEN" : "CLOSED");
}

void controlEngineLoopValve(bool open) {
  digitalWrite(ENGINE_LOOP_VALVE_PIN, open ? LOW : HIGH); // Active LOW
  engineLoopOpen = open;
  client.publish(Topic_EngineLoop_Status, open ? "OPEN" : "CLOSED");
}

void controlDieselHeater(String state) {
  dieselHeaterState = state;

  if (state == "OFF") {
    // OFF = All pins HIGH (relays off - active LOW)
    digitalWrite(DIESEL_HEATER_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, HIGH);
    Serial.println("🔥 Diesel Heater: OFF (all relays off)");
  }
  else if (state == "MID") {
    // MID = Relays 1+2 ON (pins LOW), Relay 3 OFF (pin HIGH)
    digitalWrite(DIESEL_HEATER_PIN, LOW);     // Relay 1 ON (white wire)
    digitalWrite(DIESEL_HEATER_2_PIN, LOW);   // Relay 2 ON (blue wire)
    digitalWrite(DIESEL_HEATER_3_PIN, HIGH);  // Relay 3 OFF (yellow wire)
    Serial.println("🔥 Diesel Heater: MID (relays 1+2 active)");
  }
  else if (state == "HIGH") {
    // HIGH = Relays 1+3 ON (pins LOW), Relay 2 OFF (pin HIGH)
    digitalWrite(DIESEL_HEATER_PIN, LOW);     // Relay 1 ON (white wire)
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);  // Relay 2 OFF (blue wire)
    digitalWrite(DIESEL_HEATER_3_PIN, LOW);   // Relay 3 ON (yellow wire)
    Serial.println("🔥 Diesel Heater: HIGH (relays 1+3 active)");
  }

  client.publish(Topic_DieselHtr_Status, dieselHeaterState.c_str());
}

// ============================================================================
// EEPROM CONFIGURATION PERSISTENCE
// ============================================================================

// Calculate simple checksum for data integrity
uint16_t calculateChecksum(ConfigData* config) {
  uint16_t checksum = 0;
  uint8_t* data = (uint8_t*)config;
  // Calculate checksum over all bytes except the checksum field itself
  for (size_t i = 0; i < sizeof(ConfigData) - sizeof(uint16_t); i++) {
    checksum += data[i];
  }
  return checksum;
}

// Save current configuration to EEPROM
void saveConfigToEEPROM() {
  ConfigData config;
  config.magic = EEPROM_MAGIC;
  config.version = EEPROM_VERSION;
  config.freshWaterTempTarget = freshWaterTempTarget;
  config.greyWaterTempTarget = greyWaterTempTarget;
  config.exhaustFanTempHigh = exhaustFanTempHigh;
  config.exhaustFanTempLow = exhaustFanTempLow;
  config.checksum = calculateChecksum(&config);

  EEPROM.put(0, config);
  EEPROM.commit();

  Serial.println("💾 Configuration saved to EEPROM");
}

// Load configuration from EEPROM (if valid)
void loadConfigFromEEPROM() {
  ConfigData config;
  EEPROM.get(0, config);

  // Validate magic number and version
  if (config.magic != EEPROM_MAGIC) {
    Serial.println("⚠️  EEPROM: No valid config found (wrong magic number)");
    Serial.println("   Using default temperature settings");
    return;
  }

  if (config.version != EEPROM_VERSION) {
    Serial.println("⚠️  EEPROM: Config version mismatch");
    Serial.println("   Using default temperature settings");
    return;
  }

  // Validate checksum
  uint16_t calculatedChecksum = calculateChecksum(&config);
  if (config.checksum != calculatedChecksum) {
    Serial.println("⚠️  EEPROM: Checksum mismatch (data corrupted)");
    Serial.println("   Using default temperature settings");
    return;
  }

  // Validate ranges (sanity check)
  if (config.freshWaterTempTarget < 32.0 || config.freshWaterTempTarget > 60.0 ||
      config.greyWaterTempTarget < 32.0 || config.greyWaterTempTarget > 60.0 ||
      config.exhaustFanTempHigh < 60.0 || config.exhaustFanTempHigh > 120.0 ||
      config.exhaustFanTempLow < 60.0 || config.exhaustFanTempLow > 120.0) {
    Serial.println("⚠️  EEPROM: Config values out of range");
    Serial.println("   Using default temperature settings");
    return;
  }

  // All validation passed - load the configuration
  freshWaterTempTarget = config.freshWaterTempTarget;
  greyWaterTempTarget = config.greyWaterTempTarget;
  exhaustFanTempHigh = config.exhaustFanTempHigh;
  exhaustFanTempLow = config.exhaustFanTempLow;

  Serial.println("✓ Configuration loaded from EEPROM");
  Serial.print("  Fresh Water Target: ");
  Serial.print(freshWaterTempTarget);
  Serial.println(" °F");
  Serial.print("  Grey Water Target: ");
  Serial.print(greyWaterTempTarget);
  Serial.println(" °F");
  Serial.print("  Exhaust Fan High: ");
  Serial.print(exhaustFanTempHigh);
  Serial.println(" °F");
  Serial.print("  Exhaust Fan Low: ");
  Serial.print(exhaustFanTempLow);
  Serial.println(" °F");
}
