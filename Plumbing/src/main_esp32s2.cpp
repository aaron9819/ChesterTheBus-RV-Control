#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

// ============================================================================
// ESP32-S2 MINI PLUMBING & ENVIRONMENT CONTROLLER - Chester the Bus RV
// ============================================================================
// Board Responsibilities:
// - Read temperature sensors (hydronic, fresh tank, grey tank, ambient, manifolds)
// - Control critical safety systems (exhaust fan, tank heaters)
// - Environmental monitoring (BME280: temp, humidity, pressure)
// - Cabinet lock control (servo)
// - Manifold pressure monitoring
// - Valve control (rear loop, engine loop, front loop from thermostat triggers)
// - Grey tank heat loop valve control
// - Diesel heater on/off control
// - Electric heater on/off control (GPIO37, HIGH=on)
// - All relay status publishing (authoritative source for valve states)
// ============================================================================

#define FIRMWARE_VERSION "v3.1.0"
#define CABINET_ID "RearPassSide"

// ============================================================================
// WiFi & MQTT CONFIGURATION
// ============================================================================
const char* ssid = "Chester IOT";
const char* password = "2025Chester9894";
const char* mqtt_server = "192.168.8.1";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* mqtt_client_id = "ESP32S2_Plumbing";

// OTA Configuration
const char* otaHostname = "Plumbing";
const char* otaPassword = "Chester2025";

// ============================================================================
// MQTT TOPICS
// ============================================================================
// Temperature Sensors (Publish Only)
const char* Topic_Hydronic_Temp = "HydronicTemperature";
const char* Topic_FreshWtr_Temp = "FreshWaterTemperature";
const char* Topic_GreyWtr_Temp = "GreyWaterTemperature";
const char* Topic_Temp_Return = "PlumbingTempReturnManifold";

// Environmental Sensors (Publish Only)
const char* Topic_Environment_Temp = "EnvironmentTempAmbient";
const char* Topic_Environment_Humidity = "EnvironmentHumidity";
const char* Topic_Environment_Pressure = "EnvironmentPressure";

// Manifold Pressure (Publish Only)
const char* Topic_Manifold_Pressure = "ManifoldPressure"; // PSI

// Flow Sensor (Publish Only)
const char* Topic_Flow_Rate = "HotWaterFlowRate";
const char* Topic_Flow_Status = "HotWaterFlowStatus"; // FLOWING / NOT_FLOWING

// Main Water Pump (Publish & Subscribe)
const char* Topic_MainPump_Command = "MainWaterPumpCommand"; // ON / OFF
const char* Topic_MainPump_Status = "MainWaterPumpStatus"; // ON / OFF

// Domestic Hot Water Loop (Publish & Subscribe)
const char* Topic_DomesticHW_Command = "DomesticHotWaterCommand"; // OPEN / CLOSE
const char* Topic_DomesticHW_Status = "DomesticHotWaterStatus"; // OPEN / CLOSE

// Valve Status (Publish Only - This board is authoritative)
const char* Topic_FreshWtrHeat_Status = "FreshWaterHeatStatus"; // ON / Off
const char* Topic_GreyWtrHeat_Status = "GreyWaterHeatStatus"; // ON / Off
const char* Topic_RearLoop_Status = "RearLoopStatus"; // OPEN / CLOSE
const char* Topic_EngineLoop_Status = "EngineLoopStatus"; // OPEN / CLOSE
const char* Topic_FrontLoop_Status = "FrontLoopStatus"; // OPEN / CLOSE
const char* Topic_ExhaustFan_Status = "ExhaustFanStatus"; // ON / OFF
const char* Topic_DieselHtr_Status = "DieselHeaterStatus"; // OFF / ON
const char* Topic_ElectricHtr_Status = "ElectricHeaterStatus"; // ON / OFF

// Valve Commands (Subscribe)
const char* Topic_FreshWtrHeat_Command = "FreshWaterHeatCommand"; // ON / OFF
const char* Topic_GreyWtrHeat_Command = "GreyWaterHeatCommand"; // ON / OFF
const char* Topic_RearLoop_Command = "RearLoopCommand"; // OPEN / CLOSE
const char* Topic_EngineLoop_Command = "EngineLoopCommand"; // OPEN / CLOSE
const char* Topic_FrontLoop_Command = "FrontLoopCommand"; // OPEN / CLOSE
const char* Topic_ExhaustFan_Command = "ExhaustFanCommand"; // ON / OFF
const char* Topic_DieselHtr_Command = "DieselHeaterCommand"; // OFF / ON
const char* Topic_ElectricHtr_Command = "ElectricHeaterCommand"; // ON / OFF

// Cabinet Lock Topics
const char* Topic_CabLock_Command = "CabLockRearPassSideCommand"; // LOCK / UNLOCK
const char* Topic_CabLock_Status = "CabLockRearPassSideStatus"; // LOCKED / UNLOCKED
const char* Topic_AllLock_Command = "CabLockAllCommand"; // LOCK / UNLOCK

// Mode Topics (Subscribe) - Single global mode controls all automation
const char* Topic_Plumbing_Mode = "PlumbingMode"; // AUTO / MANUAL (global - state)
const char* Topic_Plumbing_ModeCmd = "PlumbingModeCommand"; // AUTO / MANUAL (global - command)

// Configuration Topics (State - Published by board)
const char* Topic_FreshWtrHeat_TempSet = "FreshWaterHeatTempSet";
const char* Topic_GreyWtrHeat_TempSet = "GreyWaterHeatTempSet";
const char* Topic_ExhaustFan_TempSet = "ExhaustFanTempSet"; // Single setpoint for fan

// Configuration Command Topics (Commands from HA - Subscribed by board)
const char* Topic_FreshWtrHeat_TempSetCmd = "FreshWaterHeatTempSetCommand";
const char* Topic_GreyWtrHeat_TempSetCmd = "GreyWaterHeatTempSetCommand";
const char* Topic_ExhaustFan_TempSetCmd = "ExhaustFanTempSetCommand";

// Alert Topics (Publish Only) - Consolidated
// Use a single alert topic that carries descriptive text for the most
// recent alert. Retained=true so Home Assistant can show the last alert.
const char* Topic_Alert = "PlumbingAlert"; // Descriptive retained alert messages

// Forward declarations for alert helpers (implemented later)
void publishAlertRaw(const String &msg);
void refreshAlertPublish();

// Safety Command Topics (Subscribe)
const char* Topic_Clear_High_Humidity = "ClearHighHumidityCommand"; // CLEAR

// System Health (Publish Only)
const char* Topic_Plumbing_Status = "PlumbingSystemStatus"; // ONLINE / ERROR
const char* Topic_Plumbing_Uptime = "PlumbingUptime";
const char* Topic_Firmware_Version = "PlumbingFirmwareVersion";

// ============================================================================
// HARDWARE PIN DEFINITIONS - ESP32-S2 Mini
// ============================================================================
// DS18B20 Temperature Sensors (All on shared OneWire bus)
#define TEMP_SENSOR_PIN 13           // GPIO13 - Shared OneWire bus for all temperature sensors
// Sensor indices on shared bus:
// Index 0: Return manifold
// Index 1: Hydronic tank
// Index 2: Grey water tank
// Index 3: Fresh water tank

// Relay Outputs (Active LOW unless noted)
#define FRESH_WATER_HEATER_PIN 9     // GPIO9 - Fresh water tank heater
#define GREY_WATER_HEATER_PIN 2      // GPIO2 - Grey water tank heater
#define EXHAUST_FAN_PIN 12           // GPIO12 - Exhaust fan
#define MAIN_WATER_PUMP_PIN 11       // GPIO11 - Main water pump (normally closed relay)
#define DOMESTIC_HW_SOLINOID_PIN 10  // GPIO10 - Domestic hot water loop valve
#define REAR_LOOP_VALVE_PIN 5        // GPIO5 - Rear loop valve
#define ENGINE_LOOP_VALVE_PIN 3      // GPIO3 - Engine loop valve
#define FRONT_LOOP_VALVE_PIN 4       // GPIO4 - Front loop valve
#define DIESEL_HEATER_PIN 8          // GPIO8 - Diesel heater relay 1 (white wire)
#define DIESEL_HEATER_2_PIN 7        // GPIO7 - Diesel heater relay 2 (blue wire)
#define DIESEL_HEATER_3_PIN 6        // GPIO6 - Diesel heater relay 3 (yellow wire)
#define ELECTRIC_HEATER_PIN 37       // GPIO37 - Electric heater relay (HIGH = ON)

// Cabinet Lock Servo
#define SERVO_PIN 16                 // GPIO16 - Servo control

// Pressure Sensor (Analog Input) - manifold pressure sensor
#define PRESSURE_SENSOR_PIN 1        // ADC1_CH0 (GPIO1) - 0.5V-4.5V = 0-80 PSI

// Flow sensor (Digital Input)
#define FLOW_SENSOR_PIN 17           // GPIO17 - Flow sensor input

// I2C for BME280
#define BME280_SDA 33                // GPIO33 - I2C SDA
#define BME280_SCL 35                // GPIO35 - I2C SCL

// ============================================================================
// SENSOR SETUP
// ============================================================================
// Shared OneWire bus for all DS18B20 sensors
OneWire tempSensorWire(TEMP_SENSOR_PIN);

// Single DallasTemperature instance for all sensors
DallasTemperature tempSensors(&tempSensorWire);

// BME280 Environmental Sensor
Adafruit_BME280 bme;
bool bmeAvailable = false;

// Servo for cabinet lock
Servo lockServo;

// ============================================================================
// PRESSURE & FLOW CONTROL
// ============================================================================
// Manifold pressure value (psi)
float manifoldPressure = 0.0;

// Valve Control Sources (MQTT only)
bool engineLoopMqttRequest = false;  // MQTT command from GIGA
bool frontLoopMqttRequest = false;  // MQTT command from GIGA

// Flow Sensor State
volatile unsigned long flowPulseCount = 0;
volatile unsigned long lastPulseTime = 0;  // Time when last pulse occurred
volatile unsigned long recentPulseCount = 0;  // Count pulses in recent window for threshold check
float flowRate = 0.0;  // Liters per minute
bool flowDetected = false;
bool previousFlowDetected = false;  // Track flow changes for domestic HW valve
unsigned long lastFlowCalc = 0;
const unsigned long FLOW_CALC_INTERVAL = 1000;  // Calculate flow every second
const unsigned long FLOW_TIMEOUT = 3000;        // If no pulses for 3 seconds, flow has stopped
const unsigned long MIN_PULSES_FOR_FLOW = 10;   // Minimum pulses per second to consider as real flow

// Pressure Relief (open DHW valve after pump stops)
bool pressureReliefActive = false;
unsigned long pressureReliefStartTime = 0;
const unsigned long PRESSURE_RELIEF_DURATION = 2000;  // 2 seconds

// ============================================================================
// CONFIGURATION VARIABLES
// ==========================================================================
float freshWaterTempTarget = 40.0;    // °F
float greyWaterTempTarget = 35.0;     // °F
float exhaustFanTempSet = 85.0;       // °F - Exhaust fan on setpoint (hysteresis applied)

#define FREEZE_WARNING_TEMP 35.0      // °F
#define HIGH_TEMP_WARNING 180.0       // °F

// ============================================================================
// STATE VARIABLES
// ============================================================================
// Valve States
bool freshWaterHeaterOn = false;
bool greyWaterHeaterOn = false;
bool exhaustFanOn = false;
bool rearLoopOpen = false;
bool engineLoopOpen = false;
bool frontLoopOpen = false;
bool dieselHeaterOn = false;  // false=OFF, true=ON
bool electricHeaterOn = false;  // false=OFF, true=ON (HIGH=on)
bool mainWaterPumpOn = true;  // Initialized HIGH (relay OFF) = pump ON for normally closed
bool DHWSolinoidOpen = false;

// Mode - single global mode controls all automation
String plumbingMode = "AUTO"; // AUTO or MANUAL

// Cabinet Lock State
enum LockState {
  STATE_UNKNOWN,
  STATE_UNLOCKED,
  STATE_LOCKED
};
LockState currentLockState = STATE_UNKNOWN;
#define LOCKED_POSITION 90
#define UNLOCKED_POSITION 5

// Temperature Readings
float hydronicTemp = -999.0;
float freshWaterTemp = -999.0;
float greyWaterTemp = -999.0;
float returnManifoldTemp = -999.0;
float environmentTemp = -999.0;
float environmentHumidity = -999.0;
float environmentPressure = -999.0;

// Temperature Filtering - Moving Average Filter (5 samples)
#define TEMP_FILTER_SIZE 5
float freshWaterTempBuffer[TEMP_FILTER_SIZE] = {-999.0, -999.0, -999.0, -999.0, -999.0};
float greyWaterTempBuffer[TEMP_FILTER_SIZE] = {-999.0, -999.0, -999.0, -999.0, -999.0};
int tempBufferIndex = 0;
bool tempFilterInitialized = false;

// Alert States
bool freezeWarningActive = false;
bool highTempWarningActive = false;
bool highHumidityAlarmActive = false;

// Humidity Leak Detection
float lastHumidity = -999.0;
float baselineHumidity = -999.0;              // Baseline humidity before spike detected
const float HUMIDITY_SPIKE_THRESHOLD = 10.0;  // % increase that triggers alarm
const float HUMIDITY_SPIKE_TOLERANCE = 3.0;   // % tolerance for spike recovery (must return within 3% of baseline)
const float HUMIDITY_RATE_WINDOW = 60000;     // 1 minute window for rate check
const unsigned long SPIKE_RECOVERY_TIMEOUT = 30000;  // 30 second window to detect spike recovery
unsigned long lastHumidityCheck = 0;
unsigned long spikeDetectedTime = 0;          // Timestamp when spike was first detected
bool humiditySpikePending = false;            // Flag indicating spike detected, waiting for confirmation

// Timing Variables
unsigned long lastSensorRead = 0;
unsigned long lastPressureRead = 0;
unsigned long lastStatusPublish = 0;
unsigned long lastHealthPublish = 0;
const unsigned long SENSOR_READ_INTERVAL = 5000;      // 5 seconds
const unsigned long PRESSURE_READ_INTERVAL = 1000;    // 1 second
const unsigned long STATUS_PUBLISH_INTERVAL = 10000;  // 10 seconds
const unsigned long HEALTH_PUBLISH_INTERVAL = 60000;  // 60 seconds

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

// Preferences for persistent storage
Preferences preferences;

// OTA flag
bool otaInProgress = false;

// Watchdog configuration
const int WATCHDOG_TIMEOUT_SEC = 60; // seconds before WDT resets the device

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void setupWifi();
bool ensureWifiConnected(unsigned long timeoutMs);
void setupOTA();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void subscribeToTopics();
void publishHomeAssistantDiscovery();

// Temperature Sensors
void readTemperatures();
void publishTemperatures();
void readBME280();
void publishBME280();

// Pressure & Flow
float readPressure();
void calculateFlowRate();
void publishFlowRate();
void IRAM_ATTR flowSensorISR();

// Valve Control Functions
void controlFreshWaterHeater(bool turnOn);
void controlGreyWaterHeater(bool turnOn);
void controlExhaustFan(bool turnOn);
void controlRearLoopValve(bool open);
void controlEngineLoopValve(bool open);
void controlFrontLoopValve(bool open);
void controlDieselHeater(bool turnOn);
void controlElectricHeater(bool turnOn);
void controlMainWaterPump(bool turnOn);
void controlDHWSolinoid(bool open);

// Cabinet Lock
void lockCabinet();
void unlockCabinet();
void processLockCommand(String command);

// Automation & Safety
void handleCriticalAutomation();
void checkAlerts();

// Status & Health
void publishStatus();
void publishHealth();
void saveConfigToPreferences();
void loadConfigFromPreferences();
void saveValveStatesToEEPROM();
void flushEEPROM();
void restoreValveStatesFromEEPROM();
void publishAllStatesToBroker();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n\n============================================");
  Serial.println("  ESP32-S2 PLUMBING & ENVIRONMENT CONTROLLER");
  Serial.println("  Chester the Bus RV Control System");
  Serial.println("  Firmware: " + String(FIRMWARE_VERSION));
  Serial.println("  Cabinet ID: " + String(CABINET_ID));
  Serial.println("============================================\n");

  // Load saved configuration
  preferences.begin("plumbing", false);
  loadConfigFromPreferences();

  // Configure relay pins as outputs (Active LOW)
  pinMode(FRESH_WATER_HEATER_PIN, OUTPUT);
  pinMode(GREY_WATER_HEATER_PIN, OUTPUT);
  pinMode(EXHAUST_FAN_PIN, OUTPUT);
  pinMode(REAR_LOOP_VALVE_PIN, OUTPUT);
  pinMode(ENGINE_LOOP_VALVE_PIN, OUTPUT);
  pinMode(FRONT_LOOP_VALVE_PIN, OUTPUT);
  pinMode(DIESEL_HEATER_PIN, OUTPUT);
  pinMode(DIESEL_HEATER_2_PIN, OUTPUT);
  pinMode(DIESEL_HEATER_3_PIN, OUTPUT);
  pinMode(ELECTRIC_HEATER_PIN, OUTPUT);
  pinMode(MAIN_WATER_PUMP_PIN, OUTPUT);
  pinMode(DOMESTIC_HW_SOLINOID_PIN, OUTPUT);

  // Initialize all relays to OFF (HIGH = OFF for active LOW, except electric heater: LOW = OFF)
  digitalWrite(FRESH_WATER_HEATER_PIN, HIGH);
  digitalWrite(GREY_WATER_HEATER_PIN, HIGH);
  digitalWrite(EXHAUST_FAN_PIN, HIGH);
  digitalWrite(REAR_LOOP_VALVE_PIN, HIGH);
  digitalWrite(ENGINE_LOOP_VALVE_PIN, HIGH);
  digitalWrite(FRONT_LOOP_VALVE_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_3_PIN, HIGH);
  digitalWrite(ELECTRIC_HEATER_PIN, LOW);  // HIGH = ON, so LOW = OFF
  digitalWrite(MAIN_WATER_PUMP_PIN, HIGH); // RELAY IS NORMALLY CLOSED
  digitalWrite(DOMESTIC_HW_SOLINOID_PIN, HIGH);
  Serial.println("✓ Relay pins configured (Active LOW, all systems OFF)");

  // Configure pressure sensor analog input
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  analogReadResolution(12);  // 12-bit ADC resolution
  Serial.println("✓ Pressure sensor configured (ADC1_CH0)");

  // Configure flow sensor with interrupt
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, FALLING);
  Serial.println("✓ Flow sensor initialized with interrupt (GPIO17)");

  // Pump PWM initialization removed (external pump)

  // Initialize servo for cabinet lock
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  lockServo.setPeriodHertz(50);
  lockServo.attach(SERVO_PIN, 500, 2400);
  lockServo.write(UNLOCKED_POSITION);
  currentLockState = STATE_UNLOCKED;
  delay(500);
  lockServo.detach();
  Serial.println("✓ Cabinet lock servo initialized (UNLOCKED)");

  // Initialize I2C for BME280
  Wire.begin(BME280_SDA, BME280_SCL);
  if (bme.begin(0x76, &Wire)) {
    bmeAvailable = true;
    Serial.println("✓ BME280 sensor found at 0x76");
  } else if (bme.begin(0x77, &Wire)) {
    bmeAvailable = true;
    Serial.println("✓ BME280 sensor found at 0x77");
  } else {
    Serial.println("⚠ BME280 sensor NOT found");
  }

  // Initialize DS18B20 temperature sensors (shared OneWire bus on pin 13)
  Serial.println("Initializing DS18B20 temperature sensors...");

  tempSensors.begin();

  delay(100);  // Give sensors time to initialize

  // Set resolution for all sensors
  DeviceAddress addr;
  for (int i = 0; i < tempSensors.getDeviceCount(); i++) {
    if (tempSensors.getAddress(addr, i)) {
      tempSensors.setResolution(addr, 12);
    }
  }

  // Check and report sensor availability
  int sensorCount = tempSensors.getDeviceCount();

  Serial.println("Temperature sensor status:");
  Serial.print("Total sensors found on GPIO");
  Serial.print(TEMP_SENSOR_PIN);
  Serial.print(": ");
  Serial.println(sensorCount);

  // Index 0: Return manifold
  if (sensorCount > 0 && tempSensors.getAddress(addr, 0)) {
    Serial.print("  ✓ Return Manifold sensor (Index 0): 0x");
    for (uint8_t j = 0; j < 8; j++) {
      if (addr[j] < 16) Serial.print("0");
      Serial.print(addr[j], HEX);
    }
    Serial.println();
  } else {
    Serial.println("  ⚠ Return Manifold sensor NOT found (Index 0)");
  }

  // Index 1: Hydronic tank
  if (sensorCount > 1 && tempSensors.getAddress(addr, 1)) {
    Serial.print("  ✓ Hydronic sensor (Index 1): 0x");
    for (uint8_t j = 0; j < 8; j++) {
      if (addr[j] < 16) Serial.print("0");
      Serial.print(addr[j], HEX);
    }
    Serial.println();
  } else {
    Serial.println("  ⚠ Hydronic sensor NOT found (Index 1)");
  }

  // Index 2: Grey water tank
  if (sensorCount > 2 && tempSensors.getAddress(addr, 2)) {
    Serial.print("  ✓ Grey Water sensor (Index 2): 0x");
    for (uint8_t j = 0; j < 8; j++) {
      if (addr[j] < 16) Serial.print("0");
      Serial.print(addr[j], HEX);
    }
    Serial.println();
  } else {
    Serial.println("  ⚠ Grey Water sensor NOT found (Index 2)");
  }

  // Index 3: Fresh water tank
  if (sensorCount > 3 && tempSensors.getAddress(addr, 3)) {
    Serial.print("  ✓ Fresh Water sensor (Index 3): 0x");
    for (uint8_t j = 0; j < 8; j++) {
      if (addr[j] < 16) Serial.print("0");
      Serial.print(addr[j], HEX);
    }
    Serial.println();
  } else {
    Serial.println("  ⚠ Fresh Water sensor NOT found (Index 3)");
  }

  Serial.print("✓ Total DS18B20 sensors found: ");
  Serial.println(sensorCount);

  // Connect to WiFi
  setupWifi();

  // Setup OTA
  setupOTA();

  // Initialize task watchdog: reset device on panic after timeout
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&wdtConfig);
  esp_task_wdt_add(NULL); // add current task (the loop task)
  Serial.print("✓ Task watchdog enabled (");
  Serial.print(WATCHDOG_TIMEOUT_SEC);
  Serial.println("s)");

  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(1024);

  // Restore valve states from EEPROM (board is authoritative)
  restoreValveStatesFromEEPROM();
  delay(500);  // Let hardware settle

  Serial.println("\n✓ Initialization complete");
  Serial.println("============================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  ArduinoOTA.handle();

  // Feed task watchdog to indicate loop is healthy
  esp_task_wdt_reset();

  unsigned long currentMillis = millis();
  // Periodically ensure WiFi is connected (non-blocking short attempts)
  static unsigned long lastWifiCheck = 0;
  if (currentMillis - lastWifiCheck >= 10000) { // check every 10s
    lastWifiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi down - attempting quick reconnect...");
      ensureWifiConnected(3000); // short timeout
    }
  }

  if (!otaInProgress) {
    // Maintain MQTT connection
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();

    // Read pressure sensor and update PID (fast loop)
    if (currentMillis - lastPressureRead >= PRESSURE_READ_INTERVAL) {
      lastPressureRead = currentMillis;
      manifoldPressure = readPressure();
    }

    // Calculate flow rate periodically (1 second)
    if (currentMillis - lastFlowCalc >= FLOW_CALC_INTERVAL) {
      calculateFlowRate();
      publishFlowRate();
    }

    // Handle pressure relief valve timing
    if (pressureReliefActive) {
      if (currentMillis - pressureReliefStartTime >= PRESSURE_RELIEF_DURATION) {
        pressureReliefActive = false;
        // Only close valve if no flow is detected
        if (!flowDetected) {
          Serial.println("💨 Pressure relief complete - closing DHW valve");
          controlDHWSolinoid(false);
        } else {
          Serial.println("💨 Pressure relief complete - keeping DHW valve open (flow active)");
        }
      }
    }

    // Read temperature sensors
    if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
      lastSensorRead = currentMillis;
      readTemperatures();
      publishTemperatures();

      if (bmeAvailable) {
        readBME280();
        publishBME280();
      }

      handleCriticalAutomation();
      checkAlerts();
    }

    // Publish status
    if (currentMillis - lastStatusPublish >= STATUS_PUBLISH_INTERVAL) {
      lastStatusPublish = currentMillis;
      publishStatus();
    }

    // Publish health metrics
    if (currentMillis - lastHealthPublish >= HEALTH_PUBLISH_INTERVAL) {
      lastHealthPublish = currentMillis;
      publishHealth();
    }

    // Flush deferred EEPROM writes (debounced to reduce flash wear)
    flushEEPROM();
  }

  delay(10);
}

// ============================================================================
// WiFi SETUP
// ============================================================================
void setupWifi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected!");
    Serial.print("  IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("  MAC: ");
    Serial.println(WiFi.macAddress());
  } else {
    Serial.println("\n⚠ WiFi connection failed");
  }
}

// Try to ensure WiFi is connected. Uses short exponential backoff and
// returns quickly if it can't connect. Caller may choose to restart
// the interface or device after repeated failures.
bool ensureWifiConnected(unsigned long timeoutMs = 5000) {
  if (WiFi.status() == WL_CONNECTED) return true;

  unsigned long start = millis();
  unsigned long attemptDelay = 250; // start small
  const unsigned long maxDelay = 3000;

  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    Serial.print("WiFi not connected - trying to reconnect...");
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);

    unsigned long waitStart = millis();
    while ((millis() - waitStart) < attemptDelay) {
      if (WiFi.status() == WL_CONNECTED) break;
      delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(" connected");
      Serial.print("IP: "); Serial.println(WiFi.localIP());
      return true;
    }

    // increase backoff but cap it
    attemptDelay = min(maxDelay, attemptDelay * 2);
    Serial.print(" reconnect failed, next attempt in ");
    Serial.print(attemptDelay);
    Serial.println(" ms");
  }

  Serial.println("ensureWifiConnected: timed out");
  return (WiFi.status() == WL_CONNECTED);
}

// ============================================================================
// OTA SETUP
// ============================================================================
void setupOTA() {
  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    // Pump is controlled externally; firmware will not change pump state during OTA
    Serial.println("\n🔄 OTA Update Starting...");
    // Detach watchdog for this task while OTA runs to avoid timeout during long uploads
    esp_task_wdt_delete(NULL);
    Serial.println("✓ Watchdog detached for OTA");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n✓ OTA Update Complete!");
    // Re-attach watchdog after OTA completes
    esp_task_wdt_add(NULL);
    otaInProgress = false;
    Serial.println("✓ Watchdog reattached after OTA");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static int lastPercent = -1;
    int percent = (progress / (total / 100));
    if (percent != lastPercent && percent % 10 == 0) {
      Serial.printf("  Progress: %u%%\n", percent);
      lastPercent = percent;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n✗ OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    // Ensure watchdog is reattached after error path too
    esp_task_wdt_add(NULL);
    otaInProgress = false;
    Serial.println("✓ Watchdog reattached after OTA error");
  });

  ArduinoOTA.begin();
  Serial.println("✓ OTA enabled");
}

// ============================================================================
// HOME ASSISTANT MQTT DISCOVERY
// ============================================================================
void publishHomeAssistantDiscovery() {
  Serial.println("📡 Publishing Home Assistant MQTT Discovery...");

  String deviceId = "chester_plumbing";
  String deviceName = "Chester Plumbing System";

  // Device configuration (shared by all entities)
  String device = "{\"identifiers\":[\"" + deviceId + "\"],"
                  "\"name\":\"" + deviceName + "\","
                  "\"model\":\"ESP32-S2 Mini\","
                  "\"manufacturer\":\"ChesterTheBus\","
                  "\"sw_version\":\"" + String(FIRMWARE_VERSION) + "\"}";

  // Helper lambda for publishing discovery
  auto publishDiscovery = [&](const char* component, const char* objectId, const char* name,
                              const char* stateTopic, const char* deviceClass = nullptr,
                              const char* unit = nullptr, const char* commandTopic = nullptr,
                              const char* payloadOn = nullptr, const char* payloadOff = nullptr,
                              const char* icon = nullptr) {
    String topic = String("homeassistant/") + component + "/" + deviceId + "/" + objectId + "/config";
    String payload = "{\"name\":\"" + String(name) + "\","
                     "\"unique_id\":\"" + deviceId + "_" + objectId + "\","
                     "\"state_topic\":\"" + String(stateTopic) + "\"";

    if (deviceClass) payload += ",\"device_class\":\"" + String(deviceClass) + "\"";
    if (unit) payload += ",\"unit_of_measurement\":\"" + String(unit) + "\"";
    if (commandTopic) payload += ",\"command_topic\":\"" + String(commandTopic) + "\"";
    if (payloadOn) payload += ",\"payload_on\":\"" + String(payloadOn) + "\"";
    if (payloadOff) payload += ",\"payload_off\":\"" + String(payloadOff) + "\"";
    if (icon) payload += ",\"icon\":\"" + String(icon) + "\"";

    payload += ",\"device\":" + device + "}";

    client.publish(topic.c_str(), payload.c_str(), true);
    delay(50); // Small delay between discoveries
  };

  // Temperature Sensors
  publishDiscovery("sensor", "hydronic_temp", "Hydronic Temperature", Topic_Hydronic_Temp, "temperature", "°F", nullptr, nullptr, nullptr, "mdi:water-boiler");
  publishDiscovery("sensor", "fresh_water_temp", "Fresh Tank Temperature", Topic_FreshWtr_Temp, "temperature", "°F");
  publishDiscovery("sensor", "grey_water_temp", "Grey Tank Temperature", Topic_GreyWtr_Temp, "temperature", "°F");
  publishDiscovery("sensor", "return_manifold_temp", "Return Manifold Temperature", Topic_Temp_Return, "temperature", "°F");
  publishDiscovery("sensor", "ambient_temp", "Ambient Temperature", Topic_Environment_Temp, "temperature", "°F");

  // Environmental Sensors
  publishDiscovery("sensor", "humidity", "Humidity", Topic_Environment_Humidity, "humidity", "%");
  publishDiscovery("sensor", "pressure", "Atmospheric Pressure", Topic_Environment_Pressure, "pressure", "hPa");

  // Manifold Pressure Sensor
  publishDiscovery("sensor", "manifold_pressure", "Manifold Pressure", Topic_Manifold_Pressure, "pressure", "PSI", nullptr, nullptr, nullptr, "mdi:gauge");

  // Flow Sensor
  publishDiscovery("sensor", "flow_rate", "Hot Water Flow Rate", Topic_Flow_Rate, nullptr, "L/min", nullptr, nullptr, nullptr, "mdi:water");
  publishDiscovery("sensor", "flow_status", "Hot Water Flow Status", Topic_Flow_Status, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:water");

  // Switches (Valves and Controls)
  publishDiscovery("switch", "fresh_water_heat", "Fresh Water Tank Heater", Topic_FreshWtrHeat_Status, nullptr, nullptr, Topic_FreshWtrHeat_Command, "ON", "OFF", "mdi:water-thermometer");
  publishDiscovery("switch", "grey_water_heat", "Grey Water Tank Heater", Topic_GreyWtrHeat_Status, nullptr, nullptr, Topic_GreyWtrHeat_Command, "ON", "OFF", "mdi:water-thermometer");
  publishDiscovery("switch", "rear_loop", "Rear Loop Valve", Topic_RearLoop_Status, nullptr, nullptr, Topic_RearLoop_Command, "OPEN", "CLOSE", "mdi:pipe-valve");
  publishDiscovery("switch", "engine_loop", "Engine Loop Valve", Topic_EngineLoop_Status, nullptr, nullptr, Topic_EngineLoop_Command, "OPEN", "CLOSE", "mdi:pipe-valve");
  publishDiscovery("switch", "front_loop", "Front Loop Valve", Topic_FrontLoop_Status, nullptr, nullptr, Topic_FrontLoop_Command, "OPEN", "CLOSE", "mdi:pipe-valve");
  publishDiscovery("switch", "exhaust_fan", "Exhaust Fan", Topic_ExhaustFan_Status, nullptr, nullptr, Topic_ExhaustFan_Command, "ON", "OFF", "mdi:fan");
  publishDiscovery("switch", "main_pump", "Main Water Pump", Topic_MainPump_Status, nullptr, nullptr, Topic_MainPump_Command, "ON", "OFF", "mdi:pump");
  publishDiscovery("switch", "domestic_hw", "Domestic Hot Water", Topic_DomesticHW_Status, nullptr, nullptr, Topic_DomesticHW_Command, "OPEN", "CLOSE", "mdi:water-boiler");

  // Diesel Heater (simple on/off switch)
  publishDiscovery("switch", "diesel_heater", "Diesel Heater", Topic_DieselHtr_Status, nullptr, nullptr, Topic_DieselHtr_Command, "ON", "OFF", "mdi:fire");

  // Remove old select discovery entity (publish empty retained message to clean up)
  String oldHeaterSelectTopic = String("homeassistant/select/") + deviceId + "/diesel_heater/config";
  client.publish(oldHeaterSelectTopic.c_str(), "", true);
  delay(50);

  // Electric Heater (simple on/off switch)
  publishDiscovery("switch", "electric_heater", "Electric Heater", Topic_ElectricHtr_Status, nullptr, nullptr, Topic_ElectricHtr_Command, "ON", "OFF", "mdi:radiator");

  // Alerts
  publishDiscovery("sensor", "plumbing_alerts", "Plumbing Alerts", Topic_Alert, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:alert-circle");

  // Cabinet Lock
  publishDiscovery("lock", "cabinet_lock", "Rear Pass Cabinet Lock", Topic_CabLock_Status, nullptr, nullptr, Topic_CabLock_Command, "LOCK", "UNLOCK", "mdi:lock");

  // Exhaust Fan Temperature Setpoint (number entity)
  String fanSetTopic = String("homeassistant/number/") + deviceId + "/exhaust_fan_temp_set/config";
  String fanSetPayload = "{\"name\":\"Exhaust Fan Temperature Setpoint\","
                        "\"unique_id\":\"" + deviceId + "_exhaust_fan_temp_set\","
                        "\"state_topic\":\"" + String(Topic_ExhaustFan_TempSet) + "\","
                        "\"command_topic\":\"" + String(Topic_ExhaustFan_TempSetCmd) + "\","
                        "\"min\":32.0,"
                        "\"max\":140.0,"
                        "\"step\":1.0,"
                        "\"unit_of_measurement\":\"°F\","
                        "\"icon\":\"mdi:fan\","
                        "\"device\":" + device + "}";
  client.publish(fanSetTopic.c_str(), fanSetPayload.c_str(), true);
  delay(50);

  // Plumbing Mode (select: AUTO / MANUAL)
  String modeTopic = String("homeassistant/select/") + deviceId + "/plumbing_mode/config";
  String modePayload = "{\"name\":\"Plumbing Mode\","
                     "\"unique_id\":\"" + deviceId + "_plumbing_mode\","
                     "\"state_topic\":\"" + String(Topic_Plumbing_Mode) + "\","
                     "\"command_topic\":\"" + String(Topic_Plumbing_ModeCmd) + "\","
                     "\"options\":[\"AUTO\",\"MANUAL\"],"
                     "\"icon\":\"mdi:settings\","
                     "\"device\":" + device + "}";
  client.publish(modeTopic.c_str(), modePayload.c_str(), true);
  delay(50);

  // Fresh Water Heater Temperature Setpoint
  String freshTempTopic = String("homeassistant/number/") + deviceId + "/fresh_water_temp_set/config";
  String freshTempPayload = "{\"name\":\"Fresh Water Heater Setpoint\","
                            "\"unique_id\":\"" + deviceId + "_fresh_water_temp_set\","
                            "\"state_topic\":\"" + String(Topic_FreshWtrHeat_TempSet) + "\","
                            "\"command_topic\":\"" + String(Topic_FreshWtrHeat_TempSetCmd) + "\","
                            "\"min\":32.0,"
                            "\"max\":120.0,"
                            "\"step\":1.0,"
                            "\"unit_of_measurement\":\"°F\","
                            "\"icon\":\"mdi:thermometer\","
                            "\"device\":" + device + "}";
  client.publish(freshTempTopic.c_str(), freshTempPayload.c_str(), true);
  delay(50);

  // Grey Water Heater Temperature Setpoint
  String greyTempTopic = String("homeassistant/number/") + deviceId + "/grey_water_temp_set/config";
  String greyTempPayload = "{\"name\":\"Grey Water Heater Setpoint\","
                           "\"unique_id\":\"" + deviceId + "_grey_water_temp_set\","
                           "\"state_topic\":\"" + String(Topic_GreyWtrHeat_TempSet) + "\","
                           "\"command_topic\":\"" + String(Topic_GreyWtrHeat_TempSetCmd) + "\","
                           "\"min\":32.0,"
                           "\"max\":120.0,"
                           "\"step\":1.0,"
                           "\"unit_of_measurement\":\"°F\","
                           "\"icon\":\"mdi:thermometer\","
                           "\"device\":" + device + "}";
  client.publish(greyTempTopic.c_str(), greyTempPayload.c_str(), true);
  delay(50);

  Serial.println("✓ Home Assistant Discovery published for Plumbing System");
}

// ============================================================================
// MQTT CONNECTION
// ============================================================================
void reconnectMQTT() {
  static unsigned long lastAttempt = 0;
  static int mqttFailCount = 0;

  // Limit reconnect attempts frequency
  if (millis() - lastAttempt < 5000) return;
  lastAttempt = millis();

  // Ensure WiFi is up (short timeout)
  if (!ensureWifiConnected(4000)) {
    Serial.println("MQTT reconnect skipped - WiFi down");
    mqttFailCount++;
    // If WiFi/MQTT failing repeatedly, consider resetting
    if (mqttFailCount >= 8) {
      Serial.println("Repeated network failures - restarting device");
      delay(200);
      ESP.restart();
    }
    return;
  }

  if (client.connected()) {
    mqttFailCount = 0; // success
    return;
  }

  Serial.print("Connecting to MQTT...");
  // Use existing credentials (empty strings accepted)
  // Set Last Will message so broker marks us OFFLINE on unexpected disconnect
  if (client.connect(mqtt_client_id, mqtt_user, mqtt_password, Topic_Plumbing_Status, 1, true, "OFFLINE")) {
    Serial.println(" connected!");
    mqttFailCount = 0;
    // Announce ONLINE (retained) so Home Assistant and other systems see us
    client.publish(Topic_Plumbing_Status, "ONLINE", true);
    publishHomeAssistantDiscovery();
    subscribeToTopics();

    // Publish board's authoritative states to broker
    publishAllStatesToBroker();

    // Publish plumbing mode and current setpoints so HA sees them
    client.publish(Topic_Plumbing_Mode, plumbingMode.c_str(), true);
    char fwStr[8]; dtostrf(freshWaterTempTarget, 4, 1, fwStr); client.publish(Topic_FreshWtrHeat_TempSet, fwStr, true);
    char gwStr[8]; dtostrf(greyWaterTempTarget, 4, 1, gwStr); client.publish(Topic_GreyWtrHeat_TempSet, gwStr, true);
    char fanStr[8]; dtostrf(exhaustFanTempSet, 4, 1, fanStr); client.publish(Topic_ExhaustFan_TempSet, fanStr, true);

    publishHealth();
    publishStatus();
  } else {
    Serial.print(" failed, rc=");
    Serial.println(client.state());
    mqttFailCount++;
  }
}

void subscribeToTopics() {
  // Valve commands
  client.subscribe(Topic_FreshWtrHeat_Command);
  client.subscribe(Topic_GreyWtrHeat_Command);
  client.subscribe(Topic_ExhaustFan_Command);
  client.subscribe(Topic_RearLoop_Command);
  client.subscribe(Topic_EngineLoop_Command);
  client.subscribe(Topic_FrontLoop_Command);
  client.subscribe(Topic_DieselHtr_Command);
  client.subscribe(Topic_ElectricHtr_Command);

  // Global mode
  client.subscribe(Topic_Plumbing_ModeCmd);

  // Configuration - Subscribe to COMMAND topics (separate from state topics)
  // This prevents the retained message loop while allowing HA to send commands
  client.subscribe(Topic_FreshWtrHeat_TempSetCmd);
  client.subscribe(Topic_GreyWtrHeat_TempSetCmd);
  client.subscribe(Topic_ExhaustFan_TempSetCmd);

  // Cabinet lock
  client.subscribe(Topic_CabLock_Command);
  client.subscribe(Topic_AllLock_Command);

  // Main water pump and domestic HW loop
  client.subscribe(Topic_MainPump_Command);
  client.subscribe(Topic_DomesticHW_Command);

  // Safety commands
  client.subscribe(Topic_Clear_High_Humidity);

  Serial.println("✓ Subscribed to command topics");
}

// ============================================================================
// MQTT CALLBACK
// ============================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();

  Serial.print("MQTT RX [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  String topicStr = String(topic);

  // Cabinet lock commands
  if (topicStr == Topic_CabLock_Command || topicStr == Topic_AllLock_Command) {
    processLockCommand(message);
  }
  // Fresh water heater
  else if (topicStr == Topic_FreshWtrHeat_Command) {
    controlFreshWaterHeater(message == "ON");
  }
  else if (topicStr == Topic_FreshWtrHeat_TempSetCmd) {
    float newTemp = message.toFloat();
    // Validate temperature range
    if (newTemp >= 32.0 && newTemp <= 120.0) {
      freshWaterTempTarget = newTemp;
      saveConfigToPreferences();
      // Publish new state to state topic
      char tempStr[8];
      dtostrf(freshWaterTempTarget, 4, 1, tempStr);
      client.publish(Topic_FreshWtrHeat_TempSet, tempStr, true);
      Serial.print("🌡️ Fresh Water setpoint updated: ");
      Serial.print(freshWaterTempTarget);
      Serial.println(" °F");
    }
  }
  // Grey water heater
  else if (topicStr == Topic_GreyWtrHeat_Command) {
    controlGreyWaterHeater(message == "ON");
  }
  else if (topicStr == Topic_GreyWtrHeat_TempSetCmd) {
    float newTemp = message.toFloat();
    // Validate temperature range
    if (newTemp >= 32.0 && newTemp <= 120.0) {
      greyWaterTempTarget = newTemp;
      saveConfigToPreferences();
      // Publish new state to state topic
      char tempStr[8];
      dtostrf(greyWaterTempTarget, 4, 1, tempStr);
      client.publish(Topic_GreyWtrHeat_TempSet, tempStr, true);
      Serial.print("🌡️ Grey Water setpoint updated: ");
      Serial.print(greyWaterTempTarget);
      Serial.println(" °F");
    }
  }
  // Exhaust fan
  else if (topicStr == Topic_ExhaustFan_Command) {
    controlExhaustFan(message == "ON");
  }
  else if (topicStr == Topic_ExhaustFan_TempSetCmd) {
    float newTemp = message.toFloat();
    // Validate temperature range
    if (newTemp >= 32.0 && newTemp <= 140.0) {
      exhaustFanTempSet = newTemp;
      saveConfigToPreferences();
      // Publish new state to state topic
      char tempStr[8];
      dtostrf(exhaustFanTempSet, 4, 1, tempStr);
      client.publish(Topic_ExhaustFan_TempSet, tempStr, true);
      Serial.print("🌬️ Exhaust fan setpoint updated: ");
      Serial.print(exhaustFanTempSet);
      Serial.println(" °F");
    }
  }
  // Valves
  else if (topicStr == Topic_RearLoop_Command) {
    controlRearLoopValve(message == "ON" || message == "OPEN");
  }
  else if (topicStr == Topic_EngineLoop_Command) {
    // Store MQTT request and update valve
    bool newState = (message == "ON" || message == "OPEN");
    if (engineLoopMqttRequest != newState) {
      engineLoopMqttRequest = newState;
      preferences.putBool("engLoopMqtt", engineLoopMqttRequest);  // Persist to EEPROM
      Serial.print("Engine Loop MQTT: ");
      Serial.print(newState ? "OPEN" : "CLOSE");
      Serial.println(" (saved)");
    }
    controlEngineLoopValve(newState);
  }
  else if (topicStr == Topic_FrontLoop_Command) {
    // Store MQTT request and update valve
    bool newState = (message == "ON" || message == "OPEN");
    if (frontLoopMqttRequest != newState) {
      frontLoopMqttRequest = newState;
      preferences.putBool("frtLoopMqtt", frontLoopMqttRequest);  // Persist to EEPROM
      Serial.print("Front Loop MQTT: ");
      Serial.print(newState ? "OPEN" : "CLOSE");
      Serial.println(" (saved)");
    }
    controlFrontLoopValve(newState);
  }
  // Diesel heater (simple on/off)
  else if (topicStr == Topic_DieselHtr_Command) {
    controlDieselHeater(message == "ON");
  }
  // Electric heater
  else if (topicStr == Topic_ElectricHtr_Command) {
    controlElectricHeater(message == "ON");
  }
  // Main water pump
  else if (topicStr == Topic_MainPump_Command) {
    // Accept START/STOP primarily (header comment). Allow ON/OFF as backward compat.
    if (message == "START" || message == "ON") controlMainWaterPump(true);
    else if (message == "STOP" || message == "OFF") controlMainWaterPump(false);
  }
  // Domestic hot water loop
  else if (topicStr == Topic_DomesticHW_Command) {
    if (message == "OPEN") controlDHWSolinoid(true);
    else if (message == "CLOSE") controlDHWSolinoid(false);
  }
  // Safety commands
  else if (topicStr == Topic_Clear_High_Humidity) {
    if (message == "clear" || message == "CLEAR" || message == "true" || message == "TRUE") {
      if (highHumidityAlarmActive) {
        Serial.println("🔓 High humidity alarm MANUALLY CLEARED");
        highHumidityAlarmActive = false;
        humiditySpikePending = false;  // Also clear any pending spike detection
        baselineHumidity = -999.0;
        refreshAlertPublish();
        // Alert state refreshed
        // Reset humidity baseline
        lastHumidity = environmentHumidity;
        lastHumidityCheck = millis();
      }
    }
  }

  // Global plumbing mode (AUTO / MANUAL)
  else if (topicStr == Topic_Plumbing_ModeCmd) {
    if (message == "AUTO" || message == "MANUAL") {
      plumbingMode = message;
      saveConfigToPreferences();
      // Publish new state to state topic
      client.publish(Topic_Plumbing_Mode, plumbingMode.c_str(), true);
      Serial.print("⚙️ Plumbing mode set to: ");
      Serial.println(plumbingMode);
    } else {
      Serial.println("⚠️ Invalid Plumbing Mode (use AUTO or MANUAL)");
    }
  }
}

// ============================================================================
// PRESSURE SENSOR
// ============================================================================
float readPressure() {
  // Read analog voltage (0-3.3V on ESP32-S2, 12-bit ADC = 0-4095)
  int rawValue = analogRead(PRESSURE_SENSOR_PIN);

  // Calibration: ADC reads 580 at 0 PSI
  // Remove offset before calculating pressure
  const int ADC_ZERO_OFFSET = 580;
  int calibratedValue = rawValue - ADC_ZERO_OFFSET;

  // Prevent negative values
  if (calibratedValue < 0) calibratedValue = 0;

  // Convert to voltage
  float voltage = (calibratedValue / 4095.0) * 3.3;

  // Convert voltage to PSI
  // Sensor powered by 3.3V: 0V = 0 PSI, 3.3V = 80 PSI (linear)
  // PSI = (voltage / 3.3) * 80
  float psi = (voltage / 3.3) * 80.0;

  // Clamp to valid range
  if (psi < 0) psi = 0;
  if (psi > 85) psi = 85;  // Slightly over max for safety

  return psi;
}


// ============================================================================
// FLOW SENSOR
// ============================================================================
void IRAM_ATTR flowSensorISR() {
  flowPulseCount++;
  recentPulseCount++;
  lastPulseTime = millis();  // Record when pulse occurred
}

void calculateFlowRate() {
  // Disable interrupts while reading pulse count
  noInterrupts();
  unsigned long pulses = flowPulseCount;
  unsigned long recentPulses = recentPulseCount;
  unsigned long lastPulse = lastPulseTime;
  flowPulseCount = 0;
  recentPulseCount = 0;  // Reset recent counter
  interrupts();

  // Calculate flow rate (adjust calibration factor for your sensor)
  // YF-S201 sensor: ~450 pulses per liter
  // Flow rate (L/min) = (pulses / 450) * (60000 / interval_ms)
  unsigned long currentTime = millis();
  unsigned long interval = currentTime - lastFlowCalc;
  lastFlowCalc = currentTime;

  if (interval > 0) {
    flowRate = (pulses / 450.0) * (60000.0 / interval);
  }

  // Flow is detected if:
  // 1. We received pulses in the last FLOW_TIMEOUT period, AND
  // 2. The number of pulses meets the minimum threshold (filters out noise/drips)
  previousFlowDetected = flowDetected;
  unsigned long timeSinceLastPulse = currentTime - lastPulse;
  bool hasRecentPulses = (timeSinceLastPulse < FLOW_TIMEOUT);
  bool meetsThreshold = (recentPulses >= MIN_PULSES_FOR_FLOW);

  flowDetected = hasRecentPulses && meetsThreshold;

  // Instantly open domestic HW valve when flow is detected
  if (flowDetected && !previousFlowDetected) {
    Serial.println("💦 Flow DETECTED - opening domestic HW valve instantly");
    controlDHWSolinoid(true);  // Open solenoid valve immediately
    pressureReliefActive = false;  // Cancel any ongoing pressure relief
  }
  // Close domestic HW valve when flow stops (only if not doing pressure relief)
  else if (!flowDetected && previousFlowDetected) {
    if (!pressureReliefActive) {
      Serial.println("💦 Flow STOPPED - closing domestic HW valve");
      controlDHWSolinoid(false);  // Close solenoid valve
    } else {
      Serial.println("💦 Flow STOPPED - keeping valve open for pressure relief");
    }
  }

  // Log flow status changes
  if (flowDetected != previousFlowDetected) {
    Serial.print("💦 Water flow: ");
    Serial.println(flowDetected ? "DETECTED" : "STOPPED");
    if (flowDetected) {
      Serial.print("   Flow rate: ");
      Serial.print(flowRate);
      Serial.print(" L/min (");
      Serial.print(pulses);
      Serial.println(" pulses)");
    }
  }
}

void publishFlowRate() {
  if (!client.connected()) return;

  // Publish flow rate
  char flowStr[8];
  dtostrf(flowRate, 4, 2, flowStr);
  client.publish(Topic_Flow_Rate, flowStr);

  // Publish flow status
  client.publish(Topic_Flow_Status, flowDetected ? "FLOWING" : "STOPPED");
}

// ============================================================================
// TEMPERATURE SENSORS
// ============================================================================
// Helper function to calculate moving average
float calculateTempAverage(float* buffer) {
  float sum = 0.0;
  int validCount = 0;

  for (int i = 0; i < TEMP_FILTER_SIZE; i++) {
    if (buffer[i] > -999.0) {  // Only count valid readings
      sum += buffer[i];
      validCount++;
    }
  }

  if (validCount == 0) return -999.0;
  return sum / validCount;
}

void readTemperatures() {
  // Request temperatures from all sensors on shared bus
  tempSensors.requestTemperatures();

  float tempC;

  // DEBUG: Read all 4 indices and log them for sensor identification
  static unsigned long lastDebugLog = 0;
  if (client.connected() && (millis() - lastDebugLog) > 30000) {  // Log every 30 seconds
    lastDebugLog = millis();
    char debugStr[8];

    for (int i = 0; i < 4; i++) {
      tempC = tempSensors.getTempCByIndex(i);
      float tempF = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;
      dtostrf(tempF, 5, 1, debugStr);

      String debugTopic = "Debug/SensorIndex_" + String(i);
      client.publish(debugTopic.c_str(), debugStr);
    }
  }

  // Index 0: Return manifold (no filtering - usually stable)
  tempC = tempSensors.getTempCByIndex(0);
  returnManifoldTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  // Index 1: Hydronic temperature (no filtering needed - usually stable)
  tempC = tempSensors.getTempCByIndex(1);
  hydronicTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  // Index 2: Grey water temperature with filtering
  tempC = tempSensors.getTempCByIndex(2);
  float greyWaterRaw = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  // Add to circular buffer
  greyWaterTempBuffer[tempBufferIndex] = greyWaterRaw;

  // Calculate filtered value
  if (tempFilterInitialized || tempBufferIndex >= TEMP_FILTER_SIZE - 1) {
    greyWaterTemp = calculateTempAverage(greyWaterTempBuffer);
    tempFilterInitialized = true;
  } else {
    greyWaterTemp = greyWaterRaw;  // Use raw until buffer fills
  }

  // Index 3: Fresh water temperature with filtering
  tempC = tempSensors.getTempCByIndex(3);
  float freshWaterRaw = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  // Add to circular buffer
  freshWaterTempBuffer[tempBufferIndex] = freshWaterRaw;

  // Calculate filtered value
  if (tempFilterInitialized || tempBufferIndex >= TEMP_FILTER_SIZE - 1) {
    freshWaterTemp = calculateTempAverage(freshWaterTempBuffer);
  } else {
    freshWaterTemp = freshWaterRaw;  // Use raw until buffer fills
  }

  // Increment buffer index (circular)
  tempBufferIndex = (tempBufferIndex + 1) % TEMP_FILTER_SIZE;
}

void publishTemperatures() {
  char tempStr[8];

  if (hydronicTemp > -999.0) {
    dtostrf(hydronicTemp, 5, 1, tempStr);
    client.publish(Topic_Hydronic_Temp, tempStr);
  }

  if (freshWaterTemp > -999.0) {
    dtostrf(freshWaterTemp, 5, 1, tempStr);
    client.publish(Topic_FreshWtr_Temp, tempStr);
  }

  if (greyWaterTemp > -999.0) {
    dtostrf(greyWaterTemp, 5, 1, tempStr);
    client.publish(Topic_GreyWtr_Temp, tempStr);
  }

  if (returnManifoldTemp > -999.0) {
    dtostrf(returnManifoldTemp, 5, 1, tempStr);
    client.publish(Topic_Temp_Return, tempStr);
  }
}

void readBME280() {
  if (!bmeAvailable) return;

  float tempC = bme.readTemperature();
  // Convert BME Celsius to Fahrenheit for internal state and publishing
  environmentTemp = (tempC * 9.0 / 5.0) + 32.0;
  float newHumidity = bme.readHumidity();
  environmentPressure = bme.readPressure() / 100.0F;

  // Check for sudden humidity spike (possible leak detection) with spike filtering
  unsigned long currentTime = millis();
  if (lastHumidity > 0 && !highHumidityAlarmActive) {
    float humidityChange = newHumidity - lastHumidity;
    float timeDelta = (currentTime - lastHumidityCheck) / 1000.0;  // seconds

    // ---- SPIKE FILTERING LOGIC ----
    // When spike first detected: store baseline and start timer
    if (!humiditySpikePending && humidityChange > HUMIDITY_SPIKE_THRESHOLD && timeDelta < 60.0) {
      baselineHumidity = lastHumidity;
      spikeDetectedTime = currentTime;
      humiditySpikePending = true;

      Serial.println("\n🔍 Humidity spike detected (pending confirmation)...");
      Serial.print("Spike: +");
      Serial.print(humidityChange, 1);
      Serial.print("% (from ");
      Serial.print(baselineHumidity, 1);
      Serial.print("% to ");
      Serial.print(newHumidity, 1);
      Serial.println("%)");
      Serial.println("Waiting for recovery or confirmation...");
    }

    // If spike is pending, check for recovery or confirmation
    if (humiditySpikePending) {
      unsigned long spikeDuration = currentTime - spikeDetectedTime;
      float recoveryThreshold = baselineHumidity + HUMIDITY_SPIKE_TOLERANCE;

      // Check if humidity has returned to near-baseline (spike was transient)
      if (newHumidity <= recoveryThreshold) {
        Serial.println("✓ Humidity spike filtered: sensor returned to baseline (transient glitch)");
        Serial.print("Recovery time: ");
        Serial.print(spikeDuration / 1000.0, 1);
        Serial.println(" seconds");
        humiditySpikePending = false;
        baselineHumidity = -999.0;
      }
      // Timeout: if spike hasn't recovered in 30 seconds, treat as real leak
      else if (spikeDuration > SPIKE_RECOVERY_TIMEOUT) {
        Serial.println("\n⚠️⚠️⚠️ HIGH HUMIDITY ALARM (CONFIRMED) ⚠️⚠️⚠️");
        Serial.print("Humidity spike persisted for ");
        Serial.print(spikeDuration / 1000.0, 1);
        Serial.println(" seconds - treating as real leak");
        Serial.println("POSSIBLE WATER LEAK - Emergency shutdown!");

        highHumidityAlarmActive = true;
        humiditySpikePending = false;

        // EMERGENCY SHUTDOWN
        // Pump is controlled externally; firmware cannot stop the pump. Raise alert and secure valves.
        Serial.println("🛑 Pump control is external - raising alert and securing system");

        // Close all loop valves
        if (rearLoopOpen) {
          controlRearLoopValve(false);
          Serial.println("🛑 Rear loop CLOSE");
        }
        if (engineLoopOpen) {
          controlEngineLoopValve(false);
          engineLoopMqttRequest = false;
          preferences.putBool("engLoopMqtt", false);
          Serial.println("🛑 Engine loop CLOSE");
        }
        if (frontLoopOpen) {
          controlFrontLoopValve(false);
          frontLoopMqttRequest = false;
          preferences.putBool("frtLoopMqtt", false);
          Serial.println("🛑 Front loop CLOSE");
        }
        if (greyWaterHeaterOn) {
          controlGreyWaterHeater(false);
          Serial.println("🛑 Grey water heater CLOSE");
        }
        if (mainWaterPumpOn) {
          controlMainWaterPump(false);
          Serial.println("🛑 Main water pump OFF");
        }

        // Publish alerts
        publishAlertRaw("High Humidity - possible leak");
        client.publish(Topic_Plumbing_Status, "LEAK_DETECTED_SHUTDOWN", true);

        Serial.println("\nSystem locked. Send MQTT command to clear:");
        Serial.println("Topic: ClearHighHumidityCommand");
        Serial.println("Message: clear\n");
      }
    }
  }

  // Update tracking variables
  environmentHumidity = newHumidity;
  lastHumidity = newHumidity;
  lastHumidityCheck = currentTime;
}

void publishBME280() {
  if (!bmeAvailable) return;

  char tempStr[8];

  dtostrf(environmentTemp, 4, 2, tempStr);
  client.publish(Topic_Environment_Temp, tempStr);

  dtostrf(environmentHumidity, 4, 2, tempStr);
   client.publish(Topic_Environment_Humidity, tempStr);

  dtostrf(environmentPressure, 6, 2, tempStr);
  client.publish(Topic_Environment_Pressure, tempStr);
}

// ============================================================================
// VALVE CONTROL FUNCTIONS
// ============================================================================
void controlFreshWaterHeater(bool turnOn) {
  digitalWrite(FRESH_WATER_HEATER_PIN, turnOn ? LOW : HIGH);
  freshWaterHeaterOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_FreshWtrHeat_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("🔥 Fresh Water Heater: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlGreyWaterHeater(bool turnOn) {
  digitalWrite(GREY_WATER_HEATER_PIN, turnOn ? LOW : HIGH);
  greyWaterHeaterOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_GreyWtrHeat_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("🔥 Grey Water Heater: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlExhaustFan(bool turnOn) {
  digitalWrite(EXHAUST_FAN_PIN, turnOn ? LOW : HIGH);
  exhaustFanOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_ExhaustFan_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("💨 Exhaust Fan: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlRearLoopValve(bool open) {
  digitalWrite(REAR_LOOP_VALVE_PIN, open ? LOW : HIGH);
  rearLoopOpen = open;
  saveValveStatesToEEPROM();
  client.publish(Topic_RearLoop_Status, open ? "OPEN" : "CLOSE", true);
  Serial.print("🌊 Rear Loop: ");
  Serial.println(open ? "OPEN" : "CLOSE");
}

void controlEngineLoopValve(bool open) {
  digitalWrite(ENGINE_LOOP_VALVE_PIN, open ? LOW : HIGH);
  engineLoopOpen = open;
  saveValveStatesToEEPROM();
  client.publish(Topic_EngineLoop_Status, open ? "OPEN" : "CLOSE", true);
  Serial.print("🌊 Engine Loop: ");
  Serial.println(open ? "OPEN" : "CLOSE");
}

void controlFrontLoopValve(bool open) {
  digitalWrite(FRONT_LOOP_VALVE_PIN, open ? LOW : HIGH);
  frontLoopOpen = open;
  saveValveStatesToEEPROM();
  client.publish(Topic_FrontLoop_Status, open ? "OPEN" : "CLOSE", true);
  Serial.print("🌊 Front Loop: ");
  Serial.println(open ? "OPEN" : "CLOSE");
}

void controlMainWaterPump(bool turnOn) {
  // Normally closed relay: HIGH (relay OFF) = pump ON, LOW (relay ON) = pump OFF
  digitalWrite(MAIN_WATER_PUMP_PIN, turnOn ? HIGH : LOW);
  mainWaterPumpOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_MainPump_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("💧 Main Water Pump: ");
  Serial.println(turnOn ? "RUN" : "STOP");
}

void controlDHWSolinoid(bool open) {
  digitalWrite(DOMESTIC_HW_SOLINOID_PIN, open ? LOW : HIGH);
  DHWSolinoidOpen = open;
  saveValveStatesToEEPROM();
  client.publish(Topic_DomesticHW_Status, open ? "OPEN" : "CLOSE", true);
  Serial.print("🔥 Domestic Hot Water Loop: ");
  Serial.println(open ? "OPEN" : "CLOSE");
}

void controlDieselHeater(bool turnOn) {
  // Simple ON/OFF control
  // ACTIVE LOW RELAYS: LOW = relay energized (circuit ON), HIGH = relay off (circuit OFF)
  // Pin 1 (DIESEL_HEATER_PIN) = White wire (Circuit 1)
  // Pin 2 (DIESEL_HEATER_2_PIN) = Blue wire (Circuit 2)
  // Pin 3 (DIESEL_HEATER_3_PIN) = Yellow wire (Circuit 3)

  if (turnOn) {
    // ON: Pins 1&3 LOW, Pin 2 HIGH (circuits 1 & 3 ON, circuit 2 OFF)
    digitalWrite(DIESEL_HEATER_PIN, LOW);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, LOW);
  } else {
    // OFF: All pins HIGH (all relays off, circuits disabled)
    digitalWrite(DIESEL_HEATER_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, HIGH);
  }
  dieselHeaterOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_DieselHtr_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("🔥 Diesel Heater: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlElectricHeater(bool turnOn) {
  // Electric heater on pin 37: HIGH = ON, LOW = OFF
  digitalWrite(ELECTRIC_HEATER_PIN, turnOn ? HIGH : LOW);
  electricHeaterOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_ElectricHtr_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("⚡ Electric Heater: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

// ============================================================================
// CABINET LOCK CONTROL
// ============================================================================
void lockCabinet() {
  if (!lockServo.attached()) {
    lockServo.setPeriodHertz(50);
    lockServo.attach(SERVO_PIN, 500, 2400);
  }

  lockServo.write(LOCKED_POSITION);
  currentLockState = STATE_LOCKED;
  delay(500);

  lockServo.detach();

  saveValveStatesToEEPROM();
  client.publish(Topic_CabLock_Status, "LOCKED", true);
  Serial.println("🔒 Cabinet: LOCKED");
}

void unlockCabinet() {
  if (!lockServo.attached()) {
    lockServo.setPeriodHertz(50);
    lockServo.attach(SERVO_PIN, 500, 2400);
  }

  lockServo.write(UNLOCKED_POSITION);
  currentLockState = STATE_UNLOCKED;
  delay(500);

  lockServo.detach();

  saveValveStatesToEEPROM();
  client.publish(Topic_CabLock_Status, "UNLOCKED", true);
  Serial.println("🔓 Cabinet: UNLOCKED");
}

void processLockCommand(String command) {
  if (command == "LOCK") {
    lockCabinet();
  } else if (command == "UNLOCK") {
    unlockCabinet();
  } else if (command == "TOGGLE") {
    if (currentLockState == STATE_LOCKED) {
      unlockCabinet();
    } else {
      lockCabinet();
    }
  } else if (command == "STATUS") {
    const char* status = (currentLockState == STATE_LOCKED) ? "LOCKED" : "UNLOCKED";
    client.publish(Topic_CabLock_Status, status, true);
  }
}

// ============================================================================
// CRITICAL AUTOMATION
// ============================================================================
void handleCriticalAutomation() {
  // Fresh water tank heater automation (only when global plumbing mode is AUTO)
  if (plumbingMode == "AUTO") {
    if (freshWaterTemp > -999.0) {
      if (freshWaterTemp < freshWaterTempTarget && !freshWaterHeaterOn) {
        controlFreshWaterHeater(true);
      } else if (freshWaterTemp > (freshWaterTempTarget + 2.0) && freshWaterHeaterOn) {
        controlFreshWaterHeater(false);
      }
    }
  }

  // Grey water tank heater automation (only when global plumbing mode is AUTO)
  if (plumbingMode == "AUTO") {
    if (greyWaterTemp > -999.0) {
      if (greyWaterTemp < greyWaterTempTarget && !greyWaterHeaterOn) {
        controlGreyWaterHeater(true);
      } else if (greyWaterTemp > (greyWaterTempTarget + 2.0) && greyWaterHeaterOn) {
        controlGreyWaterHeater(false);
      }
    }
  }

  // Exhaust fan automation - temperature control using adjustable setpoint
  if (plumbingMode == "AUTO" && bmeAvailable) {
    if (environmentTemp > -999.0) {
      if (environmentTemp >= exhaustFanTempSet && !exhaustFanOn) {
        controlExhaustFan(true);
        Serial.print("🌡️ Exhaust fan ON - Temperature above ");
        Serial.print(exhaustFanTempSet);
        Serial.println("°F");
      }
      // Hysteresis of 2°F
      else if (environmentTemp < (exhaustFanTempSet - 3.0) && exhaustFanOn) {
        controlExhaustFan(false);
        Serial.print("🌡️ Exhaust fan OFF - Temperature below ");
        Serial.print(exhaustFanTempSet - 3.0);
        Serial.println("°F");
      }
    }
  }
}

void checkAlerts() {
  // Freeze warning
  bool freezeCondition = false;
  if (freshWaterTemp > -999.0 && freshWaterTemp < FREEZE_WARNING_TEMP) freezeCondition = true;
  if (greyWaterTemp > -999.0 && greyWaterTemp < FREEZE_WARNING_TEMP) freezeCondition = true;

  if (freezeCondition && !freezeWarningActive) {
    freezeWarningActive = true;
    publishAlertRaw("Tank Freeze Warning ACTIVE");
    Serial.println("⚠️ FREEZE WARNING!");
  } else if (!freezeCondition && freezeWarningActive) {
    freezeWarningActive = false;
    refreshAlertPublish();
  }

  // High temperature warning
  bool highTempCondition = false;
  if (hydronicTemp > HIGH_TEMP_WARNING) highTempCondition = true;

  if (highTempCondition && !highTempWarningActive) {
    highTempWarningActive = true;
    publishAlertRaw("High Hydronic Tank Temp ACTIVE");
    Serial.println("⚠️ HIGH TEMPERATURE WARNING!");
  } else if (!highTempCondition && highTempWarningActive) {
    highTempWarningActive = false;
    refreshAlertPublish();
  }
}

// ============================================================================
// STATUS & HEALTH
// ============================================================================
void publishStatus() {
  // All valve states already published by control functions with retained flag
  // Publish manifold pressure
  char pressureStr[8];
  dtostrf(manifoldPressure, 4, 1, pressureStr);
  client.publish(Topic_Manifold_Pressure, pressureStr, true);

  // Publish main water pump and domestic HW loop initial states
  client.publish(Topic_MainPump_Status, mainWaterPumpOn ? "ON" : "OFF", true);
  client.publish(Topic_DomesticHW_Status, DHWSolinoidOpen ? "OPEN" : "CLOSE", true);

  // Publish plumbing mode and setpoints so HA sees them on reconnect
  client.publish(Topic_Plumbing_Mode, plumbingMode.c_str(), true);
  char fwStr[8]; dtostrf(freshWaterTempTarget, 4, 1, fwStr); client.publish(Topic_FreshWtrHeat_TempSet, fwStr, true);
  char gwStr[8]; dtostrf(greyWaterTempTarget, 4, 1, gwStr); client.publish(Topic_GreyWtrHeat_TempSet, gwStr, true);
  char fanStr[8]; dtostrf(exhaustFanTempSet, 4, 1, fanStr); client.publish(Topic_ExhaustFan_TempSet, fanStr, true);
}

// ============================================================================
// ALERT HELPERS
// Centralize alert publishing so Topic_Alert shows the latest active alert
// or "OKAY" when no alerts are active.
// ============================================================================
String lastAlert = "OKAY";

void publishAlertRaw(const String &msg) {
  lastAlert = msg;
  client.publish(Topic_Alert, lastAlert.c_str(), true);
}

void refreshAlertPublish() {
  // Priority: high humidity > high temp > freeze warning > OKAY
  if (highHumidityAlarmActive) {
    publishAlertRaw("High Humidity - possible leak");
  } else if (highTempWarningActive) {
    publishAlertRaw("High Hydronic Tank Temp ACTIVE");
  } else if (freezeWarningActive) {
    publishAlertRaw("Tank Freeze Warning ACTIVE");
  } else {
    publishAlertRaw("OKAY");
  }
}

void publishHealth() {
  char uptimeStr[16];
  unsigned long uptime = millis() / 1000;
  snprintf(uptimeStr, sizeof(uptimeStr), "%lu", uptime);
  client.publish(Topic_Plumbing_Uptime, uptimeStr);

  client.publish(Topic_Plumbing_Status, "ONLINE", true);
  client.publish(Topic_Firmware_Version, FIRMWARE_VERSION);

  Serial.print("📊 Uptime: ");
  Serial.print(uptime);
  Serial.println("s");
}

// ============================================================================
// CONFIGURATION PERSISTENCE
// ============================================================================
void saveConfigToPreferences() {
  preferences.putFloat("fwTarget", freshWaterTempTarget);
  preferences.putFloat("gwTarget", greyWaterTempTarget);
  preferences.putFloat("fanSet", exhaustFanTempSet);
  preferences.putString("plumbingMode", plumbingMode);
  Serial.println("💾 Configuration saved");
}

void loadConfigFromPreferences() {
  freshWaterTempTarget = preferences.getFloat("fwTarget", 40.0);
  greyWaterTempTarget = preferences.getFloat("gwTarget", 35.0);
  exhaustFanTempSet = preferences.getFloat("fanSet", 85.0);
  plumbingMode = preferences.getString("plumbingMode", "AUTO");

  // Restore loop request states (MQTT commands from GIGA/thermostat)
  engineLoopMqttRequest = preferences.getBool("engLoopMqtt", false);
  frontLoopMqttRequest = preferences.getBool("frtLoopMqtt", false);

  Serial.println("💾 Configuration loaded");
  Serial.print("  Engine Loop MQTT Request: ");
  Serial.println(engineLoopMqttRequest ? "OPEN" : "CLOSE");
  Serial.print("  Front Loop MQTT Request: ");
  Serial.println(frontLoopMqttRequest ? "OPEN" : "CLOSE");
  Serial.print("  Plumbing Mode: ");
  Serial.println(plumbingMode);
  Serial.print("  Exhaust Fan Setpoint: ");
  Serial.print(exhaustFanTempSet);
  Serial.println(" °F");
}

// ============================================================================
// EEPROM STATE MANAGEMENT (Board is Authoritative)
// ============================================================================
// Deferred write: mark dirty and flush after debounce to reduce flash wear.
bool eepromDirty = false;
unsigned long eepromDirtyTime = 0;
const unsigned long EEPROM_DEBOUNCE_MS = 2000;  // Wait 2s after last change before writing

void saveValveStatesToEEPROM() {
  // Mark dirty - actual write happens in flushEEPROM() called from loop()
  eepromDirty = true;
  eepromDirtyTime = millis();
}

void flushEEPROM() {
  if (!eepromDirty) return;
  if (millis() - eepromDirtyTime < EEPROM_DEBOUNCE_MS) return;

  eepromDirty = false;
  preferences.putBool("freshHeat", freshWaterHeaterOn);
  preferences.putBool("greyHeat", greyWaterHeaterOn);
  preferences.putBool("exhaustFan", exhaustFanOn);
  preferences.putBool("rearLoop", rearLoopOpen);
  preferences.putBool("engineLoop", engineLoopOpen);
  preferences.putBool("frontLoop", frontLoopOpen);
  preferences.putBool("dieselHtr", dieselHeaterOn);
  preferences.putBool("electricHtr", electricHeaterOn);
  preferences.putBool("mainPump", mainWaterPumpOn);
  preferences.putBool("dhwSolenoid", DHWSolinoidOpen);
  preferences.putBool("cabLock", currentLockState == STATE_LOCKED);
  Serial.println("💾 Valve states saved to EEPROM");
}

void restoreValveStatesFromEEPROM() {
  Serial.println("\n🔄 Restoring valve states from EEPROM (AUTHORITATIVE)...");

  // Restore state variables from EEPROM
  freshWaterHeaterOn = preferences.getBool("freshHeat", false);
  greyWaterHeaterOn = preferences.getBool("greyHeat", false);
  exhaustFanOn = preferences.getBool("exhaustFan", false);
  rearLoopOpen = preferences.getBool("rearLoop", false);
  engineLoopOpen = preferences.getBool("engineLoop", false);
  frontLoopOpen = preferences.getBool("frontLoop", false);
  dieselHeaterOn = preferences.getBool("dieselHtr", false);
  electricHeaterOn = preferences.getBool("electricHtr", false);
  mainWaterPumpOn = preferences.getBool("mainPump", true);  // Default ON
  DHWSolinoidOpen = preferences.getBool("dhwSolenoid", false);
  bool savedLockState = preferences.getBool("cabLock", false);  // Default UNLOCKED

  // Apply restored states to physical hardware (active LOW relays)
  digitalWrite(FRESH_WATER_HEATER_PIN, freshWaterHeaterOn ? LOW : HIGH);
  digitalWrite(GREY_WATER_HEATER_PIN, greyWaterHeaterOn ? LOW : HIGH);
  digitalWrite(EXHAUST_FAN_PIN, exhaustFanOn ? LOW : HIGH);
  digitalWrite(REAR_LOOP_VALVE_PIN, rearLoopOpen ? LOW : HIGH);
  digitalWrite(ENGINE_LOOP_VALVE_PIN, engineLoopOpen ? LOW : HIGH);
  digitalWrite(FRONT_LOOP_VALVE_PIN, frontLoopOpen ? LOW : HIGH);
  digitalWrite(MAIN_WATER_PUMP_PIN, mainWaterPumpOn ? HIGH : LOW);
  digitalWrite(DOMESTIC_HW_SOLINOID_PIN, DHWSolinoidOpen ? LOW : HIGH);

  // Electric heater: HIGH = ON, LOW = OFF
  digitalWrite(ELECTRIC_HEATER_PIN, electricHeaterOn ? HIGH : LOW);

  // Restore diesel heater state (simple on/off)
  if (dieselHeaterOn) {
    // ON: Pins 1&3 LOW, Pin 2 HIGH
    digitalWrite(DIESEL_HEATER_PIN, LOW);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, LOW);
  } else {
    // OFF: All pins HIGH
    digitalWrite(DIESEL_HEATER_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, HIGH);
  }

  // Restore cabinet lock position
  if (savedLockState) {
    lockServo.setPeriodHertz(50);
    lockServo.attach(SERVO_PIN, 500, 2400);
    lockServo.write(LOCKED_POSITION);
    currentLockState = STATE_LOCKED;
    delay(500);
    lockServo.detach();
  } else {
    // Already unlocked from setup init, just update state
    currentLockState = STATE_UNLOCKED;
  }

  Serial.println("  ✓ Hardware relays restored from EEPROM");
  Serial.println("  Fresh Water Heater: " + String(freshWaterHeaterOn ? "ON" : "OFF"));
  Serial.println("  Grey Water Heater: " + String(greyWaterHeaterOn ? "ON" : "OFF"));
  Serial.println("  Exhaust Fan: " + String(exhaustFanOn ? "ON" : "OFF"));
  Serial.println("  Rear Loop: " + String(rearLoopOpen ? "OPEN" : "CLOSE"));
  Serial.println("  Engine Loop: " + String(engineLoopOpen ? "OPEN" : "CLOSE"));
  Serial.println("  Front Loop: " + String(frontLoopOpen ? "OPEN" : "CLOSE"));
  Serial.println("  Diesel Heater: " + String(dieselHeaterOn ? "ON" : "OFF"));
  Serial.println("  Electric Heater: " + String(electricHeaterOn ? "ON" : "OFF"));
  Serial.println("  Main Water Pump: " + String(mainWaterPumpOn ? "ON" : "OFF"));
  Serial.println("  Domestic HW: " + String(DHWSolinoidOpen ? "OPEN" : "CLOSE"));
  Serial.println("  Cabinet Lock: " + String(currentLockState == STATE_LOCKED ? "LOCKED" : "UNLOCKED"));
  Serial.println("✓ Board is now AUTHORITATIVE source\n");
}

void publishAllStatesToBroker() {
  Serial.println("📡 Publishing authoritative states to MQTT broker...");

  // Publish all current states with retained flag
  client.publish(Topic_FreshWtrHeat_Status, freshWaterHeaterOn ? "ON" : "OFF", true);
  client.publish(Topic_GreyWtrHeat_Status, greyWaterHeaterOn ? "ON" : "OFF", true);
  client.publish(Topic_ExhaustFan_Status, exhaustFanOn ? "ON" : "OFF", true);
  client.publish(Topic_RearLoop_Status, rearLoopOpen ? "OPEN" : "CLOSE", true);
  client.publish(Topic_EngineLoop_Status, engineLoopOpen ? "OPEN" : "CLOSE", true);
  client.publish(Topic_FrontLoop_Status, frontLoopOpen ? "OPEN" : "CLOSE", true);
  client.publish(Topic_MainPump_Status, mainWaterPumpOn ? "ON" : "OFF", true);
  client.publish(Topic_DomesticHW_Status, DHWSolinoidOpen ? "OPEN" : "CLOSE", true);
  client.publish(Topic_DieselHtr_Status, dieselHeaterOn ? "ON" : "OFF", true);
  client.publish(Topic_ElectricHtr_Status, electricHeaterOn ? "ON" : "OFF", true);

  Serial.println("✓ Broker synchronized with board's authoritative state");
}
