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
// - Hot water pump PID control with pressure sensor
// - Valve control (rear loop, engine loop, front loop from thermostat triggers)
// - Grey tank heat loop valve control
// - Diesel heater 3-state control
// - All relay status publishing (authoritative source for valve states)
// ============================================================================

#define FIRMWARE_VERSION "v2.0.0-ESP32S2"
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

// Hot Water Pump (Publish & Subscribe)
const char* Topic_Pump_Pressure = "HotWaterPumpPressure";
const char* Topic_Pump_Speed = "HotWaterPumpSpeed";
const char* Topic_Pump_Status = "HotWaterPumpStatus";
const char* Topic_Pump_Command = "HotWaterPumpCommand";
const char* Topic_Pump_SetPoint = "HotWaterPumpSetPoint";
const char* Topic_Pump_RawADC = "HotWaterPumpRawADC";
const char* Topic_Pump_Voltage = "HotWaterPumpVoltage";

// Flow Sensor (Publish Only)
const char* Topic_Flow_Rate = "HotWaterFlowRate";
const char* Topic_Flow_Status = "HotWaterFlowStatus";

// Main Water Pump (Publish & Subscribe)
const char* Topic_MainPump_Command = "MainWaterPumpCommand";
const char* Topic_MainPump_Status = "MainWaterPumpStatus";

// Domestic Hot Water Loop (Publish & Subscribe)
const char* Topic_DomesticHW_Command = "DomesticHotWaterCommand";
const char* Topic_DomesticHW_Status = "DomesticHotWaterStatus";

// Valve Status (Publish Only - This board is authoritative)
const char* Topic_FreshWtrHeat_Status = "FreshWaterHeatStatus";
const char* Topic_GreyWtrHeat_Status = "GreyWaterHeatStatus";
const char* Topic_RearLoop_Status = "RearLoopStatus";
const char* Topic_EngineLoop_Status = "EngineLoopStatus";
const char* Topic_FrontLoop_Status = "FrontLoopStatus";
const char* Topic_ExhaustFan_Status = "ExhaustFanStatus";
const char* Topic_DieselHtr_Status = "DieselHeaterStatus";

// Valve Commands (Subscribe)
const char* Topic_FreshWtrHeat_Command = "FreshWaterHeatCommand";
const char* Topic_GreyWtrHeat_Command = "GreyWaterHeatCommand";
const char* Topic_RearLoop_Command = "RearLoopCommand";
const char* Topic_EngineLoop_Command = "EngineLoopCommand";
const char* Topic_FrontLoop_Command = "FrontLoopCommand";
const char* Topic_ExhaustFan_Command = "ExhaustFanCommand";
const char* Topic_DieselHtr_Command = "DieselHeaterCommand";

// Cabinet Lock Topics
const char* Topic_CabLock_Command = "CabLockRearPassSideCommand";
const char* Topic_CabLock_Status = "CabLockRearPassSideStatus";
const char* Topic_AllLock_Command = "CabLockAllCommand";

// Mode Topics (Subscribe)
const char* Topic_FreshWtrHeat_Mode = "FreshWaterHeatMode";
const char* Topic_GreyWtrHeat_Mode = "GreyWaterHeatMode";
const char* Topic_ExhaustFan_Mode = "ExhaustFanMode";

// Configuration Topics
const char* Topic_FreshWtrHeat_TempSet = "FreshWaterHeatTempSet";
const char* Topic_GreyWtrHeat_TempSet = "GreyWaterHeatTempSet";
const char* Topic_ExhaustFan_TempHigh = "ExhaustFanTempHigh";
const char* Topic_ExhaustFan_TempLow = "ExhaustFanTempLow";

// Alert Topics (Publish Only)
const char* Topic_Alert_Freeze_Warning = "AlertFreezeWarning";
const char* Topic_Alert_High_Temp = "AlertHighTemp";
const char* Topic_Alert_System_Error = "AlertSystemError";
const char* Topic_Alert_Pump_Error = "AlertPumpError";
const char* Topic_Alert_High_Humidity = "AlertHighHumidity";

// Safety Command Topics (Subscribe)
const char* Topic_Clear_High_Humidity = "ClearHighHumidityCommand";

// System Health (Publish Only)
const char* Topic_Plumbing_Status = "PlumbingSystemStatus";
const char* Topic_Plumbing_Uptime = "PlumbingUptime";
const char* Topic_Firmware_Version = "PlumbingFirmwareVersion";

// ============================================================================
// HARDWARE PIN DEFINITIONS - ESP32-S2 Mini
// ============================================================================
// DS18B20 Temperature Sensors
#define ONE_WIRE_BUS 15              // GPIO11 for DS18B20 data

// Relay Outputs (Active LOW)
#define FRESH_WATER_HEATER_PIN 9     // GPIO10 - Fresh water tank heater
#define GREY_WATER_HEATER_PIN 2      // GPIO2 - Grey water tank heater
#define EXHAUST_FAN_PIN 21            // GPIO3 - Exhaust fan
#define MAIN_WATER_PUMP_PIN 18       // GPIO18 - Main water pump (city water pressure)
#define DOMESTIC_HW_SOLINOID_PIN 10      // GPIO10 - Domestic hot water loop valve
#define REAR_LOOP_VALVE_PIN 5        // GPIO4 - Rear loop valve
#define ENGINE_LOOP_VALVE_PIN 3      // GPIO5 - Engine loop valve
#define FRONT_LOOP_VALVE_PIN 4       // GPIO6 - Front loop valve (controlled by plumbing board)
#define DIESEL_HEATER_PIN 8          // GPIO7 - Diesel heater relay 1 (white wire)
#define DIESEL_HEATER_2_PIN 7        // GPIO8 - Diesel heater relay 2 (blue wire)
#define DIESEL_HEATER_3_PIN 6        // GPIO9 - Diesel heater relay 3 (yellow wire)

// Cabinet Lock Servo
#define SERVO_PIN 16                 // GPIO16 - Servo control

// Hot Water Pump PWM Control
#define PUMP_PWM_PIN 12              // GPIO12 - PWM output to pump controller
#define PUMP_PWM_CHANNEL 3           // PWM channel 2
#define PUMP_PWM_FREQ 10000         // 25kHz PWM frequency
#define PUMP_PWM_RESOLUTION 8        // 8-bit resolution (0-255)

// Pressure Sensor (Analog Input)
#define PRESSURE_SENSOR_PIN 1        // ADC1_CH0 (GPIO1) - 0.5V-4.5V = 0-80 PSI

// Flow sensor (Digital Input)
#define FLOW_SENSOR_PIN 17           // GPIO15 - Flow sensor input (not used in current code)

// I2C for BME280
#define BME280_SDA 33                // GPIO33 - I2C SDA
#define BME280_SCL 35                // GPIO35 - I2C SCL

// ============================================================================
// SENSOR SETUP
// ============================================================================
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);

// DS18B20 sensor indices
#define SENSOR_HYDRONIC 2
#define SENSOR_FRESH_WATER 0
#define SENSOR_GREY_WATER 1
#define SENSOR_RETURN_MANIFOLD 3

// BME280 Environmental Sensor
Adafruit_BME280 bme;
bool bmeAvailable = false;

// Servo for cabinet lock
Servo lockServo;

// ============================================================================
// PUMP SPEED CONTROL (Fixed Speeds)
// ============================================================================
// Pump speed settings (adjustable)
const float PUMP_SPEED_FLOW_DETECTED = 100.0;  // 100% when flow detected
const float PUMP_SPEED_LOOPS_OPEN = 70.0;      // 70% when valves open (no flow)
const float PUMP_MIN_SPEED = 30.0;             // Minimum threshold to run pump

// Safety limits for pressure monitoring
const float PRESSURE_MIN_ALARM = 2.0;         // Alarm if < 2 PSI while running
const float PRESSURE_MAX_ALARM = 40.0;        // Alarm if > 40 PSI while running
const unsigned long PUMP_STARTUP_GRACE_PERIOD = 5000;  // 5 seconds for pressure to build

// Pressure & Pump State
float currentPressure = 0.0;
float pumpSpeed = 0.0;  // 0-100%
bool pumpRunning = false;
bool pumpAlarm = false;
String pumpAlarmReason = "";
unsigned long pumpStartTime = 0;

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

// Pump Auto Control
String pumpMode = "AUTO";  // AUTO or MANUAL
bool pumpAutoStarted = false;  // Track if pump was auto-started
unsigned long lastManualPumpControl = 0;  // Track last manual interaction
const unsigned long MANUAL_TIMEOUT = 30000;  // 30 seconds to revert to AUTO

// Loop Timer Management (10-minute Delays before adding pressure boost)
const unsigned long LOOP_DELAY_TIME = 600000;  // 10 minutes in milliseconds 600000
unsigned long rearLoopOpenTime = 0;
unsigned long engineLoopOpenTime = 0;
unsigned long frontLoopOpenTime = 0;
unsigned long greyLoopOpenTime = 0;
bool rearLoopDelayComplete = false;
bool engineLoopDelayComplete = false;
bool frontLoopDelayComplete = false;
bool greyLoopDelayComplete = false;

// Pressure Relief (open DHW valve after pump stops)
bool pressureReliefActive = false;
unsigned long pressureReliefStartTime = 0;
const unsigned long PRESSURE_RELIEF_DURATION = 2000;  // 2 seconds

// ============================================================================
// CONFIGURATION VARIABLES
// ============================================================================
float freshWaterTempTarget = 40.0;    // °F
float greyWaterTempTarget = 35.0;     // °F
float exhaustFanTempHigh = 85.0;      // °F
float exhaustFanTempLow = 75.0;       // °F

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
bool dieselHeaterState = 0;  // 0=OFF, 1=PUMP ONLY, 2=HIGH
bool mainWaterPumpOn = true;  // Initialized HIGH (relay OFF) = pump ON for normally closed
bool DHWSolinoidOpen = false;

// Modes
String freshWaterMode = "AUTO";
String greyWaterMode = "AUTO";
String exhaustFanMode = "AUTO";

// Debug Publishing
unsigned long lastDebugPublish = 0;
const unsigned long DEBUG_PUBLISH_INTERVAL = 10000;  // 10 seconds

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

// Alert States
bool freezeWarningActive = false;
bool highTempWarningActive = false;
bool highHumidityAlarmActive = false;

// Humidity Leak Detection
float lastHumidity = -999.0;
const float HUMIDITY_SPIKE_THRESHOLD = 10.0;  // % increase that triggers alarm
const float HUMIDITY_RATE_WINDOW = 60000;     // 1 minute window for rate check
unsigned long lastHumidityCheck = 0;

// Timing Variables
unsigned long lastSensorRead = 0;
unsigned long lastPressureRead = 0;
unsigned long lastPIDUpdate = 0;
unsigned long lastStatusPublish = 0;
unsigned long lastHealthPublish = 0;
const unsigned long SENSOR_READ_INTERVAL = 5000;      // 5 seconds
const unsigned long PRESSURE_READ_INTERVAL = 100;     // 100ms (fast for PID)
const unsigned long PID_UPDATE_INTERVAL = 100;        // 100ms
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

// Pressure & Pump Control
float readPressure();
void controlPump(float speedPercent);
void checkPumpSafety();
void calculateFlowRate();
void publishFlowRate();
void handleAutoPump();
bool shouldPumpBeRunning();
void IRAM_ATTR flowSensorISR();

// Valve Control Functions
void controlFreshWaterHeater(bool turnOn);
void controlGreyWaterHeater(bool turnOn);
void controlExhaustFan(bool turnOn);
void controlRearLoopValve(bool open);
void controlEngineLoopValve(bool open);
void controlFrontLoopValve(bool open);
void controlDieselHeater(int state);
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
void restoreValveStates();

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
  pinMode(MAIN_WATER_PUMP_PIN, OUTPUT);
  pinMode(DOMESTIC_HW_SOLINOID_PIN, OUTPUT);

  // Initialize all relays to OFF (HIGH = OFF for active LOW)
  digitalWrite(FRESH_WATER_HEATER_PIN, HIGH);
  digitalWrite(GREY_WATER_HEATER_PIN, HIGH);
  digitalWrite(EXHAUST_FAN_PIN, HIGH);
  digitalWrite(REAR_LOOP_VALVE_PIN, HIGH);
  digitalWrite(ENGINE_LOOP_VALVE_PIN, HIGH);
  digitalWrite(FRONT_LOOP_VALVE_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_3_PIN, HIGH);
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

  // Initialize pump PWM
  ledcSetup(PUMP_PWM_CHANNEL, PUMP_PWM_FREQ, PUMP_PWM_RESOLUTION);
  ledcAttachPin(PUMP_PWM_PIN, PUMP_PWM_CHANNEL);
  ledcWrite(PUMP_PWM_CHANNEL, 0);  // Start at 0% duty cycle

  Serial.print("✓ Pump PWM initialized on GPIO");
  Serial.print(PUMP_PWM_PIN);
  Serial.print(" at ");
  Serial.print(PUMP_PWM_FREQ);
  Serial.println(" Hz (starting at 0%)");

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

  // Initialize DS18B20 temperature sensors
  tempSensors.begin();
  delay(100);  // Give sensors time to initialize

  int sensorCount = tempSensors.getDeviceCount();
  Serial.print("✓ Found ");
  Serial.print(sensorCount);
  Serial.println(" DS18B20 temperature sensor(s)");

  // Set resolution and print addresses for all sensors
  if (sensorCount > 0) {
    Serial.println("Sensor addresses:");
    for (int i = 0; i < sensorCount; i++) {
      DeviceAddress addr;
      if (tempSensors.getAddress(addr, i)) {
        // Set resolution for this sensor
        tempSensors.setResolution(addr, 12);

        // Print address
        Serial.print("  Sensor ");
        Serial.print(i);
        Serial.print(": 0x");
        for (uint8_t j = 0; j < 8; j++) {
          if (addr[j] < 16) Serial.print("0");
          Serial.print(addr[j], HEX);
        }
        Serial.println();
      }
    }
  }

  // Connect to WiFi
  setupWifi();

  // Setup OTA
  setupOTA();

  // Initialize task watchdog: reset device on panic after timeout
  esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL); // add current task (the loop task)
  Serial.print("✓ Task watchdog enabled (");
  Serial.print(WATCHDOG_TIMEOUT_SEC);
  Serial.println("s)");

  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(512);

  // Restore valve states to match saved MQTT requests
  restoreValveStates();

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
      currentPressure = readPressure();
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

    // Check for manual mode timeout (revert to AUTO after 30 seconds)
    if (pumpMode == "MANUAL" && (currentMillis - lastManualPumpControl >= MANUAL_TIMEOUT)) {
      pumpMode = "AUTO";
      Serial.println("⏱️ Manual timeout: Switching pump to AUTO mode");
      client.publish("HotWaterPumpMode", "AUTO");
    }

    // Handle automatic pump control (check every 100ms for fast response)
    static unsigned long lastAutoPumpCheck = 0;
    if (currentMillis - lastAutoPumpCheck >= 100) {
      lastAutoPumpCheck = currentMillis;
      handleAutoPump();

      // Debug pump state every 5 seconds
      static unsigned long lastPumpDebug = 0;
      if (currentMillis - lastPumpDebug >= 5000) {
        lastPumpDebug = currentMillis;
        Serial.print("🔍 Pump Debug: Mode=");
        Serial.print(pumpMode);
        Serial.print(" | Running=");
        Serial.print(pumpRunning);
        Serial.print(" | Auto=");
        Serial.print(pumpAutoStarted);
        Serial.print(" | Speed=");
        Serial.print(pumpSpeed);
        Serial.print("% | Pressure=");
        Serial.print(currentPressure);
        Serial.println(" PSI");
      }
    }

    // Update pump speed based on conditions
    if (currentMillis - lastPIDUpdate >= PID_UPDATE_INTERVAL) {
      lastPIDUpdate = currentMillis;

      if (pumpRunning && !pumpAlarm) {
        // Simple speed control: 100% for flow, 70% for loops
        float targetSpeed;
        if (flowDetected) {
          targetSpeed = PUMP_SPEED_FLOW_DETECTED;
        } else {
          targetSpeed = PUMP_SPEED_LOOPS_OPEN;
        }
        controlPump(targetSpeed);
        checkPumpSafety();
      } else if (!pumpRunning) {
        // Ensure pump is OFF - explicitly set PWM to 0
        controlPump(0);
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
    controlPump(0);  // Stop pump during OTA
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
  publishDiscovery("sensor", "hydronic_temp", "Hydronic Temperature", Topic_Hydronic_Temp, "temperature", "°C", nullptr, nullptr, nullptr, "mdi:water-boiler");
  publishDiscovery("sensor", "fresh_water_temp", "Fresh Water Tank Temperature", Topic_FreshWtr_Temp, "temperature", "°C");
  publishDiscovery("sensor", "grey_water_temp", "Grey Water Tank Temperature", Topic_GreyWtr_Temp, "temperature", "°C");
  publishDiscovery("sensor", "return_manifold_temp", "Return Manifold Temperature", Topic_Temp_Return, "temperature", "°C");
  publishDiscovery("sensor", "ambient_temp", "Ambient Temperature", Topic_Environment_Temp, "temperature", "°C");
  
  // Environmental Sensors
  publishDiscovery("sensor", "humidity", "Humidity", Topic_Environment_Humidity, "humidity", "%");
  publishDiscovery("sensor", "pressure", "Atmospheric Pressure", Topic_Environment_Pressure, "pressure", "hPa");
  
  // Pump Sensors
  publishDiscovery("sensor", "pump_pressure", "Hot Water Pump Pressure", Topic_Pump_Pressure, "pressure", "PSI", nullptr, nullptr, nullptr, "mdi:gauge");
  publishDiscovery("sensor", "pump_speed", "Hot Water Pump Speed", Topic_Pump_Speed, nullptr, "%", nullptr, nullptr, nullptr, "mdi:pump");
  publishDiscovery("sensor", "pump_status", "Hot Water Pump Status", Topic_Pump_Status, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:pump");
  
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
  
  // Diesel Heater (select entity for 3-state control)
  String heaterTopic = String("homeassistant/select/") + deviceId + "/diesel_heater/config";
  String heaterPayload = "{\"name\":\"Diesel Heater\","
                         "\"unique_id\":\"" + deviceId + "_diesel_heater\","
                         "\"state_topic\":\"" + String(Topic_DieselHtr_Status) + "\","
                         "\"command_topic\":\"" + String(Topic_DieselHtr_Command) + "\","
                         "\"options\":[\"OFF\",\"PUMP ONLY\",\"HIGH\"],"
                         "\"icon\":\"mdi:fire\","
                         "\"device\":" + device + "}";
  client.publish(heaterTopic.c_str(), heaterPayload.c_str(), true);
  delay(50);
  
  // Binary Sensors (Alerts)
  publishDiscovery("binary_sensor", "freeze_warning", "Freeze Warning", Topic_Alert_Freeze_Warning, nullptr, nullptr, nullptr, "ON", "OFF", "mdi:snowflake-alert");
  publishDiscovery("binary_sensor", "high_temp_alert", "High Temperature Alert", Topic_Alert_High_Temp, nullptr, nullptr, nullptr, "ON", "OFF", "mdi:thermometer-alert");
  publishDiscovery("binary_sensor", "system_error", "System Error", Topic_Alert_System_Error, nullptr, nullptr, nullptr, "ON", "OFF", "mdi:alert-circle");
  publishDiscovery("binary_sensor", "pump_error", "Pump Error", Topic_Alert_Pump_Error, nullptr, nullptr, nullptr, "ON", "OFF", "mdi:pump-off");
  publishDiscovery("binary_sensor", "high_humidity_alert", "High Humidity Alert (Leak Detection)", Topic_Alert_High_Humidity, nullptr, nullptr, nullptr, "ON", "OFF", "mdi:water-alert");
  
  // Cabinet Lock
  publishDiscovery("lock", "cabinet_lock", "Rear Pass Cabinet Lock", Topic_CabLock_Status, nullptr, nullptr, Topic_CabLock_Command, "LOCK", "UNLOCK", "mdi:lock");
  
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
  if (client.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
    Serial.println(" connected!");
    mqttFailCount = 0;
    publishHomeAssistantDiscovery();
    subscribeToTopics();
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

  // Modes
  client.subscribe(Topic_FreshWtrHeat_Mode);
  client.subscribe(Topic_GreyWtrHeat_Mode);
  client.subscribe(Topic_ExhaustFan_Mode);

  // Configuration
  client.subscribe(Topic_FreshWtrHeat_TempSet);
  client.subscribe(Topic_GreyWtrHeat_TempSet);
  client.subscribe(Topic_ExhaustFan_TempHigh);
  client.subscribe(Topic_ExhaustFan_TempLow);

  // Cabinet lock
  client.subscribe(Topic_CabLock_Command);
  client.subscribe(Topic_AllLock_Command);

  // Pump control
  client.subscribe(Topic_Pump_Command);
  client.subscribe(Topic_Pump_SetPoint);

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
  else if (topicStr == Topic_FreshWtrHeat_Mode) {
    freshWaterMode = message;
  }
  else if (topicStr == Topic_FreshWtrHeat_TempSet) {
    freshWaterTempTarget = message.toFloat();
    saveConfigToPreferences();
  }
  // Grey water heater
  else if (topicStr == Topic_GreyWtrHeat_Command) {
    controlGreyWaterHeater(message == "ON");
  }
  else if (topicStr == Topic_GreyWtrHeat_Mode) {
    greyWaterMode = message;
  }
  else if (topicStr == Topic_GreyWtrHeat_TempSet) {
    greyWaterTempTarget = message.toFloat();
    saveConfigToPreferences();
  }
  // Exhaust fan
  else if (topicStr == Topic_ExhaustFan_Command) {
    controlExhaustFan(message == "ON");
  }
  else if (topicStr == Topic_ExhaustFan_Mode) {
    exhaustFanMode = message;
  }
  else if (topicStr == Topic_ExhaustFan_TempHigh) {
    exhaustFanTempHigh = message.toFloat();
    saveConfigToPreferences();
  }
  else if (topicStr == Topic_ExhaustFan_TempLow) {
    exhaustFanTempLow = message.toFloat();
    saveConfigToPreferences();
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
      Serial.print(newState ? "OPEN" : "CLOSED");
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
      Serial.print(newState ? "OPEN" : "CLOSED");
      Serial.println(" (saved)");
    }
    controlFrontLoopValve(newState);
  }
  // Diesel heater
  else if (topicStr == Topic_DieselHtr_Command) {
    if (message == "OFF") controlDieselHeater(0);
    else if (message == "PUMP ONLY") controlDieselHeater(1);
    else if (message == "HIGH") controlDieselHeater(2);
  }
  // Main water pump
  else if (topicStr == Topic_MainPump_Command) {
    if (message == "ON") controlMainWaterPump(true);
    else if (message == "OFF") controlMainWaterPump(false);
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
        client.publish(Topic_Alert_High_Humidity, "CLEARED");
        client.publish(Topic_Pump_Status, "OK");
        // Reset humidity baseline
        lastHumidity = environmentHumidity;
        lastHumidityCheck = millis();
      }
    }
  }
  // Pump control
  else if (topicStr == Topic_Pump_Command) {
    if (message == "ON") {
      // Don't allow manual start if high humidity alarm is active
      if (highHumidityAlarmActive) {
        Serial.println("⚠️ Cannot start pump: High humidity alarm active!");
        client.publish(Topic_Pump_Status, "BLOCKED_HIGH_HUMIDITY");
        return;
      }

      pumpMode = "MANUAL";  // Switch to manual mode
      lastManualPumpControl = millis();  // Reset inactivity timer
      pumpRunning = true;
      pumpAutoStarted = false;
      pumpAlarm = false;
      pumpAlarmReason = "";
      pumpStartTime = millis();

      // Publish status immediately
      client.publish(Topic_Pump_Status, "RUNNING", true);
      client.publish("HotWaterPumpMode", "MANUAL");
      Serial.println("💧 Hot Water Pump: MANUAL START (auto-revert in 30s)");

    } else if (message == "OFF") {
      pumpMode = "MANUAL";  // Switch to manual mode
      lastManualPumpControl = millis();  // Reset inactivity timer
      pumpRunning = false;
      pumpAutoStarted = false;
      controlPump(0);

      // Publish status immediately
      client.publish(Topic_Pump_Status, "STOPPED", true);
      client.publish("HotWaterPumpMode", "MANUAL");
      Serial.println("💧 Hot Water Pump: MANUAL STOP (auto-revert in 30s)");
    } else if (message == "AUTO") {
      pumpMode = "AUTO";
      client.publish("HotWaterPumpMode", "AUTO");
      Serial.println("💧 Hot Water Pump: AUTO MODE enabled");
      // Let handleAutoPump() manage the pump state
    }
  }
}

// ============================================================================
// PRESSURE SENSOR & PID CONTROL
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

void controlPump(float speedPercent) {
  static bool wasPumpRunning = false;

  // If speed is below minimum threshold, shut pump off completely
  if (speedPercent > 0.0 && speedPercent < PUMP_MIN_SPEED) {
    pumpSpeed = 0.0;
    Serial.print("⚠️ Pump speed below ");
    Serial.print(PUMP_MIN_SPEED, 0);
    Serial.print("% (");
    Serial.print(speedPercent, 1);
    Serial.println("%) - shutting off");
  } else {
    pumpSpeed = constrain(speedPercent, 0.0, 100.0);
  }

  // Detect pump shutdown and trigger pressure relief
  // Only open valve if ALL loops are closed (no flow path open)
  if (wasPumpRunning && pumpSpeed == 0.0) {
    bool allLoopsClosed = !rearLoopOpen && !engineLoopOpen && !frontLoopOpen && !greyWaterHeaterOn;

    if (allLoopsClosed) {
      Serial.println("💨 Pump stopped (all loops closed) - opening DHW valve for pressure relief");
      pressureReliefActive = true;
      pressureReliefStartTime = millis();
      controlDHWSolinoid(true);  // Open valve to release pressure
    } else {
      Serial.println("💨 Pump stopped but loops are open - skipping pressure relief");
    }
  }

  wasPumpRunning = (pumpSpeed > 0.0);

  // Convert 0-100% to 0-255 PWM
  int pwmValue = map((int)(pumpSpeed * 10), 0, 1000, 0, 255);
  pwmValue = constrain(pwmValue, 0, 255);

  // Write PWM value to pump controller
  ledcWrite(PUMP_PWM_CHANNEL, pwmValue);

  // Publish pump speed (every 2 seconds)
  static unsigned long lastPubTime = 0;
  if (millis() - lastPubTime >= 2000) {
    lastPubTime = millis();

    char speedStr[8];
    dtostrf(pumpSpeed, 4, 1, speedStr);
    client.publish(Topic_Pump_Speed, speedStr);

    char pressureStr[8];
    dtostrf(currentPressure, 4, 1, pressureStr);
    client.publish(Topic_Pump_Pressure, pressureStr);
  }
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
// AUTOMATIC PUMP CONTROL
// ============================================================================
bool shouldPumpBeRunning() {
  // SAFETY: Never run pump if high humidity alarm is active
  if (highHumidityAlarmActive) {
    return false;
  }

  // Instant start for flow detection
  if (flowDetected) {
    return true;
  }

  // For valves, only start pump after 10-minute delay is complete
  if (greyWaterHeaterOn && greyLoopDelayComplete) {
    return true;
  }
  if (rearLoopOpen && rearLoopDelayComplete) {
    return true;
  }
  if (engineLoopOpen && engineLoopDelayComplete) {
    return true;
  }
  if (frontLoopOpen && frontLoopDelayComplete) {
    return true;
  }

  return false;
}

void handleAutoPump() {
  if (pumpMode != "AUTO") return;  // Only run in AUTO mode

  unsigned long currentTime = millis();

  // Update loop delay timers
  if (rearLoopOpen) {
    if (rearLoopOpenTime == 0) {
      rearLoopOpenTime = currentTime;
      Serial.println("🕐 Rear loop opened - starting 10-minute timer");
    }
    if (!rearLoopDelayComplete && (currentTime - rearLoopOpenTime >= LOOP_DELAY_TIME)) {
      rearLoopDelayComplete = true;
      Serial.println("✓ Rear loop 10-minute delay complete - will add pressure boost");
    }
  } else {
    rearLoopOpenTime = 0;
    rearLoopDelayComplete = false;
  }

  if (engineLoopOpen) {
    if (engineLoopOpenTime == 0) {
      engineLoopOpenTime = currentTime;
      Serial.println("🕐 Engine loop opened - starting 10-minute timer");
    }
    if (!engineLoopDelayComplete && (currentTime - engineLoopOpenTime >= LOOP_DELAY_TIME)) {
      engineLoopDelayComplete = true;
      Serial.println("✓ Engine loop 10-minute Delay complete - will add pressure boost");
    }
  } else {
    engineLoopOpenTime = 0;
    engineLoopDelayComplete = false;
  }

  if (frontLoopOpen) {
    if (frontLoopOpenTime == 0) {
      frontLoopOpenTime = currentTime;
      Serial.println("🕐 Front loop opened - starting 10-minute timer");
    }
    if (!frontLoopDelayComplete && (currentTime - frontLoopOpenTime >= LOOP_DELAY_TIME)) {
      frontLoopDelayComplete = true;
      Serial.println("✓ Front loop 10-minute Delay complete - will add pressure boost");
    }
  } else {
    frontLoopOpenTime = 0;
    frontLoopDelayComplete = false;
  }

  if (greyWaterHeaterOn) {
    if (greyLoopOpenTime == 0) {
      greyLoopOpenTime = currentTime;
      Serial.println("🕐 Grcdey loop opened - starting 10-minute timer");
    }
    if (!greyLoopDelayComplete && (currentTime - greyLoopOpenTime >= LOOP_DELAY_TIME)) {
      greyLoopDelayComplete = true;
      Serial.println("✓ Grey loop 10-minute Delay complete - will add pressure boost");
    }
  } else {
    greyLoopOpenTime = 0;
    greyLoopDelayComplete = false;
  }

  bool pumpShouldRun = shouldPumpBeRunning();

  // Start pump if needed and not running
  if (pumpShouldRun && !pumpRunning) {
    pumpRunning = true;
    pumpAutoStarted = true;
    pumpAlarm = false;
    pumpAlarmReason = "";
    pumpStartTime = millis();

    Serial.print("⚙️ AUTO START: Pump starting because ");
    if (flowDetected) Serial.print("flow detected (INSTANT) ");
    if (greyWaterHeaterOn) {
      Serial.print("grey-water-heater ");
      if (greyLoopDelayComplete) Serial.print("(boost active) ");
      else Serial.print("(waiting for boost) ");
    }
    if (rearLoopOpen) {
      Serial.print("rear-loop ");
      if (rearLoopDelayComplete) Serial.print("(boost active) ");
      else Serial.print("(waiting for boost) ");
    }
    if (engineLoopOpen) {
      Serial.print("engine-loop ");
      if (engineLoopDelayComplete) Serial.print("(boost active) ");
      else Serial.print("(waiting for boost) ");
    }
    if (frontLoopOpen) {
      Serial.print("front-loop ");
      if (frontLoopDelayComplete) Serial.print("(boost active) ");
      else Serial.print("(waiting for boost) ");
    }
    Serial.println();

    client.publish(Topic_Pump_Status, "RUNNING", true);
    // Start at appropriate speed based on condition
    float startSpeed = flowDetected ? PUMP_SPEED_FLOW_DETECTED : PUMP_SPEED_LOOPS_OPEN;
    controlPump(startSpeed);
  }
  // Stop pump if not needed and currently running (and was auto-started)
  else if (!pumpShouldRun && pumpRunning && pumpAutoStarted) {
    Serial.println("⚙️ AUTO STOP: Pump stopping (no valves open, no flow)");
    pumpRunning = false;
    pumpAutoStarted = false;
    controlPump(0);
    client.publish(Topic_Pump_Status, "STOPPED", true);
  }
}

void checkPumpSafety() {
  if (!pumpRunning) {
    // If pump is stopped and alarm is set, clear it if pressure is back in safe range
    if (pumpAlarm && currentPressure >= PRESSURE_MIN_ALARM && currentPressure <= PRESSURE_MAX_ALARM) {
      Serial.println("✓ Pump alarm cleared - pressure back to safe range");
      pumpAlarm = false;
      pumpAlarmReason = "";
      client.publish(Topic_Pump_Status, "OK");
      client.publish(Topic_Alert_Pump_Error, "CLEARED");
    }
    return;
  }

  // Allow startup grace period for pressure to build
  unsigned long pumpRunDuration = millis() - pumpStartTime;
  if (pumpRunDuration < PUMP_STARTUP_GRACE_PERIOD) {
    // Still in startup period - don't check pressure alarms yet
    return;
  }

  // Check if pressure is out of safe range
  if (currentPressure < PRESSURE_MIN_ALARM) {
    if (!pumpAlarm) {  // Only trigger alarm once
      pumpAlarm = true;
      pumpAlarmReason = "PRESSURE_LOW";
      pumpRunning = false;
      controlPump(0);

      Serial.println("⚠️ PUMP ALARM: Pressure below 2 PSI!");
      client.publish(Topic_Alert_Pump_Error, "PRESSURE_LOW");
      client.publish(Topic_Pump_Status, "ALARM_LOW_PRESSURE");
    }
  }
  else if (currentPressure > PRESSURE_MAX_ALARM) {
    if (!pumpAlarm) {  // Only trigger alarm once
      pumpAlarm = true;
      pumpAlarmReason = "PRESSURE_HIGH";
      pumpRunning = false;
      controlPump(0);

      Serial.println("⚠️ PUMP ALARM: Pressure above threshold!");
      Serial.println("💨 Opening DHW valve for emergency pressure relief");

      // Emergency pressure relief - open DHW valve
      controlDHWSolinoid(true);
      pressureReliefActive = true;
      pressureReliefStartTime = millis();

      client.publish(Topic_Alert_Pump_Error, "PRESSURE_HIGH");
      client.publish(Topic_Pump_Status, "ALARM_HIGH_PRESSURE");
    }
  }
  else if (pumpAlarm) {
    // Clear alarm if pressure returns to safe range
    Serial.println("✓ Pump alarm auto-cleared - pressure back to safe range");
    pumpAlarm = false;
    pumpAlarmReason = "";
    client.publish(Topic_Pump_Status, "OK");
    client.publish(Topic_Alert_Pump_Error, "CLEARED");
  }
}

// ============================================================================
// TEMPERATURE SENSORS
// ============================================================================
void readTemperatures() {
  tempSensors.requestTemperatures();

  float tempC;

  tempC = tempSensors.getTempCByIndex(SENSOR_HYDRONIC);
  hydronicTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  tempC = tempSensors.getTempCByIndex(SENSOR_FRESH_WATER);
  freshWaterTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  tempC = tempSensors.getTempCByIndex(SENSOR_GREY_WATER);
  greyWaterTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;

  tempC = tempSensors.getTempCByIndex(SENSOR_RETURN_MANIFOLD);
  returnManifoldTemp = (tempC != DEVICE_DISCONNECTED_C) ? (tempC * 9.0 / 5.0) + 32.0 : -999.0;
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
  environmentTemp = (tempC * 9.0 / 5.0) + 32.0;
  float newHumidity = bme.readHumidity();
  environmentPressure = bme.readPressure() / 100.0F;

  // Check for sudden humidity spike (possible leak detection)
  unsigned long currentTime = millis();
  if (lastHumidity > 0 && !highHumidityAlarmActive) {
    float humidityChange = newHumidity - lastHumidity;
    float timeDelta = (currentTime - lastHumidityCheck) / 1000.0;  // seconds

    // Detect rapid humidity increase
    if (humidityChange > HUMIDITY_SPIKE_THRESHOLD && timeDelta < 60.0) {
      Serial.println("\n⚠️⚠️⚠️ HIGH HUMIDITY ALARM ⚠️⚠️⚠️");
      Serial.print("Humidity spike detected: +");
      Serial.print(humidityChange, 1);
      Serial.print("% in ");
      Serial.print(timeDelta, 0);
      Serial.println(" seconds");
      Serial.println("POSSIBLE WATER LEAK - Emergency shutdown!");

      highHumidityAlarmActive = true;

      // EMERGENCY SHUTDOWN
      // Stop pump immediately
      if (pumpRunning) {
        pumpRunning = false;
        pumpAutoStarted = false;
        controlPump(0);
        Serial.println("🛑 Pump STOPPED");
      }

      // Close all loop valves
      if (rearLoopOpen) {
        controlRearLoopValve(false);
        Serial.println("🛑 Rear loop CLOSED");
      }
      if (engineLoopOpen) {
        controlEngineLoopValve(false);
        engineLoopMqttRequest = false;
        preferences.putBool("engLoopMqtt", false);
        Serial.println("🛑 Engine loop CLOSED");
      }
      if (frontLoopOpen) {
        controlFrontLoopValve(false);
        frontLoopMqttRequest = false;
        preferences.putBool("frtLoopMqtt", false);
        Serial.println("🛑 Front loop CLOSED");
      }
      if (greyWaterHeaterOn) {
        controlGreyWaterHeater(false);
        Serial.println("🛑 Grey water heater CLOSED");
      }

      // Publish alerts
      client.publish(Topic_Alert_High_Humidity, "ACTIVE - High Humidity, Possible Leak", true);
      client.publish(Topic_Pump_Status, "ALARM_HIGH_HUMIDITY", true);
      client.publish(Topic_Plumbing_Status, "LEAK_DETECTED_SHUTDOWN", true);

      Serial.println("\nSystem locked. Send MQTT command to clear:");
      Serial.println("Topic: ClearHighHumidityCommand");
      Serial.println("Message: clear\n");
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
  client.publish(Topic_FreshWtrHeat_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("🔥 Fresh Water Heater: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlGreyWaterHeater(bool turnOn) {
  digitalWrite(GREY_WATER_HEATER_PIN, turnOn ? LOW : HIGH);
  greyWaterHeaterOn = turnOn;
  client.publish(Topic_GreyWtrHeat_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("🔥 Grey Water Heater: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlExhaustFan(bool turnOn) {
  digitalWrite(EXHAUST_FAN_PIN, turnOn ? LOW : HIGH);
  exhaustFanOn = turnOn;
  client.publish(Topic_ExhaustFan_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("💨 Exhaust Fan: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlRearLoopValve(bool open) {
  digitalWrite(REAR_LOOP_VALVE_PIN, open ? LOW : HIGH);
  rearLoopOpen = open;
  client.publish(Topic_RearLoop_Status, open ? "OPEN" : "CLOSED", true);
  Serial.print("🌊 Rear Loop: ");
  Serial.println(open ? "OPEN" : "CLOSED");
}

void controlEngineLoopValve(bool open) {
  digitalWrite(ENGINE_LOOP_VALVE_PIN, open ? LOW : HIGH);
  engineLoopOpen = open;
  client.publish(Topic_EngineLoop_Status, open ? "OPEN" : "CLOSED", true);
  Serial.print("🌊 Engine Loop: ");
  Serial.println(open ? "OPEN" : "CLOSED");
}

void controlFrontLoopValve(bool open) {
  digitalWrite(FRONT_LOOP_VALVE_PIN, open ? LOW : HIGH);
  frontLoopOpen = open;
  client.publish(Topic_FrontLoop_Status, open ? "OPEN" : "CLOSED", true);
  Serial.print("🌊 Front Loop: ");
  Serial.println(open ? "OPEN" : "CLOSED");
}

void controlMainWaterPump(bool turnOn) {
  // Normally closed relay: HIGH (relay OFF) = pump ON, LOW (relay ON) = pump OFF
  digitalWrite(MAIN_WATER_PUMP_PIN, turnOn ? HIGH : LOW);
  mainWaterPumpOn = turnOn;
  client.publish(Topic_MainPump_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("💧 Main Water Pump: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlDHWSolinoid(bool open) {
  digitalWrite(DOMESTIC_HW_SOLINOID_PIN, open ? LOW : HIGH);
  DHWSolinoidOpen = open;
  client.publish(Topic_DomesticHW_Status, open ? "OPEN" : "CLOSED", true);
  Serial.print("🔥 Domestic Hot Water Loop: ");
  Serial.println(open ? "OPEN" : "CLOSED");
}

void controlDieselHeater(int state) {
  // state: 0=OFF, 1=MID, 2=HIGH
  // ACTIVE LOW RELAYS: LOW = relay energized (circuit ON), HIGH = relay off (circuit OFF)
  // Pin 1 (DIESEL_HEATER_PIN) = White wire (Circuit 1)
  // Pin 2 (DIESEL_HEATER_2_PIN) = Blue wire (Circuit 2)
  // Pin 3 (DIESEL_HEATER_3_PIN) = Yellow wire (Circuit 3)

  if (state == 0) {
    // OFF: All pins HIGH (all relays off, circuits disabled)
    digitalWrite(DIESEL_HEATER_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, HIGH);
    client.publish(Topic_DieselHtr_Status, "OFF", true);
  } else if (state == 1) {
    // MID: Pins 1&2 LOW, Pin 3 HIGH (circuits 1 & 2 ON, circuit 3 OFF)
    digitalWrite(DIESEL_HEATER_PIN, LOW);
    digitalWrite(DIESEL_HEATER_2_PIN, LOW);
    digitalWrite(DIESEL_HEATER_3_PIN, HIGH);
    client.publish(Topic_DieselHtr_Status, "PUMP ONLY", true);
  } else {
    // HIGH: Pins 1&3 LOW, Pin 2 HIGH (circuits 1 & 3 ON, circuit 2 OFF)
    digitalWrite(DIESEL_HEATER_PIN, LOW);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, LOW);
    client.publish(Topic_DieselHtr_Status, "HIGH", true);
  }
  dieselHeaterState = state;
  Serial.print("🔥 Diesel Heater: ");
  if (state == 0) Serial.println("OFF");
  else if (state == 1) Serial.println("PUMP ONLY");
  else Serial.println("HIGH");
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
  // Fresh water tank heater automation
  if (freshWaterMode == "AUTO") {
    if (freshWaterTemp > -999.0) {
      if (freshWaterTemp < freshWaterTempTarget && !freshWaterHeaterOn) {
        controlFreshWaterHeater(true);
      } else if (freshWaterTemp > (freshWaterTempTarget + 2.0) && freshWaterHeaterOn) {
        controlFreshWaterHeater(false);
      }
    }
  }

  // Grey water tank heater automation
  if (greyWaterMode == "AUTO") {
    if (greyWaterTemp > -999.0) {
      if (greyWaterTemp < greyWaterTempTarget && !greyWaterHeaterOn) {
        controlGreyWaterHeater(true);
      } else if (greyWaterTemp > (greyWaterTempTarget + 2.0) && greyWaterHeaterOn) {
        controlGreyWaterHeater(false);
      }
    }
  }

  // Exhaust fan automation - temperature control
  if (exhaustFanMode == "AUTO" && bmeAvailable) {
    if (environmentTemp > -999.0) {
      // Turn on fan if temp exceeds 100°F
      if (environmentTemp >= 100.0 && !exhaustFanOn) {
        controlExhaustFan(true);
        Serial.println("🌡️ Exhaust fan ON - Temperature above 100°F");
      }
      // Turn off fan if temp drops below 95°F (5° hysteresis)
      else if (environmentTemp < 95.0 && exhaustFanOn) {
        controlExhaustFan(false);
        Serial.println("🌡️ Exhaust fan OFF - Temperature below 95°F");
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
    client.publish(Topic_Alert_Freeze_Warning, "ACTIVE");
    Serial.println("⚠️ FREEZE WARNING!");
  } else if (!freezeCondition && freezeWarningActive) {
    freezeWarningActive = false;
    client.publish(Topic_Alert_Freeze_Warning, "CLEAR");
  }

  // High temperature warning
  bool highTempCondition = false;
  if (hydronicTemp > HIGH_TEMP_WARNING) highTempCondition = true;

  if (highTempCondition && !highTempWarningActive) {
    highTempWarningActive = true;
    client.publish(Topic_Alert_High_Temp, "ACTIVE");
    Serial.println("⚠️ HIGH TEMPERATURE WARNING!");
  } else if (!highTempCondition && highTempWarningActive) {
    highTempWarningActive = false;
    client.publish(Topic_Alert_High_Temp, "CLEAR");
  }
}

// ============================================================================
// STATUS & HEALTH
// ============================================================================
void publishStatus() {
  // All valve states already published by control functions with retained flag
  // Pump status
  const char* pumpStatus = pumpAlarm ? pumpAlarmReason.c_str() : (pumpRunning ? "RUNNING" : "STOPPED");
  client.publish(Topic_Pump_Status, pumpStatus);

  // Publish main water pump and domestic HW loop initial states
  client.publish(Topic_MainPump_Status, mainWaterPumpOn ? "ON" : "OFF", true);
  client.publish(Topic_DomesticHW_Status, DHWSolinoidOpen ? "OPEN" : "CLOSED", true);
}

void publishHealth() {
  char uptimeStr[16];
  unsigned long uptime = millis() / 1000;
  snprintf(uptimeStr, sizeof(uptimeStr), "%lu", uptime);
  client.publish(Topic_Plumbing_Uptime, uptimeStr);

  client.publish(Topic_Plumbing_Status, "ONLINE");
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
  preferences.putFloat("fanHigh", exhaustFanTempHigh);
  preferences.putFloat("fanLow", exhaustFanTempLow);
  Serial.println("💾 Configuration saved");
}

void loadConfigFromPreferences() {
  freshWaterTempTarget = preferences.getFloat("fwTarget", 40.0);
  greyWaterTempTarget = preferences.getFloat("gwTarget", 35.0);
  exhaustFanTempHigh = preferences.getFloat("fanHigh", 85.0);
  exhaustFanTempLow = preferences.getFloat("fanLow", 75.0);

  // Restore loop request states (MQTT commands from GIGA/thermostat)
  engineLoopMqttRequest = preferences.getBool("engLoopMqtt", false);
  frontLoopMqttRequest = preferences.getBool("frtLoopMqtt", false);

  Serial.println("💾 Configuration loaded");
  Serial.print("  Engine Loop MQTT Request: ");
  Serial.println(engineLoopMqttRequest ? "OPEN" : "CLOSE");
  Serial.print("  Front Loop MQTT Request: ");
  Serial.println(frontLoopMqttRequest ? "OPEN" : "CLOSE");
}

void restoreValveStates() {
  // Restore physical valve positions to match saved MQTT request states
  Serial.println("\n🔄 Restoring valve states from EEPROM...");

  if (engineLoopMqttRequest) {
    controlEngineLoopValve(true);
    Serial.println("  ✓ Engine loop restored to OPEN");
  }

  if (frontLoopMqttRequest) {
    controlFrontLoopValve(true);
    Serial.println("  ✓ Front loop restored to OPEN");
  }

  if (!engineLoopMqttRequest && !frontLoopMqttRequest) {
    Serial.println("  ℹ️ All loops closed (default state)");
  }

  Serial.println("✓ Valve states synchronized with MQTT status\n");
}
