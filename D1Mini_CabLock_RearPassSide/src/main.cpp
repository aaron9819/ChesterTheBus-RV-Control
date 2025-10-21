#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoOTA.h>

// ============================================================================
// D1 MINI ENVIRONMENTAL & HOT WATER CONTROLLER - Chester the Bus RV
// ============================================================================
// Board Responsibilities:
// - Cabinet lock control (servo on D4)
// - Environmental monitoring (BME280 sensor: temp, humidity, pressure)
// - Water flow detection (Hall Effect flow sensor on D8)
// - Hot water solenoid control (relay on D5)
// - Automatic hot water valve when flow detected
// ============================================================================

#define FIRMWARE_VERSION "v1.0.0"
#define BOARD_NAME "Environment"

// ============================================
// CONFIGURATION
// ============================================
// Set unique cabinet identifier
#define CABINET_ID "RearPassSide"  // Keep for backward compatibility

// WiFi Configuration
const char* ssid = "Chester IOT";
const char* password = "2025Chester9894";

// MQTT Configuration - Connect to Mosquitto broker on GL-AR300M16 router
const char* mqtt_server = "192.168.8.1";  // Router IP address
const int mqtt_port = 1883;
const char* mqtt_user = "";  // No username
const char* mqtt_password = "";  // No password

// OTA Configuration
const char* otaHostname = "D1Mini-Environment";
const char* otaPassword = "Chester2025";

// MQTT Client ID - Unique per device
String mqtt_client_id = String("CabLock_") + String(CABINET_ID);

// MQTT Topics - Cabinet Lock
String topic_cabinetLock_command = "CabLock" + String(CABINET_ID) + "Command";
String topic_cabinetLock_status = "CabLock" + String(CABINET_ID) + "Status";
const char* topic_allLock_command = "CabLockAllCommand";  // Group control

// MQTT Topics - Environmental Sensors (Publish Only)
const char* Topic_Temp_Ambient = "EnvironmentTempAmbient";
const char* Topic_Humidity_Ambient = "EnvironmentHumidity";
const char* Topic_Pressure_Ambient = "EnvironmentPressure";

// MQTT Topics - Flow Sensor (Publish Only)
const char* Topic_Flow_Rate = "HotWaterFlowRate";
const char* Topic_Flow_Status = "HotWaterFlowStatus";  // FLOWING/STOPPED

// MQTT Topics - Hot Water Solenoid (Subscribe & Publish)
const char* Topic_HotWater_Command = "HotWaterSolenoidCommand";  // ON/OFF/AUTO
const char* Topic_HotWater_Status = "HotWaterSolenoidStatus";
const char* Topic_HotWater_Mode = "HotWaterSolenoidMode";  // AUTO/MANUAL

// MQTT Topics - System Health
const char* Topic_Environment_Status = "D1EnvironmentSystemStatus";
const char* Topic_Environment_Uptime = "D1EnvironmentUptime";
const char* Topic_Firmware_Version = "D1EnvironmentFirmwareVersion";

// Hardware Configuration
#define SERVO_PIN D4                    // Cabinet lock servo
#define HOT_WATER_SOLENOID_PIN D5       // Relay for hot water solenoid
#define FLOW_SENSOR_PIN D8              // Hall Effect flow sensor (interrupt capable)
#define BME280_SDA D2                   // I2C SDA (GPIO4)
#define BME280_SCL D1                   // I2C SCL (GPIO5)

Servo lockServo;
Adafruit_BME280 bme;                    // BME280 sensor object

// Lock positions (adjust based on your servo/latch mechanism)
#define LOCKED_POSITION 90
#define UNLOCKED_POSITION 5

// MQTT Client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Lock State
enum LockState {
  STATE_UNKNOWN,
  STATE_UNLOCKED,
  STATE_LOCKED
};

LockState currentState = STATE_UNKNOWN;

// Hot Water Solenoid State
bool hotWaterSolenoidOn = false;
String hotWaterMode = "AUTO";  // AUTO or MANUAL

// Flow Sensor State
volatile unsigned long flowPulseCount = 0;
volatile unsigned long lastPulseTime = 0;  // Time when last pulse occurred
volatile unsigned long recentPulseCount = 0;  // Count pulses in recent window for threshold check
float flowRate = 0.0;  // Liters per minute
bool flowDetected = false;
unsigned long lastFlowCalc = 0;
const unsigned long FLOW_CALC_INTERVAL = 1000;  // Calculate flow every second
const unsigned long FLOW_TIMEOUT = 3000;        // If no pulses for 3 seconds, flow has stopped
const unsigned long MIN_PULSES_FOR_FLOW = 10;   // Minimum pulses per second to consider as real flow

// BME280 Sensor Readings
float temperature = 0.0;  // °F
float humidity = 0.0;     // %
float pressure = 0.0;     // hPa
bool bmeAvailable = false;

// Timing Variables
unsigned long lastStatusUpdate = 0;
unsigned long lastSensorRead = 0;
unsigned long lastFlowCheck = 0;
unsigned long lastHealthPublish = 0;
const unsigned long STATUS_INTERVAL = 60000;      // 60 seconds
const unsigned long SENSOR_READ_INTERVAL = 5000;  // 5 seconds
const unsigned long FLOW_CHECK_INTERVAL = 100;    // 100ms for fast response
const unsigned long HEALTH_PUBLISH_INTERVAL = 60000;  // 60 seconds

// Forward declarations
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void lockCabinet();
void unlockCabinet();
void sendStatus();
void processCommand(String command);
void controlHotWaterSolenoid(bool turnOn);
void readBME280();
void publishBME280();
void calculateFlowRate();
void publishFlowRate();
void publishHealth();
void handleAutoHotWater();
void setupOTA();
void IRAM_ATTR flowSensorISR();

// MQTT Callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.printf("MQTT message received on topic: %s - Message: ", topic);
  Serial.println(message);

  message.trim();

  // Cabinet lock commands
  if (String(topic) == topic_cabinetLock_command || String(topic) == topic_allLock_command) {
    processCommand(message);
  }
  // Hot water solenoid commands
  else if (String(topic) == Topic_HotWater_Command) {
    if (message == "ON") {
      controlHotWaterSolenoid(true);
    } else if (message == "OFF") {
      controlHotWaterSolenoid(false);
    }
  }
  // Hot water mode
  else if (String(topic) == Topic_HotWater_Mode) {
    if (message == "AUTO" || message == "MANUAL") {
      hotWaterMode = message;
      Serial.print("Hot water mode set to: ");
      Serial.println(hotWaterMode);
    }
  }
}

void processCommand(String command) {
  if (command == "LOCK") {
    lockCabinet();
  }
  else if (command == "UNLOCK") {
    unlockCabinet();
  }
  else if (command == "STATUS") {
    sendStatus();
  }
  else if (command == "TOGGLE") {
    if (currentState == STATE_LOCKED) {
      unlockCabinet();
    } else {
      lockCabinet();
    }
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

void reconnectMQTT() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection to ");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.print(mqtt_port);
    Serial.print(" as client: ");
    Serial.print(mqtt_client_id);
    Serial.print("...");

    // Connect with or without credentials
    bool connected;
    if (strlen(mqtt_user) > 0) {
      connected = mqttClient.connect(mqtt_client_id.c_str(), mqtt_user, mqtt_password);
    } else {
      connected = mqttClient.connect(mqtt_client_id.c_str());
    }

    if (connected) {
      Serial.println("connected");

      // Subscribe to individual command topic
      mqttClient.subscribe(topic_cabinetLock_command.c_str());
      Serial.print("Subscribed to: ");
      Serial.println(topic_cabinetLock_command);

      // Subscribe to group command topic (for controlling all locks at once)
      mqttClient.subscribe(topic_allLock_command);
      Serial.print("Subscribed to: ");
      Serial.println(topic_allLock_command);

      // Subscribe to hot water topics
      mqttClient.subscribe(Topic_HotWater_Command);
      mqttClient.subscribe(Topic_HotWater_Mode);
      Serial.println("Subscribed to hot water control topics");

      // Send initial status
      sendStatus();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n\n============================================");
  Serial.println("  D1 ENVIRONMENTAL & HOT WATER CONTROLLER");
  Serial.println("  Chester the Bus RV Control System");
  Serial.print("  Firmware: ");
  Serial.println(FIRMWARE_VERSION);
  #ifdef BUILD_DATE
  Serial.print("  Build: ");
  Serial.print(BUILD_DATE);
  Serial.print(" ");
  Serial.println(BUILD_TIME);
  #endif
  Serial.println("============================================\n");

  Serial.print("Board: ");
  Serial.println(BOARD_NAME);
  Serial.print("Cabinet ID: ");
  Serial.println(CABINET_ID);

  // Initialize I2C for BME280
  Wire.begin(BME280_SDA, BME280_SCL);
  Serial.println("I2C initialized (D2=SDA, D1=SCL)");

  // Initialize BME280 sensor
  if (bme.begin(0x76)) {  // Try default address 0x76
    bmeAvailable = true;
    Serial.println("✓ BME280 sensor found at address 0x76");
  } else if (bme.begin(0x77)) {  // Try alternate address 0x77
    bmeAvailable = true;
    Serial.println("✓ BME280 sensor found at address 0x77");
  } else {
    bmeAvailable = false;
    Serial.println("⚠ BME280 sensor NOT found!");
  }

  // Initialize servo
  lockServo.attach(SERVO_PIN);
  lockServo.write(UNLOCKED_POSITION);
  currentState = STATE_UNLOCKED;
  lockServo.detach();  // Detach to save power and reduce noise
  Serial.println("✓ Servo initialized at UNLOCKED position (D4)");

  // Initialize hot water solenoid relay (active LOW, so HIGH = OFF)
  pinMode(HOT_WATER_SOLENOID_PIN, OUTPUT);
  digitalWrite(HOT_WATER_SOLENOID_PIN, HIGH);
  hotWaterSolenoidOn = false;
  Serial.println("✓ Hot water solenoid relay initialized (D5)");

  // Initialize flow sensor
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, FALLING);
  Serial.println("✓ Flow sensor initialized (D8)");

  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());

    // Setup OTA (Over-The-Air updates)
    setupOTA();

    // Setup MQTT
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);

    // Connect to MQTT broker
    reconnectMQTT();

    // Initial sensor reading
    if (bmeAvailable) {
      readBME280();
      publishBME280();
    }

    Serial.println("\n✓ System Ready!");
    Serial.println("  - Cabinet Lock Control");
    Serial.println("  - Environmental Monitoring (BME280)");
    Serial.println("  - Water Flow Detection");
    Serial.println("  - Hot Water Solenoid Control");
  } else {
    Serial.println("\nWiFi connection failed!");
    Serial.println("Will retry in loop...");
  }
}

void lockCabinet() {
  Serial.print(">>> LOCKING Cabinet: ");
  Serial.println(CABINET_ID);

  // Ensure servo is attached before moving
  if (!lockServo.attached()) {
    lockServo.attach(SERVO_PIN);
  }

  lockServo.write(LOCKED_POSITION);
  currentState = STATE_LOCKED;
  delay(500);  // Give servo time to move

  // Disable servo to save power and reduce noise
  lockServo.detach();
  Serial.println("Servo disabled after locking");

  // Publish status via MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topic_cabinetLock_status.c_str(), "LOCKED", true);  // Retained message
    Serial.println("Status published: LOCKED");
  }
}

void unlockCabinet() {
  Serial.print(">>> UNLOCKING Cabinet: ");
  Serial.println(CABINET_ID);

  // Ensure servo is attached before moving
  if (!lockServo.attached()) {
    lockServo.attach(SERVO_PIN);
  }

  lockServo.write(UNLOCKED_POSITION);
  currentState = STATE_UNLOCKED;
  delay(500);  // Give servo time to move

  // Disable servo to save power and reduce noise
  lockServo.detach();
  Serial.println("Servo disabled after unlocking");

  // Publish status via MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topic_cabinetLock_status.c_str(), "UNLOCKED", true);  // Retained message
    Serial.println("Status published: UNLOCKED");
  }
}

void sendStatus() {
  // Publish current status via MQTT (with retained flag for Home Assistant)
  if (mqttClient.connected()) {
    const char* status;
    if (currentState == STATE_LOCKED) {
      status = "LOCKED";
    } else if (currentState == STATE_UNLOCKED) {
      status = "UNLOCKED";
    } else {
      status = "UNKNOWN";
    }

    mqttClient.publish(topic_cabinetLock_status.c_str(), status, true);
    Serial.print("Status published for ");
    Serial.print(CABINET_ID);
    Serial.print(": ");
    Serial.println(status);
  }
}

void loop() {
  // Ensure WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconnected!");
    }
  }

  // Handle OTA updates
  ArduinoOTA.handle();

  // Keep MQTT connection alive
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Calculate flow rate periodically (1 second)
  if (millis() - lastFlowCalc >= FLOW_CALC_INTERVAL) {
    calculateFlowRate();
  }

  // Check hot water automation frequently (100ms) for fast response
  if (millis() - lastFlowCheck >= FLOW_CHECK_INTERVAL) {
    lastFlowCheck = millis();
    handleAutoHotWater();
  }

  // Read BME280 sensor periodically (5 seconds)
  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();

    // Read BME280
    if (bmeAvailable) {
      readBME280();
      publishBME280();
    }

    // Publish flow rate (less frequent than calculation)
    publishFlowRate();
  }

  // Periodically send status update
  if (millis() - lastStatusUpdate >= STATUS_INTERVAL) {
    lastStatusUpdate = millis();
    sendStatus();
  }

  // Publish health metrics
  if (millis() - lastHealthPublish >= HEALTH_PUBLISH_INTERVAL) {
    lastHealthPublish = millis();
    publishHealth();
  }

  delay(10);
}

// ============================================================================
// HOT WATER SOLENOID CONTROL
// ============================================================================
void controlHotWaterSolenoid(bool turnOn) {
  digitalWrite(HOT_WATER_SOLENOID_PIN, turnOn ? LOW : HIGH); // Active LOW
  hotWaterSolenoidOn = turnOn;

  if (mqttClient.connected()) {
    mqttClient.publish(Topic_HotWater_Status, turnOn ? "ON" : "OFF");
  }

  Serial.print("💧 Hot water solenoid: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

// ============================================================================
// BME280 ENVIRONMENTAL SENSOR
// ============================================================================
void readBME280() {
  if (!bmeAvailable) return;

  float tempC = bme.readTemperature();
  temperature = (tempC * 9.0 / 5.0) + 32.0;  // Convert to Fahrenheit
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;  // Convert to hPa
}

void publishBME280() {
  if (!bmeAvailable || !mqttClient.connected()) return;

  char tempStr[8];

  // Publish temperature
  dtostrf(temperature, 4, 2, tempStr);
  mqttClient.publish(Topic_Temp_Ambient, tempStr);
  Serial.print("🌡️  Temperature: ");
  Serial.print(tempStr);
  Serial.println(" °F");

  // Publish humidity
  dtostrf(humidity, 4, 2, tempStr);
  mqttClient.publish(Topic_Humidity_Ambient, tempStr);
  Serial.print("💧 Humidity: ");
  Serial.print(tempStr);
  Serial.println(" %");

  // Publish pressure
  dtostrf(pressure, 6, 2, tempStr);
  mqttClient.publish(Topic_Pressure_Ambient, tempStr);
  Serial.print("🌍 Pressure: ");
  Serial.print(tempStr);
  Serial.println(" hPa");
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
  bool previousFlowDetected = flowDetected;
  unsigned long timeSinceLastPulse = currentTime - lastPulse;
  bool hasRecentPulses = (timeSinceLastPulse < FLOW_TIMEOUT);
  bool meetsThreshold = (recentPulses >= MIN_PULSES_FOR_FLOW);

  flowDetected = hasRecentPulses && meetsThreshold;

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
    } else {
      Serial.print("   Recent pulses: ");
      Serial.print(recentPulses);
      Serial.print(" (threshold: ");
      Serial.print(MIN_PULSES_FOR_FLOW);
      Serial.println(")");
    }
  }
}

void publishFlowRate() {
  if (!mqttClient.connected()) return;

  // Publish flow rate
  char flowStr[8];
  dtostrf(flowRate, 4, 2, flowStr);
  mqttClient.publish(Topic_Flow_Rate, flowStr);

  // Publish flow status
  mqttClient.publish(Topic_Flow_Status, flowDetected ? "FLOWING" : "STOPPED");

  if (flowDetected) {
    Serial.print("💦 Flow Rate: ");
    Serial.print(flowStr);
    Serial.println(" L/min");
  }
}

// ============================================================================
// AUTOMATIC HOT WATER CONTROL
// ============================================================================
void handleAutoHotWater() {
  if (hotWaterMode != "AUTO") return;

  // Simple logic: Turn on when flow detected, turn off when flow stopped
  // flowDetected already has 3-second timeout built in
  if (flowDetected && !hotWaterSolenoidOn) {
    Serial.println("🔥 AUTO: Opening hot water solenoid (flow detected)");
    controlHotWaterSolenoid(true);
  }
  else if (!flowDetected && hotWaterSolenoidOn) {
    Serial.println("❄️  AUTO: Closing hot water solenoid (no flow for 3+ seconds)");
    controlHotWaterSolenoid(false);
  }
}

// ============================================================================
// OTA UPDATE SETUP
// ============================================================================
void setupOTA() {
  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("\n🔄 OTA Update Starting...");
    Serial.println("   Type: " + type);

    // Turn off hot water solenoid for safety (HIGH = OFF for active LOW relay)
    digitalWrite(HOT_WATER_SOLENOID_PIN, HIGH);
    Serial.println("   Safety: Hot water solenoid disabled");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n✓ OTA Update Complete!");
    Serial.println("   Rebooting...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static int lastPercent = -1;
    int percentComplete = (progress / (total / 100));
    if (percentComplete != lastPercent && percentComplete % 10 == 0) {
      Serial.printf("   Progress: %u%%\n", percentComplete);
      lastPercent = percentComplete;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n✗ OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Authentication Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  Serial.println("✓ OTA (Over-The-Air) update enabled");
  Serial.print("  Hostname: ");
  Serial.print(otaHostname);
  Serial.println(".local");
}

// ============================================================================
// SYSTEM HEALTH
// ============================================================================
void publishHealth() {
  if (!mqttClient.connected()) return;

  // Uptime in seconds
  char uptimeStr[16];
  unsigned long uptime = millis() / 1000;
  snprintf(uptimeStr, sizeof(uptimeStr), "%lu", uptime);
  mqttClient.publish(Topic_Environment_Uptime, uptimeStr);

  // System status
  mqttClient.publish(Topic_Environment_Status, "ONLINE");
  mqttClient.publish(Topic_Firmware_Version, FIRMWARE_VERSION);

  Serial.print("📊 Health: Uptime=");
  Serial.print(uptime);
  Serial.println("s");
}
