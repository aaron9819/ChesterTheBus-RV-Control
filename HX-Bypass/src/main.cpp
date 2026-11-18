#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

// ============================================
// CONFIGURATION
// ============================================
#define DEVICE_ID "HX-Bypass"
#define FIRMWARE_VERSION "v1.0.1"

// WiFi and OTA credentials come from secrets.ini at the repo root (gitignored),
// passed in as build flags by platformio.ini. Template: secrets.example.ini.
#if !defined(SECRET_WIFI_SSID) || !defined(SECRET_WIFI_PASSWORD) || !defined(SECRET_OTA_PASSWORD)
#error "WiFi/OTA credentials missing - copy secrets.example.ini to secrets.ini at the repo root"
#endif

// WiFi Configuration
const char* ssid = SECRET_WIFI_SSID;
const char* password = SECRET_WIFI_PASSWORD;

// MQTT Configuration - Connect to Mosquitto broker on GL-AR300M16 router
const char* mqtt_server = "192.168.8.1";  // Router IP address
const int mqtt_port = 1883;
const char* mqtt_user = "";  // No username
const char* mqtt_password = "";  // No password

// OTA Configuration
const char* otaHostname = "HX-Bypass";
const char* otaPassword = SECRET_OTA_PASSWORD;

// MQTT Client ID - Unique per device
String mqtt_client_id = String("HX_Bypass");

// MQTT Topics
const char* topic_hydronic_temp = "HydronicTemperature";
String topic_valve_command = "HXBypassValveCommand";
String topic_valve_status = "HXBypassValveStatus";
String topic_auto_mode_command = "HXBypassAutoModeCommand";
String topic_auto_mode_status = "HXBypassAutoModeStatus";
String topic_temp_setpoint_command = "HXBypassTempSetpointCommand";
String topic_temp_setpoint_status = "HXBypassTempSetpointStatus";

// Hardware Configuration
#define RELAY_PIN D4  // Relay control pin

// Temperature control
float currentTemp = 0.0;
float tempSetpoint = 160.0;  // Default setpoint (130-200 range)
bool autoMode = true;  // Default to auto mode
bool valveState = false;  // false = closed, true = open
bool reverseLogic = false;  // Set to true if logic needs to be reversed
// Once switched, the valve holds until the temperature has moved back this far
// past the setpoint. Without it, a reading that hovers at the setpoint (one
// every 5 s) chatters the relay.
const float HYSTERESIS_F = 2.0;

// Networking (non-blocking)
const unsigned long MQTT_RETRY_MS = 5000;
unsigned long lastMqttAttempt = 0;
bool otaStarted = false;

// MQTT Client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Timing variables
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_INTERVAL = 60000;  // Send status every 60 seconds

// Forward declarations
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void publishHomeAssistantDiscovery();
void updateValve();
void sendStatus();

// MQTT Callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.printf("MQTT message received on topic: '%s' - Message: '%s'", topic, message.c_str());
  Serial.println();
  Serial.printf("Comparing to valve command topic: '%s'", topic_valve_command.c_str());
  Serial.println();

  String topicStr = String(topic);

  if (topicStr == topic_valve_command) {
    Serial.println("VALVE COMMAND RECEIVED!");
    Serial.printf("Message: '%s', Auto mode: %s\n", message.c_str(), autoMode ? "true" : "false");

    if (message == "ON") {
      if (!autoMode) {
        Serial.println("Setting valve to ON in manual mode");
        valveState = true;
        digitalWrite(RELAY_PIN, LOW);  // Directly control relay - LOW activates it
        Serial.println(">>> Relay activated (LOW)");
        // Immediately publish the new state back to confirm the change
        mqttClient.publish(topic_valve_status.c_str(), "ON", true);
        Serial.println("Manual valve control: ON - Status published");
      } else {
        Serial.println("Manual valve control ignored - system in auto mode");
        // Send current state to sync the UI
        mqttClient.publish(topic_valve_status.c_str(), valveState ? "ON" : "OFF", true);
      }
    }
    else if (message == "OFF") {
      if (!autoMode) {
        Serial.println("Setting valve to OFF in manual mode");
        valveState = false;
        digitalWrite(RELAY_PIN, HIGH);  // Directly control relay - HIGH deactivates it
        Serial.println(">>> Relay deactivated (HIGH)");
        // Immediately publish the new state back to confirm the change
        mqttClient.publish(topic_valve_status.c_str(), "OFF", true);
        Serial.println("Manual valve control: OFF - Status published");
      } else {
        Serial.println("Manual valve control ignored - system in auto mode");
        // Send current state to sync the UI
        mqttClient.publish(topic_valve_status.c_str(), valveState ? "ON" : "OFF", true);
      }
    }
  }
  else if (topicStr == topic_auto_mode_command) {
    Serial.printf("Auto mode command received: '%s'\n", message.c_str());

    if (message == "ON") {
      autoMode = true;
      Serial.println("Auto mode enabled");
      updateValve();  // Re-evaluate valve state based on temperature

      // Make sure Home Assistant knows the current valve state
      mqttClient.publish(topic_valve_status.c_str(), valveState ? "ON" : "OFF", true);
      Serial.println("Sent updated valve state after enabling auto mode");
    }
    else if (message == "OFF") {
      autoMode = false;
      Serial.println("Manual mode enabled - valve will maintain current state until manually changed");

      // Keep current valve state when switching to manual mode
      mqttClient.publish(topic_valve_status.c_str(), valveState ? "ON" : "OFF", true);
      Serial.printf("Sent valve state (%s) to Home Assistant\n", valveState ? "ON" : "OFF");
    }

    // Publish auto mode status
    mqttClient.publish(topic_auto_mode_status.c_str(), autoMode ? "ON" : "OFF", true);
    Serial.println("Auto mode status published");
  }
  else if (topicStr == topic_temp_setpoint_command) {
    float newSetpoint = message.toFloat();
    if (newSetpoint >= 130.0 && newSetpoint <= 200.0) {
      tempSetpoint = newSetpoint;
      Serial.print("Temperature setpoint updated to: ");
      Serial.println(tempSetpoint);

      // Publish status
      mqttClient.publish(topic_temp_setpoint_status.c_str(), String(tempSetpoint).c_str(), true);

      if (autoMode) {
        updateValve();  // Re-evaluate valve state based on new setpoint
      }
    }
  }
  else if (topicStr == topic_hydronic_temp) {
    float newTemp = message.toFloat();
    currentTemp = newTemp;
    Serial.print("Hydronic tank temperature updated: ");
    Serial.println(currentTemp);

    if (autoMode) {
      updateValve();  // Update valve state based on new temperature
    }
  }
}

// ============================================================================
// HOME ASSISTANT MQTT DISCOVERY
// ============================================================================
void publishHomeAssistantDiscovery() {
  Serial.println("📡 Publishing Home Assistant MQTT Discovery...");

  String deviceId = String("chester_hxbypass");
  String deviceName = String("Heat Exchanger Bypass");

  // Device configuration
  String device = "{\"identifiers\":[\"" + deviceId + "\"],"
                  "\"name\":\"" + deviceName + "\","
                  "\"model\":\"D1 Mini (ESP8266)\","
                  "\"manufacturer\":\"ChesterTheBus\"}";

  // 1. Valve Switch Entity (for manual control)
  String switchTopic = String("homeassistant/switch/") + deviceId + "/valve/config";
  String switchPayload = "{\"name\":\"HX Bypass Valve\","
                   "\"unique_id\":\"" + deviceId + "_valve\","
                   "\"state_topic\":\"" + topic_valve_status + "\","
                   "\"command_topic\":\"" + topic_valve_command + "\","
                   "\"icon\":\"mdi:valve\","
                   "\"device\":" + device + "}";

  mqttClient.publish(switchTopic.c_str(), switchPayload.c_str(), true);

  // 2. Auto Mode Switch
  String autoModeTopic = String("homeassistant/switch/") + deviceId + "/auto_mode/config";
  String autoModePayload = "{\"name\":\"HX Bypass Auto Mode\","
                   "\"unique_id\":\"" + deviceId + "_auto_mode\","
                   "\"state_topic\":\"" + topic_auto_mode_status + "\","
                   "\"command_topic\":\"" + topic_auto_mode_command + "\","
                   "\"icon\":\"mdi:home-automation\","
                   "\"device\":" + device + "}";

  mqttClient.publish(autoModeTopic.c_str(), autoModePayload.c_str(), true);

  // 3. Temperature Setpoint
  String setpointTopic = String("homeassistant/number/") + deviceId + "/temp_setpoint/config";
  String setpointPayload = "{\"name\":\"HX Bypass Temperature Setpoint\","
                   "\"unique_id\":\"" + deviceId + "_temp_setpoint\","
                   "\"state_topic\":\"" + topic_temp_setpoint_status + "\","
                   "\"command_topic\":\"" + topic_temp_setpoint_command + "\","
                   "\"min\":130,"
                   "\"max\":200,"
                   "\"step\":1,"
                   "\"unit_of_measurement\":\"°F\","
                   "\"icon\":\"mdi:thermometer\","
                   "\"device\":" + device + "}";

  mqttClient.publish(setpointTopic.c_str(), setpointPayload.c_str(), true);

  Serial.println("✓ Home Assistant Discovery published");
}

void updateValve() {
  bool newValveState = valveState;

  if (autoMode) {
    // In auto mode, determine valve state based on temperature (with hysteresis)
    if (!reverseLogic) {
      // Normal logic: Open valve (ON) above setpoint, close once HYSTERESIS_F below it
      newValveState = valveState ? (currentTemp > tempSetpoint - HYSTERESIS_F)
                                 : (currentTemp > tempSetpoint);
    } else {
      // Reversed logic: Open valve (ON) below setpoint, close once HYSTERESIS_F above it
      newValveState = valveState ? (currentTemp < tempSetpoint + HYSTERESIS_F)
                                 : (currentTemp < tempSetpoint);
    }
  } else {
    // In manual mode, keep the current valve state that was set via MQTT
    newValveState = valveState;
    Serial.print("Manual mode: maintaining valve state: ");
    Serial.println(valveState ? "OPEN (ON)" : "CLOSED (OFF)");
  }

  // Only update if state has changed
  if (valveState != newValveState || millis() - lastStatusUpdate > STATUS_INTERVAL) {
    valveState = newValveState;

    Serial.print(">>> Setting valve to: ");
    Serial.println(valveState ? "OPEN (ON)" : "CLOSED (OFF)");
    Serial.print(">>> Setting D4 (GPIO ");
    Serial.print(RELAY_PIN);
    Serial.print(") to: ");
    // Relay activates when pin is pulled LOW
    Serial.println(valveState ? "LOW" : "HIGH");

    // Control the relay - LOW activates the relay
    digitalWrite(RELAY_PIN, valveState ? LOW : HIGH);

    // Double-check pin state
    int pinState = digitalRead(RELAY_PIN);
    Serial.print(">>> Pin state after setting: ");
    Serial.println(pinState == HIGH ? "HIGH" : "LOW");

    // Publish status via MQTT
    if (mqttClient.connected()) {
      mqttClient.publish(topic_valve_status.c_str(), valveState ? "ON" : "OFF", true);
      Serial.println("Valve status published");
    }
  }
}

// One connection attempt at most every MQTT_RETRY_MS. This used to loop until
// connected, which stopped OTA and WiFi recovery while the broker was down.
void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (lastMqttAttempt != 0 && millis() - lastMqttAttempt < MQTT_RETRY_MS) return;
  lastMqttAttempt = millis();

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

    // Publish Home Assistant discovery
    publishHomeAssistantDiscovery();

    // Subscribe to topics
    mqttClient.subscribe(topic_valve_command.c_str());
    mqttClient.subscribe(topic_auto_mode_command.c_str());
    mqttClient.subscribe(topic_temp_setpoint_command.c_str());
    mqttClient.subscribe(topic_hydronic_temp);

    Serial.println("Subscribed to command topics");

    // Send initial status
    sendStatus();
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" - retrying in 5 seconds");
  }
}

void setupOTA() {
  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    Serial.println("\n🔄 OTA Update Starting...");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n✓ OTA Update Complete! Rebooting...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static int lastPercent = -1;
    if (total == 0) return;
    int percentComplete = (int)((progress * 100ULL) / total);
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
  otaStarted = true;
  Serial.println("✓ OTA enabled");
  Serial.print("  Hostname: ");
  Serial.print(otaHostname);
  Serial.println(".local");
}

void sendStatus() {
  // Publish current status via MQTT (with retained flag for Home Assistant)
  if (mqttClient.connected()) {
    mqttClient.publish(topic_valve_status.c_str(), valveState ? "ON" : "OFF", true);
    mqttClient.publish(topic_auto_mode_status.c_str(), autoMode ? "ON" : "OFF", true);
    mqttClient.publish(topic_temp_setpoint_status.c_str(), String(tempSetpoint).c_str(), true);

    Serial.println("Status published");
    lastStatusUpdate = millis();
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n\n=== ChesterTheBus Heat Exchanger Bypass Controller ===");
  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);

  // Initialize relay pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);  // Start with valve closed (relay inactive = HIGH)
  Serial.println("Relay initialized to OFF (valve closed)");

  // Configure static IP address
  IPAddress staticIP(192, 168, 8, 140);  // Static IP: 192.168.8.140
  IPAddress gateway(192, 168, 8, 1);     // Gateway (router): 192.168.8.1
  IPAddress subnet(255, 255, 255, 0);    // Subnet mask
  IPAddress dns(8, 8, 8, 8);             // DNS (Google)

  // Apply static IP configuration
  WiFi.config(staticIP, gateway, subnet, dns);
  Serial.print("Static IP configured: ");
  Serial.println(staticIP);

  // MQTT is configured whether or not WiFi is up yet. It used to be set up only
  // when WiFi connected within 20 s of boot; otherwise the loop retried forever
  // against an unconfigured client.
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);

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

    setupOTA();
    reconnectMQTT();
  } else {
    Serial.println("\nWiFi connection failed!");
    Serial.println("Will retry in loop...");
  }

  Serial.println("Ready to control heat exchanger bypass valve!");
  Serial.print("Firmware: ");
  Serial.println(FIRMWARE_VERSION);
}

void loop() {
  // Ensure WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");

    // Reconfigure static IP before reconnecting
    IPAddress staticIP(192, 168, 8, 140);
    IPAddress gateway(192, 168, 8, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dns(8, 8, 8, 8);

    WiFi.config(staticIP, gateway, subnet, dns);
    Serial.print("Static IP reconfigured: ");
    Serial.println(staticIP);

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

  // OTA starts the first time WiFi is up, even if that is long after boot
  if (!otaStarted && WiFi.status() == WL_CONNECTED) {
    setupOTA();
  }
  if (otaStarted) {
    ArduinoOTA.handle();
  }

  // Keep MQTT connection alive
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Periodically send status update
  if (millis() - lastStatusUpdate > STATUS_INTERVAL) {
    sendStatus();
  }

  delay(10);
}
