#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>

// ============================================
// CONFIGURATION - Change for each D1 Mini
// ============================================
// Set unique cabinet identifier for each D1 Mini
#define CABINET_ID "KitchenDriverSide"  // Options: kitchen1, kitchen2, bathroom, bedroom, etc.

// WiFi Configuration
const char* ssid = "Chester IOT";
const char* password = "2025Chester9894";

// MQTT Configuration - Connect to Mosquitto broker on GL-AR300M16 router
const char* mqtt_server = "192.168.8.1";  // Router IP address
const int mqtt_port = 1883;
const char* mqtt_user = "";  // No username
const char* mqtt_password = "";  // No password

// MQTT Client ID - Unique per device
String mqtt_client_id = String("CabLock_") + String(CABINET_ID);

// MQTT Topics - Generated based on CABINET_ID
String topic_cabinetLock_command = "CabLock" + String(CABINET_ID) + "Command";
String topic_cabinetLock_status = "CabLock" + String(CABINET_ID) + "Status";
const char* topic_allLock_command = "CabLockAllCommand";  // Group control

// Hardware Configuration
#define SERVO_PIN D4
Servo lockServo;

// Lock positions (adjust based on your servo/latch mechanism)
#define LOCKED_POSITION 5
#define UNLOCKED_POSITION 90

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
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_INTERVAL = 60000; // Send status every 60 seconds

// Forward declarations
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void publishHomeAssistantDiscovery();
void lockCabinet();
void unlockCabinet();
void sendStatus();
void processCommand(String command);

// MQTT Callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.printf("MQTT message received on topic: %s - Message: ", topic);
  Serial.println(message);

  message.trim();
  processCommand(message);
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

// ============================================================================
// HOME ASSISTANT MQTT DISCOVERY
// ============================================================================
void publishHomeAssistantDiscovery() {
  Serial.println("📡 Publishing Home Assistant MQTT Discovery...");
  
  String deviceId = String("chester_cablock_") + String(CABINET_ID);
  deviceId.toLowerCase();
  String deviceName = String("Cabinet Lock - ") + String(CABINET_ID);
  
  // Device configuration
  String device = "{\"identifiers\":[\"" + deviceId + "\"],"
                  "\"name\":\"" + deviceName + "\","
                  "\"model\":\"D1 Mini (ESP8266)\","
                  "\"manufacturer\":\"ChesterTheBus\"}";
  
  // Lock Entity
  String topic = String("homeassistant/lock/") + deviceId + "/lock/config";
  String payload = "{\"name\":\"" + deviceName + "\","
                   "\"unique_id\":\"" + deviceId + "_lock\","
                   "\"state_topic\":\"" + topic_cabinetLock_status + "\","
                   "\"command_topic\":\"" + topic_cabinetLock_command + "\","
                   "\"payload_lock\":\"LOCK\","
                   "\"payload_unlock\":\"UNLOCK\","
                   "\"state_locked\":\"LOCKED\","
                   "\"state_unlocked\":\"UNLOCKED\","
                   "\"optimistic\":false,"
                   "\"icon\":\"mdi:lock\","
                   "\"device\":" + device + "}";
  
  mqttClient.publish(topic.c_str(), payload.c_str(), true);
  
  Serial.println("✓ Home Assistant Discovery published");
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

      // Publish Home Assistant discovery
      publishHomeAssistantDiscovery();

      // Subscribe to individual command topic
      mqttClient.subscribe(topic_cabinetLock_command.c_str());
      Serial.print("Subscribed to: ");
      Serial.println(topic_cabinetLock_command);

      // Subscribe to group command topic (for controlling all locks at once)
      mqttClient.subscribe(topic_allLock_command);
      Serial.print("Subscribed to: ");
      Serial.println(topic_allLock_command);

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

  Serial.println("\n\n=== ChesterTheBus D1 Mini Cabinet Lock Controller ===");
  Serial.print("Cabinet ID: ");
  Serial.println(CABINET_ID);
  Serial.print("Command Topic: ");
  Serial.println(topic_cabinetLock_command);
  Serial.print("Status Topic: ");
  Serial.println(topic_cabinetLock_status);

  // Initialize servo
  lockServo.attach(SERVO_PIN);
  lockServo.write(UNLOCKED_POSITION);
  currentState = STATE_UNLOCKED;
  lockServo.detach();
  Serial.println("Servo initialized at UNLOCKED position");

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

    // Setup MQTT
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);

    // Connect to MQTT broker
    reconnectMQTT();

    Serial.println("Ready to receive lock/unlock commands via MQTT!");
    Serial.println("Commands: LOCK, UNLOCK, TOGGLE, STATUS");
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

  // Keep MQTT connection alive
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Periodically send status update (for monitoring/debugging)
  if (millis() - lastStatusUpdate > STATUS_INTERVAL) {
    sendStatus();
    lastStatusUpdate = millis();
  }

  delay(10);
}
