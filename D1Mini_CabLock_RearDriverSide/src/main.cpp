#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <ArduinoOTA.h>

// ============================================
// CONFIGURATION - Change for each D1 Mini
// ============================================
// Set unique cabinet identifier for each D1 Mini
#define CABINET_ID "RearDriverSide"  // Options: kitchen1, kitchen2, bathroom, bedroom, etc.
#define FIRMWARE_VERSION "v1.1.0"

// WiFi Configuration
const char* ssid = "Chester IOT";
const char* password = "2025Chester9894";

// MQTT Configuration - Connect to Mosquitto broker on GL-AR300M16 router
const char* mqtt_server = "192.168.8.1";  // Router IP address
const int mqtt_port = 1883;
const char* mqtt_user = "";  // No username
const char* mqtt_password = "";  // No password

// OTA Configuration
const char* otaHostname = "CabLock-RearDriverSide";
const char* otaPassword = "Chester2025";

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
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_INTERVAL = 60000; // Send status every 60 seconds

// Command deduplication
String lastCommand = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_COOLDOWN_MS = 1000; // Ignore duplicate commands within 1 second

// Forward declarations
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
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
  unsigned long currentMillis = millis();

  // Ignore duplicate commands within cooldown period (except STATUS)
  if (command != "STATUS") {
    if (command == lastCommand && (currentMillis - lastCommandTime) < COMMAND_COOLDOWN_MS) {
      Serial.print("⚠️  Ignoring duplicate command within cooldown: ");
      Serial.println(command);
      return;
    }
    lastCommand = command;
    lastCommandTime = currentMillis;
  }

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

    // Setup OTA (Over-The-Air) updates
    ArduinoOTA.setHostname(otaHostname);
    ArduinoOTA.setPassword(otaPassword);

    ArduinoOTA.onStart([]() {
      Serial.println("\n🔄 OTA Update Starting...");
      // Detach servo during update for safety
      if (lockServo.attached()) {
        lockServo.detach();
      }
    });

    ArduinoOTA.onEnd([]() {
      Serial.println("\n✓ OTA Update Complete! Rebooting...");
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
    Serial.println("✓ OTA enabled");
    Serial.print("  Hostname: ");
    Serial.print(otaHostname);
    Serial.println(".local");

    // Setup MQTT
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);

    // Connect to MQTT broker
    reconnectMQTT();

    Serial.println("Ready to receive lock/unlock commands via MQTT!");
    Serial.println("Commands: LOCK, UNLOCK, TOGGLE, STATUS");
    Serial.print("Firmware: ");
    Serial.println(FIRMWARE_VERSION);
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

  // Periodically send status update (for monitoring/debugging)
  if (millis() - lastStatusUpdate > STATUS_INTERVAL) {
    sendStatus();
    lastStatusUpdate = millis();
  }

  delay(10);
}
