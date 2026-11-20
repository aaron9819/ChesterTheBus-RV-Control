#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ChesterNet.h>

// ============================================
// CONFIGURATION - Change for each D1 Mini
// ============================================
// Set unique cabinet identifier for each D1 Mini
#define CABINET_ID "RearDriverSide"  // Options: kitchen1, kitchen2, bathroom, bedroom, etc.
#define FIRMWARE_VERSION "v3.2.1"

// WiFi, MQTT broker and the device (OTA) password are stored on the board by
// ChesterNet (lib_shared/ChesterNet) and editable at http://<hostname>.local/.
// secrets.ini at the repo root only supplies the defaults a board starts with.
// These are filled in from ChesterNet at the top of setup().
const char* ssid = nullptr;
const char* password = nullptr;

// MQTT Configuration
const char* mqtt_server = nullptr;
int mqtt_port = 1883;
const char* mqtt_user = "";  // No username
const char* mqtt_password = "";  // No password

// OTA Configuration
const char* otaHostname = "CabLock-RearDriverSide";
const char* otaPassword = nullptr;

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
// Lock/unlock speed. The servo is stepped one degree at a time instead of being
// sent straight to the end position, so it travels at about 30% of its own
// full speed: ~7 ms/degree, about 0.6 s for the 85 degree throw, against
// ~0.15-0.2 s for a typical SG90/MG90 at full speed. Raise to slow it further.
const unsigned long SERVO_MS_PER_DEGREE = 7;
const unsigned long SERVO_SETTLE_MS = 300;  // Held at the end position before release
int servoPosition = -1;                     // Last commanded angle; -1 until the first move after boot

// Lock state is persisted so a reboot (e.g. a brownout while cranking the
// engine) puts the latch back where it was instead of reporting a state the
// servo was never actually driven to.
// The size must be ChesterNet's: it keeps the network settings in the same
// EEPROM sector, above CHESTERNET_EEPROM_OFFSET.
#define EEPROM_SIZE CHESTERNET_EEPROM_SIZE
#define EEPROM_MAGIC 0xCB
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_STATE 1

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

// Networking (non-blocking - the loop never stalls waiting for WiFi or the broker)
const unsigned long WIFI_KICK_MS = 30000;   // Restart the WiFi attempt this often while down
const unsigned long MQTT_RETRY_MS = 5000;   // Broker connection attempt interval
unsigned long lastWifiKick = 0;
unsigned long lastMqttAttempt = 0;
bool otaStarted = false;
bool wifiWasConnected = false;

// Command deduplication
String lastCommand = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_COOLDOWN_MS = 1000; // Ignore duplicate commands within 1 second

// Forward declarations
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void publishHomeAssistantDiscovery();
void lockCabinet();
void unlockCabinet();
void sendStatus();
void processCommand(String command);
void setupOTA();

// ============================================================================
// LOCK STATE PERSISTENCE
// ============================================================================
void saveLockState() {
  // ESP8266 EEPROM only writes flash when a byte actually changed
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.write(EEPROM_ADDR_STATE, (uint8_t)currentState);
  EEPROM.commit();
}

LockState loadLockState() {
  if (EEPROM.read(EEPROM_ADDR_MAGIC) == EEPROM_MAGIC) {
    uint8_t s = EEPROM.read(EEPROM_ADDR_STATE);
    if (s == STATE_LOCKED || s == STATE_UNLOCKED) return (LockState)s;
  }
  return STATE_UNLOCKED;  // Nothing saved yet
}

// Sweep the servo to a position at the reduced speed, hold it briefly, release it
void moveServo(int position) {
  if (!lockServo.attached()) {
    lockServo.attach(SERVO_PIN);
  }
  if (servoPosition < 0) {
    // First move after boot: the latch should already be at its saved
    // position, so hold it there rather than sweep from a guessed angle
    lockServo.write(position);
  } else {
    int step = (position > servoPosition) ? 1 : -1;
    for (int angle = servoPosition; angle != position; angle += step) {
      lockServo.write(angle);
      delay(SERVO_MS_PER_DEGREE);
    }
    lockServo.write(position);
  }
  servoPosition = position;
  delay(SERVO_SETTLE_MS);
  // Disable servo to save power and reduce noise
  lockServo.detach();
}

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

// One connection attempt at most every MQTT_RETRY_MS. This used to loop until
// connected, which stopped OTA and WiFi recovery for as long as the broker was
// unreachable.
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
    Serial.println(" - retrying in 5 seconds");
  }
}

void setupOTA() {
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

  // Stored network settings (defaults from secrets.ini on first boot)
  ChesterNet::begin(otaHostname, FIRMWARE_VERSION);
  ssid = ChesterNet::wifiSsid();
  password = ChesterNet::wifiPassword();
  mqtt_server = ChesterNet::mqttHost();
  mqtt_port = ChesterNet::mqttPort();
  otaPassword = ChesterNet::devicePassword();
  ChesterNet::onUpdateStart([]() {
    // A browser upload blocks loop() until it finishes - leave the servo unpowered
    if (lockServo.attached()) lockServo.detach();
  });

  // Restore the saved lock position. Previously the servo was attached and
  // detached with no delay, so it never actually moved, yet UNLOCKED was reported.
  EEPROM.begin(EEPROM_SIZE);
  currentState = loadLockState();
  moveServo(currentState == STATE_LOCKED ? LOCKED_POSITION : UNLOCKED_POSITION);
  Serial.print("Servo restored to ");
  Serial.println(currentState == STATE_LOCKED ? "LOCKED" : "UNLOCKED");

  // MQTT is configured whether or not WiFi is up yet. It used to be set up only
  // when WiFi connected within 20 s of boot - if the router came up later (a
  // whole-bus power-up), the lock never reached the broker until power-cycled.
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);

  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  lastWifiKick = millis();

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
    wifiWasConnected = true;

    setupOTA();
    reconnectMQTT();
  } else {
    Serial.println("\nWiFi not up yet - will keep trying from the loop");
  }

  Serial.println("Ready to receive lock/unlock commands via MQTT!");
  Serial.println("Commands: LOCK, UNLOCK, TOGGLE, STATUS");
  Serial.print("Firmware: ");
  Serial.println(FIRMWARE_VERSION);
}

void lockCabinet() {
  Serial.print(">>> LOCKING Cabinet: ");
  Serial.println(CABINET_ID);

  moveServo(LOCKED_POSITION);
  currentState = STATE_LOCKED;
  saveLockState();
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

  moveServo(UNLOCKED_POSITION);
  currentState = STATE_UNLOCKED;
  saveLockState();
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
  // Settings web page, browser firmware updates, setup hotspot
  ChesterNet::loop();

  // WiFi: the core reconnects on its own; only restart the attempt occasionally
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiWasConnected) {
      wifiWasConnected = false;
      Serial.println("WiFi disconnected (auto-reconnect running)");
    }
    if (millis() - lastWifiKick >= WIFI_KICK_MS && ChesterNet::wifiRetryAllowed()) {
      lastWifiKick = millis();
      Serial.println("WiFi still down - restarting connection attempt");
      WiFi.begin(ssid, password);
    }
  } else if (!wifiWasConnected) {
    wifiWasConnected = true;
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
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

  // Periodically send status update (for monitoring/debugging)
  if (millis() - lastStatusUpdate > STATUS_INTERVAL) {
    sendStatus();
    lastStatusUpdate = millis();
  }

  delay(10);
}
