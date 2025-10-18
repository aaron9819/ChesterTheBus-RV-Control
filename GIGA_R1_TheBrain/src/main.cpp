#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"
#include <WiFi.h>
#include <PubSubClient.h>

// ============================================================================
// GIGA R1 "THE BRAIN" - ChesterTheBus RV Control
// ============================================================================
// Main controller for RV systems with touch display
// Cabinet Lock Control Interface
// ============================================================================

#define FIRMWARE_VERSION "v2.0.0"

// ----------------- WiFi Configuration -----------------
const char* ssid = "Chester IOT";
const char* password = "2025Chester9894";

// ----------------- MQTT Configuration -----------------
const char* mqtt_server = "192.168.8.1";  // GL-AR300M16 Router IP
const int mqtt_port = 1883;
const char* mqtt_client_id = "GIGA_R1_TheBrain";
const char* mqtt_user = "";
const char* mqtt_password = "";
String gigaIP;

// ----------------- MQTT Topics for Cabinet Locks -----------------
const char* topic_lock_kitchen_driver_cmd = "CabLockKitchenDriverSideCommand";
const char* topic_lock_kitchen_driver_status = "CabLockKitchenDriverSideStatus";

const char* topic_lock_kitchen_pass_cmd = "CabLockKitchenPassSideCommand";
const char* topic_lock_kitchen_pass_status = "CabLockKitchenPassSideStatus";

const char* topic_lock_rear_driver_cmd = "CabLockRearDriverSideCommand";
const char* topic_lock_rear_driver_status = "CabLockRearDriverSideStatus";

const char* topic_lock_plumbing_cmd = "CabLockRearPassSideCommand";  // D1 Plumbing cabinet
const char* topic_lock_plumbing_status = "CabLockRearPassSideStatus";

const char* topic_lock_all_cmd = "CabLockAllCommand";  // Control all locks at once

// System health topics
const char* topic_giga_status = "GIGASystemStatus";
const char* topic_giga_uptime = "GIGAUptime";

// ----------------- MQTT Client -----------------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ----------------- Display Configuration -----------------
GigaDisplay_GFX display;
Arduino_GigaDisplayTouch touchDetector;

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 480

// ----------------- Colors (RGB565) -----------------
#define COLOR_BACKGROUND 0x0000    // Black
#define COLOR_HEADER 0x001F        // Blue
#define COLOR_LOCKED 0xF800        // Red
#define COLOR_UNLOCKED 0x07E0      // Green
#define COLOR_WAITING 0xFFE0       // Yellow
#define COLOR_UNKNOWN 0x7BEF       // Gray
#define COLOR_TEXT_WHITE 0xFFFF    // White
#define COLOR_TEXT_BLACK 0x0000    // Black
#define COLOR_BORDER 0x4208        // Dark Gray

// ----------------- Touch Configuration -----------------
#define TOUCH_THRESHOLD 50
unsigned long lastTouchTime = 0;
const unsigned long TOUCH_DEBOUNCE_MS = 500;  // Increased from 300ms to 500ms

// ----------------- Lock State -----------------
enum LockState {
  STATE_UNKNOWN,
  STATE_UNLOCKED,
  STATE_LOCKED,
  STATE_WAITING
};

struct CabinetLock {
  const char* name;
  const char* cmdTopic;
  const char* statusTopic;
  LockState state;
  unsigned long lastCommandTime;
  int buttonX;
  int buttonY;
  int buttonWidth;
  int buttonHeight;
};

// Cabinet lock definitions (4 locks total)
CabinetLock locks[4] = {
  {"Kitchen Driver", topic_lock_kitchen_driver_cmd, topic_lock_kitchen_driver_status,
   STATE_UNKNOWN, 0, 50, 100, 340, 70},

  {"Kitchen Pass", topic_lock_kitchen_pass_cmd, topic_lock_kitchen_pass_status,
   STATE_UNKNOWN, 0, 410, 100, 340, 70},

  {"Rear Driver", topic_lock_rear_driver_cmd, topic_lock_rear_driver_status,
   STATE_UNKNOWN, 0, 50, 190, 340, 70},

  {"Plumbing", topic_lock_plumbing_cmd, topic_lock_plumbing_status,
   STATE_UNKNOWN, 0, 410, 190, 340, 70}
};

// Lock All / Unlock All buttons
struct ControlButton {
  const char* label;
  const char* command;
  int x;
  int y;
  int width;
  int height;
  uint16_t color;
};

ControlButton controlButtons[2] = {
  {"LOCK ALL", "LOCK", 50, 300, 340, 70, COLOR_LOCKED},
  {"UNLOCK ALL", "UNLOCK", 410, 300, 340, 70, COLOR_UNLOCKED}
};

const unsigned long COMMAND_TIMEOUT = 3000;  // 3 seconds

// ----------------- System State -----------------
unsigned long lastStatusPublish = 0;
const unsigned long STATUS_PUBLISH_INTERVAL = 60000;  // 60 seconds
bool uiNeedsRedraw = true;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void setupWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void subscribeToTopics();
void publishSystemStatus();
void drawUI();
void drawHeader();
void drawLockButton(int lockIndex);
void drawControlButton(int btnIndex);
void handleTouch();
void sendLockCommand(int lockIndex, const char* command);
void sendLockAllCommand(const char* command);
void updateLockStates();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n");
  Serial.println("\n\n=================================");
  Serial.println("  GIGA R1 - MAIN CONTROLLER");
  Serial.println("  Chester the Bus RV Control");
  Serial.println("  Firmware: " + String(FIRMWARE_VERSION));
  #ifdef BUILD_DATE
  Serial.print("  Build: ");
  Serial.print(BUILD_DATE);
  Serial.print(" ");
  Serial.println(BUILD_TIME);
  #endif
  Serial.println("=================================\n");

  // Initialize display
  display.begin();
  display.fillScreen(COLOR_BACKGROUND);
  display.setRotation(1);  // Landscape orientation
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_WHITE);

  // Show loading message
  display.setCursor(280, 220);
  display.println("Initializing...");

  Serial.println("✓ Display initialized (800x480)");

  // Initialize touch
  if (touchDetector.begin()) {
    Serial.println("✓ Touch initialized");
  } else {
    Serial.println("✗ Touch initialization failed");
  }

  // Connect to WiFi
  setupWiFi();

  // Setup MQTT
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);

  // Connect to MQTT
  reconnectMQTT();

  // Draw initial UI
  drawUI();

  Serial.println("\n✓ Initialization complete");
  Serial.println("============================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Handle touch input
  handleTouch();

  // Update lock states (check for timeouts)
  updateLockStates();

  // Redraw UI if needed
  if (uiNeedsRedraw) {
    drawUI();
    uiNeedsRedraw = false;
  }

  // Publish system status periodically
  if (currentMillis - lastStatusPublish >= STATUS_PUBLISH_INTERVAL) {
    lastStatusPublish = currentMillis;
    publishSystemStatus();
  }

  delay(10);
}

// ============================================================================
// WiFi SETUP
// ============================================================================
void setupWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected");
    gigaIP = WiFi.localIP().toString();
    Serial.print("  IP Address: ");
    Serial.println(gigaIP);
    Serial.print("  Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\n✗ WiFi connection failed!");
    gigaIP = "No Connection";
  }
}

// ============================================================================
// MQTT CONNECTION
// ============================================================================
void reconnectMQTT() {
  static unsigned long lastAttempt = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastAttempt < 5000) {
    return;  // Don't attempt too frequently
  }
  lastAttempt = currentMillis;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping MQTT reconnect");
    return;
  }

  Serial.print("Connecting to MQTT broker at ");
  Serial.print(mqtt_server);
  Serial.print(":");
  Serial.print(mqtt_port);
  Serial.print("...");

  if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
    Serial.println(" connected!");

    // Publish online status
    mqttClient.publish(topic_giga_status, "ONLINE", true);

    // Subscribe to all status topics
    subscribeToTopics();

    // Publish system info
    publishSystemStatus();

    Serial.println("✓ MQTT setup complete");

    // Request initial status from all locks
    Serial.println("Requesting initial lock states...");
    for (int i = 0; i < 4; i++) {
      mqttClient.publish(locks[i].cmdTopic, "STATUS");
    }

    uiNeedsRedraw = true;
  } else {
    Serial.print(" failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// ============================================================================
// SUBSCRIBE TO MQTT TOPICS
// ============================================================================
void subscribeToTopics() {
  // Subscribe to all cabinet lock status topics
  mqttClient.subscribe(topic_lock_kitchen_driver_status);
  mqttClient.subscribe(topic_lock_kitchen_pass_status);
  mqttClient.subscribe(topic_lock_rear_driver_status);
  mqttClient.subscribe(topic_lock_plumbing_status);

  Serial.println("✓ Subscribed to cabinet lock status topics");
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

  // Update lock states based on status messages
  for (int i = 0; i < 4; i++) {
    if (String(topic) == locks[i].statusTopic) {
      LockState newState = STATE_UNKNOWN;

      if (message == "LOCKED") {
        newState = STATE_LOCKED;
      } else if (message == "UNLOCKED") {
        newState = STATE_UNLOCKED;
      }

      if (locks[i].state != newState) {
        locks[i].state = newState;
        uiNeedsRedraw = true;
        Serial.print("  → ");
        Serial.print(locks[i].name);
        Serial.print(" state: ");
        Serial.println(message);
      }
      break;
    }
  }
}

// ============================================================================
// PUBLISH SYSTEM STATUS
// ============================================================================
void publishSystemStatus() {
  if (!mqttClient.connected()) return;

  // Publish uptime
  char uptimeStr[16];
  unsigned long uptime = millis() / 1000;
  snprintf(uptimeStr, sizeof(uptimeStr), "%lu", uptime);
  mqttClient.publish(topic_giga_uptime, uptimeStr);

  // Publish online status
  mqttClient.publish(topic_giga_status, "ONLINE");

  Serial.print("📊 Status published | Uptime: ");
  Serial.print(uptime);
  Serial.println("s");
}

// ============================================================================
// DRAW USER INTERFACE
// ============================================================================
void drawUI() {
  display.fillScreen(COLOR_BACKGROUND);

  // Draw header
  drawHeader();

  // Draw individual lock buttons
  for (int i = 0; i < 4; i++) {
    drawLockButton(i);
  }

  // Draw control buttons (Lock All / Unlock All)
  for (int i = 0; i < 2; i++) {
    drawControlButton(i);
  }

  // Draw footer with connection status
  display.setTextSize(1);
  display.setTextColor(COLOR_TEXT_WHITE);
  display.setCursor(10, 460);
  display.print("IP: ");
  display.print(gigaIP);
  display.print(" | MQTT: ");
  display.print(mqttClient.connected() ? "Connected" : "Disconnected");
  display.print(" | FW: ");
  display.print(FIRMWARE_VERSION);
}

// ============================================================================
// DRAW HEADER
// ============================================================================
void drawHeader() {
  // Header bar
  display.fillRect(0, 0, SCREEN_WIDTH, 60, COLOR_HEADER);

  // Title
  display.setTextSize(3);
  display.setTextColor(COLOR_TEXT_WHITE);
  display.setCursor(200, 18);
  display.println("ChesterTheBus RV");

  // Subtitle
  display.setTextSize(2);
  display.setCursor(270, 42);
  display.println("Cabinet Locks");
}

// ============================================================================
// DRAW LOCK BUTTON
// ============================================================================
void drawLockButton(int lockIndex) {
  CabinetLock& lock = locks[lockIndex];

  // Determine button color based on state
  uint16_t buttonColor;
  const char* stateText;

  switch (lock.state) {
    case STATE_LOCKED:
      buttonColor = COLOR_LOCKED;
      stateText = "LOCKED";
      break;
    case STATE_UNLOCKED:
      buttonColor = COLOR_UNLOCKED;
      stateText = "UNLOCKED";
      break;
    case STATE_WAITING:
      buttonColor = COLOR_WAITING;
      stateText = "WAITING...";
      break;
    default:
      buttonColor = COLOR_UNKNOWN;
      stateText = "UNKNOWN";
      break;
  }

  // Draw button background
  display.fillRoundRect(lock.buttonX, lock.buttonY,
                       lock.buttonWidth, lock.buttonHeight,
                       10, buttonColor);

  // Draw border
  display.drawRoundRect(lock.buttonX, lock.buttonY,
                       lock.buttonWidth, lock.buttonHeight,
                       10, COLOR_BORDER);

  // Draw lock name
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  int textX = lock.buttonX + (lock.buttonWidth / 2) - (strlen(lock.name) * 6);
  display.setCursor(textX, lock.buttonY + 15);
  display.println(lock.name);

  // Draw state
  display.setTextSize(2);
  textX = lock.buttonX + (lock.buttonWidth / 2) - (strlen(stateText) * 6);
  display.setCursor(textX, lock.buttonY + 42);
  display.println(stateText);
}

// ============================================================================
// DRAW CONTROL BUTTON (Lock All / Unlock All)
// ============================================================================
void drawControlButton(int btnIndex) {
  ControlButton& btn = controlButtons[btnIndex];

  // Draw button background
  display.fillRoundRect(btn.x, btn.y, btn.width, btn.height, 10, btn.color);

  // Draw border
  display.drawRoundRect(btn.x, btn.y, btn.width, btn.height, 10, COLOR_BORDER);

  // Draw label
  display.setTextSize(3);
  display.setTextColor(COLOR_TEXT_BLACK);
  int textX = btn.x + (btn.width / 2) - (strlen(btn.label) * 9);
  display.setCursor(textX, btn.y + 23);
  display.println(btn.label);
}

// ============================================================================
// HANDLE TOUCH
// ============================================================================
void handleTouch() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastTouchTime < TOUCH_DEBOUNCE_MS) {
    return;  // Debounce
  }

  uint8_t contacts;
  GDTpoint_t points[5];
  contacts = touchDetector.getTouchPoints(points);

  if (contacts > 0) {
    // Raw touch coordinates (portrait mode: 480x800)
    int rawX = points[0].x;
    int rawY = points[0].y;

    // Transform coordinates for landscape mode (rotation = 1)
    // In landscape: screen is 800x480
    // Touch sensor reports in portrait: 480x800
    // Transformation: x_landscape = rawY, y_landscape = 480 - rawX
    int x = rawY;
    int y = 480 - rawX;

    Serial.print("Touch detected - Raw: X:");
    Serial.print(rawX);
    Serial.print(" Y:");
    Serial.print(rawY);
    Serial.print(" | Transformed: X:");
    Serial.print(x);
    Serial.print(" Y:");
    Serial.println(y);

    // Check individual lock buttons
    for (int i = 0; i < 4; i++) {
      CabinetLock& lock = locks[i];
      if (x >= lock.buttonX && x <= (lock.buttonX + lock.buttonWidth) &&
          y >= lock.buttonY && y <= (lock.buttonY + lock.buttonHeight)) {

        // Ignore touch if lock is already waiting for a response
        if (lock.state == STATE_WAITING) {
          Serial.print("Ignoring touch - ");
          Serial.print(lock.name);
          Serial.println(" is already waiting for response");
          lastTouchTime = currentMillis;
          return;
        }

        // Toggle lock state
        if (lock.state == STATE_LOCKED) {
          sendLockCommand(i, "UNLOCK");
        } else {
          sendLockCommand(i, "LOCK");
        }

        lastTouchTime = currentMillis;
        return;
      }
    }

    // Check Lock All / Unlock All buttons
    for (int i = 0; i < 2; i++) {
      ControlButton& btn = controlButtons[i];
      if (x >= btn.x && x <= (btn.x + btn.width) &&
          y >= btn.y && y <= (btn.y + btn.height)) {

        sendLockAllCommand(btn.command);
        lastTouchTime = currentMillis;
        return;
      }
    }
  }
}

// ============================================================================
// SEND LOCK COMMAND
// ============================================================================
void sendLockCommand(int lockIndex, const char* command) {
  if (!mqttClient.connected()) {
    Serial.println("Cannot send command - MQTT not connected");
    return;
  }

  CabinetLock& lock = locks[lockIndex];

  Serial.print("Sending command to ");
  Serial.print(lock.name);
  Serial.print(": ");
  Serial.println(command);

  // Publish command
  mqttClient.publish(lock.cmdTopic, command);

  // Set waiting state
  lock.state = STATE_WAITING;
  lock.lastCommandTime = millis();

  uiNeedsRedraw = true;
}

// ============================================================================
// SEND LOCK ALL COMMAND
// ============================================================================
void sendLockAllCommand(const char* command) {
  if (!mqttClient.connected()) {
    Serial.println("Cannot send command - MQTT not connected");
    return;
  }

  Serial.print("Sending command to ALL locks: ");
  Serial.println(command);

  // Send command to all individual locks
  for (int i = 0; i < 4; i++) {
    mqttClient.publish(locks[i].cmdTopic, command);
    locks[i].state = STATE_WAITING;
    locks[i].lastCommandTime = millis();
  }

  // Also publish to the "all" topic if cabinet locks listen to it
  mqttClient.publish(topic_lock_all_cmd, command);

  uiNeedsRedraw = true;
}

// ============================================================================
// UPDATE LOCK STATES
// ============================================================================
void updateLockStates() {
  unsigned long currentMillis = millis();
  bool stateChanged = false;

  // Check for command timeouts
  for (int i = 0; i < 4; i++) {
    if (locks[i].state == STATE_WAITING) {
      if (currentMillis - locks[i].lastCommandTime > COMMAND_TIMEOUT) {
        // Command timed out - mark as unknown
        locks[i].state = STATE_UNKNOWN;
        stateChanged = true;

        Serial.print("⚠️  ");
        Serial.print(locks[i].name);
        Serial.println(" command timeout");
      }
    }
  }

  if (stateChanged) {
    uiNeedsRedraw = true;
  }
}
