#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoOTA.h>

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

const char* topic_lock_kitchen_upper_cmd = "CabLockKitchenUpperPassSideCommand";
const char* topic_lock_kitchen_upper_status = "CabLockKitchenUpperPassSideStatus";

const char* topic_lock_rear_driver_cmd = "CabLockRearDriverSideCommand";
const char* topic_lock_rear_driver_status = "CabLockRearDriverSideStatus";

const char* topic_lock_rear_pass_cmd = "CabLockRearPassSideCommand";  // Environment board with cabinet lock
const char* topic_lock_rear_pass_status = "CabLockRearPassSideStatus";

const char* topic_lock_all_cmd = "CabLockAllCommand";  // Control all locks at once

// System health topics
const char* topic_giga_status = "GIGASystemStatus";
const char* topic_giga_uptime = "GIGAUptime";

// Plumbing system control topics
const char* topic_fresh_water_heat_cmd = "FreshWaterHeatCommand";
const char* topic_fresh_water_heat_status = "FreshWaterHeatStatus";
const char* topic_grey_water_heat_cmd = "GreyWaterHeatCommand";
const char* topic_grey_water_heat_status = "GreyWaterHeatStatus";
const char* topic_exhaust_fan_cmd = "ExhaustFanCommand";
const char* topic_exhaust_fan_status = "ExhaustFanStatus";
const char* topic_engine_loop_cmd = "EngineLoopCommand";
const char* topic_engine_loop_status = "EngineLoopStatus";
const char* topic_diesel_heater_cmd = "DieselHeaterCommand";
const char* topic_diesel_heater_status = "DieselHeaterStatus";

// Rear loop and thermostat topics (shared between plumbing and thermostat pages)
const char* topic_rear_loop_cmd = "RearLoopCommand";
const char* topic_rear_loop_status = "RearLoopStatus";
const char* topic_front_loop_cmd = "FrontLoopCommand";
const char* topic_front_loop_status = "FrontLoopStatus";
const char* topic_pump_cmd = "HotWaterPumpCommand";
const char* topic_pump_status = "HotWaterPumpStatus";
const char* topic_main_pump_cmd = "MainWaterPumpCommand";
const char* topic_main_pump_status = "MainWaterPumpStatus";
const char* topic_domestic_hw_cmd = "DomesticHotWaterCommand";
const char* topic_domestic_hw_status = "DomesticHotWaterStatus";
const char* topic_rear_thermostat_temp = "RearThermostatTemperature";
const char* topic_rear_thermostat_humidity = "RearThermostatHumidity";
const char* topic_rear_thermostat_pressure = "RearThermostatPressure";
const char* topic_rear_thermostat_setpoint = "RearThermostatSetpoint";
const char* topic_rear_thermostat_mode = "RearThermostatMode";  // AUTO/OFF

// Additional temperature sensor topics
const char* topic_hydronic_temp = "HydronicTemperature";
const char* topic_fresh_water_temp = "FreshWaterTemperature";
const char* topic_grey_water_temp = "GreyWaterTemperature";

// ----------------- MQTT Client -----------------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ----------------- BME280 Sensor -----------------
Adafruit_BME280 bme;
bool bmeAvailable = false;

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
bool touchCurrentlyPressed = false;  // Track if touch is being held down

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

// Cabinet lock definitions (5 locks total)
CabinetLock locks[5] = {
  {"Kitchen Driver", topic_lock_kitchen_driver_cmd, topic_lock_kitchen_driver_status,
   STATE_UNKNOWN, 0, 50, 100, 340, 70},

  {"Kitchen Pass", topic_lock_kitchen_pass_cmd, topic_lock_kitchen_pass_status,
   STATE_UNKNOWN, 0, 410, 100, 340, 70},

  {"Kitchen Upper", topic_lock_kitchen_upper_cmd, topic_lock_kitchen_upper_status,
   STATE_UNKNOWN, 0, 50, 190, 340, 70},

  {"Rear Driver", topic_lock_rear_driver_cmd, topic_lock_rear_driver_status,
   STATE_UNKNOWN, 0, 410, 190, 340, 70},

  {"Rear Pass/Env", topic_lock_rear_pass_cmd, topic_lock_rear_pass_status,
   STATE_UNKNOWN, 0, 50, 280, 340, 70}
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
  {"LOCK ALL", "LOCK", 50, 370, 340, 70, COLOR_LOCKED},
  {"UNLOCK ALL", "UNLOCK", 410, 370, 340, 70, COLOR_UNLOCKED}
};

const unsigned long COMMAND_TIMEOUT = 3000;  // 3 seconds

// ----------------- System State -----------------
unsigned long lastStatusPublish = 0;
const unsigned long STATUS_PUBLISH_INTERVAL = 60000;  // 60 seconds
bool uiNeedsRedraw = true;
bool backlightOn = true;  // Track backlight state
unsigned long lastUserInteraction = 0;  // Track last touch time for screen timeout
const unsigned long SCREEN_TIMEOUT_MS = 300000;  // 5 minutes = 300,000ms

// ----------------- UI Pages -----------------
enum UIPage {
  PAGE_CABINET_LOCKS,
  PAGE_THERMOSTAT,
  PAGE_PLUMBING
};

UIPage currentPage = PAGE_CABINET_LOCKS;

// ----------------- Rear Thermostat State -----------------
float rearTemperature = 0.0;      // °F
float rearHumidity = 0.0;         // %
float rearPressure = 0.0;         // hPa
float rearThermostatSetpoint = 68.0;  // °F
String rearThermostatMode = "OFF";  // AUTO or OFF
bool rearLoopOpen = false;
unsigned long lastThermostatRead = 0;
unsigned long lastThermostatControl = 0;
unsigned long lastThermostatButtonPress = 0;  // Track button press timing
const unsigned long THERMOSTAT_READ_INTERVAL = 5000;    // 5 seconds
const unsigned long THERMOSTAT_CONTROL_INTERVAL = 30000; // 30 seconds
const unsigned long THERMOSTAT_BUTTON_COOLDOWN = 300;   // 300ms cooldown between button presses
const float THERMOSTAT_HYSTERESIS = 1.0;  // °F

// ----------------- Additional Temperature Sensors -----------------
float hydronicTemp = 0.0;         // °F
float freshWaterTemp = 0.0;       // °F
float greyWaterTemp = 0.0;        // °F

// ----------------- Plumbing System State -----------------
bool freshWaterHeaterOn = false;
bool greyWaterHeaterOn = false;
bool exhaustFanOn = false;
bool pumpRunning = false;
bool rearLoopValveOpen = false;
bool engineLoopValveOpen = false;
bool frontLoopValveOpen = false;
String dieselHeaterState = "OFF";  // OFF, PUMP ONLY, or HIGH
bool mainWaterPumpOn = true;
bool DHWSolinoidOpen = false;

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
void setupOTA();
void readBME280();
void publishThermostatReadings();
void handleThermostatControl();
void controlRearLoop(bool open);
void drawThermostatPage();
void handleThermostatTouch();
void drawPageNav();
void wakeScreen();
void checkScreenTimeout();
void drawPlumbingPage();
void handlePlumbingTouch();
void sendPlumbingCommand(const char* topic, const char* command);

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

  // Initialize I2C for BME280 (using default SDA/SCL pins)
  Wire.begin();
  Serial.println("✓ I2C initialized");

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

  // Connect to WiFi
  setupWiFi();

  // Setup MQTT
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);

  // Connect to MQTT
  reconnectMQTT();

  // Setup OTA updates
  // Note: OTA temporarily disabled - use USB upload
  // setupOTA();

  // Initial BME280 reading
  if (bmeAvailable) {
    readBME280();
  }

  // Initialize screen timeout
  lastUserInteraction = millis();

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

  // Handle OTA updates
  // ArduinoOTA.handle();  // Disabled - use USB upload

  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Handle touch input
  if (currentPage == PAGE_CABINET_LOCKS) {
    handleTouch();
  } else if (currentPage == PAGE_THERMOSTAT) {
    handleThermostatTouch();
  } else if (currentPage == PAGE_PLUMBING) {
    handlePlumbingTouch();
  }

  // Update lock states (check for timeouts)
  updateLockStates();

  // Read BME280 sensor periodically
  if (currentMillis - lastThermostatRead >= THERMOSTAT_READ_INTERVAL) {
    lastThermostatRead = currentMillis;
    if (bmeAvailable) {
      readBME280();
      publishThermostatReadings();
      if (currentPage == PAGE_THERMOSTAT) {
        uiNeedsRedraw = true;
      }
    }
  }

  // Handle thermostat control logic
  if (currentMillis - lastThermostatControl >= THERMOSTAT_CONTROL_INTERVAL) {
    lastThermostatControl = currentMillis;
    handleThermostatControl();
  }

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

  // Check for screen timeout
  checkScreenTimeout();

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
    for (int i = 0; i < 5; i++) {
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
  mqttClient.subscribe(topic_lock_kitchen_upper_status);
  mqttClient.subscribe(topic_lock_rear_driver_status);
  mqttClient.subscribe(topic_lock_rear_pass_status);

  // Subscribe to plumbing system status topics
  mqttClient.subscribe(topic_fresh_water_heat_status);
  mqttClient.subscribe(topic_grey_water_heat_status);
  mqttClient.subscribe(topic_exhaust_fan_status);
  mqttClient.subscribe(topic_rear_loop_status);
  mqttClient.subscribe(topic_engine_loop_status);
  mqttClient.subscribe(topic_diesel_heater_status);
  mqttClient.subscribe(topic_front_loop_status);
  mqttClient.subscribe(topic_pump_status);
  mqttClient.subscribe(topic_main_pump_status);
  mqttClient.subscribe(topic_domestic_hw_status);

  // Subscribe to thermostat topics
  mqttClient.subscribe(topic_rear_loop_status);

  // Subscribe to temperature sensor topics
  mqttClient.subscribe(topic_hydronic_temp);
  mqttClient.subscribe(topic_fresh_water_temp);
  mqttClient.subscribe(topic_grey_water_temp);

  Serial.println("✓ Subscribed to cabinet lock status topics (5 locks)");
  Serial.println("✓ Subscribed to plumbing system status topics");
  Serial.println("✓ Subscribed to thermostat topics");
  Serial.println("✓ Subscribed to temperature sensor topics");
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
  for (int i = 0; i < 5; i++) {
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

  // Update rear loop status
  if (String(topic) == topic_rear_loop_status) {
    bool newStatus = (message == "OPEN");
    if (rearLoopOpen != newStatus) {
      rearLoopOpen = newStatus;
      Serial.print("  → Rear loop: ");
      Serial.println(message);
      if (currentPage == PAGE_THERMOSTAT) {
        uiNeedsRedraw = true;
      }
    }
  }

  // Update plumbing system status
  if (String(topic) == topic_fresh_water_heat_status) {
    bool newStatus = (message == "ON");
    if (freshWaterHeaterOn != newStatus) {
      freshWaterHeaterOn = newStatus;
      Serial.print("  → Fresh water heater: ");
      Serial.println(message);
      if (currentPage == PAGE_PLUMBING) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_grey_water_heat_status) {
    bool newStatus = (message == "ON");
    if (greyWaterHeaterOn != newStatus) {
      greyWaterHeaterOn = newStatus;
      Serial.print("  → Grey water heater: ");
      Serial.println(message);
      if (currentPage == PAGE_PLUMBING) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_exhaust_fan_status) {
    bool newStatus = (message == "ON");
    if (exhaustFanOn != newStatus) {
      exhaustFanOn = newStatus;
      Serial.print("  → Exhaust fan: ");
      Serial.println(message);
      if (currentPage == PAGE_PLUMBING) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_pump_status) {
    bool newStatus = (message == "RUNNING" || message == "ON");
    if (pumpRunning != newStatus) {
      pumpRunning = newStatus;
      Serial.print("  → Hot water pump: ");
      Serial.println(message);
      if (currentPage == PAGE_PLUMBING) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_rear_loop_status) {
    bool newStatus = (message == "OPEN");
    if (rearLoopValveOpen != newStatus) {
      rearLoopValveOpen = newStatus;
      Serial.print("  → Rear loop valve: ");
      Serial.println(message);
      if (currentPage == PAGE_PLUMBING) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_engine_loop_status) {
    bool newStatus = (message == "OPEN");
    if (engineLoopValveOpen != newStatus) {
      engineLoopValveOpen = newStatus;
      Serial.print("  → Engine loop valve: ");
      Serial.println(message);
      if (currentPage == PAGE_PLUMBING) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_diesel_heater_status) {
    if (dieselHeaterState != message) {
      dieselHeaterState = message;
      Serial.print("  → Diesel heater: ");
      Serial.println(message);
      if (currentPage == PAGE_PLUMBING) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_front_loop_status) {
    bool newState = (message == "ON" || message == "OPEN");
    if (frontLoopValveOpen != newState) {
      frontLoopValveOpen = newState;
      Serial.print("  → Front loop valve: ");
      Serial.println(message);
      if (currentPage == PAGE_PLUMBING) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_main_pump_status) {
    bool newStatus = (message == "ON");
    if (mainWaterPumpOn != newStatus) {
      mainWaterPumpOn = newStatus;
      Serial.print("  → Main water pump: ");
      Serial.println(message);
      if (currentPage == PAGE_PLUMBING) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_domestic_hw_status) {
    bool newStatus = (message == "OPEN");
    if (DHWSolinoidOpen != newStatus) {
      DHWSolinoidOpen = newStatus;
      Serial.print("  → Domestic HW Solinoid: ");
      Serial.println(message);
      if (currentPage == PAGE_PLUMBING) {
        uiNeedsRedraw = true;
      }
    }
  }

  // Handle temperature sensor updates
  if (String(topic) == topic_hydronic_temp) {
    float newTemp = atof(message.c_str());
    if (abs(hydronicTemp - newTemp) > 0.1) {
      hydronicTemp = newTemp;
      Serial.print("  → Hydronic temp: ");
      Serial.print(hydronicTemp);
      Serial.println("°F");
      if (currentPage == PAGE_THERMOSTAT) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_fresh_water_temp) {
    float newTemp = atof(message.c_str());
    if (abs(freshWaterTemp - newTemp) > 0.1) {
      freshWaterTemp = newTemp;
      Serial.print("  → Fresh water temp: ");
      Serial.print(freshWaterTemp);
      Serial.println("°F");
      if (currentPage == PAGE_THERMOSTAT) {
        uiNeedsRedraw = true;
      }
    }
  }

  if (String(topic) == topic_grey_water_temp) {
    float newTemp = atof(message.c_str());
    if (abs(greyWaterTemp - newTemp) > 0.1) {
      greyWaterTemp = newTemp;
      Serial.print("  → Grey water temp: ");
      Serial.print(greyWaterTemp);
      Serial.println("°F");
      if (currentPage == PAGE_THERMOSTAT) {
        uiNeedsRedraw = true;
      }
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

  // If backlight is off, just show black screen
  if (!backlightOn) {
    return;
  }

  if (currentPage == PAGE_CABINET_LOCKS) {
    // Draw header
    drawHeader();

    // Draw page navigation
    drawPageNav();

    // Draw individual lock buttons
    for (int i = 0; i < 5; i++) {
      drawLockButton(i);
    }

    // Draw control buttons (Lock All / Unlock All)
    for (int i = 0; i < 2; i++) {
      drawControlButton(i);
    }
  } else if (currentPage == PAGE_THERMOSTAT) {
    drawThermostatPage();
  } else if (currentPage == PAGE_PLUMBING) {
    drawPlumbingPage();
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

  // Page navigation tabs in header
  drawPageNav();
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

  uint8_t contacts;
  GDTpoint_t points[5];
  contacts = touchDetector.getTouchPoints(points);

  // Detect touch release
  if (contacts == 0) {
    if (touchCurrentlyPressed) {
      touchCurrentlyPressed = false;
      Serial.println("Touch released");
    }
    return;
  }

  // If touch is still being held down from previous press, ignore
  if (touchCurrentlyPressed) {
    return;
  }

  // Debounce check
  if (currentMillis - lastTouchTime < TOUCH_DEBOUNCE_MS) {
    return;
  }

  if (contacts > 0) {
    // If screen is off, just wake it and don't process the touch
    if (!backlightOn) {
      wakeScreen();
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;  // Prevent immediate button press after wake
      return;
    }

    // Update last user interaction time
    lastUserInteraction = currentMillis;
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

    // Check page navigation buttons first (in header: y 10-50)
    if (y >= 10 && y <= 50) {
      if (x >= 395 && x <= 525) {
        // Locks page button
        if (currentPage != PAGE_CABINET_LOCKS) {
          currentPage = PAGE_CABINET_LOCKS;
          uiNeedsRedraw = true;
          Serial.println("Switched to Locks page");
        }
        lastTouchTime = currentMillis;
        touchCurrentlyPressed = true;
        return;
      } else if (x >= 535 && x <= 665) {
        // Thermostat page button
        if (currentPage != PAGE_THERMOSTAT) {
          currentPage = PAGE_THERMOSTAT;
          uiNeedsRedraw = true;
          Serial.println("Switched to Thermostat page");
        }
        lastTouchTime = currentMillis;
        touchCurrentlyPressed = true;
        return;
      } else if (x >= 675 && x <= 790) {
        // Plumbing page button
        if (currentPage != PAGE_PLUMBING) {
          currentPage = PAGE_PLUMBING;
          uiNeedsRedraw = true;
          Serial.println("Switched to Plumbing page");
        }
        lastTouchTime = currentMillis;
        touchCurrentlyPressed = true;
        return;
      }
    }

    // Check individual lock buttons
    for (int i = 0; i < 5; i++) {
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
  for (int i = 0; i < 5; i++) {
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
  for (int i = 0; i < 5; i++) {
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

// ============================================================================
// SETUP OTA UPDATES
// ============================================================================
void setupOTA() {
  // GIGA R1 uses InternalStorage (defined as extern in library)
  auto ip = WiFi.localIP();

  // Begin OTA with GIGA R1 specific API
  ArduinoOTA.begin(ip, "GIGA-TheBrain", "Chester2025", InternalStorage);

  Serial.println("✓ OTA initialized");
  Serial.print("  IP Address: ");
  Serial.println(ip);
  Serial.println("  Name: GIGA-TheBrain");
  Serial.println("  Password: Chester2025");
  Serial.println("  Note: Use Arduino IDE for OTA uploads");
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
// dtostrf replacement for GIGA R1 (ARM doesn't have dtostrf)
char* dtostrf(double val, signed char width, unsigned char prec, char* sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}

// ============================================================================
// BME280 SENSOR READING
// ============================================================================
void readBME280() {
  if (!bmeAvailable) return;

  float tempC = bme.readTemperature();
  rearTemperature = (tempC * 9.0 / 5.0) + 32.0;  // Convert to Fahrenheit
  rearHumidity = bme.readHumidity();
  rearPressure = bme.readPressure() / 100.0F;  // Convert to hPa
}

void publishThermostatReadings() {
  if (!mqttClient.connected()) return;

  char tempStr[8];

  // Publish temperature
  dtostrf(rearTemperature, 4, 2, tempStr);
  mqttClient.publish(topic_rear_thermostat_temp, tempStr);

  // Publish humidity
  dtostrf(rearHumidity, 4, 2, tempStr);
  mqttClient.publish(topic_rear_thermostat_humidity, tempStr);

  // Publish pressure
  dtostrf(rearPressure, 6, 2, tempStr);
  mqttClient.publish(topic_rear_thermostat_pressure, tempStr);

  // Publish setpoint and mode
  dtostrf(rearThermostatSetpoint, 4, 1, tempStr);
  mqttClient.publish(topic_rear_thermostat_setpoint, tempStr);
  mqttClient.publish(topic_rear_thermostat_mode, rearThermostatMode.c_str());
}

// ============================================================================
// THERMOSTAT CONTROL LOGIC
// ============================================================================
void handleThermostatControl() {
  if (!bmeAvailable || rearThermostatMode != "AUTO") return;

  // Hysteresis logic
  if (rearTemperature < (rearThermostatSetpoint - THERMOSTAT_HYSTERESIS)) {
    // Too cold - open rear loop to allow heat
    if (!rearLoopOpen) {
      Serial.println("🌡️ Thermostat: Opening rear loop (heating)");
      controlRearLoop(true);
    }
  }
  else if (rearTemperature > (rearThermostatSetpoint + THERMOSTAT_HYSTERESIS)) {
    // Too hot - close rear loop
    if (rearLoopOpen) {
      Serial.println("🌡️ Thermostat: Closing rear loop (cooling)");
      controlRearLoop(false);
    }
  }
  // Within hysteresis band - maintain current state
}

void controlRearLoop(bool open) {
  if (!mqttClient.connected()) return;

  mqttClient.publish(topic_rear_loop_cmd, open ? "OPEN" : "CLOSE");

  Serial.print("💧 Rear loop command sent: ");
  Serial.println(open ? "OPEN" : "CLOSE");
}

// ============================================================================
// DRAW THERMOSTAT PAGE
// ============================================================================
void drawThermostatPage() {
  // Header
  display.fillRect(0, 0, SCREEN_WIDTH, 60, COLOR_HEADER);
  display.setTextSize(3);
  display.setTextColor(COLOR_TEXT_WHITE);
  display.setCursor(250, 18);
  display.println("Rear Thermostat");

  // Page navigation tabs in header
  drawPageNav();

  // Current temperature - large display
  display.setTextSize(6);
  display.setTextColor(COLOR_TEXT_WHITE);
  display.setCursor(280, 90);
  char tempStr[10];
  dtostrf(rearTemperature, 4, 1, tempStr);
  display.print(tempStr);
  display.setTextSize(4);
  display.println(" F");

  // Humidity and Pressure
  display.setTextSize(2);
  display.setCursor(250, 180);
  display.print("Humidity: ");
  dtostrf(rearHumidity, 4, 1, tempStr);
  display.print(tempStr);
  display.println("%");

  display.setCursor(230, 210);
  display.print("Pressure: ");
  dtostrf(rearPressure, 6, 1, tempStr);
  display.print(tempStr);
  display.println(" hPa");

  // Setpoint display
  display.setTextSize(3);
  display.setCursor(280, 260);
  display.print("Set: ");
  dtostrf(rearThermostatSetpoint, 4, 1, tempStr);
  display.print(tempStr);
  display.println(" F");

  // Temp down button
  display.fillRoundRect(100, 320, 120, 80, 10, COLOR_HEADER);
  display.drawRoundRect(100, 320, 120, 80, 10, COLOR_BORDER);
  display.setTextSize(5);
  display.setTextColor(COLOR_TEXT_WHITE);
  display.setCursor(135, 340);
  display.println("-");

  // Temp up button
  display.fillRoundRect(580, 320, 120, 80, 10, COLOR_HEADER);
  display.drawRoundRect(580, 320, 120, 80, 10, COLOR_BORDER);
  display.setTextSize(5);
  display.setTextColor(COLOR_TEXT_WHITE);
  display.setCursor(615, 340);
  display.println("+");

  // Mode button (AUTO/OFF)
  uint16_t modeColor = (rearThermostatMode == "AUTO") ? COLOR_UNLOCKED : COLOR_UNKNOWN;
  display.fillRoundRect(300, 320, 200, 80, 10, modeColor);
  display.drawRoundRect(300, 320, 200, 80, 10, COLOR_BORDER);
  display.setTextSize(3);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(335, 345);
  display.println(rearThermostatMode);

  // Rear loop status indicator
  uint16_t statusColor = rearLoopOpen ? COLOR_LOCKED : COLOR_UNLOCKED;
  const char* statusText = rearLoopOpen ? "HEATING" : "IDLE";
  display.fillRoundRect(250, 420, 300, 30, 5, statusColor);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  int textWidth = strlen(statusText) * 12;
  display.setCursor(400 - textWidth/2, 425);
  display.println(statusText);

  // Additional Temperature Sensors Display (bottom of page)
  display.setTextSize(1);
  display.setTextColor(COLOR_TEXT_WHITE);

  // Draw a separator line
  display.drawLine(10, 455, 790, 455, COLOR_BORDER);

  // Display temperature sensors in columns
  // Left column
  display.setCursor(20, 80);
  display.print("Hydronic: ");
  if (hydronicTemp != 0.0) {
    display.print(hydronicTemp, 1);
    display.print("F");
  } else {
    display.print("---");
  }

  // Middle column
  display.setCursor(20, 120);
  display.print("Fresh Tank: ");
  if (freshWaterTemp != 0.0) {
    display.print(freshWaterTemp, 1);
    display.print("F");
  } else {
    display.print("---");
  }

  // Right column
  display.setCursor(20, 160);
  display.print("Grey Tank: ");
  if (greyWaterTemp != 0.0) {
    display.print(greyWaterTemp, 1);
    display.print("F");
  } else {
    display.print("---");
  }
}

// ============================================================================
// HANDLE THERMOSTAT TOUCH
// ============================================================================
void handleThermostatTouch() {
  unsigned long currentMillis = millis();

  uint8_t contacts;
  GDTpoint_t points[5];
  contacts = touchDetector.getTouchPoints(points);

  // Detect touch release
  if (contacts == 0) {
    if (touchCurrentlyPressed) {
      touchCurrentlyPressed = false;
      Serial.println("Touch released");
    }
    return;
  }

  // If screen is off, just wake it and don't process the touch
  if (!backlightOn) {
    wakeScreen();
    lastTouchTime = currentMillis;
    touchCurrentlyPressed = true;  // Prevent immediate button press
    return;
  }

  // Update last user interaction time
  lastUserInteraction = currentMillis;

  // If touch is still being held down from previous press, ignore
  if (touchCurrentlyPressed) {
    return;
  }

  // Debounce check
  if (currentMillis - lastTouchTime < TOUCH_DEBOUNCE_MS) {
    return;
  }

  if (contacts > 0) {
    int rawX = points[0].x;
    int rawY = points[0].y;
    int x = rawY;
    int y = 480 - rawX;

    Serial.print("Thermostat touch - X:");
    Serial.print(x);
    Serial.print(" Y:");
    Serial.println(y);

    // Check page navigation buttons first (in header: y 10-50)
    if (y >= 10 && y <= 50) {
      if (x >= 395 && x <= 525) {
        // Locks page button
        if (currentPage != PAGE_CABINET_LOCKS) {
          currentPage = PAGE_CABINET_LOCKS;
          uiNeedsRedraw = true;
          Serial.println("Switched to Locks page");
        }
        lastTouchTime = currentMillis;
        touchCurrentlyPressed = true;
        return;
      } else if (x >= 535 && x <= 665) {
        // Thermostat page button
        if (currentPage != PAGE_THERMOSTAT) {
          currentPage = PAGE_THERMOSTAT;
          uiNeedsRedraw = true;
          Serial.println("Switched to Thermostat page");
        }
        lastTouchTime = currentMillis;
        touchCurrentlyPressed = true;
        return;
      } else if (x >= 675 && x <= 790) {
        // Plumbing page button
        if (currentPage != PAGE_PLUMBING) {
          currentPage = PAGE_PLUMBING;
          uiNeedsRedraw = true;
          Serial.println("Switched to Plumbing page");
        }
        lastTouchTime = currentMillis;
        touchCurrentlyPressed = true;
        return;
      }
    }

    // Temp down button (100, 320, 120, 80)
    if (x >= 100 && x <= 220 && y >= 320 && y <= 400) {
      // Check if enough time has passed since last button press
      if (currentMillis - lastThermostatButtonPress < THERMOSTAT_BUTTON_COOLDOWN) {
        Serial.println("Ignoring temp down - button cooldown active");
        lastTouchTime = currentMillis;
        return;
      }

      rearThermostatSetpoint -= 1.0;
      if (rearThermostatSetpoint < 50.0) rearThermostatSetpoint = 50.0;
      Serial.print("Setpoint decreased to: ");
      Serial.println(rearThermostatSetpoint);
      publishThermostatReadings();
      uiNeedsRedraw = true;
      lastTouchTime = currentMillis;
      lastThermostatButtonPress = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Temp up button (580, 320, 120, 80)
    if (x >= 580 && x <= 700 && y >= 320 && y <= 400) {
      // Check if enough time has passed since last button press
      if (currentMillis - lastThermostatButtonPress < THERMOSTAT_BUTTON_COOLDOWN) {
        Serial.println("Ignoring temp up - button cooldown active");
        lastTouchTime = currentMillis;
        return;
      }

      rearThermostatSetpoint += 1.0;
      if (rearThermostatSetpoint > 85.0) rearThermostatSetpoint = 85.0;
      Serial.print("Setpoint increased to: ");
      Serial.println(rearThermostatSetpoint);
      publishThermostatReadings();
      uiNeedsRedraw = true;
      lastTouchTime = currentMillis;
      lastThermostatButtonPress = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Mode button (300, 320, 200, 80)
    if (x >= 300 && x <= 500 && y >= 320 && y <= 400) {
      // Check if enough time has passed since last button press
      if (currentMillis - lastThermostatButtonPress < THERMOSTAT_BUTTON_COOLDOWN) {
        Serial.println("Ignoring mode change - button cooldown active");
        lastTouchTime = currentMillis;
        return;
      }

      if (rearThermostatMode == "AUTO") {
        rearThermostatMode = "OFF";
        // Turn off rear loop when switching to OFF
        controlRearLoop(false);
      } else {
        rearThermostatMode = "AUTO";
      }
      Serial.print("Mode changed to: ");
      Serial.println(rearThermostatMode);
      publishThermostatReadings();
      uiNeedsRedraw = true;
      lastTouchTime = currentMillis;
      lastThermostatButtonPress = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }
  }
}

// ============================================================================
// SCREEN TIMEOUT MANAGEMENT
// ============================================================================
void wakeScreen() {
  if (!backlightOn) {
    backlightOn = true;
    Serial.println("💡 Screen waking up");
    uiNeedsRedraw = true;  // Redraw screen when waking
  }
  lastUserInteraction = millis();  // Reset timeout
}

void checkScreenTimeout() {
  if (backlightOn && (millis() - lastUserInteraction >= SCREEN_TIMEOUT_MS)) {
    backlightOn = false;
    Serial.println("� Screen timeout - going dark");
    display.fillScreen(COLOR_BACKGROUND);  // Black screen
  }
}// ============================================================================
// DRAW PAGE NAVIGATION
// ============================================================================
void drawPageNav() {
  // Page navigation buttons in header (top right)
  // Locks button
  uint16_t locksColor = (currentPage == PAGE_CABINET_LOCKS) ? COLOR_UNLOCKED : COLOR_UNKNOWN;
  display.fillRoundRect(395, 10, 130, 40, 5, locksColor);
  display.drawRoundRect(395, 10, 130, 40, 5, COLOR_BORDER);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(420, 18);
  display.println("LOCKS");

  // Thermostat button
  uint16_t thermoColor = (currentPage == PAGE_THERMOSTAT) ? COLOR_UNLOCKED : COLOR_UNKNOWN;
  display.fillRoundRect(535, 10, 130, 40, 5, thermoColor);
  display.drawRoundRect(535, 10, 130, 40, 5, COLOR_BORDER);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(548, 18);
  display.println("THERM");

  // Plumbing button
  uint16_t plumbColor = (currentPage == PAGE_PLUMBING) ? COLOR_UNLOCKED : COLOR_UNKNOWN;
  display.fillRoundRect(675, 10, 115, 40, 5, plumbColor);
  display.drawRoundRect(675, 10, 115, 40, 5, COLOR_BORDER);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(683, 18);
  display.println("PLUMB");
}

// ============================================================================
// DRAW PLUMBING SYSTEM CONTROL PAGE
// ============================================================================
void drawPlumbingPage() {
  display.fillScreen(COLOR_BACKGROUND);

  // Header
  display.setTextSize(3);
  display.setTextColor(COLOR_TEXT_WHITE);
  display.setCursor(250, 15);
  display.println("PLUMBING SYSTEM");

  // Page navigation
  drawPageNav();

  // Button layout: 4 rows of 3 buttons each (12 buttons total)
  // Row 1: Fresh Water | Grey Water | Exhaust Fan
  // Row 2: Rear Loop | Engine Loop | Diesel Heater
  // Row 3: Front Loop | Hot Water Pump | Main Water Pump
  // Row 4: Domestic HW Solinoid | (empty) | (empty)

  int buttonWidth = 240;
  int buttonHeight = 85;
  int col1X = 20;
  int col2X = 275;
  int col3X = 530;
  int row1Y = 70;
  int row2Y = 170;
  int row3Y = 270;
  int row4Y = 370;

  // Row 1, Col 1: Fresh Water Heater
  uint16_t freshColor = freshWaterHeaterOn ? COLOR_UNLOCKED : COLOR_LOCKED;
  display.fillRoundRect(col1X, row1Y, buttonWidth, buttonHeight, 10, freshColor);
  display.drawRoundRect(col1X, row1Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(col1X + 25, row1Y + 25);
  display.println("FRESH WATER");
  display.setCursor(col1X + 60, row1Y + 55);
  display.println(freshWaterHeaterOn ? "ON" : "OFF");

  // Row 1, Col 2: Grey Water Heater
  uint16_t greyColor = greyWaterHeaterOn ? COLOR_UNLOCKED : COLOR_LOCKED;
  display.fillRoundRect(col2X, row1Y, buttonWidth, buttonHeight, 10, greyColor);
  display.drawRoundRect(col2X, row1Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(col2X + 25, row1Y + 25);
  display.println("GREY WATER");
  display.setCursor(col2X + 60, row1Y + 55);
  display.println(greyWaterHeaterOn ? "ON" : "OFF");

  // Row 1, Col 3: Exhaust Fan
  uint16_t fanColor = exhaustFanOn ? COLOR_UNLOCKED : COLOR_LOCKED;
  display.fillRoundRect(col3X, row1Y, buttonWidth, buttonHeight, 10, fanColor);
  display.drawRoundRect(col3X, row1Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(col3X + 25, row1Y + 25);
  display.println("EXHAUST FAN");
  display.setCursor(col3X + 60, row1Y + 55);
  display.println(exhaustFanOn ? "ON" : "OFF");

  // Row 2, Col 1: Rear Loop Valve
  uint16_t rearColor = rearLoopValveOpen ? COLOR_UNLOCKED : COLOR_LOCKED;
  display.fillRoundRect(col1X, row2Y, buttonWidth, buttonHeight, 10, rearColor);
  display.drawRoundRect(col1X, row2Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(col1X + 45, row2Y + 25);
  display.println("REAR LOOP");
  display.setCursor(col1X + 60, row2Y + 55);
  display.println(rearLoopValveOpen ? "ON" : "OFF");

  // Row 2, Col 2: Engine Loop Valve
  uint16_t engineColor = engineLoopValveOpen ? COLOR_UNLOCKED : COLOR_LOCKED;
  display.fillRoundRect(col2X, row2Y, buttonWidth, buttonHeight, 10, engineColor);
  display.drawRoundRect(col2X, row2Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(col2X + 30, row2Y + 25);
  display.println("ENGINE LOOP");
  display.setCursor(col2X + 60, row2Y + 55);
  display.println(engineLoopValveOpen ? "ON" : "OFF");

  // Row 2, Col 3: Diesel Heater (cycles through OFF -> PUMP ONLY -> HIGH)
  uint16_t dieselColor;
  if (dieselHeaterState == "HIGH") {
    dieselColor = 0xF800;  // Red for HIGH
  } else if (dieselHeaterState == "PUMP ONLY") {
    dieselColor = 0xFFE0;  // Yellow for PUMP ONLY
  } else {
    dieselColor = COLOR_UNKNOWN;  // Gray for OFF
  }
  display.fillRoundRect(col3X, row2Y, buttonWidth, buttonHeight, 10, dieselColor);
  display.drawRoundRect(col3X, row2Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(col3X + 8, row2Y + 25);
  display.println("DIESEL HEATER");
  display.setTextSize(3);
  display.setCursor(col3X + 75, row2Y + 55);
  display.println(dieselHeaterState);

  // Row 3, Col 1: Front Loop Valve
  uint16_t frontColor = frontLoopValveOpen ? COLOR_UNLOCKED : COLOR_LOCKED;
  display.fillRoundRect(col1X, row3Y, buttonWidth, buttonHeight, 10, frontColor);
  display.drawRoundRect(col1X, row3Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(col1X + 40, row3Y + 25);
  display.println("FRONT LOOP");
  display.setCursor(col1X + 60, row3Y + 55);
  display.println(frontLoopValveOpen ? "ON" : "OFF");

  // Row 3, Col 2: Hot Water Pump
  uint16_t pumpColor = pumpRunning ? COLOR_UNLOCKED : COLOR_LOCKED;
  display.fillRoundRect(col2X, row3Y, buttonWidth, buttonHeight, 10, pumpColor);
  display.drawRoundRect(col2X, row3Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(col2X + 8, row3Y + 25);
  display.println("HOT WATER PUMP");
  display.setCursor(col2X + 60, row3Y + 55);
  display.println(pumpRunning ? "ON" : "OFF");

  // Row 3, Col 3: Main Water Pump
  uint16_t mainPumpColor = mainWaterPumpOn ? COLOR_UNLOCKED : COLOR_LOCKED;
  display.fillRoundRect(col3X, row3Y, buttonWidth, buttonHeight, 10, mainPumpColor);
  display.drawRoundRect(col3X, row3Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(col3X + 8, row3Y + 20);
  display.println("MAIN WATER");
  display.setCursor(col3X + 35, row3Y + 45);
  display.println("PUMP");
  display.setCursor(col3X + 60, row3Y + 65);
  display.setTextSize(2);
  display.println(mainWaterPumpOn ? "ON" : "OFF");

  // Row 4, Col 1: Domestic Hot Water Solinoid
  uint16_t domesticHWColor = DHWSolinoidOpen ? COLOR_UNLOCKED : COLOR_LOCKED;
  display.fillRoundRect(col1X, row4Y, buttonWidth, buttonHeight, 10, domesticHWColor);
  display.drawRoundRect(col1X, row4Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
  display.setTextSize(2);
  display.setTextColor(COLOR_TEXT_BLACK);
  display.setCursor(col1X + 20, row4Y + 25);
  display.println("DOMESTIC HW");
  display.setCursor(col1X + 40, row4Y + 55);
  display.println(DHWSolinoidOpen ? "OPEN" : "CLOSED");

  // Connection status
  display.setCursor(10, 462);
  display.setTextSize(1);
  display.print("IP: ");
  display.print(gigaIP);
  display.print(" | MQTT: ");
  display.print(mqttClient.connected() ? "Connected" : "Disconnected");
  display.print(" | FW: ");
  display.print(FIRMWARE_VERSION);
}

// ============================================================================
// HANDLE PLUMBING PAGE TOUCH
// ============================================================================
void handlePlumbingTouch() {
  unsigned long currentMillis = millis();

  // Debounce: ignore if touched within debounce period
  if (currentMillis - lastTouchTime < TOUCH_DEBOUNCE_MS) {
    return;
  }

  // Get touch points
  uint8_t contacts;
  GDTpoint_t points[5];
  contacts = touchDetector.getTouchPoints(points);

  if (contacts > 0) {
    // If touch is still held from previous detection, ignore
    if (touchCurrentlyPressed) {
      return;
    }

    // Wake screen if sleeping
    if (!backlightOn) {
      wakeScreen();
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Update last user interaction time
    lastUserInteraction = currentMillis;

    // Transform coordinates for landscape mode
    int rawX = points[0].x;
    int rawY = points[0].y;
    int x = rawY;
    int y = 480 - rawX;

    Serial.print("Plumbing Touch - Raw: X:");
    Serial.print(rawX);
    Serial.print(" Y:");
    Serial.print(rawY);
    Serial.print(" | Transformed: X:");
    Serial.print(x);
    Serial.print(" Y:");
    Serial.println(y);

    // Check page navigation buttons (y 10-50)
    if (y >= 10 && y <= 50) {
      if (x >= 395 && x <= 525) {
        // Locks page
        currentPage = PAGE_CABINET_LOCKS;
        uiNeedsRedraw = true;
        Serial.println("Switched to Locks page");
        lastTouchTime = currentMillis;
        touchCurrentlyPressed = true;
        return;
      } else if (x >= 535 && x <= 665) {
        // Thermostat page
        currentPage = PAGE_THERMOSTAT;
        uiNeedsRedraw = true;
        Serial.println("Switched to Thermostat page");
        lastTouchTime = currentMillis;
        touchCurrentlyPressed = true;
        return;
      } else if (x >= 675 && x <= 790) {
        // Already on plumbing page
        lastTouchTime = currentMillis;
        touchCurrentlyPressed = true;
        return;
      }
    }

    int buttonWidth = 240;
    int buttonHeight = 85;
    int col1X = 20;
    int col2X = 275;
    int col3X = 530;
    int row1Y = 70;
    int row2Y = 170;
    int row3Y = 270;
    int row4Y = 370;

    // Row 1, Col 1: Fresh Water Heater
    if (x >= col1X && x <= (col1X + buttonWidth) && y >= row1Y && y <= (row1Y + buttonHeight)) {
      sendPlumbingCommand(topic_fresh_water_heat_cmd, freshWaterHeaterOn ? "OFF" : "ON");
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Row 1, Col 2: Grey Water Heater
    if (x >= col2X && x <= (col2X + buttonWidth) && y >= row1Y && y <= (row1Y + buttonHeight)) {
      sendPlumbingCommand(topic_grey_water_heat_cmd, greyWaterHeaterOn ? "OFF" : "ON");
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Row 1, Col 3: Exhaust Fan
    if (x >= col3X && x <= (col3X + buttonWidth) && y >= row1Y && y <= (row1Y + buttonHeight)) {
      sendPlumbingCommand(topic_exhaust_fan_cmd, exhaustFanOn ? "OFF" : "ON");
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Row 2, Col 1: Rear Loop Valve
    if (x >= col1X && x <= (col1X + buttonWidth) && y >= row2Y && y <= (row2Y + buttonHeight)) {
      sendPlumbingCommand(topic_rear_loop_cmd, rearLoopValveOpen ? "CLOSE" : "OPEN");
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Row 2, Col 2: Engine Loop Valve
    if (x >= col2X && x <= (col2X + buttonWidth) && y >= row2Y && y <= (row2Y + buttonHeight)) {
      sendPlumbingCommand(topic_engine_loop_cmd, engineLoopValveOpen ? "CLOSE" : "OPEN");
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Row 2, Col 3: Diesel Heater (cycles through states)
    if (x >= col3X && x <= (col3X + buttonWidth) && y >= row2Y && y <= (row2Y + buttonHeight)) {
      String nextState;
      if (dieselHeaterState == "OFF") {
        nextState = "PUMP ONLY";
      } else if (dieselHeaterState == "PUMP ONLY") {
        nextState = "HIGH";
      } else {
        nextState = "OFF";
      }
      sendPlumbingCommand(topic_diesel_heater_cmd, nextState.c_str());
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Row 3, Col 1: Front Loop Valve
    if (x >= col1X && x <= (col1X + buttonWidth) && y >= row3Y && y <= (row3Y + buttonHeight)) {
      sendPlumbingCommand(topic_front_loop_cmd, frontLoopValveOpen ? "OFF" : "ON");
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Row 3, Col 2: Hot Water Pump
    if (x >= col2X && x <= (col2X + buttonWidth) && y >= row3Y && y <= (row3Y + buttonHeight)) {
      sendPlumbingCommand(topic_pump_cmd, pumpRunning ? "OFF" : "ON");
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Row 3, Col 3: Main Water Pump
    if (x >= col3X && x <= (col3X + buttonWidth) && y >= row3Y && y <= (row3Y + buttonHeight)) {
      sendPlumbingCommand(topic_main_pump_cmd, mainWaterPumpOn ? "OFF" : "ON");
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }

    // Row 4, Col 1: Domestic HW Solinoid
    if (x >= col1X && x <= (col1X + buttonWidth) && y >= row4Y && y <= (row4Y + buttonHeight)) {
      sendPlumbingCommand(topic_domestic_hw_cmd, DHWSolinoidOpen ? "CLOSE" : "OPEN");
      lastTouchTime = currentMillis;
      touchCurrentlyPressed = true;
      return;
    }
  } else {
    // No touch detected - reset the flag
    touchCurrentlyPressed = false;
  }
}

// ============================================================================
// SEND PLUMBING COMMAND
// ============================================================================
void sendPlumbingCommand(const char* topic, const char* command) {
  if (!mqttClient.connected()) {
    Serial.println("Cannot send plumbing command - MQTT not connected");
    return;
  }

  Serial.print("Sending plumbing command to ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(command);

  mqttClient.publish(topic, command);
}

// ============================================================================
// UPDATE HANDLE TOUCH TO INCLUDE PAGE NAVIGATION
// ============================================================================
