#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================
// CONFIGURATION - Change for each ESP32-S2 Mini
// ============================================
// Set unique cabinet identifier for each board
#define CABINET_ID "KitchenPassSide"  // Options: kitchen1, kitchen2, bathroom, bedroom, etc.
#define FIRMWARE_VERSION "v2.0.0"

// WiFi Configuration
const char* ssid = "Chester IOT";
const char* password = "2025Chester9894";

// MQTT Configuration - Connect to Mosquitto broker on GL-AR300M16 router
const char* mqtt_server = "192.168.8.1";  // Router IP address
const int mqtt_port = 1883;
const char* mqtt_user = "";  // No username
const char* mqtt_password = "";  // No password

// OTA Configuration
const char* otaHostname = "CabLock-KitchenPassSide";
const char* otaPassword = "Chester2025";

// MQTT Client ID - Unique per device
String mqtt_client_id = String("CabLock_") + String(CABINET_ID);

// MQTT Topics - Generated based on CABINET_ID
String topic_cabinetLock_command = "CabLock" + String(CABINET_ID) + "Command";
String topic_cabinetLock_status = "CabLock" + String(CABINET_ID) + "Status";
const char* topic_allLock_command = "CabLockAllCommand";  // Group control

// Thermostat Topics
String topic_thermostat_temp = "ThermostatTemp";           // Current temp in Celsius
String topic_thermostat_tempF = "ThermostatTempF";         // Current temp in Fahrenheit
String topic_thermostat_setTemp = "ThermostatSetTemp";     // Set temp in Celsius
String topic_thermostat_setTempF = "ThermostatSetTempF";   // Set temp in Fahrenheit
String topic_thermostat_setTempF_cmd = "ThermostatSetTempFCommand";  // Command to set temp in Fahrenheit
String topic_thermostat_heating = "ThermostatHeating";
String topic_engine_loop_status = "EngineLoopStatus";
String topic_fan_speed = "FanSpeed";

// Loop Control Topics (MQTT commands to plumbing board)
const char* topic_front_loop_command = "FrontLoopCommand";
const char* topic_engine_loop_command = "EngineLoopCommand";

// Diesel Heater Topics
const char* topic_diesel_heater_command = "DieselHeaterCommand";
const char* topic_diesel_heater_status = "DieselHeaterStatus";

// Hardware Configuration - ESP32-S2 Mini GPIO pins
#define SERVO_PIN 8          // for servo control
// #define FRONT_LOOP_PIN 1     // -> Hardware trigger to plumbing board GPIO14 (DISABLED - using MQTT)
// #define ENGINE_LOOP_PIN 2    // -> Hardware trigger to plumbing board GPIO13 (DISABLED - using MQTT)
#define FAN_PWM_PIN 10       // for PWM fan control
#define TEMP_UP_BUTTON 21    // for temperature up button
#define TEMP_DOWN_BUTTON 17  // for temperature down button
#define BME280_SDA 33        // for I2C SDA
#define BME280_SCL 35        // for I2C SCL

// PWM Configuration for Fan
#define PWM_FREQ 25000       // 25kHz PWM frequency (standard for fans)
#define PWM_CHANNEL 2        // PWM channel 2 (servo uses 0 and 1)
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)

Servo lockServo;

// BME280 Temperature Sensor
Adafruit_BME280 bme;
bool bmeConnected = false;

// OLED Display - 64x48 pixels
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 48
#define OLED_RESET -1  // Reset pin not used (shared I2C reset)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayConnected = false;

// Front Loop State
bool frontLoopValveOpen = false;

// Diesel Heater State
String dieselHeaterStatus = "UNKNOWN";  // Tracks current heater status from plumbing board

// Thermostat State
float currentTemp = 0.0;
float currentHumidity = 0.0;
float setTemp = 21.0;  // Default 21°C
const float TEMP_CALIBRATION_OFFSET = -6.0;  // Temperature offset in °C (adjust as needed)

// Heat Mode: OFF, AUTO
enum HeatMode {
  HEAT_OFF,    // Completely off, no heating
  HEAT_AUTO    // Automatic temperature control
};
HeatMode heatMode = HEAT_AUTO;  // Default to AUTO

bool heatingActive = false;
bool engineLoopOpen = false;
bool fanRunning = false;
int fanSpeed = 0;  // 0-255 PWM value

// Two-stage heating logic
unsigned long heatingStartTime = 0;        // When heating first started
unsigned long engineLoopOpenTime = 0;      // When engine loop was opened
const unsigned long STAGE2_DELAY = 1200000;  // 20 minutes in milliseconds
const unsigned long FAN_DELAY = 630000;      // >10 minutes delay for fan after engine loop opens
const float STAGE2_ERROR_THRESHOLD = 1.0;  // °C error to trigger stage 2 immediately

// PI Controller State
float integralError = 0.0;      // Accumulated error for integral term
float lastControlOutput = 0.0;  // Previous control output (0-100%)
unsigned long lastControlUpdate = 0;  // Last time PI controller ran

// PI Controller Parameters (tuned for Celsius)
const float U_BIAS = 30.0;      // Fan floor: 30% baseline
const float KP = 11.0;          // Proportional gain: 11% per °C
const float KI = 0.09;          // Integral gain: 0.09% per (°C·s)
const float DEADBAND = 0.5;     // Ignore ±0.5°C error
const float BOOST_THRESHOLD = 3.0;  // If error > 3°C, go to boost mode
const float BOOST_OUTPUT = 90.0;    // Boost mode output: 90%
const float RATE_LIMIT = 10.0;  // Max change: 10%/second

// Timing for valve control
unsigned long valveOpenTime = 0;

// Button state tracking with debouncing
struct ButtonState {
  bool currentlyPressed;
  bool wasPressed;
  unsigned long pressStartTime;
  unsigned long lastChangeTime;
  bool longPressTriggered;
};

ButtonState btnUp = {false, false, 0, 0, false};
ButtonState btnDown = {false, false, 0, 0, false};

const unsigned long BUTTON_DEBOUNCE_MS = 50;   // Debounce time
const unsigned long LONG_PRESS_MS = 1000;       // 1 second for long press
const unsigned long REPEAT_RATE_MS = 200;       // Repeat rate for held buttons

// Display state management
enum DisplayMode {
  DISPLAY_CURRENT_TEMP,
  DISPLAY_SET_TEMP,
  DISPLAY_MODE_CHANGE
};
DisplayMode displayMode = DISPLAY_CURRENT_TEMP;
unsigned long displayModeChangeTime = 0;
const unsigned long MODE_DISPLAY_DURATION = 2000;  // Show mode for 2 seconds
const unsigned long SET_TEMP_DISPLAY_DURATION = 3000;  // Show set temp for 3 seconds
String displayMessage = "";

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
void readTemperature();
void updateThermostat();
void controlEngineLoop(bool open);
float calculatePIControl(float error, float deltaTime);
void controlFan(float controlOutputPercent);
void checkButtons();
void publishThermostatStatus();
void updateDisplay();

// MQTT Callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.printf("MQTT message received on topic: %s - Message: ", topic);
  Serial.println(message);

  message.trim();

  // Handle cabinet lock commands
  if (String(topic) == topic_cabinetLock_command || String(topic) == topic_allLock_command) {
    processCommand(message);
  }
  // Handle temperature set command in Fahrenheit
  else if (String(topic) == topic_thermostat_setTempF_cmd) {
    float tempF = message.toFloat();
    if (tempF >= 50.0 && tempF <= 86.0) {  // Valid range 50-86°F (10-30°C)
      setTemp = (tempF - 32.0) * 5.0 / 9.0;  // Convert F to C
      Serial.print("🌡️ Set temperature changed via MQTT to: ");
      Serial.print(tempF, 1);
      Serial.print("°F (");
      Serial.print(setTemp, 1);
      Serial.println("°C)");
      publishThermostatStatus();
    } else {
      Serial.print("⚠️ Invalid temperature: ");
      Serial.print(tempF, 1);
      Serial.println("°F (must be 50-86°F)");
    }
  }
  // Handle diesel heater status updates
  else if (String(topic) == topic_diesel_heater_status) {
    dieselHeaterStatus = message;
    Serial.print("🔥 Diesel heater status: ");
    Serial.println(dieselHeaterStatus);
  }
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

      // Subscribe to temperature set command (Fahrenheit)
      mqttClient.subscribe(topic_thermostat_setTempF_cmd.c_str());
      Serial.print("Subscribed to: ");
      Serial.println(topic_thermostat_setTempF_cmd);

      // Subscribe to diesel heater status
      mqttClient.subscribe(topic_diesel_heater_status);
      Serial.print("Subscribed to: ");
      Serial.println(topic_diesel_heater_status);

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

  Serial.println("\n\n=== ChesterTheBus ESP32-S2 Cabinet Lock Controller ===");
  Serial.print("Cabinet ID: ");
  Serial.println(CABINET_ID);
  Serial.print("Command Topic: ");
  Serial.println(topic_cabinetLock_command);
  Serial.print("Status Topic: ");
  Serial.println(topic_cabinetLock_status);

  // Initialize servo - ESP32Servo library handles PWM channel allocation
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  lockServo.setPeriodHertz(50);  // Standard 50Hz servo
  lockServo.attach(SERVO_PIN, 500, 2400);  // Min/max pulse width in microseconds
  lockServo.write(UNLOCKED_POSITION);
  currentState = STATE_UNLOCKED;
  delay(500);  // Give servo time to move
  lockServo.detach();
  Serial.println("Servo initialized at UNLOCKED position");

  // Initialize hardware trigger pins (outputs to plumbing board)
  // DISABLED - Now using MQTT commands instead of hardware triggers
  // pinMode(FRONT_LOOP_PIN, OUTPUT);
  // digitalWrite(FRONT_LOOP_PIN, HIGH);  // Start HIGH (inactive)
  frontLoopValveOpen = false;
  Serial.println("Front loop control: MQTT mode (hardware triggers disabled)");

  // pinMode(ENGINE_LOOP_PIN, OUTPUT);
  // digitalWrite(ENGINE_LOOP_PIN, HIGH);  // Start HIGH (inactive)
  engineLoopOpen = false;
  Serial.println("Engine loop control: MQTT mode (hardware triggers disabled)");

  // Initialize button pins with internal pullup
  pinMode(TEMP_UP_BUTTON, INPUT_PULLUP);
  pinMode(TEMP_DOWN_BUTTON, INPUT_PULLUP);
  Serial.println("Temperature control buttons initialized");

  // Initialize BME280 sensor
  Wire.begin(BME280_SDA, BME280_SCL);
  if (bme.begin(0x76, &Wire)) {  // Try address 0x76 first, then 0x77
    bmeConnected = true;
    Serial.println("BME280 sensor initialized at 0x76");
  } else if (bme.begin(0x77, &Wire)) {
    bmeConnected = true;
    Serial.println("BME280 sensor initialized at 0x77");
  } else {
    bmeConnected = false;
    Serial.println("WARNING: BME280 sensor not found!");
  }

  // Initialize PWM fan control
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN_PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);  // Start at 0% (off)
  fanSpeed = 0;
  Serial.print("Fan PWM initialized on GPIO");
  Serial.print(FAN_PWM_PIN);
  Serial.print(" at ");
  Serial.print(PWM_FREQ);
  Serial.println(" Hz (OFF)");

  // Initialize OLED display (D1 Mini Shield - address 0x3C or 0x3D)
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    displayConnected = true;
    Serial.println("OLED display initialized at 0x3C");

    // Use default library settings - no custom commands

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Chester");
    display.println("Thermo");
    display.println("Starting");
    display.display();
  } else if (display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    displayConnected = true;
    Serial.println("OLED display initialized at 0x3D");

    // Use default library settings - no custom commands

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Chester");
    display.println("Thermo");
    display.println("Starting");
    display.display();
  } else {
    displayConnected = false;
    Serial.println("WARNING: OLED display not found!");
  }

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
    lockServo.setPeriodHertz(50);
    lockServo.attach(SERVO_PIN, 500, 2400);
  }

  lockServo.write(LOCKED_POSITION);
  currentState = STATE_LOCKED;
  delay(500);  // Give servo time to move

  // Disable servo to save power and reduce noise
  //lockServo.detach();
  //Serial.println("Servo disabled after locking");

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
    lockServo.setPeriodHertz(50);
    lockServo.attach(SERVO_PIN, 500, 2400);
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
    // Cabinet lock status
    const char* lockStatus;
    if (currentState == STATE_LOCKED) {
      lockStatus = "LOCKED";
    } else if (currentState == STATE_UNLOCKED) {
      lockStatus = "UNLOCKED";
    } else {
      lockStatus = "UNKNOWN";
    }
    mqttClient.publish(topic_cabinetLock_status.c_str(), lockStatus, true);
    Serial.print("Lock status published for ");
    Serial.print(CABINET_ID);
    Serial.print(": ");
    Serial.println(lockStatus);
  }
}

// ============================================================================
// TEMPERATURE SENSOR FUNCTIONS
// ============================================================================
void readTemperature() {
  if (bmeConnected) {
    currentTemp = bme.readTemperature() + TEMP_CALIBRATION_OFFSET;
    currentHumidity = bme.readHumidity();

    // Publish temperature to MQTT (both Celsius and Fahrenheit)
    if (mqttClient.connected()) {
      // Celsius
      char tempStr[8];
      dtostrf(currentTemp, 5, 1, tempStr);
      mqttClient.publish(topic_thermostat_temp.c_str(), tempStr, false);

      // Fahrenheit
      float currentTempF = (currentTemp * 9.0 / 5.0) + 32.0;
      char tempStrF[8];
      dtostrf(currentTempF, 5, 1, tempStrF);
      mqttClient.publish(topic_thermostat_tempF.c_str(), tempStrF, false);
    }
  }
}

// ============================================================================
// THERMOSTAT CONTROL FUNCTIONS
// ============================================================================
void updateThermostat() {
  if (!bmeConnected) return;
  if (heatMode == HEAT_OFF) {
    // Heat is OFF - ensure everything is disabled
    if (heatingActive || engineLoopOpen || frontLoopValveOpen || fanRunning) {
      heatingActive = false;
      engineLoopOpen = false;
      frontLoopValveOpen = false;
      // digitalWrite(ENGINE_LOOP_PIN, HIGH);  // Hardware trigger disabled
      // digitalWrite(FRONT_LOOP_PIN, HIGH);   // Hardware trigger disabled
      if (mqttClient.connected()) {
        mqttClient.publish(topic_engine_loop_command, "CLOSE", false);
        mqttClient.publish(topic_front_loop_command, "CLOSE", false);
      }
      controlFan(0);
      integralError = 0;
      lastControlOutput = 0;
      Serial.println("🚫 Heat mode OFF - loops and fan disabled (diesel heater unchanged)");
    }
    return;
  }

  unsigned long currentTime = millis();
  float deltaTime = (lastControlUpdate > 0) ? (currentTime - lastControlUpdate) / 1000.0 : 0.1;
  lastControlUpdate = currentTime;

  // Calculate error: setpoint - current (positive = need heat)
  float error = setTemp - currentTemp;  // TWO-STAGE HEATING LOGIC
  // Stage 1: Front loop only (always on when heat needed)
  // Stage 2: Engine loop + Fan (only if error >1.5°C OR heating for >20min)

  if (error > 0.5) {  // Need heat (0.5°C threshold)
    if (!heatingActive) {
      heatingActive = true;
      heatingStartTime = currentTime;
      Serial.println("🔥 STAGE 1: Heating activated (front loop only)");
      integralError = 0;  // Reset integral when starting
    }

    // STAGE 1: Always keep front loop active when heating
    if (!frontLoopValveOpen) {
      // digitalWrite(FRONT_LOOP_PIN, LOW);  // Hardware trigger disabled
      if (mqttClient.connected()) {
        mqttClient.publish(topic_front_loop_command, "OPEN", true);  // Retained
      }
      frontLoopValveOpen = true;
      Serial.println("⚡ Front loop: OPEN (via MQTT)");
      valveOpenTime = millis();
    }

    // Ensure diesel heater is set to HIGH when heating
    if (mqttClient.connected() && dieselHeaterStatus != "HIGH") {
      mqttClient.publish(topic_diesel_heater_command, "HIGH", false);
      Serial.println("🔥 Commanding diesel heater to HIGH");
    }

    // Determine if Stage 2 should be active
    unsigned long heatingDuration = currentTime - heatingStartTime;
    bool needsStage2 = (error > STAGE2_ERROR_THRESHOLD) || (heatingDuration > STAGE2_DELAY);

    if (needsStage2) {
      // STAGE 2: Activate engine loop and fans
      if (!engineLoopOpen) {
        Serial.print("🔥🔥 STAGE 2: Engine loop activated - ");
        if (error > STAGE2_ERROR_THRESHOLD) {
          Serial.print("Error ");
          Serial.print(error, 1);
          Serial.println("°C > threshold");
        } else {
          Serial.print("Heating duration ");
          Serial.print(heatingDuration / 60000);
          Serial.println(" min > 20 min");
        }
        // digitalWrite(ENGINE_LOOP_PIN, LOW);  // Hardware trigger disabled
        if (mqttClient.connected()) {
          mqttClient.publish(topic_engine_loop_command, "OPEN", true);  // Retained
        }
        engineLoopOpen = true;
        engineLoopOpenTime = currentTime;  // Record when engine loop opened
        Serial.println("⚡ Engine loop: OPEN (via MQTT)");
        Serial.println("⏱️  Fan will start in 5 minutes...");
      }

      // Only run fan if engine loop has been open for at least 5 minutes
      unsigned long engineLoopDuration = currentTime - engineLoopOpenTime;
      if (engineLoopDuration >= FAN_DELAY) {
        // Calculate PI control and set fan speed
        float controlOutput = calculatePIControl(error, deltaTime);
        controlFan(controlOutput);
      } else {
        // Fan waiting period - keep it off
        if (fanRunning) {
          controlFan(0);
          Serial.println("⏱️  Fan delayed - engine loop warming up");
        }
      }
    } else {
      // STAGE 1 only: Keep engine loop off, minimal/no fan
      if (engineLoopOpen) {
        Serial.println("⬇️ Dropping to STAGE 1: Engine loop deactivated");
        // digitalWrite(ENGINE_LOOP_PIN, HIGH);  // Hardware trigger disabled
        if (mqttClient.connected()) {
          mqttClient.publish(topic_engine_loop_command, "CLOSE", true);  // Retained
        }
        engineLoopOpen = false;
        engineLoopOpenTime = 0;  // Reset timer
        controlFan(0);
        integralError = 0;  // Reset integral when dropping stage
      }
    }

  } else if (error < -0.5) {  // Too hot
    if (heatingActive) {
      heatingActive = false;
      Serial.println("❄️ Heating deactivated - too hot");

      // Reset PI controller state
      integralError = 0;
      lastControlOutput = 0;

      // Deactivate ALL triggers
      // digitalWrite(ENGINE_LOOP_PIN, HIGH);  // Hardware trigger disabled
      // digitalWrite(FRONT_LOOP_PIN, HIGH);   // Hardware trigger disabled
      if (mqttClient.connected()) {
        mqttClient.publish(topic_engine_loop_command, "CLOSE", true);  // Retained
        mqttClient.publish(topic_front_loop_command, "CLOSE", true);   // Retained
      }
      engineLoopOpen = false;
      frontLoopValveOpen = false;
      Serial.println("⚡ All loops closed (diesel heater unchanged)");

      // Turn off fan
      controlFan(0);
    }
  } else {
    // Within deadband (error between -0.5 and +0.5)
    // If heating was active, keep it active (hysteresis)
    // Only deactivate if we're clearly too hot (error < -0.5)
    if (heatingActive) {
      // Maintain front loop while in deadband with heating active
      if (!frontLoopValveOpen) {
        // digitalWrite(FRONT_LOOP_PIN, LOW);  // Hardware trigger disabled
        if (mqttClient.connected()) {
          mqttClient.publish(topic_front_loop_command, "OPEN", true);  // Retained
        }
        frontLoopValveOpen = true;
      }
    } else {
      // Not heating - ensure everything is off
      if (frontLoopValveOpen || engineLoopOpen || fanRunning) {
        // digitalWrite(ENGINE_LOOP_PIN, HIGH);  // Hardware trigger disabled
        // digitalWrite(FRONT_LOOP_PIN, HIGH);   // Hardware trigger disabled
        if (mqttClient.connected()) {
          mqttClient.publish(topic_engine_loop_command, "CLOSE", true);  // Retained
          mqttClient.publish(topic_front_loop_command, "CLOSE", true);   // Retained
        }
        engineLoopOpen = false;
        frontLoopValveOpen = false;
        controlFan(0);
      }
    }
  }
}

// This function is no longer used - triggers are controlled directly in updateThermostat()
// Kept for compatibility but does nothing
void controlEngineLoop(bool open) {
  // Hardware triggers are now controlled directly in thermostat logic
  // This function is deprecated
}

// PI Controller for fan speed with anti-windup, deadband, rate limiting, and boost
float calculatePIControl(float error, float deltaTime) {
  // Boost mode: if error is large, override to high speed
  if (error > BOOST_THRESHOLD) {
    Serial.print("🚀 BOOST MODE: error ");
    Serial.print(error, 1);
    Serial.println("°C > threshold");
    integralError = 0;  // Reset integral during boost
    lastControlOutput = BOOST_OUTPUT;
    return BOOST_OUTPUT;
  }

  // Apply deadband to error
  float activeError = error;
  if (abs(error) < DEADBAND) {
    activeError = 0;
    // Don't integrate in deadband
  }

  // Calculate proportional term
  float proportional = KP * activeError;

  // Calculate integral term with anti-windup
  // Only integrate if we're not saturated
  bool atUpperLimit = (lastControlOutput >= 100.0 && activeError > 0);
  bool atLowerLimit = (lastControlOutput <= 0.0 && activeError < 0);

  if (!atUpperLimit && !atLowerLimit) {
    integralError += activeError * deltaTime;
    // Limit integral windup (prevent excessive accumulation)
    float maxIntegral = 100.0 / KI;  // Max value that could produce 100% output
    integralError = constrain(integralError, -maxIntegral, maxIntegral);
  }

  float integral = KI * integralError;

  // Calculate raw control output
  float output = U_BIAS + proportional + integral;

  // Apply rate limiting (smooth changes)
  float maxChange = RATE_LIMIT * deltaTime;
  float change = output - lastControlOutput;
  if (abs(change) > maxChange) {
    output = lastControlOutput + (change > 0 ? maxChange : -maxChange);
  }

  // Clamp to 0-100%
  output = constrain(output, 0.0, 100.0);

  // Debug output
  Serial.print("PI: e=");
  Serial.print(error, 2);
  Serial.print("°C, P=");
  Serial.print(proportional, 1);
  Serial.print("%, I=");
  Serial.print(integral, 1);
  Serial.print("%, u=");
  Serial.print(output, 1);
  Serial.println("%");

  lastControlOutput = output;
  return output;
}

void controlFan(float controlOutputPercent) {
  // Convert 0-100% to 0-255 PWM value
  int newFanSpeed = map((int)(controlOutputPercent * 10), 0, 1000, 0, 255);
  fanSpeed = constrain(newFanSpeed, 0, 255);

  // Write PWM duty cycle directly (0 = off, 255 = full speed)
  ledcWrite(PWM_CHANNEL, fanSpeed);

  bool wasRunning = fanRunning;
  fanRunning = (fanSpeed > 0);

  if (fanRunning != wasRunning) {
    Serial.print("🌀 Fan: ");
    Serial.println(fanRunning ? "ON" : "OFF");
  }

  if (fanRunning) {
    Serial.print("   Speed: ");
    Serial.print((fanSpeed * 100) / 255);
    Serial.print("% (PWM: ");
    Serial.print(fanSpeed);
    Serial.println("/255)");
  }

  if (mqttClient.connected()) {
    char speedStr[8];
    itoa(fanSpeed, speedStr, 10);
    mqttClient.publish(topic_fan_speed.c_str(), speedStr, false);
  }
}

void checkButtons() {
  unsigned long currentTime = millis();

  // Read current button states
  bool upPressed = (digitalRead(TEMP_UP_BUTTON) == LOW);
  bool downPressed = (digitalRead(TEMP_DOWN_BUTTON) == LOW);

  // ========== PROCESS UP BUTTON ==========
  if (upPressed != btnUp.currentlyPressed) {
    // Button state changed - check debounce
    if (currentTime - btnUp.lastChangeTime >= BUTTON_DEBOUNCE_MS) {
      btnUp.currentlyPressed = upPressed;
      btnUp.lastChangeTime = currentTime;

      if (upPressed) {
        // Button just pressed
        btnUp.pressStartTime = currentTime;
        btnUp.longPressTriggered = false;
        Serial.println("⬆️ UP button pressed");
      } else {
        // Button released
        unsigned long pressDuration = currentTime - btnUp.pressStartTime;

        if (!btnUp.longPressTriggered && pressDuration < LONG_PRESS_MS) {
          // Short press - increase temperature
          setTemp += 0.5;
          if (setTemp > 30.0) setTemp = 30.0;
          Serial.print("🔼 Set temp increased to: ");
          Serial.print(setTemp);
          Serial.println("°C");
          displayMode = DISPLAY_SET_TEMP;
          displayModeChangeTime = currentTime;
          updateDisplay();
          publishThermostatStatus();
        }
        Serial.println("⬆️ UP button released");
      }
    }
  }

  // Check for long press while button is held
  if (btnUp.currentlyPressed && !btnUp.longPressTriggered) {
    if (currentTime - btnUp.pressStartTime >= LONG_PRESS_MS) {
      btnUp.longPressTriggered = true;
      if (heatMode != HEAT_AUTO) {
        heatMode = HEAT_AUTO;
        Serial.println("🔥 LONG PRESS UP: Heat mode = AUTO");
        displayMessage = "HEAT AUTO";
        displayMode = DISPLAY_MODE_CHANGE;
        displayModeChangeTime = currentTime;
        updateDisplay();
        publishThermostatStatus();
      }
    }
  }

  // ========== PROCESS DOWN BUTTON ==========
  if (downPressed != btnDown.currentlyPressed) {
    // Button state changed - check debounce
    if (currentTime - btnDown.lastChangeTime >= BUTTON_DEBOUNCE_MS) {
      btnDown.currentlyPressed = downPressed;
      btnDown.lastChangeTime = currentTime;

      if (downPressed) {
        // Button just pressed
        btnDown.pressStartTime = currentTime;
        btnDown.longPressTriggered = false;
        Serial.println("⬇️ DOWN button pressed");
      } else {
        // Button released
        unsigned long pressDuration = currentTime - btnDown.pressStartTime;

        if (!btnDown.longPressTriggered && pressDuration < LONG_PRESS_MS) {
          // Short press - decrease temperature
          setTemp -= 0.5;
          if (setTemp < 10.0) setTemp = 10.0;
          Serial.print("🔽 Set temp decreased to: ");
          Serial.print(setTemp);
          Serial.println("°C");
          displayMode = DISPLAY_SET_TEMP;
          displayModeChangeTime = currentTime;
          updateDisplay();
          publishThermostatStatus();
        }
        Serial.println("⬇️ DOWN button released");
      }
    }
  }

  // Check for long press while button is held
  if (btnDown.currentlyPressed && !btnDown.longPressTriggered) {
    if (currentTime - btnDown.pressStartTime >= LONG_PRESS_MS) {
      btnDown.longPressTriggered = true;
      if (heatMode != HEAT_OFF) {
        heatMode = HEAT_OFF;
        Serial.println("🚫 LONG PRESS DOWN: Heat mode = OFF");
        displayMessage = "HEAT OFF";
        displayMode = DISPLAY_MODE_CHANGE;
        displayModeChangeTime = currentTime;
        updateDisplay();
        publishThermostatStatus();
      }
    }
  }
}

void publishThermostatStatus() {
  if (mqttClient.connected()) {
    // Publish set temperature in Celsius
    char setTempStr[8];
    dtostrf(setTemp, 5, 1, setTempStr);
    mqttClient.publish(topic_thermostat_setTemp.c_str(), setTempStr, true);

    // Publish set temperature in Fahrenheit
    float setTempF = (setTemp * 9.0 / 5.0) + 32.0;
    char setTempStrF[8];
    dtostrf(setTempF, 5, 1, setTempStrF);
    mqttClient.publish(topic_thermostat_setTempF.c_str(), setTempStrF, true);

    // Publish heating status
    mqttClient.publish(topic_thermostat_heating.c_str(), heatingActive ? "ON" : "OFF", false);

    // Publish heat mode
    const char* modeStr = (heatMode == HEAT_OFF) ? "OFF" : "AUTO";
    mqttClient.publish("ThermostatMode", modeStr, true);

    // Publish current loop states (retained for plumbing board sync)
    mqttClient.publish(topic_front_loop_command, frontLoopValveOpen ? "OPEN" : "CLOSE", true);
    mqttClient.publish(topic_engine_loop_command, engineLoopOpen ? "OPEN" : "CLOSE", true);
  }
}

// ============================================================================
// DISPLAY UPDATE FUNCTION
// ============================================================================
void updateDisplay() {
  if (!displayConnected) return;

  unsigned long currentTime = millis();

  // Check if we should return to showing current temp
  if (displayMode == DISPLAY_MODE_CHANGE && (currentTime - displayModeChangeTime > MODE_DISPLAY_DURATION)) {
    displayMode = DISPLAY_CURRENT_TEMP;
  }
  if (displayMode == DISPLAY_SET_TEMP && (currentTime - displayModeChangeTime > SET_TEMP_DISPLAY_DURATION)) {
    displayMode = DISPLAY_CURRENT_TEMP;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Shift everything down by 20 pixels to account for display offset
  int yOffset = 20;

  if (displayMode == DISPLAY_MODE_CHANGE) {
    // Show mode change message
    display.setTextSize(1);
    display.setCursor(4, 16 + yOffset);
    display.print(displayMessage);
  } else if (displayMode == DISPLAY_SET_TEMP) {
    // Show set temperature
    display.setTextSize(2);
    display.setCursor(8, 8 + yOffset);
    float setTempF = (setTemp * 9.0 / 5.0) + 32.0;
    display.print((int)setTempF);
    display.print("F");
  } else {
    // Show current temperature
    display.setTextSize(2);
    display.setCursor(8, 8 + yOffset);
    if (bmeConnected) {
      float currentTempF = (currentTemp * 9.0 / 5.0) + 32.0;
      display.print((int)currentTempF);
      display.print("F");
    } else {
      display.setTextSize(1);
      display.setCursor(10, 16 + yOffset);
      display.print("NO SENSOR");
    }
  }

  display.display();
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

  // Check temperature control buttons
  checkButtons();

  // Read temperature sensor (every loop is fine, BME280 is slow)
  static unsigned long lastTempRead = 0;
  if (millis() - lastTempRead >= 5000) {  // Read every 5 seconds
    readTemperature();
    lastTempRead = millis();
  }

  // Update thermostat logic (every loop)
  updateThermostat();

  // Update OLED display
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 1000) {  // Update display every 1 second
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  // Periodically send status update (for monitoring/debugging)
  if (millis() - lastStatusUpdate > STATUS_INTERVAL) {
    sendStatus();
    publishThermostatStatus();
    lastStatusUpdate = millis();
  }

  // Republish loop states every 30 seconds for resilience
  static unsigned long lastLoopStatePublish = 0;
  if (millis() - lastLoopStatePublish >= 30000) {
    if (mqttClient.connected()) {
      mqttClient.publish(topic_front_loop_command, frontLoopValveOpen ? "OPEN" : "CLOSE", true);
      mqttClient.publish(topic_engine_loop_command, engineLoopOpen ? "OPEN" : "CLOSE", true);
      Serial.println("🔄 Loop states republished (periodic sync)");
    }
    lastLoopStatePublish = millis();
  }

  delay(10);
}
