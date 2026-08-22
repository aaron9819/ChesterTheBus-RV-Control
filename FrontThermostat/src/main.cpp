#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// ============================================
// CONFIGURATION
// ============================================
#define CABINET_ID "KitchenPassSide"
#define FIRMWARE_VERSION "v4.1.0"

// WiFi Configuration
const char* ssid = "Chester IOT";
const char* password = "2025Chester9894";

// MQTT Configuration
const char* mqtt_server = "192.168.8.1";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";

// OTA Configuration
const char* otaHostname = "Thermostat-" CABINET_ID;
const char* otaPassword = "Chester2025";

String mqtt_client_id = String("Thermostat_") + String(CABINET_ID);

// Cabinet Lock Topics
String topic_cabinetLock_command = "CabLock" + String(CABINET_ID) + "Command";
String topic_cabinetLock_status = "CabLock" + String(CABINET_ID) + "Status";
const char* topic_allLock_command = "CabLockAllCommand";

// Thermostat Topics
const char* topic_thermostat_tempF = "ThermostatTempF";
const char* topic_thermostat_setTempF = "ThermostatSetTempF";
const char* topic_thermostat_setTempF_cmd = "ThermostatSetTempFCommand";
const char* topic_thermostat_heating = "ThermostatHeating";
const char* topic_engine_loop_command = "EngineLoopCommand";
const char* topic_fan_percent = "ThermostatFanPercent";
const char* topic_fan_command = "ThermostatFanCommand";
const char* topic_fan_speed_command = "ThermostatFanSpeedCommand";
const char* topic_fan_preset = "ThermostatFanPreset";
const char* topic_fan_preset_command = "ThermostatFanPresetCommand";
const char* topic_thermostat_mode = "ThermostatMode";
const char* topic_thermostat_mode_cmd = "ThermostatModeCommand";
const char* topic_thermostat_humidity = "ThermostatHumidity";
const char* topic_thermostat_pressure = "ThermostatPressure";

// Hardware Configuration
#define SERVO_PIN 8
#define FAN_PWM_PIN 10
#define TEMP_UP_BUTTON 21
#define TEMP_DOWN_BUTTON 17
#define BME280_SDA 33
#define BME280_SCL 35

// PWM Configuration for Fan
#define PWM_FREQ 25000
#define PWM_CHANNEL 2
#define PWM_RESOLUTION 8

// EEPROM Configuration
#define EEPROM_SIZE 64
#define EEPROM_MAGIC 0xA5B3  // Magic number to validate EEPROM data
#define EEPROM_VERSION 1

struct ThermostatState {
  uint16_t magic;       // Magic number for validation
  uint8_t version;      // Version of the state structure
  uint8_t heatMode;     // HeatMode enum value
  float setTemp;        // Set temperature in Celsius
  uint16_t checksum;    // Simple checksum for validation
};

Servo lockServo;

// BME280 Temperature Sensor
Adafruit_BME280 bme;
bool bmeConnected = false;

// OLED Display
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 48
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayConnected = false;

// Thermostat State
float currentTemp = 0.0;
float currentHumidity = 0.0;
float currentPressure = 0.0;
float setTemp = 21.0;  // Default 21°C
const float TEMP_CALIBRATION_OFFSET = -6.0;

// Heat Mode
enum HeatMode {
  HEAT_OFF,
  HEAT_AUTO
};
HeatMode heatMode = HEAT_OFF;  // Default to OFF

bool heatingActive = false;
bool engineLoopOpen = false;
bool fanRunning = false;
int fanSpeed = 0;  // 0-255 PWM value
bool manualFanOverride = false;  // True when fan is being controlled from HA
String fanPreset = "auto";  // auto | low | medium | high

// Fan preset speed percentages
const int FAN_PRESET_LOW_PERCENT = 33;
const int FAN_PRESET_MEDIUM_PERCENT = 66;
const int FAN_PRESET_HIGH_PERCENT = 100;

// Two-stage heating timing
unsigned long heatingStartTime = 0;
unsigned long engineLoopOpenTime = 0;
unsigned long engineLoopLastClosedTime = 0;
const unsigned long FAN_DELAY = 630000;  // >10 minutes delay for fan

// PI Controller State
float integralError = 0.0;
float lastControlOutput = 0.0;
unsigned long lastControlUpdate = 0;

// PI Controller Parameters
const float U_BIAS = 30.0;
const float KP = 11.0;
const float KI = 0.09;
const float DEADBAND = 0.5;
const float BOOST_THRESHOLD = 3.0;
const float BOOST_OUTPUT = 90.0;
const float RATE_LIMIT = 10.0;

// Button state tracking
struct ButtonState {
  bool currentlyPressed;
  bool wasPressed;
  unsigned long pressStartTime;
  unsigned long lastChangeTime;
  bool longPressTriggered;
};

ButtonState btnUp = {false, false, 0, 0, false};
ButtonState btnDown = {false, false, 0, 0, false};

const unsigned long BUTTON_DEBOUNCE_MS = 50;
const unsigned long LONG_PRESS_MS = 1000;
const unsigned long REPEAT_RATE_MS = 200;

// Display state
enum DisplayMode {
  DISPLAY_CURRENT_TEMP,
  DISPLAY_SET_TEMP,
  DISPLAY_MODE_CHANGE
};
DisplayMode displayMode = DISPLAY_CURRENT_TEMP;
unsigned long displayModeChangeTime = 0;
const unsigned long MODE_DISPLAY_DURATION = 2000;
const unsigned long SET_TEMP_DISPLAY_DURATION = 3000;
String displayMessage = "";

// Lock positions
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
const unsigned long STATUS_INTERVAL = 60000;

// Recovery / Watchdog
const unsigned long WIFI_RECOVERY_TIMEOUT = 120000;
const unsigned long MQTT_RECOVERY_TIMEOUT = 120000;
const unsigned long DISPLAY_INIT_RETRY_MS = 10000;

unsigned long wifiLostSince = 0;
unsigned long mqttLostSince = 0;
unsigned long lastDisplayInitAttempt = 0;
bool otaInitialized = false;

// Command deduplication
String lastCommand = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_COOLDOWN_MS = 1000;

// ============================================
// FORWARD DECLARATIONS
// ============================================
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool reconnectMQTT();
void publishHomeAssistantDiscovery();
void lockCabinet();
void unlockCabinet();
void sendStatus();
void processCommand(String command);
void readTemperature();
void updateThermostat();
float calculatePIControl(float error, float deltaTime);
void controlFan(float controlOutputPercent);
void checkButtons();
void publishThermostatStatus();
void updateDisplay();
void saveStateToEEPROM();
void loadStateFromEEPROM();
uint16_t calculateChecksum(ThermostatState* state);

// ============================================
// EEPROM STATE MANAGEMENT
// ============================================
uint16_t calculateChecksum(ThermostatState* state) {
  uint16_t sum = 0;
  uint8_t* ptr = (uint8_t*)state;
  // Calculate checksum on all bytes except the checksum field itself
  for (size_t i = 0; i < sizeof(ThermostatState) - sizeof(uint16_t); i++) {
    sum += ptr[i];
  }
  return sum;
}

void saveStateToEEPROM() {
  ThermostatState state;
  state.magic = EEPROM_MAGIC;
  state.version = EEPROM_VERSION;
  state.heatMode = (uint8_t)heatMode;
  state.setTemp = setTemp;
  state.checksum = calculateChecksum(&state);

  EEPROM.put(0, state);
  EEPROM.commit();

  Serial.println("💾 State saved to EEPROM");
  Serial.print("   Mode: ");
  Serial.println(heatMode == HEAT_OFF ? "OFF" : "AUTO");
  Serial.print("   SetTemp: ");
  Serial.print(setTemp, 1);
  Serial.println("°C");
}

void loadStateFromEEPROM() {
  ThermostatState state;
  EEPROM.get(0, state);

  // Validate magic number, version, and checksum
  if (state.magic == EEPROM_MAGIC &&
      state.version == EEPROM_VERSION &&
      state.checksum == calculateChecksum(&state)) {

    // Restore state
    heatMode = (HeatMode)state.heatMode;
    setTemp = state.setTemp;

    Serial.println("💾 State loaded from EEPROM");
    Serial.print("   Mode: ");
    Serial.println(heatMode == HEAT_OFF ? "OFF" : "AUTO");
    Serial.print("   SetTemp: ");
    Serial.print(setTemp, 1);
    Serial.println("°C");
  } else {
    Serial.println("⚠️  No valid state in EEPROM, using defaults");
    Serial.println("   Mode: OFF");
    Serial.print("   SetTemp: ");
    Serial.print(setTemp, 1);
    Serial.println("°C");
  }
}

// ============================================
// MQTT CALLBACK
// ============================================
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
  // Set temperature command (Fahrenheit)
  else if (String(topic) == topic_thermostat_setTempF_cmd) {
    float tempF = message.toFloat();
    if (tempF >= 50.0 && tempF <= 86.0) {
      setTemp = (tempF - 32.0) * 5.0 / 9.0;
      Serial.print("🌡️ Set temperature changed to: ");
      Serial.print(tempF, 1);
      Serial.println("°F");
      saveStateToEEPROM();
      publishThermostatStatus();
    }
  }
  // Thermostat mode commands
  else if (String(topic) == topic_thermostat_mode_cmd) {
    String m = message;
    m.trim();
    if (m.equalsIgnoreCase("off")) {
      heatMode = HEAT_OFF;
      Serial.println("⚙️ Mode set via MQTT: OFF");
      displayMessage = "HEAT OFF";
      displayMode = DISPLAY_MODE_CHANGE;
      displayModeChangeTime = millis();
      saveStateToEEPROM();
      publishThermostatStatus();
    } else if (m.equalsIgnoreCase("heat") || m.equalsIgnoreCase("auto")) {
      heatMode = HEAT_AUTO;
      Serial.println("⚙️ Mode set via MQTT: AUTO");
      displayMessage = "HEAT AUTO";
      displayMode = DISPLAY_MODE_CHANGE;
      displayModeChangeTime = millis();
      saveStateToEEPROM();
      publishThermostatStatus();
    }
  }
  // Fan on/off command
  else if (String(topic) == topic_fan_command) {
    String cmd = message;
    cmd.trim();
    if (cmd.equalsIgnoreCase("ON")) {
      manualFanOverride = true;
      if (fanSpeed == 0) {
        controlFan(50);  // Default to 50% when turned on with no prior speed
      }
      Serial.println("🌀 Fan ON via HA");
      publishThermostatStatus();
    } else if (cmd.equalsIgnoreCase("OFF")) {
      manualFanOverride = false;
      controlFan(0);
      Serial.println("🌀 Fan OFF via HA");
      publishThermostatStatus();
    }
  }
  // Fan speed percentage command
  else if (String(topic) == topic_fan_speed_command) {
    int percent = message.toInt();
    if (percent >= 0 && percent <= 100) {
      manualFanOverride = (percent > 0);
      controlFan((float)percent);
      Serial.print("🌀 Fan speed set via HA: ");
      Serial.print(percent);
      Serial.println("%");
      publishThermostatStatus();
    }
  }
  // Fan preset command (auto / low / medium / high)
  else if (String(topic) == topic_fan_preset_command) {
    String preset = message;
    preset.trim();
    preset.toLowerCase();
    if (preset == "auto") {
      fanPreset = "auto";
      manualFanOverride = false;
      controlFan(0);  // PI controller will resume next cycle if HEAT_AUTO
    } else if (preset == "low") {
      fanPreset = "low";
      manualFanOverride = true;
      controlFan(FAN_PRESET_LOW_PERCENT);
    } else if (preset == "medium") {
      fanPreset = "medium";
      manualFanOverride = true;
      controlFan(FAN_PRESET_MEDIUM_PERCENT);
    } else if (preset == "high") {
      fanPreset = "high";
      manualFanOverride = true;
      controlFan(FAN_PRESET_HIGH_PERCENT);
    } else {
      return;
    }
    Serial.print("🌀 Fan preset: ");
    Serial.println(fanPreset);
    publishThermostatStatus();
  }
}

// ============================================
// MQTT CONNECTION
// ============================================
bool reconnectMQTT() {
  if (mqttClient.connect(mqtt_client_id.c_str(), mqtt_user, mqtt_password)) {
    Serial.println("✓ MQTT connected");
    mqttClient.subscribe(topic_cabinetLock_command.c_str());
    mqttClient.subscribe(topic_allLock_command);
    mqttClient.subscribe(topic_thermostat_setTempF_cmd);
    mqttClient.subscribe(topic_thermostat_mode_cmd);
    mqttClient.subscribe(topic_fan_command);
    mqttClient.subscribe(topic_fan_speed_command);
    mqttClient.subscribe(topic_fan_preset_command);

    publishHomeAssistantDiscovery();
    sendStatus();
    publishThermostatStatus();
    return true;
  }
  return false;
}

// ============================================
// HOME ASSISTANT DISCOVERY
// ============================================
void publishHomeAssistantDiscovery() {
  if (!mqttClient.connected()) return;

  String deviceId = String("thermostat_") + String(CABINET_ID);

  // Device information (shared by all entities)
  String device = "\"device\":{";
  device += "\"identifiers\":[\"" + deviceId + "\"],";
  device += "\"name\":\"Front Thermostat\",";
  device += "\"model\":\"ESP32-S2 Thermostat\",";
  device += "\"manufacturer\":\"Chester RV Control\",";
  device += "\"sw_version\":\"" + String(FIRMWARE_VERSION) + "\"";
  device += "}";

  // Climate entity (thermostat)
  String climateTopic = "homeassistant/climate/" + deviceId + "/config";
  String climateConfig = "{";
  climateConfig += "\"name\":\"Front Thermostat\",";
  climateConfig += "\"unique_id\":\"" + deviceId + "_climate\",";
  climateConfig += "\"mode_command_topic\":\"" + String(topic_thermostat_mode_cmd) + "\",";
  climateConfig += "\"mode_state_topic\":\"" + String(topic_thermostat_mode) + "\",";
  climateConfig += "\"modes\":[\"off\",\"heat\"],";
  climateConfig += "\"temperature_command_topic\":\"" + String(topic_thermostat_setTempF_cmd) + "\",";
  climateConfig += "\"temperature_state_topic\":\"" + String(topic_thermostat_setTempF) + "\",";
  climateConfig += "\"current_temperature_topic\":\"" + String(topic_thermostat_tempF) + "\",";
  climateConfig += "\"temperature_unit\":\"F\",";
  climateConfig += "\"temp_step\":0.5,";
  climateConfig += "\"min_temp\":50,";
  climateConfig += "\"max_temp\":86,";
  climateConfig += "\"precision\":0.1,";
  climateConfig += device;
  climateConfig += "}";
  mqttClient.publish(climateTopic.c_str(), climateConfig.c_str(), true);

  // Humidity sensor
  String humidityTopic = "homeassistant/sensor/" + deviceId + "_humidity/config";
  String humidityConfig = "{";
  humidityConfig += "\"name\":\"Front Humidity\",";
  humidityConfig += "\"unique_id\":\"" + deviceId + "_humidity\",";
  humidityConfig += "\"state_topic\":\"" + String(topic_thermostat_humidity) + "\",";
  humidityConfig += "\"unit_of_measurement\":\"%\",";
  humidityConfig += "\"device_class\":\"humidity\",";
  humidityConfig += device;
  humidityConfig += "}";
  mqttClient.publish(humidityTopic.c_str(), humidityConfig.c_str(), true);

  // Pressure sensor
  String pressureTopic = "homeassistant/sensor/" + deviceId + "_pressure/config";
  String pressureConfig = "{";
  pressureConfig += "\"name\":\"Front Pressure\",";
  pressureConfig += "\"unique_id\":\"" + deviceId + "_pressure\",";
  pressureConfig += "\"state_topic\":\"" + String(topic_thermostat_pressure) + "\",";
  pressureConfig += "\"unit_of_measurement\":\"hPa\",";
  pressureConfig += "\"device_class\":\"pressure\",";
  pressureConfig += device;
  pressureConfig += "}";
  mqttClient.publish(pressureTopic.c_str(), pressureConfig.c_str(), true);

  // Remove old fan sensor entity if it exists
  String oldFanTopic = "homeassistant/sensor/" + deviceId + "_fan/config";
  mqttClient.publish(oldFanTopic.c_str(), "", true);

  // Fan entity with speed control
  String fanTopic = "homeassistant/fan/" + deviceId + "_fan/config";
  String fanConfig = "{";
  fanConfig += "\"name\":\"Thermostat Fan\",";
  fanConfig += "\"unique_id\":\"" + deviceId + "_fan\",";
  fanConfig += "\"state_topic\":\"" + String(topic_fan_percent) + "\",";
  fanConfig += "\"state_value_template\":\"{{ 'ON' if value | int > 0 else 'OFF' }}\",";
  fanConfig += "\"command_topic\":\"" + String(topic_fan_command) + "\",";
  fanConfig += "\"percentage_state_topic\":\"" + String(topic_fan_percent) + "\",";
  fanConfig += "\"percentage_command_topic\":\"" + String(topic_fan_speed_command) + "\",";
  fanConfig += "\"speed_range_min\":1,";
  fanConfig += "\"speed_range_max\":100,";
  fanConfig += "\"preset_modes\":[\"auto\",\"low\",\"medium\",\"high\"],";
  fanConfig += "\"preset_mode_command_topic\":\"" + String(topic_fan_preset_command) + "\",";
  fanConfig += "\"preset_mode_state_topic\":\"" + String(topic_fan_preset) + "\",";
  fanConfig += device;
  fanConfig += "}";
  mqttClient.publish(fanTopic.c_str(), fanConfig.c_str(), true);

  // Binary sensor for heating status
  String heatingTopic = "homeassistant/binary_sensor/" + deviceId + "_heating/config";
  String heatingConfig = "{";
  heatingConfig += "\"name\":\"Thermostat Heating\",";
  heatingConfig += "\"unique_id\":\"" + deviceId + "_heating\",";
  heatingConfig += "\"state_topic\":\"" + String(topic_thermostat_heating) + "\",";
  heatingConfig += "\"payload_on\":\"ON\",";
  heatingConfig += "\"payload_off\":\"OFF\",";
  heatingConfig += "\"device_class\":\"heat\",";
  heatingConfig += device;
  heatingConfig += "}";
  mqttClient.publish(heatingTopic.c_str(), heatingConfig.c_str(), true);

  // Lock entity
  String lockTopic = "homeassistant/lock/" + deviceId + "_lock/config";
  String lockConfig = "{";
  lockConfig += "\"name\":\"Cabinet Lock\",";
  lockConfig += "\"unique_id\":\"" + deviceId + "_lock\",";
  lockConfig += "\"command_topic\":\"" + String(topic_cabinetLock_command) + "\",";
  lockConfig += "\"state_topic\":\"" + String(topic_cabinetLock_status) + "\",";
  lockConfig += "\"payload_lock\":\"LOCK\",";
  lockConfig += "\"payload_unlock\":\"UNLOCK\",";
  lockConfig += "\"state_locked\":\"LOCKED\",";
  lockConfig += "\"state_unlocked\":\"UNLOCKED\",";
  lockConfig += device;
  lockConfig += "}";
  mqttClient.publish(lockTopic.c_str(), lockConfig.c_str(), true);

  Serial.println("✓ Home Assistant discovery published");
}

// ============================================
// CABINET LOCK FUNCTIONS
// ============================================
void lockCabinet() {
  lockServo.attach(SERVO_PIN);
  lockServo.write(LOCKED_POSITION);
  delay(1000);
  lockServo.detach();
  currentState = STATE_LOCKED;
  Serial.println("🔒 Cabinet LOCKED");
  sendStatus();
}

void unlockCabinet() {
  lockServo.attach(SERVO_PIN);
  lockServo.write(UNLOCKED_POSITION);
  delay(1000);
  lockServo.detach();
  currentState = STATE_UNLOCKED;
  Serial.println("🔓 Cabinet UNLOCKED");
  sendStatus();
}

void sendStatus() {
  if (mqttClient.connected()) {
    const char* lockStatus;
    if (currentState == STATE_LOCKED) {
      lockStatus = "LOCKED";
    } else if (currentState == STATE_UNLOCKED) {
      lockStatus = "UNLOCKED";
    } else {
      lockStatus = "UNKNOWN";
    }
    mqttClient.publish(topic_cabinetLock_status.c_str(), lockStatus, true);
    Serial.print("Lock status: ");
    Serial.println(lockStatus);
  }
}

void processCommand(String command) {
  unsigned long currentTime = millis();

  if (command == lastCommand && (currentTime - lastCommandTime) < COMMAND_COOLDOWN_MS) {
    Serial.println("⚠️ Duplicate command ignored");
    return;
  }

  lastCommand = command;
  lastCommandTime = currentTime;

  if (command == "LOCK") {
    lockCabinet();
  } else if (command == "UNLOCK") {
    unlockCabinet();
  }
}

// ============================================
// TEMPERATURE SENSOR
// ============================================
void readTemperature() {
  if (bmeConnected) {
    float tempReading = bme.readTemperature();
    float humReading = bme.readHumidity();
    float presReading = bme.readPressure() / 100.0F;

    // Guard against NaN from BME280 communication errors
    if (isnan(tempReading) || isnan(humReading) || isnan(presReading)) {
      Serial.println("⚠️ BME280 returned NaN - skipping reading");
      return;
    }

    currentTemp = tempReading + TEMP_CALIBRATION_OFFSET;
    currentHumidity = humReading;
    currentPressure = presReading;

    if (mqttClient.connected()) {
      float currentTempF = (currentTemp * 9.0 / 5.0) + 32.0;
      char tempStrF[8];
      dtostrf(currentTempF, 5, 1, tempStrF);
      mqttClient.publish(topic_thermostat_tempF, tempStrF, true);
      mqttClient.publish(topic_thermostat_humidity, String(currentHumidity, 1).c_str(), true);
      mqttClient.publish(topic_thermostat_pressure, String(currentPressure, 1).c_str(), true);
    }
  }
}

// ============================================
// THERMOSTAT CONTROL
// ============================================
void updateThermostat() {
  if (!bmeConnected) return;

  unsigned long currentTime = millis();

  if (heatMode == HEAT_OFF) {
    // OFF mode - close engine loop, stop auto heating
    // But allow manual fan control from HA
    if (heatingActive || engineLoopOpen || (fanRunning && !manualFanOverride)) {
      heatingActive = false;
      integralError = 0;
      lastControlOutput = 0;

      if (!manualFanOverride) {
        controlFan(0);
      }

      // Close engine loop when switching to OFF mode
      if (engineLoopOpen) {
        if (mqttClient.connected()) {
          mqttClient.publish(topic_engine_loop_command, "CLOSE", false);
        }
        engineLoopOpen = false;
        engineLoopLastClosedTime = currentTime;
        Serial.println("⚡ Engine loop CLOSED (thermostat OFF)");
      }

      Serial.println("🚫 Thermostat OFF - heating disabled");
    }
    return;
  }

  if (heatMode == HEAT_AUTO) {
    // AUTO mode - PI controller drives fan unless a manual preset is overriding
    float deltaTime = (lastControlUpdate > 0) ? (currentTime - lastControlUpdate) / 1000.0 : 0.1;
    lastControlUpdate = currentTime;

    float error = setTemp - currentTemp;  // Positive = need heat

    if (error > 0.5) {
      // Need heat
      if (!heatingActive) {
        heatingActive = true;
        heatingStartTime = currentTime;
        integralError = 0;

        if (!engineLoopOpen) {
          if (mqttClient.connected()) {
            mqttClient.publish(topic_engine_loop_command, "OPEN", false);
          }
          engineLoopOpen = true;

          // Handle FAN_DELAY timing
          unsigned long timeSinceClosed = 0;
          if (engineLoopLastClosedTime != 0 && currentTime > engineLoopLastClosedTime) {
            timeSinceClosed = currentTime - engineLoopLastClosedTime;
          }
          unsigned long effectiveElapsed = (timeSinceClosed > FAN_DELAY) ? FAN_DELAY : timeSinceClosed;
          engineLoopOpenTime = currentTime - effectiveElapsed;
          Serial.println("⚡ Engine loop OPEN");
          Serial.println("⏱️  Fan will start after delay...");
        }
        publishThermostatStatus();
      }

      // Fan control with delay (skip writes if manual preset is active)
      unsigned long engineLoopDuration = currentTime - engineLoopOpenTime;
      if (engineLoopDuration >= FAN_DELAY) {
        float controlOutput = calculatePIControl(error, deltaTime);
        if (!manualFanOverride) controlFan(controlOutput);
      } else {
        if (fanRunning && !manualFanOverride) {
          controlFan(0);
          Serial.println("⏱️  Fan delayed - engine loop warming up");
        }
      }
    } else if (error <= 0 && heatingActive) {
      // Target reached - turn off heating
      heatingActive = false;
      Serial.println("❄️ Heating off - target reached");

      integralError = 0;
      lastControlOutput = 0;

      if (mqttClient.connected()) {
        mqttClient.publish(topic_engine_loop_command, "CLOSE", false);
      }
      engineLoopOpen = false;
      engineLoopLastClosedTime = currentTime;
      Serial.println("⚡ Engine loop CLOSED");

      if (!manualFanOverride) controlFan(0);
      publishThermostatStatus();
    }
  }
}

// ============================================
// PI CONTROLLER
// ============================================
float calculatePIControl(float error, float deltaTime) {
  // Boost mode for large errors
  if (error > BOOST_THRESHOLD) {
    Serial.print("🚀 BOOST MODE: error ");
    Serial.print(error, 1);
    Serial.println("°C");
    integralError = 0;
    lastControlOutput = BOOST_OUTPUT;
    return BOOST_OUTPUT;
  }

  // Apply deadband
  float activeError = error;
  if (abs(error) < DEADBAND) {
    activeError = 0;
  }

  // Proportional term
  float proportional = KP * activeError;

  // Integral term with anti-windup
  bool atUpperLimit = (lastControlOutput >= 100.0 && activeError > 0);
  bool atLowerLimit = (lastControlOutput <= 0.0 && activeError < 0);

  if (!atUpperLimit && !atLowerLimit) {
    integralError += activeError * deltaTime;
    float maxIntegral = 100.0 / KI;
    integralError = constrain(integralError, -maxIntegral, maxIntegral);
  }

  float integral = KI * integralError;

  // Calculate output
  float output = U_BIAS + proportional + integral;
  output = constrain(output, 0.0, 100.0);

  // Rate limiting
  float maxChange = RATE_LIMIT * deltaTime;
  float change = output - lastControlOutput;
  if (abs(change) > maxChange) {
    output = lastControlOutput + (change > 0 ? maxChange : -maxChange);
  }

  lastControlOutput = output;
  return output;
}

// ============================================
// FAN CONTROL
// ============================================
void controlFan(float controlOutputPercent) {
  int pwmValue = (int)((controlOutputPercent / 100.0) * 255.0);
  pwmValue = constrain(pwmValue, 0, 255);

  if (pwmValue != fanSpeed) {
    fanSpeed = pwmValue;
    ledcWrite(FAN_PWM_PIN, fanSpeed);
    fanRunning = (fanSpeed > 0);

    Serial.print("🌀 Fan speed: ");
    Serial.print((int)controlOutputPercent);
    Serial.println("%");
  }
}

// ============================================
// BUTTON HANDLING
// ============================================
void checkButtons() {
  unsigned long currentTime = millis();
  bool upPressed = (digitalRead(TEMP_UP_BUTTON) == LOW);
  bool downPressed = (digitalRead(TEMP_DOWN_BUTTON) == LOW);

  // Process UP button
  if (upPressed != btnUp.currentlyPressed) {
    if (currentTime - btnUp.lastChangeTime >= BUTTON_DEBOUNCE_MS) {
      btnUp.currentlyPressed = upPressed;
      btnUp.lastChangeTime = currentTime;

      if (upPressed) {
        btnUp.pressStartTime = currentTime;
        btnUp.longPressTriggered = false;
        Serial.println("⬆️ UP button pressed");
      } else {
        unsigned long pressDuration = currentTime - btnUp.pressStartTime;

        if (!btnUp.longPressTriggered && pressDuration < LONG_PRESS_MS) {
          // Short press - increase temperature
          setTemp += 0.5;
          if (setTemp > 30.0) setTemp = 30.0;
          Serial.print("🔼 Set temp: ");
          Serial.print(setTemp);
          Serial.println("°C");
          displayMode = DISPLAY_SET_TEMP;
          displayModeChangeTime = currentTime;
          updateDisplay();
          saveStateToEEPROM();
          publishThermostatStatus();
        }
        Serial.println("⬆️ UP button released");
      }
    }
  }

  // Check for long press on UP button
  if (btnUp.currentlyPressed && !btnUp.longPressTriggered) {
    if (currentTime - btnUp.pressStartTime >= LONG_PRESS_MS) {
      btnUp.longPressTriggered = true;
      if (heatMode != HEAT_AUTO) {
        heatMode = HEAT_AUTO;
        Serial.println("🔥 Mode: AUTO");
        displayMessage = "HEAT AUTO";
        displayMode = DISPLAY_MODE_CHANGE;
        displayModeChangeTime = currentTime;
        updateDisplay();
        saveStateToEEPROM();
        publishThermostatStatus();
      }
    }
  }

  // Process DOWN button
  if (downPressed != btnDown.currentlyPressed) {
    if (currentTime - btnDown.lastChangeTime >= BUTTON_DEBOUNCE_MS) {
      btnDown.currentlyPressed = downPressed;
      btnDown.lastChangeTime = currentTime;

      if (downPressed) {
        btnDown.pressStartTime = currentTime;
        btnDown.longPressTriggered = false;
        Serial.println("⬇️ DOWN button pressed");
      } else {
        unsigned long pressDuration = currentTime - btnDown.pressStartTime;

        if (!btnDown.longPressTriggered && pressDuration < LONG_PRESS_MS) {
          // Short press - decrease temperature
          setTemp -= 0.5;
          if (setTemp < 10.0) setTemp = 10.0;
          Serial.print("🔽 Set temp: ");
          Serial.print(setTemp);
          Serial.println("°C");
          displayMode = DISPLAY_SET_TEMP;
          displayModeChangeTime = currentTime;
          updateDisplay();
          saveStateToEEPROM();
          publishThermostatStatus();
        }
        Serial.println("⬇️ DOWN button released");
      }
    }
  }

  // Check for long press on DOWN button
  if (btnDown.currentlyPressed && !btnDown.longPressTriggered) {
    if (currentTime - btnDown.pressStartTime >= LONG_PRESS_MS) {
      btnDown.longPressTriggered = true;
      if (heatMode != HEAT_OFF) {
        heatMode = HEAT_OFF;
        Serial.println("🚫 Mode: OFF");
        displayMessage = "HEAT OFF";
        displayMode = DISPLAY_MODE_CHANGE;
        displayModeChangeTime = currentTime;
        updateDisplay();
        saveStateToEEPROM();
        publishThermostatStatus();
      }
    }
  }
}

void publishThermostatStatus() {
  if (mqttClient.connected()) {
    // Set temperature in Fahrenheit
    float setTempF = (setTemp * 9.0 / 5.0) + 32.0;
    char setTempStrF[8];
    dtostrf(setTempF, 5, 1, setTempStrF);
    mqttClient.publish(topic_thermostat_setTempF, setTempStrF, true);

    // Heating status
    mqttClient.publish(topic_thermostat_heating, heatingActive ? "ON" : "OFF", true);

    // Mode
    const char* modeStr = (heatMode == HEAT_OFF) ? "off" : "heat";
    mqttClient.publish(topic_thermostat_mode, modeStr, true);

    // Fan percent
    int percent = (fanSpeed * 100) / 255;
    char pctStr[8];
    itoa(percent, pctStr, 10);
    mqttClient.publish(topic_fan_percent, pctStr, true);

    // Fan preset
    mqttClient.publish(topic_fan_preset, fanPreset.c_str(), true);
  }
}

// ============================================
// DISPLAY UPDATE
// ============================================
void updateDisplay() {
  if (!displayConnected) return;

  unsigned long currentTime = millis();

  // Return to current temp display after timeout
  if (displayMode == DISPLAY_MODE_CHANGE && (currentTime - displayModeChangeTime > MODE_DISPLAY_DURATION)) {
    displayMode = DISPLAY_CURRENT_TEMP;
  }
  if (displayMode == DISPLAY_SET_TEMP && (currentTime - displayModeChangeTime > SET_TEMP_DISPLAY_DURATION)) {
    displayMode = DISPLAY_CURRENT_TEMP;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  int yOffset = 20;  // Display offset

  if (displayMode == DISPLAY_MODE_CHANGE) {
    display.setTextSize(1);
    display.setCursor(4, 16 + yOffset);
    display.print(displayMessage);
  } else if (displayMode == DISPLAY_SET_TEMP) {
    display.setTextSize(2);
    display.setCursor(8, 8 + yOffset);
    float setTempF = (setTemp * 9.0 / 5.0) + 32.0;
    display.print((int)setTempF);
    display.print("F");
  } else {
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

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n================================");
  Serial.println("RV Thermostat " + String(FIRMWARE_VERSION));
  Serial.println("Cabinet: " + String(CABINET_ID));
  Serial.println("================================\n");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("✓ EEPROM initialized");

  // Load saved state
  loadStateFromEEPROM();

  // Initialize I2C
  Wire.begin(BME280_SDA, BME280_SCL);
  Serial.println("I2C initialized");

  // Initialize BME280
  if (bme.begin(0x76)) {
    bmeConnected = true;
    Serial.println("✓ BME280 sensor connected");
  } else {
    Serial.println("⚠️ BME280 sensor not found");
  }

  // Initialize display
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    displayConnected = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println("Thermostat");
    display.println("Starting...");
    display.display();
    Serial.println("✓ OLED display connected");
  } else {
    Serial.println("⚠️ OLED display not found");
  }

  // Initialize servo
  lockServo.attach(SERVO_PIN);
  lockServo.write(UNLOCKED_POSITION);
  delay(1000);
  lockServo.detach();
  Serial.println("✓ Servo initialized");

  // Initialize fan PWM
  ledcAttach(FAN_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(FAN_PWM_PIN, 0);
  Serial.println("✓ Fan PWM initialized");

  // Initialize buttons
  pinMode(TEMP_UP_BUTTON, INPUT_PULLUP);
  pinMode(TEMP_DOWN_BUTTON, INPUT_PULLUP);
  Serial.println("✓ Buttons initialized");

  // Connect to WiFi
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 40) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n⚠️ WiFi connection failed");
  }

  // Initialize MQTT
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);

  // Initialize OTA
  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Update Starting...");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Complete!");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.begin();
    otaInitialized = true;
    Serial.println("✓ OTA initialized");
  }

  Serial.println("\n✓ Setup complete\n");

  if (displayConnected) {
    updateDisplay();
  }
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // WiFi reconnection
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiLostSince == 0) wifiLostSince = millis();

    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 6) {
      delay(250);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✓ WiFi reconnected");
      wifiLostSince = 0;
      if (!otaInitialized) {
        ArduinoOTA.begin();
        otaInitialized = true;
      }
      if (displayConnected) {
        displayMode = DISPLAY_CURRENT_TEMP;
        updateDisplay();
      }
    } else if (millis() - wifiLostSince > WIFI_RECOVERY_TIMEOUT) {
      Serial.println("\n⚠️ WiFi timeout - restarting...");
      ESP.restart();
    }
  } else {
    wifiLostSince = 0;
  }

  // MQTT connection
  if (!mqttClient.connected()) {
    if (mqttLostSince == 0) mqttLostSince = millis();

    if (millis() - mqttLostSince > 5000) {
      Serial.println("MQTT reconnecting...");
      if (reconnectMQTT()) {
        mqttLostSince = 0;
      } else if (millis() - mqttLostSince > MQTT_RECOVERY_TIMEOUT) {
        Serial.println("⚠️ MQTT timeout - restarting...");
        ESP.restart();
      }
    }
  } else {
    mqttLostSince = 0;
  }

  mqttClient.loop();

  if (otaInitialized) {
    ArduinoOTA.handle();
  }

  // Display retry
  if (!displayConnected && (millis() - lastDisplayInitAttempt > DISPLAY_INIT_RETRY_MS)) {
    lastDisplayInitAttempt = millis();
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      displayConnected = true;
      Serial.println("✓ Display reconnected");
      updateDisplay();
    }
  }

  // Read temperature every 5 seconds
  static unsigned long lastTempRead = 0;
  if (millis() - lastTempRead > 5000) {
    lastTempRead = millis();
    readTemperature();
    updateDisplay();
  }

  // Update thermostat control
  static unsigned long lastThermostatUpdate = 0;
  if (millis() - lastThermostatUpdate > 1000) {
    lastThermostatUpdate = millis();
    updateThermostat();
  }

  // Check buttons
  checkButtons();

  // Periodic status update
  if (millis() - lastStatusUpdate > STATUS_INTERVAL) {
    lastStatusUpdate = millis();
    sendStatus();
    publishThermostatStatus();
  }

  delay(50);
}
