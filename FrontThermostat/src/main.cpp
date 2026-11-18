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
#include <stddef.h>
#include <math.h>
#include "secrets.h"   // include/secrets.h (gitignored) - see include/secrets.example.h

// ============================================
// CONFIGURATION
// ============================================
#define CABINET_ID "KitchenPassSide"
#define FIRMWARE_VERSION "v5.2.0"

// WiFi Configuration
const char* ssid = SECRET_WIFI_SSID;
const char* password = SECRET_WIFI_PASSWORD;

// MQTT Configuration
const char* mqtt_server = "192.168.8.1";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";

// OTA Configuration
const char* otaHostname = "Thermostat-" CABINET_ID;
const char* otaPassword = SECRET_OTA_PASSWORD;

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
const char* topic_thermostat_action = "ThermostatAction";
const char* topic_engine_loop_command = "EngineLoopCommand";
const char* topic_engine_loop_status = "EngineLoopStatus";
const char* topic_engine_loop_valve_state = "EngineLoopValveState";  // Plumbing's valve-travel model
const char* topic_plumbing_status = "PlumbingSystemStatus";          // Plumbing LWT: OFFLINE when gone
const char* topic_fan_percent = "ThermostatFanPercent";
const char* topic_fan_command = "ThermostatFanCommand";
const char* topic_fan_speed_command = "ThermostatFanSpeedCommand";
const char* topic_fan_preset = "ThermostatFanPreset";
const char* topic_fan_preset_command = "ThermostatFanPresetCommand";
const char* topic_thermostat_mode = "ThermostatMode";
const char* topic_thermostat_mode_cmd = "ThermostatModeCommand";
const char* topic_thermostat_humidity = "ThermostatHumidity";
const char* topic_thermostat_pressure = "ThermostatPressure";
const char* topic_sensor_fault = "ThermostatSensorFault";
const char* topic_valve_fault = "ThermostatValveFault";
const char* topic_availability = "ThermostatAvailability";  // LWT: online / offline

// Hardware Configuration
#define SERVO_PIN 8
#define FAN_PWM_PIN 10
#define TEMP_UP_BUTTON 21
#define TEMP_DOWN_BUTTON 17
#define BME280_SDA 33
#define BME280_SCL 35
#define BME280_ADDR 0x76
#define OLED_ADDR 0x3C

// PWM / LEDC allocation.
// LEDC channels share timers in pairs (timer = (channel / 2) % 4). The servo
// library is pinned to timer 0 (channels 0-1) with ESP32PWM::allocateTimer(0),
// and the fan gets its own fixed channel on timer 2, so the two can never
// collide on a channel or reconfigure each other's timer.
#define PWM_FREQ 25000
#define PWM_RESOLUTION 8
#define FAN_LEDC_CHANNEL 4
#define SERVO_LEDC_TIMER 0

// Lock positions
#define LOCKED_POSITION 90
#define UNLOCKED_POSITION 5
const unsigned long SERVO_MOVE_MS = 1000;

// ============================================
// STATE TYPES
// ============================================
enum HeatMode { HEAT_OFF = 0, HEAT_AUTO = 1 };
enum LockState { STATE_UNKNOWN = 0, STATE_UNLOCKED = 1, STATE_LOCKED = 2 };

// EEPROM Configuration
#define EEPROM_SIZE 64
#define EEPROM_MAGIC 0xA5B3
#define EEPROM_VERSION 2   // v2: setpoint stored in F, lock state added, checksum fixed

// Explicit layout (no hidden padding before the checksum).
struct ThermostatState {
  uint16_t magic;
  uint8_t version;
  uint8_t heatMode;
  uint8_t lockState;
  uint8_t reserved[3];
  float setTempF;
  uint16_t checksum;   // covers every byte before this field
  uint16_t pad;
};
static_assert(sizeof(ThermostatState) <= EEPROM_SIZE, "ThermostatState does not fit in EEPROM");

// ============================================
// THERMOSTAT SETTINGS
// ============================================
const float SETPOINT_MIN_F = 50.0;
const float SETPOINT_MAX_F = 86.0;
const float DEFAULT_SETPOINT_F = 70.0;
const float TEMP_CALIBRATION_OFFSET = -6.0;  // C, compensates enclosure self-heating

// Fan call (PI control). Independent of the engine loop below.
const float HEAT_ON_ERROR = 0.5;   // C below setpoint -> call for heat (fan)
const float HEAT_OFF_ERROR = 0.0;  // C below setpoint -> stop the fan (0.5 C hysteresis)

// Engine loop. While heat is on, the loop is held OPEN whenever the room is at or
// below the setpoint, so the coil stays hot and the fan never waits out a cold-coil
// warm-up. It only closes once the room climbs this far ABOVE the setpoint, and
// re-opens as soon as the room is back at the setpoint. The margin keeps sensor noise
// at the setpoint from cycling the valve.
const float LOOP_CLOSE_ABOVE_F = 0.5;   // F above setpoint -> close the engine loop

// Fan warm-up: don't blow air until hot coolant has reached the coil.
const unsigned long FAN_DELAY = 630000;  // 10.5 min of flow for a cold coil

// Engine-loop valve feedback (from Plumbing's EngineLoopStatus)
const unsigned long VALVE_CONFIRM_MS = 20000;

// PI Controller Parameters (C units, output in %)
const float U_BIAS = 30.0;
const float KP = 11.0;
const float KI = 0.09;
const float DEADBAND = 0.5;
const float BOOST_ENTER = 3.0;
const float BOOST_EXIT = 2.5;
const float BOOST_OUTPUT = 90.0;
const float RATE_LIMIT = 10.0;          // %/s for the automatic fan
const float INTEGRAL_BLEED_S = 120.0;   // integral decay time constant inside the deadband

// Fan preset speed percentages
const int FAN_PRESET_LOW_PERCENT = 33;
const int FAN_PRESET_MEDIUM_PERCENT = 66;
const int FAN_PRESET_HIGH_PERCENT = 100;
const int FAN_ON_DEFAULT_PERCENT = 50;

// Sensor
const unsigned long SENSOR_READ_MS = 5000;
const unsigned long SENSOR_STALE_MS = 60000;
const unsigned long SENSOR_RETRY_MS = 10000;

// Buttons
const unsigned long BUTTON_DEBOUNCE_MS = 30;
const unsigned long REPEAT_DELAY_MS = 600;
const unsigned long REPEAT_RATE_MS = 250;
const unsigned long REPEAT_FAST_AFTER_MS = 2500;
const unsigned long REPEAT_FAST_MS = 100;
const unsigned long MODE_HOLD_MS = 1500;   // hold BOTH buttons to toggle heat on/off

// Display: 0.66" 64x48 SSD1306 (LOLIN OLED shield style)
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 48
#define OLED_RESET -1
// This glass is wired for the SSD1306's *alternative* COM-pin layout (0x12, as U8g2
// and LOLIN's own library configure it). Adafruit_SSD1306 has no 64x48 case and sends
// the sequential layout (0x02), which interleaves the picture: buffer rows 16-23 never
// appear, rows 24-47 are stretched over every other line of the full height, and rows
// 0-15 land in between. beginDisplay() sends the right value, after which all 48 rows
// map 1:1 and the layout below can start at row 0.
const uint8_t OLED_COM_PINS = 0x12;
const int DISP_BODY_Y = 10;  // the big value is centred in rows 10-47, below the status line
const unsigned long SET_VIEW_MS = 4000;
const unsigned long MESSAGE_VIEW_MS = 2000;
const unsigned long DISPLAY_DIM_AFTER_MS = 30000;
const unsigned long DISPLAY_OFF_AFTER_MS = 600000;
const unsigned long DISPLAY_PROBE_MS = 2000;
const unsigned long DISPLAY_RENDER_MS = 100;

// Networking
const unsigned long WIFI_KICK_MS = 30000;
const unsigned long WIFI_RESTART_MS = 900000;   // 15 min with no WiFi -> reboot
const unsigned long MQTT_RETRY_MS = 5000;
const unsigned long MQTT_RESTART_MS = 900000;   // 15 min WiFi up but no broker -> reboot
const unsigned long STATUS_INTERVAL = 60000;
const unsigned long STATUS_MIN_INTERVAL_MS = 1000;
const unsigned long STATE_SAVE_DELAY_MS = 2000;
const unsigned long COMMAND_COOLDOWN_MS = 1000;

// ============================================
// GLOBALS
// ============================================
Servo lockServo;
Adafruit_BME280 bme;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Persistent settings
HeatMode heatMode = HEAT_OFF;
float setTempF = DEFAULT_SETPOINT_F;
LockState currentState = STATE_UNKNOWN;
bool stateDirty = false;
unsigned long stateDirtySince = 0;

// Sensor state
bool bmePresent = false;
bool sensorOk = false;          // a plausible reading newer than SENSOR_STALE_MS
bool sensorEverValid = false;
unsigned long lastValidReading = 0;
unsigned long lastSensorRetry = 0;
unsigned long lastTempRead = 0;
float currentTemp = 0.0;        // C, calibrated
float currentHumidity = 0.0;
float currentPressure = 0.0;
char lastPubTemp[16] = "";
char lastPubHum[16] = "";
char lastPubPres[16] = "";

// Heating state
bool heatingActive = false;     // calling for heat

// Engine-loop request (held state, delivered until acknowledged by the broker)
bool loopWanted = false;
bool loopDelivered = false;     // false at boot -> CLOSE is asserted after the first connect
unsigned long loopDeliveredAt = 0;
bool plumbingStatusKnown = false;
bool plumbingLoopOpen = false;
bool valveFault = false;        // valve closed by someone else while we asked for heat

// Plumbing's model of the valve's travel (EngineLoopValveState). Plumbing owns the
// valve and keeps counting through a thermostat reboot, so whenever it is online and
// publishing this, the coil counts as hot exactly when it reports OPEN.
bool valveTravelKnown = false;
bool valveFullyOpen = false;
bool plumbingOnline = false;

// Local coil warm-up model - fallback for when Plumbing is offline
bool coilFlowing = false;       // hot coolant believed to be flowing through the coil
unsigned long coilFlowStart = 0;
bool coilEverStopped = false;
unsigned long coilStoppedAt = 0;
unsigned long coilWarmthAtStop = 0;   // ms of warm-up credit at the moment flow stopped

// PI state
float integralError = 0.0;
bool boostActive = false;
bool controlInitialized = false;
unsigned long lastControlUpdate = 0;
unsigned long lastThermostatUpdate = 0;

// Fan state
enum FanControl { FAN_CTRL_AUTO, FAN_CTRL_MANUAL, FAN_CTRL_HOLD_OFF };
FanControl fanControl = FAN_CTRL_AUTO;
int manualFanPercent = 0;
const char* manualPresetName = nullptr;   // "low"/"medium"/"high", or nullptr for a custom speed
int fanPercent = 0;                       // what the fan is actually driven at, 0-100
float autoFanOutput = 0.0;                // rate-limited automatic output
float autoFanTarget = 0.0;

// Buttons
struct Button {
  uint8_t pin;
  bool raw;
  bool pressed;
  unsigned long rawChangedAt;
  unsigned long pressedAt;
  unsigned long nextRepeatAt;
  bool holdActive;
};
Button btnUp = {TEMP_UP_BUTTON, false, false, 0, 0, 0, false};
Button btnDown = {TEMP_DOWN_BUTTON, false, false, 0, 0, 0, false};
bool comboActive = false;
bool comboFired = false;
unsigned long comboSince = 0;

// Display
enum View { VIEW_HOME, VIEW_SETPOINT, VIEW_MESSAGE };
View view = VIEW_HOME;
unsigned long viewStart = 0;
unsigned long viewDuration = 0;
char msgTop[16] = "";
char msgBig[12] = "";
bool displayConnected = false;
bool displayDimmed = false;
bool displayOff = false;
bool displayForce = true;
unsigned long lastInteraction = 0;
unsigned long lastDisplayProbe = 0;
unsigned long lastRender = 0;
String lastFrameSig = "";

// Servo
bool servoMoving = false;
unsigned long servoMoveStart = 0;

// Networking
bool wifiDown = false;
unsigned long wifiDownSince = 0;
unsigned long lastWifiKick = 0;
bool mqttDown = true;
unsigned long mqttDownSince = 0;
unsigned long lastMqttAttempt = 0;
bool otaInitialized = false;
bool otaInProgress = false;
bool statusDirty = true;
unsigned long lastStatusPublish = 0;
unsigned long lastHeartbeat = 0;

// Command deduplication
String lastCommand = "";
unsigned long lastCommandTime = 0;

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
void publishSensorReadings(bool force);
void updateThermostat();
float calculatePIControl(float error, float deltaTime);
void applyFan(float deltaTime);
void setFanPercent(int percent, bool forceWrite);
void checkButtons(unsigned long now);
void publishThermostatStatus();
void requestStatusPublish();
void renderDisplay(bool force);
void saveStateNow();
void markStateDirty();
void loadStateFromEEPROM();
void setLoopWanted(bool want);
void trySendLoopRequest();
bool coilReady(unsigned long now);
void setHeatMode(HeatMode mode, bool showFeedback);
void setSetpointF(float f, bool showOnDisplay);
void showMessage(const char* top, const char* big, unsigned long ms);
void noteInteraction();

// ============================================
// HELPERS
// ============================================
float cToF(float c) { return c * 9.0f / 5.0f + 32.0f; }
float fToC(float f) { return (f - 32.0f) * 5.0f / 9.0f; }

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)addr, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

// Saturation vapour pressure (Magnus), hPa
float saturationVapourPressure(float tC) {
  return 6.112f * expf(17.62f * tC / (243.12f + tC));
}

// ============================================
// EEPROM STATE MANAGEMENT
// ============================================
uint16_t calculateChecksum(const ThermostatState& state) {
  // Fletcher-16 over every byte before the checksum field
  const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&state);
  uint16_t a = 0, b = 0;
  for (size_t i = 0; i < offsetof(ThermostatState, checksum); i++) {
    a = (a + ptr[i]) % 255;
    b = (b + a) % 255;
  }
  return (uint16_t)((b << 8) | a);
}

void saveStateNow() {
  ThermostatState state = {};   // zeroed: no indeterminate bytes
  state.magic = EEPROM_MAGIC;
  state.version = EEPROM_VERSION;
  state.heatMode = (uint8_t)heatMode;
  state.lockState = (uint8_t)currentState;
  state.setTempF = setTempF;
  state.checksum = calculateChecksum(state);

  EEPROM.put(0, state);
  bool ok = EEPROM.commit();
  stateDirty = false;

  Serial.printf("State saved%s: mode=%s set=%.1fF lock=%d\n", ok ? "" : " (COMMIT FAILED)",
                heatMode == HEAT_OFF ? "OFF" : "AUTO", setTempF, (int)currentState);
}

// Setpoint changes from buttons can arrive several per second; batch them.
void markStateDirty() {
  if (!stateDirty) stateDirtySince = millis();
  stateDirty = true;
}

void flushStateIfDirty(unsigned long now) {
  if (stateDirty && now - stateDirtySince >= STATE_SAVE_DELAY_MS) saveStateNow();
}

void loadStateFromEEPROM() {
  ThermostatState state = {};
  EEPROM.get(0, state);

  bool valid = state.magic == EEPROM_MAGIC &&
               state.version == EEPROM_VERSION &&
               state.checksum == calculateChecksum(state) &&
               state.heatMode <= HEAT_AUTO &&
               state.lockState <= STATE_LOCKED &&
               !isnan(state.setTempF) &&
               state.setTempF >= SETPOINT_MIN_F && state.setTempF <= SETPOINT_MAX_F;

  if (valid) {
    heatMode = (HeatMode)state.heatMode;
    setTempF = state.setTempF;
    currentState = (LockState)state.lockState;
    Serial.printf("State loaded: mode=%s set=%.1fF lock=%d\n",
                  heatMode == HEAT_OFF ? "OFF" : "AUTO", setTempF, (int)currentState);
  } else {
    Serial.printf("No valid state in EEPROM, using defaults: mode=OFF set=%.1fF lock=unlocked\n", setTempF);
  }
}

// ============================================
// MQTT CALLBACK
// ============================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Copy topic and payload first: PubSubClient reuses its buffer when we publish.
  String t(topic);
  String message;
  message.reserve(length);
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  message.trim();

  Serial.printf("MQTT %s: %s\n", t.c_str(), message.c_str());

  if (t == topic_cabinetLock_command || t == topic_allLock_command) {
    processCommand(message);
  }
  else if (t == topic_engine_loop_status) {
    String m = message;
    m.toUpperCase();
    plumbingStatusKnown = true;
    plumbingLoopOpen = (m == "OPEN" || m == "ON");
  }
  else if (t == topic_engine_loop_valve_state || t == topic_plumbing_status) {
    bool wasReady = coilReady(millis());
    String m = message;
    m.toUpperCase();
    if (t == topic_plumbing_status) {
      plumbingOnline = (m.length() > 0 && m != "OFFLINE");
    } else {
      // An empty (cleared) or unknown payload drops back to the local model
      valveTravelKnown = (m == "OPEN" || m == "OPENING" || m == "CLOSING" || m == "CLOSED");
      valveFullyOpen = (m == "OPEN");
    }
    bool ready = coilReady(millis());
    if (ready != wasReady) {
      Serial.printf("Coil %s (%s)\n", ready ? "hot" : "not hot",
                    (valveTravelKnown && plumbingOnline) ? "Plumbing valve model" : "local model");
      requestStatusPublish();
      displayForce = true;
    }
  }
  else if (t == topic_thermostat_setTempF_cmd) {
    const char* s = message.c_str();
    char* end = nullptr;
    float tempF = strtof(s, &end);
    if (end != s && !isnan(tempF) && tempF >= SETPOINT_MIN_F && tempF <= SETPOINT_MAX_F) {
      setSetpointF(tempF, true);
    } else {
      Serial.printf("Rejected setpoint '%s' (must be %.0f-%.0f F)\n", s, SETPOINT_MIN_F, SETPOINT_MAX_F);
      requestStatusPublish();   // re-sync HA to the real setpoint
    }
  }
  else if (t == topic_thermostat_mode_cmd) {
    if (message.equalsIgnoreCase("off")) {
      setHeatMode(HEAT_OFF, true);
    } else if (message.equalsIgnoreCase("heat") || message.equalsIgnoreCase("auto")) {
      setHeatMode(HEAT_AUTO, true);
    } else {
      requestStatusPublish();
    }
  }
  else if (t == topic_fan_command) {
    if (message.equalsIgnoreCase("ON")) {
      manualFanPercent = (fanPercent > 0) ? fanPercent : FAN_ON_DEFAULT_PERCENT;
      manualPresetName = nullptr;
      fanControl = FAN_CTRL_MANUAL;
      Serial.printf("Fan ON via HA (%d%%)\n", manualFanPercent);
    } else if (message.equalsIgnoreCase("OFF")) {
      // While heating, OFF holds the fan off for the rest of this heat cycle.
      // Otherwise it simply returns the fan to automatic control (which is off).
      fanControl = heatingActive ? FAN_CTRL_HOLD_OFF : FAN_CTRL_AUTO;
      autoFanOutput = 0;
      Serial.println(heatingActive ? "Fan OFF via HA (held off for this heat cycle)" : "Fan OFF via HA");
    }
    applyFan(0);
    requestStatusPublish();
  }
  else if (t == topic_fan_speed_command) {
    int percent = message.toInt();
    if (percent >= 0 && percent <= 100) {
      if (percent == 0) {
        fanControl = heatingActive ? FAN_CTRL_HOLD_OFF : FAN_CTRL_AUTO;
        autoFanOutput = 0;
      } else {
        manualFanPercent = percent;
        manualPresetName = nullptr;
        fanControl = FAN_CTRL_MANUAL;
      }
      Serial.printf("Fan speed via HA: %d%%\n", percent);
      applyFan(0);
    }
    requestStatusPublish();
  }
  else if (t == topic_fan_preset_command) {
    String preset = message;
    preset.toLowerCase();
    if (preset == "auto") {
      fanControl = FAN_CTRL_AUTO;
      autoFanOutput = fanPercent;   // ramp from where the fan is now
    } else if (preset == "low") {
      fanControl = FAN_CTRL_MANUAL; manualPresetName = "low"; manualFanPercent = FAN_PRESET_LOW_PERCENT;
    } else if (preset == "medium") {
      fanControl = FAN_CTRL_MANUAL; manualPresetName = "medium"; manualFanPercent = FAN_PRESET_MEDIUM_PERCENT;
    } else if (preset == "high") {
      fanControl = FAN_CTRL_MANUAL; manualPresetName = "high"; manualFanPercent = FAN_PRESET_HIGH_PERCENT;
    } else {
      requestStatusPublish();
      return;
    }
    Serial.printf("Fan preset: %s\n", preset.c_str());
    applyFan(0);
    requestStatusPublish();
  }
}

// ============================================
// MQTT CONNECTION
// ============================================
bool reconnectMQTT() {
  Serial.println("MQTT connecting...");
  if (!mqttClient.connect(mqtt_client_id.c_str(), mqtt_user, mqtt_password,
                          topic_availability, 0, true, "offline")) {
    Serial.printf("MQTT connect failed, state=%d\n", mqttClient.state());
    return false;
  }
  Serial.println("MQTT connected");
  mqttClient.publish(topic_availability, "online", true);

  mqttClient.subscribe(topic_cabinetLock_command.c_str());
  mqttClient.subscribe(topic_allLock_command);
  mqttClient.subscribe(topic_thermostat_setTempF_cmd);
  mqttClient.subscribe(topic_thermostat_mode_cmd);
  mqttClient.subscribe(topic_fan_command);
  mqttClient.subscribe(topic_fan_speed_command);
  mqttClient.subscribe(topic_fan_preset_command);
  mqttClient.subscribe(topic_engine_loop_status);
  mqttClient.subscribe(topic_engine_loop_valve_state);
  mqttClient.subscribe(topic_plumbing_status);

  publishHomeAssistantDiscovery();
  sendStatus();
  publishThermostatStatus();
  publishSensorReadings(true);
  trySendLoopRequest();   // deliver a valve request that was made while offline
  return true;
}

// ============================================
// HOME ASSISTANT DISCOVERY
// ============================================
void publishHomeAssistantDiscovery() {
  if (!mqttClient.connected()) return;

  String deviceId = String("thermostat_") + String(CABINET_ID);
  String avail = "\"availability_topic\":\"" + String(topic_availability) + "\",";

  // Device information (shared by all entities). Entity names below are
  // relative to the device name, per HA's has_entity_name convention.
  String device = "\"device\":{";
  device += "\"identifiers\":[\"" + deviceId + "\"],";
  device += "\"name\":\"Front Thermostat\",";
  device += "\"model\":\"ESP32-S2 Thermostat\",";
  device += "\"manufacturer\":\"Chester RV Control\",";
  device += "\"sw_version\":\"" + String(FIRMWARE_VERSION) + "\"";
  device += "}";

  // Climate entity (thermostat) - name null = use the device name
  String climateTopic = "homeassistant/climate/" + deviceId + "/config";
  String climateConfig = "{";
  climateConfig += "\"name\":null,";
  climateConfig += "\"unique_id\":\"" + deviceId + "_climate\",";
  climateConfig += avail;
  climateConfig += "\"mode_command_topic\":\"" + String(topic_thermostat_mode_cmd) + "\",";
  climateConfig += "\"mode_state_topic\":\"" + String(topic_thermostat_mode) + "\",";
  climateConfig += "\"modes\":[\"off\",\"heat\"],";
  climateConfig += "\"action_topic\":\"" + String(topic_thermostat_action) + "\",";
  climateConfig += "\"temperature_command_topic\":\"" + String(topic_thermostat_setTempF_cmd) + "\",";
  climateConfig += "\"temperature_state_topic\":\"" + String(topic_thermostat_setTempF) + "\",";
  climateConfig += "\"current_temperature_topic\":\"" + String(topic_thermostat_tempF) + "\",";
  climateConfig += "\"current_humidity_topic\":\"" + String(topic_thermostat_humidity) + "\",";
  climateConfig += "\"temperature_unit\":\"F\",";
  climateConfig += "\"temp_step\":1,";
  climateConfig += "\"min_temp\":50,";
  climateConfig += "\"max_temp\":86,";
  climateConfig += "\"precision\":0.1,";
  climateConfig += device;
  climateConfig += "}";
  mqttClient.publish(climateTopic.c_str(), climateConfig.c_str(), true);

  // Humidity sensor
  String humidityTopic = "homeassistant/sensor/" + deviceId + "_humidity/config";
  String humidityConfig = "{";
  humidityConfig += "\"name\":\"Humidity\",";
  humidityConfig += "\"unique_id\":\"" + deviceId + "_humidity\",";
  humidityConfig += avail;
  humidityConfig += "\"state_topic\":\"" + String(topic_thermostat_humidity) + "\",";
  humidityConfig += "\"unit_of_measurement\":\"%\",";
  humidityConfig += "\"device_class\":\"humidity\",";
  humidityConfig += "\"state_class\":\"measurement\",";
  humidityConfig += device;
  humidityConfig += "}";
  mqttClient.publish(humidityTopic.c_str(), humidityConfig.c_str(), true);

  // Pressure sensor
  String pressureTopic = "homeassistant/sensor/" + deviceId + "_pressure/config";
  String pressureConfig = "{";
  pressureConfig += "\"name\":\"Pressure\",";
  pressureConfig += "\"unique_id\":\"" + deviceId + "_pressure\",";
  pressureConfig += avail;
  pressureConfig += "\"state_topic\":\"" + String(topic_thermostat_pressure) + "\",";
  pressureConfig += "\"unit_of_measurement\":\"hPa\",";
  pressureConfig += "\"device_class\":\"pressure\",";
  pressureConfig += "\"state_class\":\"measurement\",";
  pressureConfig += device;
  pressureConfig += "}";
  mqttClient.publish(pressureTopic.c_str(), pressureConfig.c_str(), true);

  // Remove old fan sensor entity if it exists
  String oldFanTopic = "homeassistant/sensor/" + deviceId + "_fan/config";
  mqttClient.publish(oldFanTopic.c_str(), "", true);

  // Fan entity with speed control
  String fanTopic = "homeassistant/fan/" + deviceId + "_fan/config";
  String fanConfig = "{";
  fanConfig += "\"name\":\"Fan\",";
  fanConfig += "\"unique_id\":\"" + deviceId + "_fan\",";
  fanConfig += avail;
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

  // Binary sensor for heating status (calling for heat)
  String heatingTopic = "homeassistant/binary_sensor/" + deviceId + "_heating/config";
  String heatingConfig = "{";
  heatingConfig += "\"name\":\"Heating\",";
  heatingConfig += "\"unique_id\":\"" + deviceId + "_heating\",";
  heatingConfig += avail;
  heatingConfig += "\"state_topic\":\"" + String(topic_thermostat_heating) + "\",";
  heatingConfig += "\"payload_on\":\"ON\",";
  heatingConfig += "\"payload_off\":\"OFF\",";
  heatingConfig += "\"device_class\":\"heat\",";
  heatingConfig += device;
  heatingConfig += "}";
  mqttClient.publish(heatingTopic.c_str(), heatingConfig.c_str(), true);

  // Diagnostic: temperature sensor fault
  String sensorFaultTopic = "homeassistant/binary_sensor/" + deviceId + "_sensor_fault/config";
  String sensorFaultConfig = "{";
  sensorFaultConfig += "\"name\":\"Sensor Fault\",";
  sensorFaultConfig += "\"unique_id\":\"" + deviceId + "_sensor_fault\",";
  sensorFaultConfig += avail;
  sensorFaultConfig += "\"state_topic\":\"" + String(topic_sensor_fault) + "\",";
  sensorFaultConfig += "\"payload_on\":\"ON\",";
  sensorFaultConfig += "\"payload_off\":\"OFF\",";
  sensorFaultConfig += "\"device_class\":\"problem\",";
  sensorFaultConfig += "\"entity_category\":\"diagnostic\",";
  sensorFaultConfig += device;
  sensorFaultConfig += "}";
  mqttClient.publish(sensorFaultTopic.c_str(), sensorFaultConfig.c_str(), true);

  // Diagnostic: heat valve closed by something other than the thermostat
  String valveFaultTopic = "homeassistant/binary_sensor/" + deviceId + "_valve_fault/config";
  String valveFaultConfig = "{";
  valveFaultConfig += "\"name\":\"Heat Valve Fault\",";
  valveFaultConfig += "\"unique_id\":\"" + deviceId + "_valve_fault\",";
  valveFaultConfig += avail;
  valveFaultConfig += "\"state_topic\":\"" + String(topic_valve_fault) + "\",";
  valveFaultConfig += "\"payload_on\":\"ON\",";
  valveFaultConfig += "\"payload_off\":\"OFF\",";
  valveFaultConfig += "\"device_class\":\"problem\",";
  valveFaultConfig += "\"entity_category\":\"diagnostic\",";
  valveFaultConfig += device;
  valveFaultConfig += "}";
  mqttClient.publish(valveFaultTopic.c_str(), valveFaultConfig.c_str(), true);

  // Lock entity
  String lockTopic = "homeassistant/lock/" + deviceId + "_lock/config";
  String lockConfig = "{";
  lockConfig += "\"name\":\"Cabinet Lock\",";
  lockConfig += "\"unique_id\":\"" + deviceId + "_lock\",";
  lockConfig += avail;
  lockConfig += "\"command_topic\":\"" + String(topic_cabinetLock_command) + "\",";
  lockConfig += "\"state_topic\":\"" + String(topic_cabinetLock_status) + "\",";
  lockConfig += "\"payload_lock\":\"LOCK\",";
  lockConfig += "\"payload_unlock\":\"UNLOCK\",";
  lockConfig += "\"state_locked\":\"LOCKED\",";
  lockConfig += "\"state_unlocked\":\"UNLOCKED\",";
  lockConfig += device;
  lockConfig += "}";
  mqttClient.publish(lockTopic.c_str(), lockConfig.c_str(), true);

  Serial.println("Home Assistant discovery published");
}

// ============================================
// CABINET LOCK FUNCTIONS (non-blocking servo)
// ============================================
void moveServo(int position) {
  if (!lockServo.attached()) lockServo.attach(SERVO_PIN);
  lockServo.write(position);
  servoMoving = true;
  servoMoveStart = millis();
}

void handleServo(unsigned long now) {
  if (servoMoving && now - servoMoveStart >= SERVO_MOVE_MS) {
    lockServo.detach();
    servoMoving = false;
  }
}

void lockCabinet() {
  moveServo(LOCKED_POSITION);
  currentState = STATE_LOCKED;
  Serial.println("Cabinet LOCKED");
  saveStateNow();
  sendStatus();
}

void unlockCabinet() {
  moveServo(UNLOCKED_POSITION);
  currentState = STATE_UNLOCKED;
  Serial.println("Cabinet UNLOCKED");
  saveStateNow();
  sendStatus();
}

void sendStatus() {
  if (!mqttClient.connected() || currentState == STATE_UNKNOWN) return;
  const char* lockStatus = (currentState == STATE_LOCKED) ? "LOCKED" : "UNLOCKED";
  mqttClient.publish(topic_cabinetLock_status.c_str(), lockStatus, true);
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command == "STATUS") {
    sendStatus();
    return;
  }

  unsigned long now = millis();
  if (command == lastCommand && now - lastCommandTime < COMMAND_COOLDOWN_MS) {
    Serial.println("Duplicate lock command ignored");
    return;
  }
  lastCommand = command;
  lastCommandTime = now;

  if (command == "LOCK") {
    lockCabinet();
  } else if (command == "UNLOCK") {
    unlockCabinet();
  } else if (command == "TOGGLE") {
    if (currentState == STATE_LOCKED) unlockCabinet(); else lockCabinet();
  } else {
    Serial.printf("Unknown lock command: %s\n", command.c_str());
  }
}

// ============================================
// TEMPERATURE SENSOR
// ============================================
void readTemperature() {
  unsigned long now = millis();
  lastTempRead = now;

  if (!bmePresent && now - lastSensorRetry >= SENSOR_RETRY_MS) {
    lastSensorRetry = now;
    if (bme.begin(BME280_ADDR)) {
      bmePresent = true;
      Serial.println("BME280 sensor connected");
    }
  }

  if (bmePresent) {
    uint8_t ctrlMeas = 0;
    if (!i2cRead8(BME280_ADDR, 0xF4, ctrlMeas)) {
      // Sensor no longer answers on I2C (the library would return garbage, not NaN)
      Serial.println("BME280 not responding");
      bmePresent = false;
      lastSensorRetry = now;
    } else if ((ctrlMeas & 0x03) == 0) {
      // Power-on reset: back in sleep mode with reset-value data registers. Re-initialise.
      Serial.println("BME280 was reset - reinitialising");
      if (!bme.begin(BME280_ADDR)) {
        bmePresent = false;
        lastSensorRetry = now;
      }
    } else {
      float rawT = bme.readTemperature();
      float rawH = bme.readHumidity();
      float pres = bme.readPressure() / 100.0F;
      float t = rawT + TEMP_CALIBRATION_OFFSET;

      bool plausible = !isnan(rawT) && !isnan(rawH) && !isnan(pres) &&
                       t > -30.0f && t < 60.0f &&
                       rawH >= 0.0f && rawH <= 100.0f &&
                       pres > 300.0f && pres < 1100.0f;
      if (plausible) {
        currentTemp = t;
        // The sensor reads RH at its own (self-heated) temperature; convert to RH at room temperature.
        float rh = rawH * saturationVapourPressure(rawT) / saturationVapourPressure(t);
        currentHumidity = constrain(rh, 0.0f, 100.0f);
        currentPressure = pres;
        lastValidReading = now;
        sensorEverValid = true;
      } else {
        Serial.printf("BME280 implausible reading: T=%.2f H=%.1f P=%.1f - ignored\n", t, rawH, pres);
      }
    }
  }

  bool wasOk = sensorOk;
  sensorOk = sensorEverValid && (now - lastValidReading < SENSOR_STALE_MS);
  if (wasOk != sensorOk) {
    Serial.println(sensorOk ? "Temperature sensor OK" : "TEMPERATURE SENSOR FAULT - heating disabled");
    requestStatusPublish();
    displayForce = true;
  }

  publishSensorReadings(false);
}

void publishSensorReadings(bool force) {
  if (!mqttClient.connected() || !sensorOk) return;
  char t[16], h[16], p[16];
  snprintf(t, sizeof(t), "%.1f", cToF(currentTemp));
  snprintf(h, sizeof(h), "%.1f", currentHumidity);
  snprintf(p, sizeof(p), "%.1f", currentPressure);
  if (force || strcmp(t, lastPubTemp) != 0) { mqttClient.publish(topic_thermostat_tempF, t, true); strcpy(lastPubTemp, t); }
  if (force || strcmp(h, lastPubHum) != 0) { mqttClient.publish(topic_thermostat_humidity, h, true); strcpy(lastPubHum, h); }
  if (force || strcmp(p, lastPubPres) != 0) { mqttClient.publish(topic_thermostat_pressure, p, true); strcpy(lastPubPres, p); }
}

// ============================================
// ENGINE LOOP REQUEST + COIL WARM-UP MODEL
// ============================================
void coilFlowStarted(unsigned long now) {
  if (coilFlowing) return;
  // Credit warmth the coil still holds from the previous flow period. Warmth
  // builds while coolant flows (up to FAN_DELAY) and drains at the same rate
  // once flow stops, so a coil closed 1 min ago is nearly hot and one closed
  // an hour ago is cold.
  unsigned long credit = 0;
  if (coilEverStopped) {
    unsigned long sinceStop = now - coilStoppedAt;
    credit = (coilWarmthAtStop > sinceStop) ? coilWarmthAtStop - sinceStop : 0;
  }
  coilFlowStart = now - credit;
  coilFlowing = true;
  Serial.printf("Local coil model: warm-up started (%lus credit, fan in %lus)\n", credit / 1000, (FAN_DELAY - credit) / 1000);
}

void coilFlowStopped(unsigned long now) {
  if (!coilFlowing) return;
  unsigned long warmth = now - coilFlowStart;
  coilWarmthAtStop = (warmth > FAN_DELAY) ? FAN_DELAY : warmth;
  coilStoppedAt = now;
  coilEverStopped = true;
  coilFlowing = false;
}

bool coilReady(unsigned long now) {
  // Plumbing's valve-travel model when available: it survives a thermostat
  // reboot, so the fan no longer waits out a full warm-up after an OTA or a power
  // blip while the loop has been open all along.
  if (valveTravelKnown && plumbingOnline) return valveFullyOpen;
  return coilFlowing && (now - coilFlowStart >= FAN_DELAY);
}

void setLoopWanted(bool want) {
  if (want == loopWanted) return;
  loopWanted = want;
  loopDelivered = false;
  valveFault = false;
  if (!want) coilFlowStopped(millis());
  trySendLoopRequest();
  requestStatusPublish();
}

// The request is held until the broker accepts it, so a request made while
// MQTT is down is delivered on reconnect instead of being lost.
void trySendLoopRequest() {
  if (loopDelivered || !mqttClient.connected()) return;
  if (mqttClient.publish(topic_engine_loop_command, loopWanted ? "OPEN" : "CLOSE", false)) {
    loopDelivered = true;
    loopDeliveredAt = millis();
    Serial.printf("Engine loop %s requested\n", loopWanted ? "OPEN" : "CLOSE");
    if (loopWanted) coilFlowStarted(loopDeliveredAt);
  }
}

// Compare our request with Plumbing's reported valve state. If the valve is
// closed while we want heat (Plumbing leak shutdown, or HA's valve switch), we
// do NOT re-open it automatically - that would override a safety shutdown.
// We stop running the fan and raise a fault; a user action retries.
void checkValveFeedback(unsigned long now) {
  if (!loopWanted || !loopDelivered || !plumbingStatusKnown) return;

  if (!valveFault && !plumbingLoopOpen && now - loopDeliveredAt >= VALVE_CONFIRM_MS) {
    valveFault = true;
    coilFlowStopped(now);
    Serial.println("HEAT VALVE FAULT: engine loop reported CLOSED while heat is requested");
    requestStatusPublish();
    displayForce = true;
  } else if (valveFault && plumbingLoopOpen) {
    valveFault = false;
    coilFlowStarted(now);
    Serial.println("Engine loop reported OPEN again - heat valve fault cleared");
    requestStatusPublish();
    displayForce = true;
  }
}

// A deliberate user action (button, mode or setpoint change) re-sends the heat
// request after a valve fault.
void retryValveAfterUserAction() {
  if (valveFault && loopWanted) {
    valveFault = false;
    loopDelivered = false;
    Serial.println("Retrying engine loop OPEN after user action");
    trySendLoopRequest();
  }
}

// ============================================
// THERMOSTAT CONTROL
// ============================================
void resetPI() {
  integralError = 0;
  boostActive = false;
}

void updateThermostat() {
  unsigned long now = millis();
  float deltaTime = controlInitialized ? (now - lastControlUpdate) / 1000.0f : 1.0f;
  deltaTime = constrain(deltaTime, 0.0f, 5.0f);   // never integrate over a long gap
  lastControlUpdate = now;
  controlInitialized = true;

  checkValveFeedback(now);

  if (heatMode == HEAT_OFF || !sensorOk) {
    if (heatingActive) {
      heatingActive = false;
      resetPI();
      Serial.println(heatMode == HEAT_OFF ? "Thermostat OFF - heating stopped" : "Sensor fault - heating stopped");
      requestStatusPublish();
    }
    setLoopWanted(false);
    autoFanTarget = 0;
  } else {
    float error = fToC(setTempF) - currentTemp;   // C, positive = need heat

    // Engine loop: open at or below the setpoint, closed only above it (+margin).
    // Evaluated before the fan call so the loop is always open when the fan wants heat.
    if (!loopWanted && error >= 0.0f) {
      Serial.printf("Room at/below setpoint (%.2f C under) - engine loop OPEN\n", error);
      setLoopWanted(true);
    } else if (loopWanted && -error > LOOP_CLOSE_ABOVE_F * 5.0f / 9.0f) {
      Serial.printf("Room %.2f C above setpoint - engine loop CLOSE\n", -error);
      setLoopWanted(false);
    }

    // Fan call: the PI controller runs the fan while the room is HEAT_ON_ERROR below
    // the setpoint, until it reaches the setpoint. The loop stays open throughout.
    if (!heatingActive && error > HEAT_ON_ERROR) {
      heatingActive = true;
      resetPI();
      Serial.printf("Calling for heat (error %.2f C)\n", error);
      requestStatusPublish();
    } else if (heatingActive && error <= HEAT_OFF_ERROR) {
      heatingActive = false;
      resetPI();
      Serial.println("Target reached - fan off (engine loop stays open)");
      requestStatusPublish();
    }

    if (heatingActive && !valveFault && coilReady(now)) {
      autoFanTarget = calculatePIControl(error, deltaTime);
    } else {
      autoFanTarget = 0;
    }
  }

  // A "fan off" hold only lasts for the heat cycle it was given in.
  if (fanControl == FAN_CTRL_HOLD_OFF && !heatingActive) {
    fanControl = FAN_CTRL_AUTO;
    requestStatusPublish();
  }

  applyFan(deltaTime);
}

// ============================================
// PI CONTROLLER
// ============================================
float calculatePIControl(float error, float deltaTime) {
  // Boost with hysteresis so the output doesn't flip around 3.0 C
  if (!boostActive && error > BOOST_ENTER) {
    boostActive = true;
    Serial.printf("BOOST: error %.1f C\n", error);
  } else if (boostActive && error < BOOST_EXIT) {
    boostActive = false;
    integralError = 0;
  }
  if (boostActive) return BOOST_OUTPUT;

  float maxIntegral = (100.0f - U_BIAS) / KI;
  float activeError = error;

  if (error < DEADBAND) {
    // Close to the setpoint: no proportional push, and let the integral bleed
    // off so the fan winds down instead of staying pinned until the cutoff.
    activeError = 0;
    integralError -= integralError * fminf(deltaTime / INTEGRAL_BLEED_S, 1.0f);
  } else {
    float trial = U_BIAS + KP * activeError + KI * integralError;
    if (trial < 100.0f) {   // anti-windup: stop integrating once saturated
      integralError += activeError * deltaTime;
    }
  }
  integralError = constrain(integralError, 0.0f, maxIntegral);

  float output = U_BIAS + KP * activeError + KI * integralError;
  return constrain(output, 0.0f, 100.0f);
}

// ============================================
// FAN CONTROL
// ============================================
void applyFan(float deltaTime) {
  switch (fanControl) {
    case FAN_CTRL_MANUAL:
      autoFanOutput = manualFanPercent;
      setFanPercent(manualFanPercent, false);
      break;
    case FAN_CTRL_HOLD_OFF:
      autoFanOutput = 0;
      setFanPercent(0, false);
      break;
    case FAN_CTRL_AUTO:
    default: {
      if (boostActive && autoFanTarget >= BOOST_OUTPUT) {
        autoFanOutput = autoFanTarget;   // boost jumps straight to its output
      } else {
        float maxStep = RATE_LIMIT * deltaTime;
        float change = autoFanTarget - autoFanOutput;
        if (fabsf(change) > maxStep) change = (change > 0) ? maxStep : -maxStep;
        autoFanOutput += change;
      }
      setFanPercent((int)lroundf(autoFanOutput), false);
      break;
    }
  }
}

void setFanPercent(int percent, bool forceWrite) {
  percent = constrain(percent, 0, 100);
  if (percent == fanPercent && !forceWrite) return;
  bool changed = (percent != fanPercent);
  fanPercent = percent;
  uint32_t duty = (uint32_t)((percent * 255 + 50) / 100);
  ledcWrite(FAN_PWM_PIN, duty);
  if (changed) {
    Serial.printf("Fan speed: %d%%\n", fanPercent);
    requestStatusPublish();
  }
}

const char* fanPresetState() {
  if (fanControl == FAN_CTRL_AUTO) return "auto";
  if (fanControl == FAN_CTRL_MANUAL && manualPresetName) return manualPresetName;
  return "None";   // HA's payload_reset_preset_mode: manual speed, no preset
}

const char* hvacAction() {
  if (heatMode == HEAT_OFF) return fanPercent > 0 ? "fan" : "off";
  if (heatingActive && !valveFault) {
    return coilReady(millis()) ? "heating" : "preheating";
  }
  return fanPercent > 0 ? "fan" : "idle";
}

// ============================================
// SETTINGS CHANGES (shared by buttons and MQTT)
// ============================================
void setHeatMode(HeatMode mode, bool showFeedback) {
  bool changed = (mode != heatMode);
  heatMode = mode;
  if (changed) {
    Serial.printf("Mode: %s\n", mode == HEAT_OFF ? "OFF" : "AUTO");
    if (mode == HEAT_OFF && fanControl == FAN_CTRL_HOLD_OFF) fanControl = FAN_CTRL_AUTO;
    markStateDirty();
  }
  retryValveAfterUserAction();
  requestStatusPublish();
  if (showFeedback) showMessage("MODE", mode == HEAT_OFF ? "OFF" : "HEAT", MESSAGE_VIEW_MS);
}

void setSetpointF(float f, bool showOnDisplay) {
  f = constrain(f, SETPOINT_MIN_F, SETPOINT_MAX_F);
  if (f != setTempF) {
    setTempF = f;
    Serial.printf("Setpoint: %.1f F\n", setTempF);
    markStateDirty();
  }
  retryValveAfterUserAction();
  requestStatusPublish();
  if (showOnDisplay && displayConnected && !displayOff) {
    view = VIEW_SETPOINT;
    viewStart = millis();
    viewDuration = SET_VIEW_MS;
    displayForce = true;
  }
}

// ============================================
// STATUS PUBLISHING
// ============================================
void requestStatusPublish() {
  statusDirty = true;
}

void publishThermostatStatus() {
  if (!mqttClient.connected()) return;
  char buf[16];

  snprintf(buf, sizeof(buf), "%.1f", setTempF);
  mqttClient.publish(topic_thermostat_setTempF, buf, true);
  mqttClient.publish(topic_thermostat_heating, (heatingActive && !valveFault) ? "ON" : "OFF", true);
  mqttClient.publish(topic_thermostat_mode, heatMode == HEAT_OFF ? "off" : "heat", true);
  mqttClient.publish(topic_thermostat_action, hvacAction(), true);
  snprintf(buf, sizeof(buf), "%d", fanPercent);
  mqttClient.publish(topic_fan_percent, buf, true);
  mqttClient.publish(topic_fan_preset, fanPresetState(), true);
  mqttClient.publish(topic_sensor_fault, sensorOk ? "OFF" : "ON", true);
  mqttClient.publish(topic_valve_fault, valveFault ? "ON" : "OFF", true);

  statusDirty = false;
  lastStatusPublish = millis();
}

// ============================================
// BUTTON HANDLING
//   Press UP/DOWN        first press shows the setpoint, further presses change it 1 F
//   Hold UP/DOWN         repeats, accelerating after 2.5 s
//   Hold BOTH 1.5 s      toggles heat on/off
//   Any press on a dark screen only wakes it
// ============================================
bool debounceButton(Button& b, unsigned long now) {
  bool r = (digitalRead(b.pin) == LOW);
  if (r != b.raw) {
    b.raw = r;
    b.rawChangedAt = now;
  }
  if (b.raw != b.pressed && now - b.rawChangedAt >= BUTTON_DEBOUNCE_MS) {
    b.pressed = b.raw;
    if (b.pressed) {
      b.pressedAt = now;
      return true;
    }
    b.holdActive = false;
  }
  return false;
}

void stepSetpoint(int direction) {
  float stepped = (float)lroundf(setTempF) + direction;
  setSetpointF(stepped, true);
}

void handlePress(Button& b, int direction, unsigned long now) {
  if (displayConnected && displayOff) {
    noteInteraction();   // wake only
    b.holdActive = false;
    return;
  }
  noteInteraction();
  b.holdActive = true;
  b.nextRepeatAt = now + REPEAT_DELAY_MS;

  if (displayConnected && view != VIEW_SETPOINT) {
    setSetpointF(setTempF, true);   // just show it
    return;
  }
  stepSetpoint(direction);
}

void handleHold(Button& b, int direction, unsigned long now) {
  if (!b.pressed || !b.holdActive) return;
  if ((long)(now - b.nextRepeatAt) >= 0) {
    noteInteraction();
    stepSetpoint(direction);
    unsigned long held = now - b.pressedAt;
    b.nextRepeatAt = now + (held > REPEAT_FAST_AFTER_MS ? REPEAT_FAST_MS : REPEAT_RATE_MS);
  }
}

void checkButtons(unsigned long now) {
  bool upEdge = debounceButton(btnUp, now);
  bool downEdge = debounceButton(btnDown, now);

  if (btnUp.pressed && btnDown.pressed) {
    if (!comboActive) {
      comboActive = true;
      comboFired = false;
      comboSince = now;
      btnUp.holdActive = false;
      btnDown.holdActive = false;
      noteInteraction();
    } else if (!comboFired && now - comboSince >= MODE_HOLD_MS) {
      comboFired = true;
      setHeatMode(heatMode == HEAT_OFF ? HEAT_AUTO : HEAT_OFF, true);
    }
    return;
  }
  if (comboActive) {
    // Ignore the leftover button until both are released
    if (!btnUp.pressed && !btnDown.pressed) comboActive = false;
    return;
  }

  if (upEdge) handlePress(btnUp, +1, now);
  if (downEdge) handlePress(btnDown, -1, now);
  handleHold(btnUp, +1, now);
  handleHold(btnDown, -1, now);
}

// ============================================
// DISPLAY
//   Home:     top line  OFF / AUTO 70 / WARM>70 / HEAT>70 / SENSOR! / VALVE!
//             flags     M = fan under manual control, ! = not connected to MQTT
//             big       room temperature, rounded (--F when unknown)
//   Setpoint: SET (or SET (OFF) when heat is off) + setpoint
// ============================================
void noteInteraction() {
  lastInteraction = millis();
}

void showMessage(const char* top, const char* big, unsigned long ms) {
  strncpy(msgTop, top, sizeof(msgTop) - 1);
  msgTop[sizeof(msgTop) - 1] = '\0';
  strncpy(msgBig, big, sizeof(msgBig) - 1);
  msgBig[sizeof(msgBig) - 1] = '\0';
  view = VIEW_MESSAGE;
  viewStart = millis();
  viewDuration = ms;
  displayForce = true;
}

// display.begin() plus the COM-pin layout this panel actually needs (see OLED_COM_PINS)
bool beginDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false)) return false;
  display.ssd1306_command(SSD1306_SETCOMPINS);
  display.ssd1306_command(OLED_COM_PINS);
  return true;
}

void probeDisplay(unsigned long now) {
  if (now - lastDisplayProbe < DISPLAY_PROBE_MS) return;
  lastDisplayProbe = now;

  // Adafruit_SSD1306::begin() succeeds even with no panel attached, so check for an ACK.
  bool ack = i2cPresent(OLED_ADDR);
  if (displayConnected && !ack) {
    displayConnected = false;
    Serial.println("Display lost");
  } else if (!displayConnected && ack) {
    if (beginDisplay()) {
      displayConnected = true;
      displayOff = false;
      displayDimmed = false;
      displayForce = true;
      lastFrameSig = "";
      Serial.println("Display connected");
    }
  }
}

void updateDisplayPower(unsigned long now) {
  if (!displayConnected) return;
  unsigned long idle = now - lastInteraction;
  bool fault = !sensorOk || valveFault;
  bool wantOff = !fault && !otaInProgress && idle >= DISPLAY_OFF_AFTER_MS;
  bool wantDim = idle >= DISPLAY_DIM_AFTER_MS && !otaInProgress;

  if (wantOff != displayOff) {
    display.ssd1306_command(wantOff ? SSD1306_DISPLAYOFF : SSD1306_DISPLAYON);
    displayOff = wantOff;
    if (!wantOff) displayForce = true;
  }
  if (wantDim != displayDimmed) {
    display.dim(wantDim);
    displayDimmed = wantDim;
  }
}

void buildHomeTop(char* out, size_t len) {
  int setF = (int)lroundf(setTempF);
  if (!sensorOk) {
    snprintf(out, len, "SENSOR!");
  } else if (heatMode == HEAT_OFF) {
    snprintf(out, len, "OFF");
  } else if (valveFault) {
    snprintf(out, len, "VALVE!");
  } else if (heatingActive) {
    snprintf(out, len, "%s>%d", coilReady(millis()) ? "HEAT" : "WARM", setF);
  } else {
    snprintf(out, len, "AUTO %d", setF);
  }
}

void renderDisplay(bool force) {
  if (!displayConnected || displayOff) return;
  unsigned long now = millis();
  if (view != VIEW_HOME && now - viewStart >= viewDuration) {
    view = VIEW_HOME;
  }

  char top[16];
  char big[12];
  char flags[4] = "";

  if (view == VIEW_MESSAGE) {
    snprintf(top, sizeof(top), "%s", msgTop);
    snprintf(big, sizeof(big), "%s", msgBig);
  } else if (view == VIEW_SETPOINT) {
    snprintf(top, sizeof(top), "%s", heatMode == HEAT_OFF ? "SET (OFF)" : "SET");
    snprintf(big, sizeof(big), "%dF", (int)lroundf(setTempF));
  } else {
    buildHomeTop(top, sizeof(top));
    if (sensorOk) snprintf(big, sizeof(big), "%dF", (int)lroundf(cToF(currentTemp)));
    else snprintf(big, sizeof(big), "--F");
    size_t n = 0;
    if (fanControl != FAN_CTRL_AUTO) flags[n++] = 'M';
    if (!mqttClient.connected()) flags[n++] = '!';
    flags[n] = '\0';
  }

  String sig = String((int)view) + "|" + top + "|" + big + "|" + flags;
  if (!force && sig == lastFrameSig) return;
  lastFrameSig = sig;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);

  // Status line across the top
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(top);
  if (flags[0]) {
    display.setCursor(SCREEN_WIDTH - 6 * (int)strlen(flags), 0);
    display.print(flags);
  }

  // Big value: the largest text size that fits the width, centred below the status
  // line. Font cells are 6x8 per size step; the last column and row are spacing.
  int bigLen = strlen(big);
  int size = 3;
  while (size > 1 && bigLen * 6 * size - size > SCREEN_WIDTH) size--;
  int inkW = bigLen * 6 * size - size;
  int inkH = 7 * size;
  int x = (SCREEN_WIDTH - inkW) / 2;
  int y = DISP_BODY_Y + (SCREEN_HEIGHT - DISP_BODY_Y - inkH) / 2;
  display.setTextSize(size);
  display.setCursor(x > 0 ? x : 0, y);
  display.print(big);
  display.display();
}

// ============================================
// NETWORK MAINTENANCE (non-blocking)
// ============================================
void handleWifi(unsigned long now) {
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiDown) {
      wifiDown = false;
      Serial.print("WiFi connected, IP: ");
      Serial.println(WiFi.localIP());
    }
    if (!otaInitialized) {
      ArduinoOTA.begin();
      otaInitialized = true;
      Serial.println("OTA initialized");
    }
    return;
  }

  if (!wifiDown) {
    wifiDown = true;
    wifiDownSince = now;
    lastWifiKick = now;
    Serial.println("WiFi disconnected (auto-reconnect running)");
  }
  // The core reconnects on its own; only restart the attempt occasionally,
  // never while an association/DHCP may still be in progress.
  if (now - lastWifiKick >= WIFI_KICK_MS) {
    lastWifiKick = now;
    Serial.println("WiFi still down - restarting connection attempt");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
  }
  if (now - wifiDownSince >= WIFI_RESTART_MS) {
    Serial.println("WiFi down too long - restarting");
    if (stateDirty) saveStateNow();
    ESP.restart();
  }
}

void handleMqtt(unsigned long now) {
  if (mqttClient.connected()) {
    if (mqttDown) {
      mqttDown = false;
      displayForce = true;
    }
    return;
  }
  if (!mqttDown) {
    mqttDown = true;
    mqttDownSince = now;
    lastMqttAttempt = now - MQTT_RETRY_MS;   // try again right away
    displayForce = true;
    Serial.println("MQTT disconnected");
  }
  if (WiFi.status() != WL_CONNECTED) return;
  if (now - lastMqttAttempt < MQTT_RETRY_MS) return;

  lastMqttAttempt = now;
  if (reconnectMQTT()) {
    mqttDown = false;
    displayForce = true;
  } else if (now - mqttDownSince >= MQTT_RESTART_MS) {
    Serial.println("MQTT unreachable too long - restarting");
    if (stateDirty) saveStateNow();
    ESP.restart();
  }
}

// ============================================
// SETUP
// ============================================
void setup() {
  // Drive the fan PWM low before anything else so the fan input never floats at boot.
  bool fanPwmOk = ledcAttachChannel(FAN_PWM_PIN, PWM_FREQ, PWM_RESOLUTION, FAN_LEDC_CHANNEL);
  ledcWrite(FAN_PWM_PIN, 0);

  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n================================");
  Serial.println("RV Thermostat " + String(FIRMWARE_VERSION));
  Serial.println("Cabinet: " + String(CABINET_ID));
  Serial.println("================================\n");
  Serial.println(fanPwmOk ? "Fan PWM initialized (LEDC ch 4)" : "FAN PWM ATTACH FAILED");

  EEPROM.begin(EEPROM_SIZE);
  loadStateFromEEPROM();

  pinMode(TEMP_UP_BUTTON, INPUT_PULLUP);
  pinMode(TEMP_DOWN_BUTTON, INPUT_PULLUP);

  Wire.begin(BME280_SDA, BME280_SCL);

  // Display first, so the boot screen shows while the rest starts
  if (i2cPresent(OLED_ADDR) && beginDisplay()) {
    displayConnected = true;
    showMessage("Starting", "--F", 3000);
    renderDisplay(true);
    Serial.println("OLED display connected");
  } else {
    Serial.println("OLED display not found (will keep checking)");
  }

  if (bme.begin(BME280_ADDR)) {
    bmePresent = true;
    Serial.println("BME280 sensor connected");
  } else {
    Serial.println("BME280 sensor not found (will keep retrying)");
  }
  lastSensorRetry = millis();
  readTemperature();

  // Servo: restricted to LEDC timer 0 so it never touches the fan's timer/channel.
  ESP32PWM::allocateTimer(SERVO_LEDC_TIMER);
  if (currentState == STATE_UNKNOWN) currentState = STATE_UNLOCKED;   // no saved state: default unlocked
  moveServo(currentState == STATE_LOCKED ? LOCKED_POSITION : UNLOCKED_POSITION);
  Serial.printf("Servo restored to %s\n", currentState == STATE_LOCKED ? "LOCKED" : "UNLOCKED");

  // WiFi: non-blocking; loop() keeps the thermostat running while it connects.
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);
  mqttClient.setSocketTimeout(5);
  mqttDownSince = millis();
  lastMqttAttempt = millis() - MQTT_RETRY_MS;   // first attempt as soon as WiFi is up

  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Update Starting...");
    otaInProgress = true;
    noteInteraction();
    if (displayOff) { display.ssd1306_command(SSD1306_DISPLAYON); displayOff = false; }
    showMessage("UPDATING", "0%", 600000);
    renderDisplay(true);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Complete!");
    if (stateDirty) saveStateNow();
    showMessage("UPDATE", "DONE", 5000);
    renderDisplay(true);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned int lastShown = 0;
    if (total == 0) return;
    unsigned int pct = (unsigned int)((progress * 100ULL) / total);
    Serial.printf("Progress: %u%%\r", pct);
    if (pct / 10 != lastShown / 10) {
      lastShown = pct;
      char b[8];
      snprintf(b, sizeof(b), "%u%%", pct);
      showMessage("UPDATING", b, 600000);
      renderDisplay(true);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    Serial.printf("OTA Error[%u]\n", error);
    showMessage("UPDATE", "FAIL", 5000);
    renderDisplay(true);
  });

  lastInteraction = millis();
  Serial.println("\nSetup complete\n");
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  unsigned long now = millis();

  handleWifi(now);
  handleMqtt(now);
  mqttClient.loop();
  if (otaInitialized) ArduinoOTA.handle();

  now = millis();
  handleServo(now);
  checkButtons(now);

  if (now - lastTempRead >= SENSOR_READ_MS) {
    readTemperature();
  }

  if (now - lastThermostatUpdate >= 1000) {
    lastThermostatUpdate = now;
    updateThermostat();
  }

  if (statusDirty && mqttClient.connected() && now - lastStatusPublish >= STATUS_MIN_INTERVAL_MS) {
    publishThermostatStatus();
  }

  if (now - lastHeartbeat >= STATUS_INTERVAL) {
    lastHeartbeat = now;
    sendStatus();
    publishThermostatStatus();
    publishSensorReadings(true);
    setFanPercent(fanPercent, true);   // re-assert PWM duty
    if (mqttClient.connected()) mqttClient.publish(topic_availability, "online", true);
  }

  flushStateIfDirty(now);

  probeDisplay(now);
  updateDisplayPower(now);
  if (displayForce || now - lastRender >= DISPLAY_RENDER_MS) {
    lastRender = now;
    bool force = displayForce;
    displayForce = false;
    renderDisplay(force);
  }

  delay(10);
}
