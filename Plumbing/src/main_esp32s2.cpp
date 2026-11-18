#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

// ============================================================================
// ESP32-S2 MINI PLUMBING & ENVIRONMENT CONTROLLER - Chester the Bus RV
// ============================================================================
// Board Responsibilities:
// - Read temperature sensors (hydronic, fresh tank, grey tank, ambient, manifolds)
// - Control critical safety systems (exhaust fan, tank heaters)
// - Environmental monitoring (BME280: temp, humidity, pressure)
// - Cabinet lock control (servo)
// - Valve control (rear loop, engine loop, front loop from thermostat triggers)
// - Grey tank heat loop valve control
// - Diesel heater on/off control
// - Electric heater on/off control (GPIO37, HIGH=on)
// - All relay status publishing (authoritative source for valve states)
// ============================================================================

#define FIRMWARE_VERSION "v3.5.0"
#define CABINET_ID "RearPassSide"

// ============================================================================
// WiFi & MQTT CONFIGURATION
// ============================================================================
// WiFi and OTA credentials come from secrets.ini at the repo root (gitignored),
// passed in as build flags by platformio.ini. Template: secrets.example.ini.
#if !defined(SECRET_WIFI_SSID) || !defined(SECRET_WIFI_PASSWORD) || !defined(SECRET_OTA_PASSWORD)
#error "WiFi/OTA credentials missing - copy secrets.example.ini to secrets.ini at the repo root"
#endif

const char* ssid = SECRET_WIFI_SSID;
const char* password = SECRET_WIFI_PASSWORD;
const char* mqtt_server = "192.168.8.1";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* mqtt_client_id = "ESP32S2_Plumbing";

// OTA Configuration
const char* otaHostname = "Plumbing";
const char* otaPassword = SECRET_OTA_PASSWORD;

// ============================================================================
// MQTT TOPICS
// ============================================================================
// Temperature Sensors (Publish Only)
const char* Topic_Hydronic_Temp = "HydronicTemperature";
const char* Topic_FreshWtr_Temp = "FreshWaterTemperature";
const char* Topic_GreyWtr_Temp = "GreyWaterTemperature";
const char* Topic_Temp_Return = "PlumbingTempReturnManifold";
const char* Topic_Temp_Supply = "PlumbingTempSupplyManifold";

// DS18B20 harness management
const char* Topic_TempSensors_Found = "PlumbingTempSensorsFound";     // Devices on the OneWire bus
const char* Topic_TempSensors_Live = "PlumbingTempSensorsLive";       // Positions bound and reading
const char* Topic_TempSensor_Scan = "PlumbingTempSensorScan";         // JSON bus map
const char* Topic_TempSensor_Command = "PlumbingTempSensorCommand";   // SCAN / REPORT / CLEAR
const char* Topic_TempAssign_Command = "PlumbingTempAssignCommand";   // "<role>=<ROM hex|bus index>"
const char* Topic_TempIdentify_Command = "PlumbingTempIdentifyCommand"; // "<role>" or CANCEL
const char* Topic_TempIdentify_State = "PlumbingTempIdentifyState";
const char* Topic_TempPresence_Prefix = "PlumbingTempPresence/";      // + role key -> ONLINE/OFFLINE

// Environmental Sensors (Publish Only)
const char* Topic_Environment_Temp = "EnvironmentTempAmbient";
const char* Topic_Environment_Humidity = "EnvironmentHumidity";
const char* Topic_Environment_Pressure = "EnvironmentPressure";

// Flow Sensor (Publish Only)
const char* Topic_Flow_Rate = "HotWaterFlowRate";           // L/min
const char* Topic_Flow_Rate_GPM = "HotWaterFlowRateGPM";    // US gal/min
const char* Topic_Flow_Status = "HotWaterFlowStatus";       // FLOWING / STOPPED
const char* Topic_Flow_Total = "HotWaterFlowTotal";         // Litres since boot
const char* Topic_Flow_Total_Gal = "HotWaterFlowTotalGal";  // US gallons since boot
const char* Topic_Flow_Pulses = "HotWaterFlowPulses";       // Raw pulse count since boot
const char* Topic_Flow_KFactor = "HotWaterFlowKFactor";     // Pulses per litre (retained)
const char* Topic_Flow_Calibrate = "HotWaterFlowCalibrateCommand"; // START | <volume>[L|G]

// Main Water Pump (Publish & Subscribe)
const char* Topic_MainPump_Command = "MainWaterPumpCommand"; // ON / OFF
const char* Topic_MainPump_Status = "MainWaterPumpStatus"; // ON / OFF

// Domestic Hot Water Loop (Publish & Subscribe)
const char* Topic_DomesticHW_Command = "DomesticHotWaterCommand"; // OPEN / CLOSE
const char* Topic_DomesticHW_Status = "DomesticHotWaterStatus"; // OPEN / CLOSE

// Valve Status (Publish Only - This board is authoritative)
const char* Topic_FreshWtrHeat_Status = "FreshWaterHeatStatus"; // ON / Off
const char* Topic_GreyWtrHeat_Status = "GreyWaterHeatStatus"; // ON / Off
const char* Topic_RearLoop_Status = "RearLoopStatus"; // OPEN / CLOSE
const char* Topic_EngineLoop_Status = "EngineLoopStatus"; // OPEN / CLOSE (relay)
const char* Topic_EngineLoop_ValveState = "EngineLoopValveState";       // OPEN / OPENING / CLOSING / CLOSED (modelled travel)
const char* Topic_EngineLoop_ValvePosition = "EngineLoopValvePosition"; // 0-100 % open (modelled travel)
const char* Topic_FrontLoop_Status = "FrontLoopStatus"; // OPEN / CLOSE
const char* Topic_ExhaustFan_Status = "ExhaustFanStatus"; // ON / OFF
const char* Topic_DieselHtr_Status = "DieselHeaterStatus"; // OFF / ON
const char* Topic_ElectricHtr_Status = "ElectricHeaterStatus"; // ON / OFF

// Valve Commands (Subscribe)
const char* Topic_FreshWtrHeat_Command = "FreshWaterHeatCommand"; // ON / OFF
const char* Topic_GreyWtrHeat_Command = "GreyWaterHeatCommand"; // ON / OFF
const char* Topic_RearLoop_Command = "RearLoopCommand"; // OPEN / CLOSE
const char* Topic_EngineLoop_Command = "EngineLoopCommand"; // OPEN / CLOSE
const char* Topic_FrontLoop_Command = "FrontLoopCommand"; // OPEN / CLOSE
const char* Topic_ExhaustFan_Command = "ExhaustFanCommand"; // ON / OFF
const char* Topic_DieselHtr_Command = "DieselHeaterCommand"; // OFF / ON
const char* Topic_ElectricHtr_Command = "ElectricHeaterCommand"; // ON / OFF

// Cabinet Lock Topics
const char* Topic_CabLock_Command = "CabLockRearPassSideCommand"; // LOCK / UNLOCK
const char* Topic_CabLock_Status = "CabLockRearPassSideStatus"; // LOCKED / UNLOCKED
const char* Topic_AllLock_Command = "CabLockAllCommand"; // LOCK / UNLOCK

// Mode Topics (Subscribe) - Single global mode controls all automation
const char* Topic_Plumbing_Mode = "PlumbingMode"; // AUTO / MANUAL (global - state)
const char* Topic_Plumbing_ModeCmd = "PlumbingModeCommand"; // AUTO / MANUAL (global - command)

// Configuration Topics (State - Published by board)
const char* Topic_FreshWtrHeat_TempSet = "FreshWaterHeatTempSet";
const char* Topic_GreyWtrHeat_TempSet = "GreyWaterHeatTempSet";
const char* Topic_ExhaustFan_TempSet = "ExhaustFanTempSet"; // Single setpoint for fan

// Configuration Command Topics (Commands from HA - Subscribed by board)
const char* Topic_FreshWtrHeat_TempSetCmd = "FreshWaterHeatTempSetCommand";
const char* Topic_GreyWtrHeat_TempSetCmd = "GreyWaterHeatTempSetCommand";
const char* Topic_ExhaustFan_TempSetCmd = "ExhaustFanTempSetCommand";

// Alert Topics (Publish Only) - Consolidated
// Use a single alert topic that carries descriptive text for the most
// recent alert. Retained=true so Home Assistant can show the last alert.
const char* Topic_Alert = "PlumbingAlert"; // Descriptive retained alert messages

// Forward declarations for alert helpers (implemented later)
void publishAlertRaw(const String &msg);
void refreshAlertPublish();

// Safety Command Topics (Subscribe)
const char* Topic_Clear_High_Humidity = "ClearHighHumidityCommand"; // CLEAR

// System Health (Publish Only)
const char* Topic_Plumbing_Status = "PlumbingSystemStatus"; // ONLINE / ERROR
const char* Topic_Plumbing_Uptime = "PlumbingUptime";
const char* Topic_Plumbing_IP = "PlumbingIP";            // Retained - findable without mDNS
const char* Topic_Plumbing_Command = "PlumbingCommand";  // RESTART
const char* Topic_Firmware_Version = "PlumbingFirmwareVersion";

// ============================================================================
// HARDWARE PIN DEFINITIONS - ESP32-S2 Mini
// ============================================================================
// DS18B20 Temperature Sensors (All on shared OneWire bus)
#define TEMP_SENSOR_PIN 13           // GPIO13 - Shared OneWire bus, 4.7k pullup to 3.3V
// Five probes share this bus. Each harness position is bound to a sensor's 64-bit
// ROM address (stored in NVS), never to its discovery index - see the harness
// section below for why.

// Relay Outputs (Active LOW unless noted)
#define FRESH_WATER_HEATER_PIN 9     // GPIO9 - Fresh water tank heater
#define GREY_WATER_HEATER_PIN 2      // GPIO2 - Grey water tank heater
#define EXHAUST_FAN_PIN 12           // GPIO12 - Exhaust fan
#define MAIN_WATER_PUMP_PIN 11       // GPIO11 - Main water pump (normally closed relay)
#define DOMESTIC_HW_SOLINOID_PIN 10  // GPIO10 - Domestic hot water loop valve
#define REAR_LOOP_VALVE_PIN 5        // GPIO5 - Rear loop valve
#define ENGINE_LOOP_VALVE_PIN 3      // GPIO3 - Engine loop valve
#define FRONT_LOOP_VALVE_PIN 4       // GPIO4 - Front loop valve
#define DIESEL_HEATER_PIN 8          // GPIO8 - Diesel heater relay 1 (white wire)
#define DIESEL_HEATER_2_PIN 7        // GPIO7 - Diesel heater relay 2 (blue wire)
#define DIESEL_HEATER_3_PIN 6        // GPIO6 - Diesel heater relay 3 (yellow wire)
#define ELECTRIC_HEATER_PIN 37       // GPIO37 - Electric heater relay (HIGH = ON)

// Cabinet Lock Servo
#define SERVO_PIN 16                 // GPIO16 - Servo control

// Flow sensor (Digital Input)
#define FLOW_SENSOR_PIN 17           // GPIO17 - Flow sensor input

// I2C for BME280
#define BME280_SDA 33                // GPIO33 - I2C SDA
#define BME280_SCL 35                // GPIO35 - I2C SCL

// ============================================================================
// SENSOR SETUP
// ============================================================================
// Shared OneWire bus for all DS18B20 sensors
OneWire tempSensorWire(TEMP_SENSOR_PIN);

// Single DallasTemperature instance for all sensors
DallasTemperature tempSensors(&tempSensorWire);

// BME280 Environmental Sensor
Adafruit_BME280 bme;
bool bmeAvailable = false;

// Servo for cabinet lock
Servo lockServo;

// ============================================================================
// VALVE CONTROL SOURCES
// ============================================================================
// Valve Control Sources (MQTT only)
bool engineLoopMqttRequest = false;  // MQTT command from GIGA
bool frontLoopMqttRequest = false;  // MQTT command from GIGA

// ============================================================================
// FLOW SENSOR
// ============================================================================
// The ISR only ever counts. Everything else works off the monotonic total, so
// no window can be "missed" between evaluations.
volatile unsigned long flowPulseTotal = 0;   // Monotonic since boot, never reset
volatile unsigned long lastPulseTime = 0;    // millis() of the most recent pulse

float flowRate = 0.0;              // L/min
bool flowDetected = false;         // Debounced - drives the DHW solenoid
unsigned long lastFlowCalc = 0;
unsigned long lastRatePulseTotal = 0;
const unsigned long FLOW_CALC_INTERVAL = 1000;   // Rate + totals recomputed every second

// ---- Flow presence debounce ----
// Presence is deliberately hair-trigger: a single pulse counts, and stays
// counting until the bus has been quiet for FLOW_PULSE_IDLE_MS. That makes a
// trickle register. Noise rejection is done by DURATION instead - a burst has
// to sustain for FLOW_ON_DELAY_MS before the valve opens.
//
// This is what stops the valve reopening after a tap closes: slamming the
// solenoid shut kicks the turbine for a few hundred ms, which used to clear the
// old "10 pulses in a 1 s window" test outright. It can no longer outlast the
// on-delay.
const unsigned long FLOW_EVAL_INTERVAL   = 50;    // State machine tick
const unsigned long FLOW_PULSE_IDLE_MS   = 500;   // Quiet for this long = flow gone
const unsigned long FLOW_ON_DELAY_MS     = 600;   // Sustained flow before the valve opens
const unsigned long FLOW_OFF_DELAY_MS    = 1500;  // Sustained quiet before it closes
const unsigned long FLOW_MIN_PULSES_OPEN = 5;     // Pulses that must land during the on-delay
const unsigned long FLOW_CLOSE_LOCKOUT_MS = 1000; // Ignore flow this long after WE closed the valve

enum FlowState { FLOW_IDLE, FLOW_STARTING, FLOW_ACTIVE, FLOW_STOPPING };
FlowState flowState = FLOW_IDLE;
unsigned long flowStateSince = 0;
unsigned long flowStartPulses = 0;
unsigned long lastFlowEval = 0;
unsigned long flowValveClosedAt = 0;

// ---- Metering ----
// Pulses per litre. 450 is the YF-S201 figure (F = 7.5 x Q L/min). Other
// turbines differ by an order of magnitude, so this is calibrated by bucket
// test and persisted rather than hardcoded - see handleFlowCalibration().
float flowKFactor = 450.0;
const float FLOW_K_MIN = 50.0;
const float FLOW_K_MAX = 20000.0;
const float LITRES_PER_US_GALLON = 3.785411784;

bool flowCalActive = false;
unsigned long flowCalStartPulses = 0;

// ============================================================================
// CONFIGURATION VARIABLES
// ==========================================================================
float freshWaterTempTarget = 40.0;    // °F
float greyWaterTempTarget = 35.0;     // °F
float exhaustFanTempSet = 85.0;       // °F - Exhaust fan on setpoint (hysteresis applied)

#define FREEZE_WARNING_TEMP 35.0      // °F
#define HIGH_TEMP_WARNING 180.0       // °F

// ============================================================================
// STATE VARIABLES
// ============================================================================
// Valve States
bool freshWaterHeaterOn = false;
bool greyWaterHeaterOn = false;
bool exhaustFanOn = false;
bool rearLoopOpen = false;
bool engineLoopOpen = false;
bool frontLoopOpen = false;
bool dieselHeaterOn = false;  // false=OFF, true=ON
bool electricHeaterOn = false;  // false=OFF, true=ON (HIGH=on)
bool mainWaterPumpOn = true;  // Initialized HIGH (relay OFF) = pump ON for normally closed
bool DHWSolinoidOpen = false;

// ---- Engine loop valve travel model ----
// The relay only says which way the valve is being driven. The valve is modelled
// as taking ENGINE_LOOP_TRAVEL_MS to travel fully either way, starting from
// wherever it is when the command changes: closed for 3 min then reopened, it is
// 3 min from fully open again, not 10.5. The FrontThermostat holds its fan until
// EngineLoopValveState reads OPEN (keep its FAN_DELAY fallback in step with this).
enum ValveTravelState : uint8_t { VALVE_CLOSED, VALVE_OPENING, VALVE_OPEN, VALVE_CLOSING };
const char* const VALVE_TRAVEL_NAMES[] = { "CLOSED", "OPENING", "OPEN", "CLOSING" };
const unsigned long ENGINE_LOOP_TRAVEL_MS = 630000UL;   // 10.5 min to travel fully, each direction
const unsigned long ENGINE_LOOP_SAVE_MS = 60000UL;      // NVS snapshot interval while moving
const unsigned long ENGINE_LOOP_PUBLISH_MS = 30000UL;   // position publish interval while moving
unsigned long engineLoopTravelAtMark = 0;  // modelled travel at the last mark: 0 = closed, TRAVEL = open
unsigned long engineLoopMarkAt = 0;        // millis() of that mark
int engineLoopLastState = -1;              // state at the previous update (snapshot trigger)
int engineLoopPublishedState = -1;         // last state that reached the broker
int engineLoopPublishedPct = -1;
unsigned long engineLoopLastSave = 0;
unsigned long engineLoopLastPublish = 0;

// Mode - single global mode controls all automation
String plumbingMode = "AUTO"; // AUTO or MANUAL

// Cabinet Lock State
enum LockState {
  STATE_UNKNOWN,
  STATE_UNLOCKED,
  STATE_LOCKED
};
LockState currentLockState = STATE_UNKNOWN;
#define LOCKED_POSITION 90
#define UNLOCKED_POSITION 5

// Temperature Readings
float hydronicTemp = -999.0;        // Buffer tank ("buffer_tank" position)
float freshWaterTemp = -999.0;
float greyWaterTemp = -999.0;
float returnManifoldTemp = -999.0;
float supplyManifoldTemp = -999.0;
float environmentTemp = -999.0;
float environmentHumidity = -999.0;
float environmentPressure = -999.0;

// Temperature Filtering - Moving Average Filter (5 samples)
// Applied only to the tank probes, where DS18B20 quantisation noise would
// otherwise chatter the heater relays around their setpoint.
#define TEMP_FILTER_SIZE 5

// ============================================================================
// DS18B20 HARNESS - ADDRESS-BASED ROLE ASSIGNMENT
// ============================================================================
// Five probes share the GPIO13 OneWire bus. Each harness position is bound to a
// sensor's 64-bit ROM address, held in NVS - never to its discovery index.
// Index order IS ROM-address order, so it shifts whenever a probe is added,
// replaced, or drops off the bus, silently swapping readings between positions.
// On a bus that drives tank heaters that is not an acceptable failure mode.
// Addresses do not shift.
//
// Positions are labelled by hand: trigger Identify for a position, warm that
// probe with your hand, and the sensor that rises fastest takes the position.

#define TEMP_ROLE_COUNT 5
#define TEMP_MAX_BUS_SENSORS 10

// A DS18B20 is rated -55..+125 C. Anything outside that window is a failed
// read, not a temperature.
#define TEMP_VALID_MIN_F -67.0
#define TEMP_VALID_MAX_F 257.0

// Named indices into TEMP_ROLES
#define TEMP_ROLE_SUPPLY   0
#define TEMP_ROLE_RETURN   1
#define TEMP_ROLE_BUFFER   2
#define TEMP_ROLE_GREY     3
#define TEMP_ROLE_FRESH    4

struct TempRole {
  const char* key;        // Identify/assign payload, presence topic
  const char* nvsKey;     // NVS key holding this position's ROM address (<=15 chars)
  int legacyIndex;        // Old "ts<n>" NVS key this position used, for one-time migration
  const char* name;       // Home Assistant entity name
  const char* objectId;   // Home Assistant object id (unchanged for existing entities)
  const char* stateTopic;
  float* value;           // The existing global this position drives
  bool filtered;          // 5-sample moving average
  const char* icon;
};

// nvsKey is keyed to the position, never to its index in this table, so a
// position can be added or removed without shuffling everyone else's stored
// probe address. legacyIndex maps back to the original index-keyed layout
// (supply 0, return 1, buffer 2, grey 3, boiler_exhaust 4, fresh 5) that
// boards in the field are still running; loadTempAssignments() migrates it
// once and then forgets about it.
const TempRole TEMP_ROLES[TEMP_ROLE_COUNT] = {
  { "supply_manifold", "ts_supply", 0, "Supply Manifold Temperature", "supply_manifold_temp", Topic_Temp_Supply,   &supplyManifoldTemp, false, "mdi:pipe" },
  { "return_manifold", "ts_return", 1, "Return Manifold Temperature", "return_manifold_temp", Topic_Temp_Return,   &returnManifoldTemp, false, "mdi:pipe" },
  { "buffer_tank",     "ts_buffer", 2, "Hydronic Temperature",        "hydronic_temp",        Topic_Hydronic_Temp, &hydronicTemp,       false, "mdi:water-boiler" },
  { "grey_tank",       "ts_grey",   3, "Grey Tank Temperature",       "grey_water_temp",      Topic_GreyWtr_Temp,  &greyWaterTemp,      true,  "mdi:storage-tank" },
  { "fresh_tank",      "ts_fresh",  5, "Fresh Tank Temperature",      "fresh_water_temp",     Topic_FreshWtr_Temp, &freshWaterTemp,     true,  "mdi:storage-tank" }
};

float tempFilterBuf[TEMP_ROLE_COUNT][TEMP_FILTER_SIZE];
int tempFilterIdx[TEMP_ROLE_COUNT] = {0};
int tempFilterFill[TEMP_ROLE_COUNT] = {0};

// Why a probe read failed. Published per sensor in PlumbingTempSensorScan, because
// DallasTemperature::getTempC() collapses every failure into the same -127 C.
enum TempReadErr : uint8_t {
  TERR_NONE = 0,
  TERR_NO_PRESENCE,   // no presence pulse on reset: bus shorted/open, or held low
  TERR_NO_REPLY,      // scratchpad all 0xFF: nothing answered MATCH ROM
  TERR_ALL_ZERO,      // scratchpad all 0x00: data line held low during the read
  TERR_CRC,           // scratchpad CRC mismatch: signal integrity (pullup, length, splices)
  TERR_POWER_ON,      // 85 C power-on value: conversion never ran (VDD / parasite power)
  TERR_RANGE          // outside the DS18B20's -55..125 C rating
};
const char* const TEMP_ERR_NAMES[] = {
  "", "no_presence", "no_reply", "all_zero", "crc", "power_on_85C", "range"
};

// Everything currently answering on the bus, in search order
struct TempBusSensor {
  DeviceAddress addr;
  char hex[17];
  bool isDS18B20;
  bool valid;
  float tempF;
  float baselineF;   // Snapshot taken when an Identify run starts
  int role;          // Index into TEMP_ROLES, or -1
  uint32_t reads;    // Read attempts since boot
  uint32_t errors;   // Failed attempts since boot
  uint8_t lastErr;   // TempReadErr of the most recent attempt
  uint8_t scratch[9];// Raw scratchpad from the most recent attempt
};

TempBusSensor tempBus[TEMP_MAX_BUS_SENSORS];
int tempBusCount = 0;

struct TempRoleState {
  DeviceAddress addr;
  bool assigned;              // An address is stored for this position
  bool present;               // ...and that address answered on the last scan
  bool valid;                 // ...and produced a plausible reading
  bool presencePublished;
  bool presenceKnown;
  unsigned long missingSince; // 0 while healthy
};

TempRoleState tempRoleState[TEMP_ROLE_COUNT];

bool tempConversionPending = false;
unsigned long tempRescanLast = 0;
const unsigned long TEMP_RESCAN_INTERVAL = 60000;
int lastPublishedBusCount = -1;
int lastPublishedLiveCount = -1;

// Identify-by-warming
bool tempIdentifyActive = false;
int tempIdentifyRole = -1;
unsigned long tempIdentifyStart = 0;
String tempIdentifyState = "IDLE";
const unsigned long TEMP_IDENTIFY_WINDOW_MS = 60000;
const float TEMP_IDENTIFY_RISE_F = 4.0;    // Rise that counts as "you touched this one"
const float TEMP_IDENTIFY_MARGIN_F = 2.0;  // Winner must beat runner-up by this much

// Probe-loss fail-safe. A tank heater whose own probe has died has no control
// input at all - the automation below can no longer decide to turn it off, so
// it would stay energised indefinitely. In AUTO mode the heater is switched OFF
// after its probe has been missing this long. Set false to keep the old
// behaviour (heater holds its last state, alert only).
const bool TEMP_PROBE_LOSS_FAILSAFE = true;
const unsigned long TEMP_PROBE_LOSS_GRACE_MS = 60000;

// Alert state for missing probes
bool tempProbeAlertActive = false;
String tempProbeAlertText = "";

// Alert States
bool freezeWarningActive = false;
bool highTempWarningActive = false;
bool highHumidityAlarmActive = false;

// Humidity Leak Detection
float lastHumidity = -999.0;
float baselineHumidity = -999.0;              // Baseline humidity before spike detected
const float HUMIDITY_SPIKE_THRESHOLD = 10.0;  // % increase that triggers alarm
const float HUMIDITY_SPIKE_TOLERANCE = 3.0;   // % tolerance for spike recovery (must return within 3% of baseline)
const float HUMIDITY_RATE_WINDOW = 60000;     // 1 minute window for rate check
const unsigned long SPIKE_RECOVERY_TIMEOUT = 30000;  // 30 second window to detect spike recovery
unsigned long lastHumidityCheck = 0;
unsigned long spikeDetectedTime = 0;          // Timestamp when spike was first detected
bool humiditySpikePending = false;            // Flag indicating spike detected, waiting for confirmation

// Timing Variables
unsigned long lastSensorRead = 0;
unsigned long lastStatusPublish = 0;
unsigned long lastHealthPublish = 0;
const unsigned long SENSOR_READ_INTERVAL = 5000;      // 5 seconds
const unsigned long STATUS_PUBLISH_INTERVAL = 10000;  // 10 seconds
const unsigned long HEALTH_PUBLISH_INTERVAL = 60000;  // 60 seconds

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

// Preferences for persistent storage
Preferences preferences;

// OTA flag
bool otaInProgress = false;

// True while ArduinoOTA.begin() is live on the CURRENT WiFi session. begin() is
// what starts the mDNS responder, and mDNS does not survive a WiFi reconnect -
// so this has to be re-run on every reconnect or Plumbing.local silently stops
// resolving while MQTT carries on working perfectly.
bool otaListenerUp = false;

// Watchdog configuration
const int WATCHDOG_TIMEOUT_SEC = 60; // seconds before WDT resets the device

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void setupWifi();
bool ensureWifiConnected(unsigned long timeoutMs);
void setupOTA();
void publishNetworkInfo();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void subscribeToTopics();
void publishHomeAssistantDiscovery();

// Temperature Sensors
void readTemperatures();
void publishTemperatures();
void readBME280();
void publishBME280();

// DS18B20 harness
void rescanTempBus();
void mapTempRolesToBus();
void collectTemperatures();
bool tempBusNeedsRescan();
void assignTempRole(int r, const DeviceAddress addr, bool save);
void clearTempRole(int r, bool save);
int  tempRoleIndexFromKey(const String& key);
void startTempIdentify(int r);
void handleTempIdentify();
void setTempIdentifyState(const String& state);
void loadTempAssignments();
void saveTempAssignments();
void publishTempPresence(int r, bool force);
void publishTempCounts(bool force);
void publishTempScan();
void printTempBusReport();
void tempAddrToHex(const DeviceAddress addr, char* out);
bool tempHexToAddr(const String& hex, DeviceAddress out);

// Flow
void calculateFlowRate();
void evaluateFlowState();
void publishFlowRate(bool force);
void handleFlowCalibration(const String& payload);
void IRAM_ATTR flowSensorISR();

// Valve Control Functions
void controlFreshWaterHeater(bool turnOn);
void controlGreyWaterHeater(bool turnOn);
void controlExhaustFan(bool turnOn);
void controlRearLoopValve(bool open);
void controlEngineLoopValve(bool open);
void controlFrontLoopValve(bool open);
void initEngineLoopTravel();
void updateEngineLoopValve(bool forcePublish);
void controlDieselHeater(bool turnOn);
void controlElectricHeater(bool turnOn);
void controlMainWaterPump(bool turnOn);
void controlDHWSolinoid(bool open);

// Cabinet Lock
void lockCabinet();
void unlockCabinet();
void processLockCommand(String command);

// Automation & Safety
void handleCriticalAutomation();
void checkAlerts();

// Status & Health
void publishStatus();
void publishHealth();
void saveConfigToPreferences();
void loadConfigFromPreferences();
void saveValveStatesToEEPROM();
void flushEEPROM();
void restoreValveStatesFromEEPROM();
void publishAllStatesToBroker();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n\n============================================");
  Serial.println("  ESP32-S2 PLUMBING & ENVIRONMENT CONTROLLER");
  Serial.println("  Chester the Bus RV Control System");
  Serial.println("  Firmware: " + String(FIRMWARE_VERSION));
  Serial.println("  Cabinet ID: " + String(CABINET_ID));
  Serial.println("============================================\n");

  // Load saved configuration
  preferences.begin("plumbing", false);
  loadConfigFromPreferences();

  // Configure relay pins as outputs (Active LOW)
  pinMode(FRESH_WATER_HEATER_PIN, OUTPUT);
  pinMode(GREY_WATER_HEATER_PIN, OUTPUT);
  pinMode(EXHAUST_FAN_PIN, OUTPUT);
  pinMode(REAR_LOOP_VALVE_PIN, OUTPUT);
  pinMode(ENGINE_LOOP_VALVE_PIN, OUTPUT);
  pinMode(FRONT_LOOP_VALVE_PIN, OUTPUT);
  pinMode(DIESEL_HEATER_PIN, OUTPUT);
  pinMode(DIESEL_HEATER_2_PIN, OUTPUT);
  pinMode(DIESEL_HEATER_3_PIN, OUTPUT);
  pinMode(ELECTRIC_HEATER_PIN, OUTPUT);
  pinMode(MAIN_WATER_PUMP_PIN, OUTPUT);
  pinMode(DOMESTIC_HW_SOLINOID_PIN, OUTPUT);

  // Initialize all relays to OFF (HIGH = OFF for active LOW, except electric heater: LOW = OFF)
  digitalWrite(FRESH_WATER_HEATER_PIN, HIGH);
  digitalWrite(GREY_WATER_HEATER_PIN, HIGH);
  digitalWrite(EXHAUST_FAN_PIN, HIGH);
  digitalWrite(REAR_LOOP_VALVE_PIN, HIGH);
  digitalWrite(ENGINE_LOOP_VALVE_PIN, HIGH);
  digitalWrite(FRONT_LOOP_VALVE_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
  digitalWrite(DIESEL_HEATER_3_PIN, HIGH);
  digitalWrite(ELECTRIC_HEATER_PIN, LOW);  // HIGH = ON, so LOW = OFF
  digitalWrite(MAIN_WATER_PUMP_PIN, HIGH); // RELAY IS NORMALLY CLOSED
  digitalWrite(DOMESTIC_HW_SOLINOID_PIN, HIGH);
  Serial.println("✓ Relay pins configured (Active LOW, all systems OFF)");

  // Restore valve/relay/lock states right away (board is authoritative). This
  // used to run after WiFi, which left every relay OFF for up to 15 s on each
  // reboot - cycling the diesel heater, dropping the heating loops, and
  // unlocking a locked cabinet - and the network-failure restarts below can
  // reboot the board every minute while the router is down.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  restoreValveStatesFromEEPROM();

  // Configure flow sensor with interrupt
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, FALLING);
  Serial.println("✓ Flow sensor initialized with interrupt (GPIO17)");

  // Initialize I2C for BME280
  Wire.begin(BME280_SDA, BME280_SCL);
  if (bme.begin(0x76, &Wire)) {
    bmeAvailable = true;
    Serial.println("✓ BME280 sensor found at 0x76");
  } else if (bme.begin(0x77, &Wire)) {
    bmeAvailable = true;
    Serial.println("✓ BME280 sensor found at 0x77");
  } else {
    Serial.println("⚠ BME280 sensor NOT found");
  }

  // Initialize DS18B20 temperature sensors (shared OneWire bus on pin 13)
  Serial.println("Initializing DS18B20 harness (GPIO13)...");

  tempSensors.begin();
  // Conversions run non-blocking. Left at the default, requestTemperatures()
  // sits in a 750ms delay on every read cycle, which stalls flow-sensor
  // response and MQTT servicing for 15% of the board's runtime.
  tempSensors.setWaitForConversion(false);
  delay(100);  // Give sensors time to initialize

  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    for (int i = 0; i < TEMP_FILTER_SIZE; i++) tempFilterBuf[r][i] = -999.0;
  }

  Serial.println("Restoring harness positions from NVS:");
  loadTempAssignments();

  rescanTempBus();
  printTempBusReport();

  // Connect to WiFi
  setupWifi();

  // Setup OTA
  setupOTA();

  // Initialize task watchdog: reset device on panic after timeout
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&wdtConfig);
  esp_task_wdt_add(NULL); // add current task (the loop task)
  Serial.print("✓ Task watchdog enabled (");
  Serial.print(WATCHDOG_TIMEOUT_SEC);
  Serial.println("s)");

  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(2048);   // temp-bus scan JSON with diagnostics runs ~900 bytes

  Serial.println("\n✓ Initialization complete");
  Serial.println("============================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  ArduinoOTA.handle();

  // Feed task watchdog to indicate loop is healthy
  esp_task_wdt_reset();

  unsigned long currentMillis = millis();
  // Periodically ensure WiFi is connected (non-blocking short attempts)
  static unsigned long lastWifiCheck = 0;
  if (currentMillis - lastWifiCheck >= 10000) { // check every 10s
    lastWifiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi down - attempting quick reconnect...");
      otaListenerUp = false;   // the mDNS responder died with the connection
      ensureWifiConnected(3000); // short timeout
    }

    // Re-arm OTA + mDNS after any reconnect. Without this the board stays
    // perfectly healthy on MQTT while Plumbing.local stops resolving, and the
    // only recovery is a physical power cycle.
    if (!otaListenerUp && WiFi.status() == WL_CONNECTED) {
      ArduinoOTA.begin();
      otaListenerUp = true;
      Serial.print("✓ OTA/mDNS responder (re)started - IP: ");
      Serial.println(WiFi.localIP());
      publishNetworkInfo();
    }
  }

  if (!otaInProgress) {
    // Maintain MQTT connection
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();

    // Flow presence + DHW solenoid. Runs at 50ms so the on/off delays are
    // accurate; the old code only looked once a second, which is why a
    // water-hammer blip showed up as a full second of the valve reopening.
    if (currentMillis - lastFlowEval >= FLOW_EVAL_INTERVAL) {
      lastFlowEval = currentMillis;
      evaluateFlowState();
    }

    // Flow rate, totals and publishing (1 second)
    if (currentMillis - lastFlowCalc >= FLOW_CALC_INTERVAL) {
      calculateFlowRate();
      publishFlowRate(false);
    }

    // Engine loop valve travel model (1 second)
    static unsigned long lastEngineLoopTick = 0;
    if (currentMillis - lastEngineLoopTick >= 1000) {
      lastEngineLoopTick = currentMillis;
      updateEngineLoopValve(false);
    }

    // Read temperature sensors
    if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
      lastSensorRead = currentMillis;
      readTemperatures();
      publishTemperatures();

      if (bmeAvailable) {
        readBME280();
        publishBME280();
      }

      handleCriticalAutomation();
      checkAlerts();
    }

    // Publish status
    if (currentMillis - lastStatusPublish >= STATUS_PUBLISH_INTERVAL) {
      lastStatusPublish = currentMillis;
      publishStatus();
    }

    // Publish health metrics
    if (currentMillis - lastHealthPublish >= HEALTH_PUBLISH_INTERVAL) {
      lastHealthPublish = currentMillis;
      publishHealth();
    }

    // Flush deferred EEPROM writes (debounced to reduce flash wear)
    flushEEPROM();
  }

  delay(10);
}

// ============================================================================
// WiFi SETUP
// ============================================================================
void setupWifi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected!");
    Serial.print("  IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("  MAC: ");
    Serial.println(WiFi.macAddress());
  } else {
    Serial.println("\n⚠ WiFi connection failed");
  }
}

// Try to ensure WiFi is connected. Uses short exponential backoff and
// returns quickly if it can't connect. Caller may choose to restart
// the interface or device after repeated failures.
bool ensureWifiConnected(unsigned long timeoutMs = 5000) {
  if (WiFi.status() == WL_CONNECTED) return true;

  unsigned long start = millis();
  unsigned long attemptDelay = 250; // start small
  const unsigned long maxDelay = 3000;

  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    Serial.print("WiFi not connected - trying to reconnect...");
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);

    unsigned long waitStart = millis();
    while ((millis() - waitStart) < attemptDelay) {
      if (WiFi.status() == WL_CONNECTED) break;
      delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(" connected");
      Serial.print("IP: "); Serial.println(WiFi.localIP());
      return true;
    }

    // increase backoff but cap it
    attemptDelay = min(maxDelay, attemptDelay * 2);
    Serial.print(" reconnect failed, next attempt in ");
    Serial.print(attemptDelay);
    Serial.println(" ms");
  }

  Serial.println("ensureWifiConnected: timed out");
  return (WiFi.status() == WL_CONNECTED);
}

// ============================================================================
// OTA SETUP
// ============================================================================
void setupOTA() {
  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    // Pump is controlled externally; firmware will not change pump state during OTA
    Serial.println("\n🔄 OTA Update Starting...");
    // Detach watchdog for this task while OTA runs to avoid timeout during long uploads
    esp_task_wdt_delete(NULL);
    Serial.println("✓ Watchdog detached for OTA");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n✓ OTA Update Complete!");
    // Re-attach watchdog after OTA completes
    esp_task_wdt_add(NULL);
    otaInProgress = false;
    Serial.println("✓ Watchdog reattached after OTA");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static int lastPercent = -1;
    int percent = (progress / (total / 100));
    if (percent != lastPercent && percent % 10 == 0) {
      Serial.printf("  Progress: %u%%\n", percent);
      lastPercent = percent;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n✗ OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    // Ensure watchdog is reattached after error path too
    esp_task_wdt_add(NULL);
    otaInProgress = false;
    Serial.println("✓ Watchdog reattached after OTA error");
  });

  ArduinoOTA.begin();
  otaListenerUp = (WiFi.status() == WL_CONNECTED);
  Serial.println("✓ OTA enabled");
}

// The board's IP, retained, so it can always be found without mDNS.
void publishNetworkInfo() {
  if (!client.connected()) return;
  client.publish(Topic_Plumbing_IP, WiFi.localIP().toString().c_str(), true);
}

// ============================================================================
// HOME ASSISTANT MQTT DISCOVERY
// ============================================================================
void publishHomeAssistantDiscovery() {
  Serial.println("📡 Publishing Home Assistant MQTT Discovery...");

  String deviceId = "chester_plumbing";
  String deviceName = "Chester Plumbing System";

  // Device configuration (shared by all entities)
  String device = "{\"identifiers\":[\"" + deviceId + "\"],"
                  "\"name\":\"" + deviceName + "\","
                  "\"model\":\"ESP32-S2 Mini\","
                  "\"manufacturer\":\"ChesterTheBus\","
                  "\"sw_version\":\"" + String(FIRMWARE_VERSION) + "\"}";

  // Helper lambda for publishing discovery
  auto publishDiscovery = [&](const char* component, const char* objectId, const char* name,
                              const char* stateTopic, const char* deviceClass = nullptr,
                              const char* unit = nullptr, const char* commandTopic = nullptr,
                              const char* payloadOn = nullptr, const char* payloadOff = nullptr,
                              const char* icon = nullptr, const char* entityCategory = nullptr,
                              const char* stateClass = nullptr) {
    String topic = String("homeassistant/") + component + "/" + deviceId + "/" + objectId + "/config";
    String payload = "{\"name\":\"" + String(name) + "\","
                     "\"unique_id\":\"" + deviceId + "_" + objectId + "\","
                     "\"state_topic\":\"" + String(stateTopic) + "\"";

    if (deviceClass) payload += ",\"device_class\":\"" + String(deviceClass) + "\"";
    if (unit) payload += ",\"unit_of_measurement\":\"" + String(unit) + "\"";
    if (commandTopic) payload += ",\"command_topic\":\"" + String(commandTopic) + "\"";
    if (payloadOn) payload += ",\"payload_on\":\"" + String(payloadOn) + "\"";
    if (payloadOff) payload += ",\"payload_off\":\"" + String(payloadOff) + "\"";
    if (icon) payload += ",\"icon\":\"" + String(icon) + "\"";
    if (entityCategory) payload += ",\"entity_category\":\"" + String(entityCategory) + "\"";
    if (stateClass) payload += ",\"state_class\":\"" + String(stateClass) + "\"";

    payload += ",\"device\":" + device + "}";

    client.publish(topic.c_str(), payload.c_str(), true);
    delay(50); // Small delay between discoveries
  };

  // Temperature Sensors - one per harness position.
  // Each carries its own availability topic on top of the board's, so a probe
  // that drops off the bus reads "unavailable" in Home Assistant instead of
  // freezing on its last value.
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    String presenceTopic = String(Topic_TempPresence_Prefix) + TEMP_ROLES[r].key;
    String topic = String("homeassistant/sensor/") + deviceId + "/" + TEMP_ROLES[r].objectId + "/config";
    String payload = "{\"name\":\"" + String(TEMP_ROLES[r].name) + "\","
                     "\"unique_id\":\"" + deviceId + "_" + TEMP_ROLES[r].objectId + "\","
                     "\"state_topic\":\"" + String(TEMP_ROLES[r].stateTopic) + "\","
                     "\"device_class\":\"temperature\","
                     "\"unit_of_measurement\":\"°F\","
                     "\"state_class\":\"measurement\","
                     "\"suggested_display_precision\":1,"
                     "\"icon\":\"" + String(TEMP_ROLES[r].icon) + "\","
                     "\"availability_mode\":\"all\","
                     "\"availability\":["
                       "{\"topic\":\"" + String(Topic_Plumbing_Status) + "\",\"payload_available\":\"ONLINE\",\"payload_not_available\":\"OFFLINE\"},"
                       "{\"topic\":\"" + presenceTopic + "\",\"payload_available\":\"ONLINE\",\"payload_not_available\":\"OFFLINE\"}"
                     "],"
                     "\"device\":" + device + "}";
    client.publish(topic.c_str(), payload.c_str(), true);
    delay(50);
  }

  publishDiscovery("sensor", "ambient_temp", "Ambient Temperature", Topic_Environment_Temp, "temperature", "°F");

  // Harness diagnostics
  publishDiscovery("sensor", "temp_sensors_found", "Temp Sensors Found", Topic_TempSensors_Found, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:counter", "diagnostic");
  publishDiscovery("sensor", "temp_sensors_live", "Temp Sensors Live", Topic_TempSensors_Live, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:check-network", "diagnostic");
  publishDiscovery("sensor", "temp_identify_state", "Temp Identify State", Topic_TempIdentify_State, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:hand-back-right", "diagnostic");

  // Identify buttons - press one, then warm that probe with your hand
  auto publishButton = [&](const char* objectId, const String& name, const char* commandTopic,
                           const String& payloadPress, const char* icon, bool configCategory = true) {
    String topic = String("homeassistant/button/") + deviceId + "/" + objectId + "/config";
    String payload = "{\"name\":\"" + name + "\","
                     "\"unique_id\":\"" + deviceId + "_" + objectId + "\","
                     "\"command_topic\":\"" + String(commandTopic) + "\","
                     "\"payload_press\":\"" + payloadPress + "\","
                     "\"icon\":\"" + String(icon) + "\","
                     + String(configCategory ? "\"entity_category\":\"config\"," : "") +
                     "\"device\":" + device + "}";
    client.publish(topic.c_str(), payload.c_str(), true);
    delay(50);
  };

  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    publishButton((String("identify_") + TEMP_ROLES[r].key).c_str(),
                  "Identify " + String(TEMP_ROLES[r].name),
                  Topic_TempIdentify_Command, String(TEMP_ROLES[r].key), "mdi:hand-back-right");
  }
  publishButton("identify_cancel", "Cancel Temp Identify", Topic_TempIdentify_Command, "CANCEL", "mdi:cancel");
  publishButton("temp_rescan", "Rescan Temp Bus", Topic_TempSensor_Command, "SCAN", "mdi:magnify-scan");
  publishButton("temp_report", "Print Temp Bus Report", Topic_TempSensor_Command, "REPORT", "mdi:table-search");
  publishButton("flow_cal_start", "Start Flow Calibration", Topic_Flow_Calibrate, "START", "mdi:ruler");
  publishButton("restart", "Restart Plumbing Board", Topic_Plumbing_Command, "RESTART", "mdi:restart");
  // The leak shutdown latches until cleared; this was MQTT-publish-only before
  publishButton("clear_leak_alarm", "Clear Leak Alarm", Topic_Clear_High_Humidity, "CLEAR", "mdi:water-check", false);

  // Environmental Sensors
  publishDiscovery("sensor", "humidity", "Humidity", Topic_Environment_Humidity, "humidity", "%");
  publishDiscovery("sensor", "pressure", "Atmospheric Pressure", Topic_Environment_Pressure, "pressure", "hPa");

  // Flow Sensor
  publishDiscovery("sensor", "flow_rate", "Hot Water Flow Rate", Topic_Flow_Rate, "volume_flow_rate", "L/min", nullptr, nullptr, nullptr, "mdi:water", nullptr, "measurement");
  publishDiscovery("sensor", "flow_rate_gpm", "Hot Water Flow Rate (GPM)", Topic_Flow_Rate_GPM, "volume_flow_rate", "gal/min", nullptr, nullptr, nullptr, "mdi:water", nullptr, "measurement");
  publishDiscovery("sensor", "flow_status", "Hot Water Flow Status", Topic_Flow_Status, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:water");
  // Session total - resets to 0 on reboot, which total_increasing handles
  publishDiscovery("sensor", "flow_total_gal", "Hot Water Used", Topic_Flow_Total_Gal, "water", "gal", nullptr, nullptr, nullptr, "mdi:water-pump", nullptr, "total_increasing");
  publishDiscovery("sensor", "flow_pulses", "Hot Water Flow Pulses", Topic_Flow_Pulses, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:pulse", "diagnostic", "total_increasing");
  publishDiscovery("sensor", "flow_kfactor", "Flow K-Factor", Topic_Flow_KFactor, nullptr, "pulses/L", nullptr, nullptr, nullptr, "mdi:tune", "diagnostic");
  publishDiscovery("sensor", "ip_address", "IP Address", Topic_Plumbing_IP, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:ip-network", "diagnostic");

  // Switches (Valves and Controls)
  publishDiscovery("switch", "fresh_water_heat", "Fresh Water Tank Heater", Topic_FreshWtrHeat_Status, nullptr, nullptr, Topic_FreshWtrHeat_Command, "ON", "OFF", "mdi:water-thermometer");
  publishDiscovery("switch", "grey_water_heat", "Grey Water Tank Heater", Topic_GreyWtrHeat_Status, nullptr, nullptr, Topic_GreyWtrHeat_Command, "ON", "OFF", "mdi:water-thermometer");
  publishDiscovery("switch", "rear_loop", "Rear Loop Valve", Topic_RearLoop_Status, nullptr, nullptr, Topic_RearLoop_Command, "OPEN", "CLOSE", "mdi:pipe-valve");
  publishDiscovery("switch", "engine_loop", "Engine Loop Valve", Topic_EngineLoop_Status, nullptr, nullptr, Topic_EngineLoop_Command, "OPEN", "CLOSE", "mdi:pipe-valve");
  publishDiscovery("sensor", "engine_loop_valve_state", "Engine Loop Valve State", Topic_EngineLoop_ValveState, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:valve");
  publishDiscovery("sensor", "engine_loop_valve_position", "Engine Loop Valve Position", Topic_EngineLoop_ValvePosition, nullptr, "%", nullptr, nullptr, nullptr, "mdi:valve", nullptr, "measurement");
  publishDiscovery("switch", "front_loop", "Front Loop Valve", Topic_FrontLoop_Status, nullptr, nullptr, Topic_FrontLoop_Command, "OPEN", "CLOSE", "mdi:pipe-valve");
  publishDiscovery("switch", "exhaust_fan", "Exhaust Fan", Topic_ExhaustFan_Status, nullptr, nullptr, Topic_ExhaustFan_Command, "ON", "OFF", "mdi:fan");
  publishDiscovery("switch", "main_pump", "Main Water Pump", Topic_MainPump_Status, nullptr, nullptr, Topic_MainPump_Command, "ON", "OFF", "mdi:pump");
  publishDiscovery("switch", "domestic_hw", "Domestic Hot Water", Topic_DomesticHW_Status, nullptr, nullptr, Topic_DomesticHW_Command, "OPEN", "CLOSE", "mdi:water-boiler");

  // Diesel Heater (simple on/off switch)
  publishDiscovery("switch", "diesel_heater", "Diesel Heater", Topic_DieselHtr_Status, nullptr, nullptr, Topic_DieselHtr_Command, "ON", "OFF", "mdi:fire");

  // ---- Retired entities ----
  // These were removed from the firmware, but their discovery configs and state
  // values are RETAINED on the broker, so Home Assistant keeps showing them
  // until something publishes an empty retained message over the top. That is
  // what this block does. It is idempotent, so it can stay indefinitely.
  const char* retiredDiscovery[][2] = {
    { "select",        "diesel_heater" },     // replaced by a plain switch
    { "sensor",        "pump_pressure" },     // PWM hot water pump, removed
    { "sensor",        "pump_speed" },        //   "
    { "sensor",        "pump_status" },       //   "
    { "binary_sensor", "pump_error" },        //   "
    { "sensor",        "manifold_pressure" },  // manifold transducer, removed
    { "sensor",        "boiler_exhaust_temp" }, // exhaust probe removed from the harness
    { "sensor",        "boiler_flame_status" }, // flame confirmation, its only input was the exhaust probe
    { "button",        "identify_boiler_exhaust" }
  };
  for (auto& e : retiredDiscovery) {
    String t = String("homeassistant/") + e[0] + "/" + deviceId + "/" + e[1] + "/config";
    client.publish(t.c_str(), "", true);
    delay(50);
  }

  // ...and the retained state topics they left behind
  const char* retiredTopics[] = {
    "HotWaterPumpPressure", "HotWaterPumpSpeed", "HotWaterPumpStatus",
    "HotWaterPumpCommand", "HotWaterPumpSetPoint", "HotWaterPumpRawADC",
    "HotWaterPumpVoltage", "HotWaterPumpMode", "AlertPumpError",
    "ManifoldPressure",
    "DieselBoilerExhaustTemp", "DieselBoilerFlameStatus",
    "PlumbingTempPresence/boiler_exhaust"
  };
  for (auto& t : retiredTopics) {
    client.publish(t, "", true);
    delay(20);
  }

  // Electric Heater (simple on/off switch)
  publishDiscovery("switch", "electric_heater", "Electric Heater", Topic_ElectricHtr_Status, nullptr, nullptr, Topic_ElectricHtr_Command, "ON", "OFF", "mdi:radiator");

  // Alerts
  publishDiscovery("sensor", "plumbing_alerts", "Plumbing Alerts", Topic_Alert, nullptr, nullptr, nullptr, nullptr, nullptr, "mdi:alert-circle");

  // Cabinet Lock
  publishDiscovery("lock", "cabinet_lock", "Rear Pass Cabinet Lock", Topic_CabLock_Status, nullptr, nullptr, Topic_CabLock_Command, "LOCK", "UNLOCK", "mdi:lock");

  // Exhaust Fan Temperature Setpoint (number entity)
  String fanSetTopic = String("homeassistant/number/") + deviceId + "/exhaust_fan_temp_set/config";
  String fanSetPayload = "{\"name\":\"Exhaust Fan Temperature Setpoint\","
                        "\"unique_id\":\"" + deviceId + "_exhaust_fan_temp_set\","
                        "\"state_topic\":\"" + String(Topic_ExhaustFan_TempSet) + "\","
                        "\"command_topic\":\"" + String(Topic_ExhaustFan_TempSetCmd) + "\","
                        "\"min\":32.0,"
                        "\"max\":140.0,"
                        "\"step\":1.0,"
                        "\"unit_of_measurement\":\"°F\","
                        "\"icon\":\"mdi:fan\","
                        "\"device\":" + device + "}";
  client.publish(fanSetTopic.c_str(), fanSetPayload.c_str(), true);
  delay(50);

  // Plumbing Mode (select: AUTO / MANUAL)
  String modeTopic = String("homeassistant/select/") + deviceId + "/plumbing_mode/config";
  String modePayload = "{\"name\":\"Plumbing Mode\","
                     "\"unique_id\":\"" + deviceId + "_plumbing_mode\","
                     "\"state_topic\":\"" + String(Topic_Plumbing_Mode) + "\","
                     "\"command_topic\":\"" + String(Topic_Plumbing_ModeCmd) + "\","
                     "\"options\":[\"AUTO\",\"MANUAL\"],"
                     "\"icon\":\"mdi:cog\","
                     "\"device\":" + device + "}";
  client.publish(modeTopic.c_str(), modePayload.c_str(), true);
  delay(50);

  // Fresh Water Heater Temperature Setpoint
  String freshTempTopic = String("homeassistant/number/") + deviceId + "/fresh_water_temp_set/config";
  String freshTempPayload = "{\"name\":\"Fresh Water Heater Setpoint\","
                            "\"unique_id\":\"" + deviceId + "_fresh_water_temp_set\","
                            "\"state_topic\":\"" + String(Topic_FreshWtrHeat_TempSet) + "\","
                            "\"command_topic\":\"" + String(Topic_FreshWtrHeat_TempSetCmd) + "\","
                            "\"min\":32.0,"
                            "\"max\":120.0,"
                            "\"step\":1.0,"
                            "\"unit_of_measurement\":\"°F\","
                            "\"icon\":\"mdi:thermometer\","
                            "\"device\":" + device + "}";
  client.publish(freshTempTopic.c_str(), freshTempPayload.c_str(), true);
  delay(50);

  // Grey Water Heater Temperature Setpoint
  String greyTempTopic = String("homeassistant/number/") + deviceId + "/grey_water_temp_set/config";
  String greyTempPayload = "{\"name\":\"Grey Water Heater Setpoint\","
                           "\"unique_id\":\"" + deviceId + "_grey_water_temp_set\","
                           "\"state_topic\":\"" + String(Topic_GreyWtrHeat_TempSet) + "\","
                           "\"command_topic\":\"" + String(Topic_GreyWtrHeat_TempSetCmd) + "\","
                           "\"min\":32.0,"
                           "\"max\":120.0,"
                           "\"step\":1.0,"
                           "\"unit_of_measurement\":\"°F\","
                           "\"icon\":\"mdi:thermometer\","
                           "\"device\":" + device + "}";
  client.publish(greyTempTopic.c_str(), greyTempPayload.c_str(), true);
  delay(50);

  Serial.println("✓ Home Assistant Discovery published for Plumbing System");
}

// ============================================================================
// MQTT CONNECTION
// ============================================================================
void reconnectMQTT() {
  static unsigned long lastAttempt = 0;
  static int mqttFailCount = 0;

  // Limit reconnect attempts frequency
  if (millis() - lastAttempt < 5000) return;
  lastAttempt = millis();

  // Ensure WiFi is up (short timeout)
  if (!ensureWifiConnected(4000)) {
    Serial.println("MQTT reconnect skipped - WiFi down");
    mqttFailCount++;
    // If WiFi/MQTT failing repeatedly, consider resetting
    if (mqttFailCount >= 8) {
      Serial.println("Repeated network failures - restarting device");
      delay(200);
      ESP.restart();
    }
    return;
  }

  if (client.connected()) {
    mqttFailCount = 0; // success
    return;
  }

  Serial.print("Connecting to MQTT...");
  // Use existing credentials (empty strings accepted)
  // Set Last Will message so broker marks us OFFLINE on unexpected disconnect
  if (client.connect(mqtt_client_id, mqtt_user, mqtt_password, Topic_Plumbing_Status, 1, true, "OFFLINE")) {
    Serial.println(" connected!");
    mqttFailCount = 0;
    // Announce ONLINE (retained) so Home Assistant and other systems see us
    client.publish(Topic_Plumbing_Status, "ONLINE", true);
    publishHomeAssistantDiscovery();
    subscribeToTopics();

    // Publish board's authoritative states to broker
    publishAllStatesToBroker();

    // Publish plumbing mode and current setpoints so HA sees them
    client.publish(Topic_Plumbing_Mode, plumbingMode.c_str(), true);
    char fwStr[8]; dtostrf(freshWaterTempTarget, 4, 1, fwStr); client.publish(Topic_FreshWtrHeat_TempSet, fwStr, true);
    char gwStr[8]; dtostrf(greyWaterTempTarget, 4, 1, gwStr); client.publish(Topic_GreyWtrHeat_TempSet, gwStr, true);
    char fanStr[8]; dtostrf(exhaustFanTempSet, 4, 1, fanStr); client.publish(Topic_ExhaustFan_TempSet, fanStr, true);

    publishHealth();
    publishStatus();
  } else {
    Serial.print(" failed, rc=");
    Serial.println(client.state());
    mqttFailCount++;
  }
}

void subscribeToTopics() {
  // Valve commands
  client.subscribe(Topic_FreshWtrHeat_Command);
  client.subscribe(Topic_GreyWtrHeat_Command);
  client.subscribe(Topic_ExhaustFan_Command);
  client.subscribe(Topic_RearLoop_Command);
  client.subscribe(Topic_EngineLoop_Command);
  client.subscribe(Topic_FrontLoop_Command);
  client.subscribe(Topic_DieselHtr_Command);
  client.subscribe(Topic_ElectricHtr_Command);

  // Global mode
  client.subscribe(Topic_Plumbing_ModeCmd);

  // Configuration - Subscribe to COMMAND topics (separate from state topics)
  // This prevents the retained message loop while allowing HA to send commands
  client.subscribe(Topic_FreshWtrHeat_TempSetCmd);
  client.subscribe(Topic_GreyWtrHeat_TempSetCmd);
  client.subscribe(Topic_ExhaustFan_TempSetCmd);

  // Cabinet lock
  client.subscribe(Topic_CabLock_Command);
  client.subscribe(Topic_AllLock_Command);

  // Main water pump and domestic HW loop
  client.subscribe(Topic_MainPump_Command);
  client.subscribe(Topic_DomesticHW_Command);

  // Safety commands
  client.subscribe(Topic_Clear_High_Humidity);

  // Flow meter calibration
  client.subscribe(Topic_Flow_Calibrate);

  // Board maintenance
  client.subscribe(Topic_Plumbing_Command);

  // DS18B20 harness management
  client.subscribe(Topic_TempSensor_Command);
  client.subscribe(Topic_TempAssign_Command);
  client.subscribe(Topic_TempIdentify_Command);

  Serial.println("✓ Subscribed to command topics");
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

  String topicStr = String(topic);

  // ---- DS18B20 harness management ----
  if (topicStr == Topic_TempSensor_Command) {
    String cmd = message;
    cmd.toUpperCase();
    if (cmd == "SCAN") {
      tempRescanLast = millis();
      rescanTempBus();
      printTempBusReport();
      publishTempScan();
      publishTempCounts(true);
    } else if (cmd == "REPORT") {
      printTempBusReport();
      publishTempScan();
      publishTempCounts(true);
    } else if (cmd == "CLEAR") {
      for (int r = 0; r < TEMP_ROLE_COUNT; r++) clearTempRole(r, false);
      saveTempAssignments();
      mapTempRolesToBus();
      publishTempCounts(true);
      publishTempScan();
      Serial.println("🌡 All harness positions cleared");
    } else {
      Serial.println("⚠ Unknown temp command (SCAN|REPORT|CLEAR)");
    }
    return;
  }

  // "<role>=<ROM hex>" binds by address, "<role>=<n>" by bus index,
  // "<role>=" clears the position.
  if (topicStr == Topic_TempAssign_Command) {
    int eq = message.indexOf('=');
    if (eq < 0) {
      Serial.println("⚠ Assign payload must be <role>=<addr|index>");
      return;
    }
    int r = tempRoleIndexFromKey(message.substring(0, eq));
    String value = message.substring(eq + 1);
    value.trim();

    if (r < 0) {
      Serial.println("⚠ Unknown harness position");
      return;
    }
    if (value.length() == 0) {
      clearTempRole(r, true);
      mapTempRolesToBus();
      publishTempCounts(true);
      publishTempScan();
      return;
    }

    DeviceAddress addr;
    if (tempHexToAddr(value, addr)) {
      assignTempRole(r, addr, true);
      return;
    }

    bool numeric = value.length() > 0;
    for (unsigned int i = 0; i < value.length(); i++) {
      if (!isDigit(value.charAt(i))) { numeric = false; break; }
    }
    int idx = value.toInt();
    if (numeric && idx >= 0 && idx < tempBusCount) {
      assignTempRole(r, tempBus[idx].addr, true);
    } else {
      Serial.printf("⚠ '%s' is neither a 16-char ROM address nor a bus index 0-%d\n",
                    value.c_str(), tempBusCount - 1);
    }
    return;
  }

  if (topicStr == Topic_Plumbing_Command) {
    String cmd = message;
    cmd.toUpperCase();
    if (cmd == "RESTART" || cmd == "REBOOT") {
      Serial.println("🔄 Restart requested over MQTT");
      client.publish(Topic_Plumbing_Status, "OFFLINE", true);
      delay(200);
      ESP.restart();
    } else {
      Serial.println("⚠ Unknown PlumbingCommand (RESTART)");
    }
    return;
  }

  if (topicStr == Topic_Flow_Calibrate) {
    handleFlowCalibration(message);
    return;
  }

  if (topicStr == Topic_TempIdentify_Command) {
    if (message.equalsIgnoreCase("CANCEL")) {
      if (tempIdentifyActive) {
        tempIdentifyActive = false;
        tempIdentifyRole = -1;
        Serial.println("🖐 Identify cancelled");
      }
      setTempIdentifyState("IDLE");
      return;
    }
    int r = tempRoleIndexFromKey(message);
    if (r < 0) {
      Serial.println("⚠ Unknown harness position for identify");
      return;
    }
    startTempIdentify(r);
    return;
  }

  // Cabinet lock commands
  if (topicStr == Topic_CabLock_Command || topicStr == Topic_AllLock_Command) {
    processLockCommand(message);
  }
  // Fresh water heater
  else if (topicStr == Topic_FreshWtrHeat_Command) {
    controlFreshWaterHeater(message == "ON");
  }
  else if (topicStr == Topic_FreshWtrHeat_TempSetCmd) {
    float newTemp = message.toFloat();
    // Validate temperature range
    if (newTemp >= 32.0 && newTemp <= 120.0) {
      freshWaterTempTarget = newTemp;
      saveConfigToPreferences();
      // Publish new state to state topic
      char tempStr[8];
      dtostrf(freshWaterTempTarget, 4, 1, tempStr);
      client.publish(Topic_FreshWtrHeat_TempSet, tempStr, true);
      Serial.print("🌡️ Fresh Water setpoint updated: ");
      Serial.print(freshWaterTempTarget);
      Serial.println(" °F");
    }
  }
  // Grey water heater
  else if (topicStr == Topic_GreyWtrHeat_Command) {
    controlGreyWaterHeater(message == "ON");
  }
  else if (topicStr == Topic_GreyWtrHeat_TempSetCmd) {
    float newTemp = message.toFloat();
    // Validate temperature range
    if (newTemp >= 32.0 && newTemp <= 120.0) {
      greyWaterTempTarget = newTemp;
      saveConfigToPreferences();
      // Publish new state to state topic
      char tempStr[8];
      dtostrf(greyWaterTempTarget, 4, 1, tempStr);
      client.publish(Topic_GreyWtrHeat_TempSet, tempStr, true);
      Serial.print("🌡️ Grey Water setpoint updated: ");
      Serial.print(greyWaterTempTarget);
      Serial.println(" °F");
    }
  }
  // Exhaust fan
  else if (topicStr == Topic_ExhaustFan_Command) {
    controlExhaustFan(message == "ON");
  }
  else if (topicStr == Topic_ExhaustFan_TempSetCmd) {
    float newTemp = message.toFloat();
    // Validate temperature range
    if (newTemp >= 32.0 && newTemp <= 140.0) {
      exhaustFanTempSet = newTemp;
      saveConfigToPreferences();
      // Publish new state to state topic
      char tempStr[8];
      dtostrf(exhaustFanTempSet, 4, 1, tempStr);
      client.publish(Topic_ExhaustFan_TempSet, tempStr, true);
      Serial.print("🌬️ Exhaust fan setpoint updated: ");
      Serial.print(exhaustFanTempSet);
      Serial.println(" °F");
    }
  }
  // Valves
  else if (topicStr == Topic_RearLoop_Command) {
    controlRearLoopValve(message == "ON" || message == "OPEN");
  }
  else if (topicStr == Topic_EngineLoop_Command) {
    // Store MQTT request and update valve
    bool newState = (message == "ON" || message == "OPEN");
    if (engineLoopMqttRequest != newState) {
      engineLoopMqttRequest = newState;
      preferences.putBool("engLoopMqtt", engineLoopMqttRequest);  // Persist to EEPROM
      Serial.print("Engine Loop MQTT: ");
      Serial.print(newState ? "OPEN" : "CLOSE");
      Serial.println(" (saved)");
    }
    controlEngineLoopValve(newState);
  }
  else if (topicStr == Topic_FrontLoop_Command) {
    // Store MQTT request and update valve
    bool newState = (message == "ON" || message == "OPEN");
    if (frontLoopMqttRequest != newState) {
      frontLoopMqttRequest = newState;
      preferences.putBool("frtLoopMqtt", frontLoopMqttRequest);  // Persist to EEPROM
      Serial.print("Front Loop MQTT: ");
      Serial.print(newState ? "OPEN" : "CLOSE");
      Serial.println(" (saved)");
    }
    controlFrontLoopValve(newState);
  }
  // Diesel heater (simple on/off)
  else if (topicStr == Topic_DieselHtr_Command) {
    controlDieselHeater(message == "ON");
  }
  // Electric heater
  else if (topicStr == Topic_ElectricHtr_Command) {
    controlElectricHeater(message == "ON");
  }
  // Main water pump
  else if (topicStr == Topic_MainPump_Command) {
    // Accept START/STOP primarily (header comment). Allow ON/OFF as backward compat.
    if (message == "START" || message == "ON") controlMainWaterPump(true);
    else if (message == "STOP" || message == "OFF") controlMainWaterPump(false);
  }
  // Domestic hot water loop
  else if (topicStr == Topic_DomesticHW_Command) {
    if (message == "OPEN") controlDHWSolinoid(true);
    else if (message == "CLOSE") controlDHWSolinoid(false);
  }
  // Safety commands
  else if (topicStr == Topic_Clear_High_Humidity) {
    if (message == "clear" || message == "CLEAR" || message == "true" || message == "TRUE") {
      if (highHumidityAlarmActive) {
        Serial.println("🔓 High humidity alarm MANUALLY CLEARED");
        highHumidityAlarmActive = false;
        humiditySpikePending = false;  // Also clear any pending spike detection
        baselineHumidity = -999.0;
        refreshAlertPublish();
        // Alert state refreshed
        // Reset humidity baseline
        lastHumidity = environmentHumidity;
        lastHumidityCheck = millis();
      }
    }
  }

  // Global plumbing mode (AUTO / MANUAL)
  else if (topicStr == Topic_Plumbing_ModeCmd) {
    if (message == "AUTO" || message == "MANUAL") {
      plumbingMode = message;
      saveConfigToPreferences();
      // Publish new state to state topic
      client.publish(Topic_Plumbing_Mode, plumbingMode.c_str(), true);
      Serial.print("⚙️ Plumbing mode set to: ");
      Serial.println(plumbingMode);
    } else {
      Serial.println("⚠️ Invalid Plumbing Mode (use AUTO or MANUAL)");
    }
  }
}

// ============================================================================
// FLOW SENSOR
// ============================================================================
void IRAM_ATTR flowSensorISR() {
  flowPulseTotal++;
  lastPulseTime = millis();
}

// ----------------------------------------------------------------------------
// Flow presence state machine (50ms tick)
// ----------------------------------------------------------------------------
// IDLE -> STARTING -> ACTIVE -> STOPPING -> IDLE
//
// STARTING and STOPPING are the hysteresis. A burst that does not last
// FLOW_ON_DELAY_MS never opens the valve; a momentary dropout mid-shower does
// not close it. The solenoid is only touched on the two committed transitions.
void evaluateFlowState() {
  noInterrupts();
  unsigned long total = flowPulseTotal;
  unsigned long lastPulse = lastPulseTime;
  interrupts();

  unsigned long now = millis();

  // Hair-trigger presence: any pulse keeps this true until the line goes quiet
  bool present = (lastPulse != 0) && ((now - lastPulse) < FLOW_PULSE_IDLE_MS);

  switch (flowState) {
    case FLOW_IDLE:
      // Pulses arriving right after we shut the valve are our own water hammer,
      // not somebody opening a tap. Do not even start the on-delay for them.
      if (present && (flowValveClosedAt == 0 || (now - flowValveClosedAt) >= FLOW_CLOSE_LOCKOUT_MS)) {
        flowState = FLOW_STARTING;
        flowStateSince = now;
        flowStartPulses = total;
      }
      break;

    case FLOW_STARTING:
      if (!present) {
        // Did not sustain - a transient, most likely the solenoid's own
        // water hammer. Valve stays shut.
        flowState = FLOW_IDLE;
      } else if (now - flowStateSince >= FLOW_ON_DELAY_MS) {
        if ((total - flowStartPulses) >= FLOW_MIN_PULSES_OPEN) {
          flowState = FLOW_ACTIVE;
          flowDetected = true;
          Serial.printf("💦 Flow confirmed after %lums (%lu pulses) - opening DHW valve\n",
                        now - flowStateSince, total - flowStartPulses);
          controlDHWSolinoid(true);
          publishFlowRate(true);
        } else {
          // Presence held but almost nothing actually turned - electrical noise
          flowState = FLOW_IDLE;
        }
      }
      break;

    case FLOW_ACTIVE:
      if (!present) {
        flowState = FLOW_STOPPING;
        flowStateSince = now;
      }
      break;

    case FLOW_STOPPING:
      if (present) {
        flowState = FLOW_ACTIVE;   // Brief dropout - hold the valve open
      } else if (now - flowStateSince >= FLOW_OFF_DELAY_MS) {
        flowState = FLOW_IDLE;
        flowDetected = false;
        flowRate = 0.0;   // The line has been quiet for FLOW_OFF_DELAY_MS
        flowValveClosedAt = now;
        Serial.println("💦 Flow stopped - closing DHW valve");
        controlDHWSolinoid(false);
        publishFlowRate(true);
      }
      break;
  }
}

// ----------------------------------------------------------------------------
// Rate and totals
// ----------------------------------------------------------------------------
void calculateFlowRate() {
  noInterrupts();
  unsigned long total = flowPulseTotal;
  interrupts();

  unsigned long delta = total - lastRatePulseTotal;
  lastRatePulseTotal = total;

  unsigned long currentTime = millis();
  unsigned long interval = currentTime - lastFlowCalc;
  lastFlowCalc = currentTime;

  // pulses / (pulses per litre) = litres; x intervals-per-minute = L/min
  if (interval > 0 && flowKFactor > 0.0) {
    flowRate = (delta / flowKFactor) * (60000.0 / interval);
  }
}

void publishFlowRate(bool force) {
  if (!client.connected()) return;

  // Publishing every second around the clock is mostly zeroes. Send on change
  // while water is moving, and a heartbeat every 30s when it is not.
  static unsigned long lastPublish = 0;
  static float lastRate = -1.0;
  static bool lastStatus = false;
  unsigned long now = millis();

  bool changed = (fabs(flowRate - lastRate) >= 0.01) || (flowDetected != lastStatus);
  if (!force && !changed && (now - lastPublish) < 30000) return;

  noInterrupts();
  unsigned long total = flowPulseTotal;
  interrupts();

  float litres = (flowKFactor > 0.0) ? (total / flowKFactor) : 0.0;

  char buf[12];
  dtostrf(flowRate, 4, 2, buf);
  client.publish(Topic_Flow_Rate, buf);

  dtostrf(flowRate / LITRES_PER_US_GALLON, 4, 2, buf);
  client.publish(Topic_Flow_Rate_GPM, buf);

  dtostrf(litres, 4, 2, buf);
  client.publish(Topic_Flow_Total, buf);

  dtostrf(litres / LITRES_PER_US_GALLON, 4, 2, buf);
  client.publish(Topic_Flow_Total_Gal, buf);

  snprintf(buf, sizeof(buf), "%lu", total);
  client.publish(Topic_Flow_Pulses, buf);

  client.publish(Topic_Flow_Status, flowDetected ? "FLOWING" : "STOPPED");

  lastRate = flowRate;
  lastStatus = flowDetected;
  lastPublish = now;
}

// ----------------------------------------------------------------------------
// Bucket-test calibration
// ----------------------------------------------------------------------------
// The L/min conversion itself is dimensionally correct, so a reading that is
// "way off" means the pulses-per-litre constant does not match the turbine
// fitted. Rather than guess a part number, measure it:
//
//   1. publish START to HotWaterFlowCalibrateCommand
//   2. run water into a measured container
//   3. publish the volume that came out, e.g. "1G" or "2.5L"
//
// K = pulses counted / litres delivered, saved to NVS.
void handleFlowCalibration(const String& payload) {
  String v = payload;
  v.trim();
  v.toUpperCase();

  noInterrupts();
  unsigned long total = flowPulseTotal;
  interrupts();

  if (v == "START" || v == "RESET") {
    flowCalActive = true;
    flowCalStartPulses = total;
    Serial.println("📏 Flow calibration started - run a measured volume, then publish it");
    return;
  }

  if (!flowCalActive) {
    Serial.println("⚠ Publish START to HotWaterFlowCalibrateCommand before the volume");
    return;
  }

  // Trailing G = US gallons, L or bare = litres
  float litres;
  if (v.endsWith("G")) {
    litres = v.substring(0, v.length() - 1).toFloat() * LITRES_PER_US_GALLON;
  } else if (v.endsWith("L")) {
    litres = v.substring(0, v.length() - 1).toFloat();
  } else {
    litres = v.toFloat();
  }

  unsigned long pulses = total - flowCalStartPulses;

  if (litres <= 0.0) {
    Serial.println("⚠ Calibration rejected - volume must be greater than zero");
    return;
  }
  if (pulses < 100) {
    Serial.printf("⚠ Calibration rejected - only %lu pulses counted, run more water\n", pulses);
    return;
  }

  float newK = pulses / litres;
  if (newK < FLOW_K_MIN || newK > FLOW_K_MAX) {
    Serial.printf("⚠ Calibration rejected - %.1f pulses/L is outside %.0f-%.0f\n",
                  newK, FLOW_K_MIN, FLOW_K_MAX);
    return;
  }

  flowKFactor = newK;
  flowCalActive = false;
  saveConfigToPreferences();

  Serial.printf("📏 Calibrated: %lu pulses over %.3f L -> %.1f pulses/L\n",
                pulses, litres, flowKFactor);

  char buf[12];
  dtostrf(flowKFactor, 6, 1, buf);
  client.publish(Topic_Flow_KFactor, buf, true);
  publishFlowRate(true);
}

// ============================================================================
// TEMPERATURE SENSORS
// ============================================================================
// Helper function to calculate moving average
float calculateTempAverage(float* buffer) {
  float sum = 0.0;
  int validCount = 0;

  for (int i = 0; i < TEMP_FILTER_SIZE; i++) {
    if (buffer[i] > -999.0) {  // Only count valid readings
      sum += buffer[i];
      validCount++;
    }
  }

  if (validCount == 0) return -999.0;
  return sum / validCount;
}

// ----------------------------------------------------------------------------
// Address helpers
// ----------------------------------------------------------------------------
void tempAddrToHex(const DeviceAddress addr, char* out) {
  for (uint8_t i = 0; i < 8; i++) sprintf(out + (i * 2), "%02X", addr[i]);
  out[16] = '\0';
}

// Accepts "28FF641E1234ABCD", with or without a 0x prefix, any case.
bool tempHexToAddr(const String& hex, DeviceAddress out) {
  String v = hex;
  v.trim();
  v.toUpperCase();
  if (v.startsWith("0X")) v = v.substring(2);
  v.replace(":", "");
  v.replace("-", "");
  if (v.length() != 16) return false;

  for (uint8_t i = 0; i < 8; i++) {
    char hi = v.charAt(i * 2);
    char lo = v.charAt(i * 2 + 1);
    if (!isHexadecimalDigit(hi) || !isHexadecimalDigit(lo)) return false;
    out[i] = (uint8_t)strtoul(v.substring(i * 2, i * 2 + 2).c_str(), NULL, 16);
  }
  return true;
}

static bool tempAddrEqual(const DeviceAddress a, const DeviceAddress b) {
  return memcmp(a, b, 8) == 0;
}

static bool tempAddrIsZero(const DeviceAddress a) {
  for (uint8_t i = 0; i < 8; i++) if (a[i] != 0) return false;
  return true;
}

int tempRoleIndexFromKey(const String& key) {
  String k = key;
  k.trim();
  k.toLowerCase();
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    if (k == TEMP_ROLES[r].key) return r;
  }
  return -1;
}

// ----------------------------------------------------------------------------
// Bus enumeration
// ----------------------------------------------------------------------------
// Searched directly rather than through DallasTemperature so CRC failures and
// non-temperature devices are visible instead of being silently dropped.
void rescanTempBus() {
  TempBusSensor previous[TEMP_MAX_BUS_SENSORS];
  int previousCount = tempBusCount;
  memcpy(previous, tempBus, sizeof(tempBus));

  tempBusCount = 0;
  memset(tempBus, 0, sizeof(tempBus));

  // Let DallasTemperature do its own self-contained enumeration first; it
  // refreshes the internal device count used elsewhere in the library.
  tempSensors.begin();

  DeviceAddress addr;
  int crcFailures = 0;
  int duplicates = 0;
  int iterations = 0;

  // NOTHING may touch the bus between search() calls. The search walks a binary
  // tree using state held inside the OneWire object, and any call that issues
  // reset_search() sends it back to the top - it then re-finds the same devices
  // forever. DallasTemperature::setResolution() does exactly that internally
  // whenever more than one device is present, so resolution is configured in a
  // second pass below rather than inside this loop.
  tempSensorWire.reset_search();
  while (tempSensorWire.search(addr)) {
    if (++iterations > TEMP_MAX_BUS_SENSORS * 4) {
      Serial.println("⚠ OneWire search did not terminate - aborting scan");
      break;
    }
    if (OneWire::crc8(addr, 7) != addr[7]) {
      crcFailures++;
      continue;
    }

    bool duplicate = false;
    for (int i = 0; i < tempBusCount; i++) {
      if (tempAddrEqual(tempBus[i].addr, addr)) { duplicate = true; break; }
    }
    if (duplicate) { duplicates++; continue; }

    if (tempBusCount >= TEMP_MAX_BUS_SENSORS) {
      Serial.printf("⚠ More than %d distinct devices on the bus - extras ignored\n",
                    TEMP_MAX_BUS_SENSORS);
      break;
    }

    TempBusSensor* b = &tempBus[tempBusCount];
    memcpy(b->addr, addr, 8);
    tempAddrToHex(b->addr, b->hex);
    // 0x28 DS18B20, 0x22 DS1822, 0x10 DS18S20
    b->isDS18B20 = (addr[0] == 0x28 || addr[0] == 0x22 || addr[0] == 0x10);
    b->role = -1;
    b->valid = false;
    b->tempF = -999.0;
    b->baselineF = -999.0;   // Never let a freshly seen sensor win an Identify

    for (int p = 0; p < previousCount; p++) {
      if (tempAddrEqual(previous[p].addr, b->addr)) {
        b->tempF     = previous[p].tempF;
        b->valid     = previous[p].valid;
        b->baselineF = previous[p].baselineF;
        b->reads     = previous[p].reads;
        b->errors    = previous[p].errors;
        b->lastErr   = previous[p].lastErr;
        memcpy(b->scratch, previous[p].scratch, sizeof(b->scratch));
        break;
      }
    }
    tempBusCount++;
  }

  // Second pass - safe now that the search has finished. The `true` skips
  // DallasTemperature's global bit-resolution recalculation, which is the part
  // that calls reset_search().
  for (int i = 0; i < tempBusCount; i++) {
    if (tempBus[i].isDS18B20) {
      tempSensors.setResolution(tempBus[i].addr, 12, true);
    } else {
      Serial.printf("⚠ Non-temperature device on bus: %s (family 0x%02X)\n",
                    tempBus[i].hex, tempBus[i].addr[0]);
    }
  }

  if (crcFailures > 0) {
    Serial.printf("⚠ %d CRC failure(s) during search - check pullup, wire length, splices\n",
                  crcFailures);
  }
  if (duplicates > 0) {
    Serial.printf("⚠ Search returned %d repeat address(es) - marginal bus, %d distinct kept\n",
                  duplicates, tempBusCount);
  }
  if (tempBusCount != previousCount) {
    Serial.printf("🌡 OneWire device count: %d -> %d\n", previousCount, tempBusCount);
  }

  mapTempRolesToBus();
}

// Bind each stored position address to whatever is on the bus right now.
void mapTempRolesToBus() {
  // Drop every existing tag first - this runs after assign/clear too, where
  // tempBus[] still carries tags from the previous mapping. Leaving a stale tag
  // would let a reassigned position read the old sensor.
  for (int i = 0; i < tempBusCount; i++) tempBus[i].role = -1;

  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    tempRoleState[r].present = false;
    if (!tempRoleState[r].assigned) continue;

    for (int i = 0; i < tempBusCount; i++) {
      if (tempAddrEqual(tempBus[i].addr, tempRoleState[r].addr)) {
        tempBus[i].role = r;
        tempRoleState[r].present = true;
        break;
      }
    }
    if (!tempRoleState[r].present) tempRoleState[r].valid = false;
  }
}

// Re-enumerating costs ~100ms of bus traffic, so it only runs when something is
// actually wrong. With every position bound and reading, this never fires.
bool tempBusNeedsRescan() {
  if (tempBusCount == 0) return true;
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    if (!tempRoleState[r].assigned || !tempRoleState[r].present) return true;
  }
  return false;
}

// ----------------------------------------------------------------------------
// Reading
// ----------------------------------------------------------------------------
// Reads one probe's scratchpad directly so a failure can be classified.
// Same MATCH ROM + READ SCRATCHPAD sequence DallasTemperature uses, and the same
// raw-to-C conversion for the DS18B20/DS1822 format.
uint8_t readTempProbe(TempBusSensor* b, float& outC) {
  uint8_t* sp = b->scratch;
  memset(sp, 0, sizeof(b->scratch));

  if (b->addr[0] == 0x10) {
    // DS18S20 uses a different temperature format - leave it to the library
    float c = tempSensors.getTempC(b->addr);
    if (c == DEVICE_DISCONNECTED_C) return TERR_CRC;
    outC = c;
    return TERR_NONE;
  }

  if (!tempSensorWire.reset()) return TERR_NO_PRESENCE;
  tempSensorWire.select(b->addr);
  tempSensorWire.write(0xBE);   // READ SCRATCHPAD
  for (uint8_t i = 0; i < 9; i++) sp[i] = tempSensorWire.read();
  tempSensorWire.reset();

  bool allZero = true, allOnes = true;
  for (uint8_t i = 0; i < 9; i++) {
    if (sp[i] != 0x00) allZero = false;
    if (sp[i] != 0xFF) allOnes = false;
  }
  if (allOnes) return TERR_NO_REPLY;
  if (allZero) return TERR_ALL_ZERO;
  if (OneWire::crc8(sp, 8) != sp[8]) return TERR_CRC;

  int16_t raw = (int16_t)(((uint16_t)sp[1] << 8) | sp[0]);
  // 0x0550 (85 C) with the reserved byte still at its reset value 0x0C is the
  // power-on default - the conversion never ran. Not a temperature.
  if (raw == 0x0550 && sp[6] == 0x0C) return TERR_POWER_ON;

  float c = raw / 16.0f;
  float f = c * 9.0f / 5.0f + 32.0f;
  if (f < TEMP_VALID_MIN_F || f > TEMP_VALID_MAX_F) return TERR_RANGE;

  outC = c;
  return TERR_NONE;
}

void collectTemperatures() {
  bool anyFailed = false;

  for (int i = 0; i < tempBusCount; i++) {
    TempBusSensor* b = &tempBus[i];
    if (!b->isDS18B20) continue;

    // One retry: a single corrupted read on a long bus should not drop a probe
    float c = 0.0f;
    uint8_t err = TERR_NONE;
    for (int attempt = 0; attempt < 2; attempt++) {
      b->reads++;
      err = readTempProbe(b, c);
      if (err == TERR_NONE) break;
      b->errors++;
    }

    if (err != TERR_NONE && b->lastErr != err) {
      Serial.printf("⚠ Probe %s read failed: %s (%lu/%lu failed)\n", b->hex,
                    TEMP_ERR_NAMES[err], (unsigned long)b->errors, (unsigned long)b->reads);
    }
    b->lastErr = err;

    if (err != TERR_NONE) {
      b->valid = false;
      anyFailed = true;
    } else {
      b->valid = true;
      b->tempF = (c * 9.0f / 5.0f) + 32.0f;
    }
  }

  // Keep the retained bus map current while something is failing, so the
  // failure reasons can be read from Home Assistant without a serial cable.
  static unsigned long lastFaultScanPublish = 0;
  if (anyFailed && (lastFaultScanPublish == 0 || millis() - lastFaultScanPublish >= 60000)) {
    lastFaultScanPublish = millis();
    publishTempScan();
  }

  // Push each bound sensor through to the global its position drives. Positions
  // with no live probe are forced to -999.0, which every automation below
  // already treats as "no data".
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    bool live = false;
    float reading = -999.0;

    if (tempRoleState[r].assigned && tempRoleState[r].present) {
      for (int i = 0; i < tempBusCount; i++) {
        if (tempBus[i].role != r) continue;
        if (tempBus[i].valid) {
          live = true;
          reading = tempBus[i].tempF;
        }
        break;
      }
    }

    tempRoleState[r].valid = live;

    if (!live) {
      *TEMP_ROLES[r].value = -999.0;
      if (tempRoleState[r].missingSince == 0) tempRoleState[r].missingSince = millis();
    } else {
      tempRoleState[r].missingSince = 0;
      if (TEMP_ROLES[r].filtered) {
        tempFilterBuf[r][tempFilterIdx[r]] = reading;
        tempFilterIdx[r] = (tempFilterIdx[r] + 1) % TEMP_FILTER_SIZE;
        if (tempFilterFill[r] < TEMP_FILTER_SIZE) tempFilterFill[r]++;
        // Use raw readings until the buffer has filled
        if (tempFilterFill[r] >= TEMP_FILTER_SIZE) {
          reading = calculateTempAverage(tempFilterBuf[r]);
        }
      }
      *TEMP_ROLES[r].value = reading;
    }

    publishTempPresence(r, false);
  }

  handleTempIdentify();
}

void readTemperatures() {
  // Collect the conversion started on the previous cycle (non-blocking)
  if (tempConversionPending) {
    tempConversionPending = false;
    collectTemperatures();
  } else if (tempBusCount == 0) {
    // Nothing on the bus means no conversion was ever started, but every
    // position still has to be marked missing - otherwise a total bus failure
    // is the one case the probe-loss fail-safe would never see.
    collectTemperatures();
  }

  // Re-enumerate only when a position is missing, and never mid-conversion
  if (tempBusNeedsRescan() && (millis() - tempRescanLast >= TEMP_RESCAN_INTERVAL)) {
    tempRescanLast = millis();
    rescanTempBus();
  }

  // Start the next conversion; results are collected on the next cycle
  if (tempBusCount > 0) {
    tempSensors.requestTemperatures();
    tempConversionPending = true;
  }
}

void publishTemperatures() {
  if (!client.connected()) return;

  char tempStr[10];
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    if (!tempRoleState[r].valid) continue;
    dtostrf(*TEMP_ROLES[r].value, 5, 1, tempStr);
    client.publish(TEMP_ROLES[r].stateTopic, tempStr);
  }
  publishTempCounts(false);
}

// ----------------------------------------------------------------------------
// Position assignment
// ----------------------------------------------------------------------------
void assignTempRole(int r, const DeviceAddress addr, bool save) {
  char hex[17];
  tempAddrToHex(addr, hex);

  // One address can only own one position - take it from whoever held it
  for (int other = 0; other < TEMP_ROLE_COUNT; other++) {
    if (other != r && tempRoleState[other].assigned &&
        tempAddrEqual(tempRoleState[other].addr, addr)) {
      Serial.printf("🌡 %s was held by %s - reassigning\n", hex, TEMP_ROLES[other].key);
      clearTempRole(other, false);
    }
  }

  memcpy(tempRoleState[r].addr, addr, 8);
  tempRoleState[r].assigned = true;
  tempRoleState[r].valid = false;
  tempRoleState[r].missingSince = 0;
  tempFilterIdx[r] = 0;
  tempFilterFill[r] = 0;

  if (save) saveTempAssignments();
  mapTempRolesToBus();
  publishTempPresence(r, true);
  publishTempCounts(true);
  publishTempScan();

  Serial.printf("🌡 %s -> %s\n", TEMP_ROLES[r].key, hex);
}

void clearTempRole(int r, bool save) {
  memset(tempRoleState[r].addr, 0, 8);
  tempRoleState[r].assigned = false;
  tempRoleState[r].present = false;
  tempRoleState[r].valid = false;
  *TEMP_ROLES[r].value = -999.0;

  if (save) saveTempAssignments();
  publishTempPresence(r, true);
  Serial.printf("🌡 %s unassigned\n", TEMP_ROLES[r].key);
}

// ----------------------------------------------------------------------------
// Identify by warming
// ----------------------------------------------------------------------------
// Trigger Identify for a position, then hold that probe in your hand. The
// sensor whose temperature climbs fastest takes the position. The margin
// requirement is what stops a bundle of probes sitting in the same warm bay
// from producing a coin-flip.
void setTempIdentifyState(const String& state) {
  tempIdentifyState = state;
  if (client.connected()) client.publish(Topic_TempIdentify_State, state.c_str(), true);
}

void startTempIdentify(int r) {
  if (tempBusCount == 0) {
    Serial.println("⚠ Cannot identify - no devices on the OneWire bus");
    setTempIdentifyState("NO_SENSORS");
    return;
  }

  tempIdentifyActive = true;
  tempIdentifyRole = r;
  tempIdentifyStart = millis();

  for (int i = 0; i < tempBusCount; i++) {
    tempBus[i].baselineF = tempBus[i].valid ? tempBus[i].tempF : -999.0;
  }

  Serial.printf("🖐 Identify: warm the %s probe now - watching %d device(s) for %lus\n",
                TEMP_ROLES[r].key, tempBusCount, TEMP_IDENTIFY_WINDOW_MS / 1000);
  setTempIdentifyState("WAITING: " + String(TEMP_ROLES[r].key));
}

void handleTempIdentify() {
  if (!tempIdentifyActive) return;

  unsigned long elapsed = millis() - tempIdentifyStart;

  int bestIdx = -1, secondIdx = -1;
  float bestRise = -1000.0, secondRise = -1000.0;

  for (int i = 0; i < tempBusCount; i++) {
    if (!tempBus[i].valid || tempBus[i].baselineF < TEMP_VALID_MIN_F) continue;
    float rise = tempBus[i].tempF - tempBus[i].baselineF;
    if (rise > bestRise) {
      secondRise = bestRise; secondIdx = bestIdx;
      bestRise = rise;       bestIdx = i;
    } else if (rise > secondRise) {
      secondRise = rise;     secondIdx = i;
    }
  }

  if (bestIdx >= 0) {
    Serial.printf("🖐 Identify %2lus left | leader %s +%.1fF | runner-up +%.1fF\n",
                  (TEMP_IDENTIFY_WINDOW_MS - elapsed) / 1000, tempBus[bestIdx].hex,
                  bestRise, secondIdx >= 0 ? secondRise : 0.0);
  }

  bool decided = (bestIdx >= 0 &&
                  bestRise >= TEMP_IDENTIFY_RISE_F &&
                  (secondIdx < 0 || (bestRise - secondRise) >= TEMP_IDENTIFY_MARGIN_F));

  if (decided) {
    tempIdentifyActive = false;
    int r = tempIdentifyRole;
    tempIdentifyRole = -1;
    Serial.printf("🖐 %s rose %.1fF - assigning to %s\n",
                  tempBus[bestIdx].hex, bestRise, TEMP_ROLES[r].key);
    assignTempRole(r, tempBus[bestIdx].addr, true);
    setTempIdentifyState("DONE: " + String(TEMP_ROLES[r].key) + "=" + String(tempBus[bestIdx].hex));
    return;
  }

  if (elapsed >= TEMP_IDENTIFY_WINDOW_MS) {
    tempIdentifyActive = false;
    Serial.printf("🖐 Identify timeout for %s - best rise %.1fF (need %.1fF, and %.1fF clear of runner-up)\n",
                  TEMP_ROLES[tempIdentifyRole].key, bestRise > -999.0 ? bestRise : 0.0,
                  TEMP_IDENTIFY_RISE_F, TEMP_IDENTIFY_MARGIN_F);
    tempIdentifyRole = -1;
    setTempIdentifyState("TIMEOUT");
  }
}

// ----------------------------------------------------------------------------
// Persistence, presence, reporting
// ----------------------------------------------------------------------------
void saveTempAssignments() {
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    if (tempRoleState[r].assigned) {
      preferences.putBytes(TEMP_ROLES[r].nvsKey, tempRoleState[r].addr, 8);
    } else {
      preferences.remove(TEMP_ROLES[r].nvsKey);
    }
  }
  Serial.println("💾 Harness positions saved");
}

void loadTempAssignments() {
  char legacyKey[8];
  char hex[17];
  int loaded = 0;

  memset(tempRoleState, 0, sizeof(tempRoleState));

  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    DeviceAddress addr;
    size_t n = preferences.getBytes(TEMP_ROLES[r].nvsKey, addr, 8);
    bool migrated = false;

    // Nothing under the position-keyed name: this board was last flashed with
    // the index-keyed layout, so read the old slot and carry the assignment
    // across. Done once - the write below means the next boot takes the fast
    // path above.
    if (n != 8 && TEMP_ROLES[r].legacyIndex >= 0) {
      snprintf(legacyKey, sizeof(legacyKey), "ts%d", TEMP_ROLES[r].legacyIndex);
      n = preferences.getBytes(legacyKey, addr, 8);
      migrated = (n == 8);
    }

    if (n == 8 && !tempAddrIsZero(addr) && OneWire::crc8(addr, 7) == addr[7]) {
      memcpy(tempRoleState[r].addr, addr, 8);
      tempRoleState[r].assigned = true;
      tempAddrToHex(addr, hex);
      if (migrated) {
        if (preferences.putBytes(TEMP_ROLES[r].nvsKey, addr, 8) == 8) {
          preferences.remove(legacyKey);
        }
        Serial.printf("  %-16s <- %s (migrated from %s)\n", TEMP_ROLES[r].key, hex, legacyKey);
      } else {
        Serial.printf("  %-16s <- %s\n", TEMP_ROLES[r].key, hex);
      }
      loaded++;
    } else {
      Serial.printf("  %-16s <- (unassigned)\n", TEMP_ROLES[r].key);
    }
  }
  Serial.printf("💾 %d of %d harness positions restored\n", loaded, TEMP_ROLE_COUNT);

  // The boiler-exhaust position is gone; drop its old slot so the stale
  // address is not left occupying NVS forever.
  preferences.remove("ts4");
}

// Per-entity availability, so a lost probe reads "unavailable" in Home
// Assistant rather than holding its last value forever.
void publishTempPresence(int r, bool force) {
  if (!client.connected()) return;

  bool live = tempRoleState[r].assigned && tempRoleState[r].present && tempRoleState[r].valid;
  if (!force && tempRoleState[r].presenceKnown && live == tempRoleState[r].presencePublished) return;

  String topic = String(Topic_TempPresence_Prefix) + TEMP_ROLES[r].key;
  client.publish(topic.c_str(), live ? "ONLINE" : "OFFLINE", true);
  tempRoleState[r].presencePublished = live;
  tempRoleState[r].presenceKnown = true;
}

void publishTempCounts(bool force) {
  if (!client.connected()) return;

  int live = 0;
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    if (tempRoleState[r].assigned && tempRoleState[r].present && tempRoleState[r].valid) live++;
  }

  if (force || tempBusCount != lastPublishedBusCount) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", tempBusCount);
    client.publish(Topic_TempSensors_Found, buf, true);
    lastPublishedBusCount = tempBusCount;
  }
  if (force || live != lastPublishedLiveCount) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", live);
    client.publish(Topic_TempSensors_Live, buf, true);
    lastPublishedLiveCount = live;
  }
}

void publishTempScan() {
  if (!client.connected()) return;

  String json = "{\"found\":" + String(tempBusCount);
  json += ",\"parasite\":" + String(tempSensors.isParasitePowerMode() ? "true" : "false");
  json += ",\"sensors\":[";
  for (int i = 0; i < tempBusCount; i++) {
    const TempBusSensor& b = tempBus[i];
    if (i > 0) json += ",";
    json += "{\"i\":" + String(i);
    json += ",\"addr\":\"" + String(b.hex) + "\"";
    json += ",\"role\":\"" + String(b.role >= 0 ? TEMP_ROLES[b.role].key : "") + "\"";
    json += ",\"f\":" + String(b.valid ? b.tempF : -999.0, 1);
    json += ",\"reads\":" + String(b.reads) + ",\"errors\":" + String(b.errors);
    if (b.lastErr != TERR_NONE) {
      // Failure reason plus the raw scratchpad that produced it
      char sp[19];
      for (int k = 0; k < 9; k++) sprintf(sp + k * 2, "%02X", b.scratch[k]);
      json += ",\"err\":\"" + String(TEMP_ERR_NAMES[b.lastErr]) + "\",\"sp\":\"" + String(sp) + "\"";
    }
    json += "}";
  }
  json += "],\"missing\":[";
  bool first = true;
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    if (tempRoleState[r].assigned && tempRoleState[r].present) continue;
    if (!first) json += ",";
    json += "\"" + String(TEMP_ROLES[r].key) + "\"";
    first = false;
  }
  json += "]}";

  client.publish(Topic_TempSensor_Scan, json.c_str(), true);
}

void printTempBusReport() {
  Serial.printf("\n--- DS18B20 harness --- GPIO%d  found=%d  parasite=%s\n", TEMP_SENSOR_PIN,
                tempBusCount, tempSensors.isParasitePowerMode() ? "YES" : "no");
  if (tempBusCount == 0) {
    Serial.println("  NO DEVICES. Check 4.7k pullup to 3.3V, data on GPIO13, common ground.");
  }
  Serial.println("  #  ROM Address        Position          Temp F   Reads  Errors  Last error");
  for (int i = 0; i < tempBusCount; i++) {
    char tf[10];
    if (tempBus[i].valid) dtostrf(tempBus[i].tempF, 7, 1, tf); else strcpy(tf, "   ----");
    Serial.printf("  %d  %-16s   %-16s  %s  %6lu  %6lu  %s%s\n", i, tempBus[i].hex,
                  tempBus[i].role >= 0 ? TEMP_ROLES[tempBus[i].role].key : "(unassigned)",
                  tf, (unsigned long)tempBus[i].reads, (unsigned long)tempBus[i].errors,
                  TEMP_ERR_NAMES[tempBus[i].lastErr],
                  tempBus[i].isDS18B20 ? "" : "  <- NOT A DS18B20");
  }
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    if (!tempRoleState[r].assigned) {
      Serial.printf("  UNASSIGNED: %s\n", TEMP_ROLES[r].key);
    } else if (!tempRoleState[r].present) {
      char hex[17];
      tempAddrToHex(tempRoleState[r].addr, hex);
      Serial.printf("  MISSING:    %-16s %s\n", TEMP_ROLES[r].key, hex);
    }
  }
  Serial.println();
}

void readBME280() {
  if (!bmeAvailable) return;

  float tempC = bme.readTemperature();
  // Convert BME Celsius to Fahrenheit for internal state and publishing
  environmentTemp = (tempC * 9.0 / 5.0) + 32.0;
  float newHumidity = bme.readHumidity();
  environmentPressure = bme.readPressure() / 100.0F;

  // Check for sudden humidity spike (possible leak detection) with spike filtering
  unsigned long currentTime = millis();
  if (lastHumidity > 0 && !highHumidityAlarmActive) {
    float humidityChange = newHumidity - lastHumidity;
    float timeDelta = (currentTime - lastHumidityCheck) / 1000.0;  // seconds

    // ---- SPIKE FILTERING LOGIC ----
    // When spike first detected: store baseline and start timer
    if (!humiditySpikePending && humidityChange > HUMIDITY_SPIKE_THRESHOLD && timeDelta < 60.0) {
      baselineHumidity = lastHumidity;
      spikeDetectedTime = currentTime;
      humiditySpikePending = true;

      Serial.println("\n🔍 Humidity spike detected (pending confirmation)...");
      Serial.print("Spike: +");
      Serial.print(humidityChange, 1);
      Serial.print("% (from ");
      Serial.print(baselineHumidity, 1);
      Serial.print("% to ");
      Serial.print(newHumidity, 1);
      Serial.println("%)");
      Serial.println("Waiting for recovery or confirmation...");
    }

    // If spike is pending, check for recovery or confirmation
    if (humiditySpikePending) {
      unsigned long spikeDuration = currentTime - spikeDetectedTime;
      float recoveryThreshold = baselineHumidity + HUMIDITY_SPIKE_TOLERANCE;

      // Check if humidity has returned to near-baseline (spike was transient)
      if (newHumidity <= recoveryThreshold) {
        Serial.println("✓ Humidity spike filtered: sensor returned to baseline (transient glitch)");
        Serial.print("Recovery time: ");
        Serial.print(spikeDuration / 1000.0, 1);
        Serial.println(" seconds");
        humiditySpikePending = false;
        baselineHumidity = -999.0;
      }
      // Timeout: if spike hasn't recovered in 30 seconds, treat as real leak
      else if (spikeDuration > SPIKE_RECOVERY_TIMEOUT) {
        Serial.println("\n⚠️⚠️⚠️ HIGH HUMIDITY ALARM (CONFIRMED) ⚠️⚠️⚠️");
        Serial.print("Humidity spike persisted for ");
        Serial.print(spikeDuration / 1000.0, 1);
        Serial.println(" seconds - treating as real leak");
        Serial.println("POSSIBLE WATER LEAK - Emergency shutdown!");

        highHumidityAlarmActive = true;
        humiditySpikePending = false;

        // EMERGENCY SHUTDOWN
        // Pump is controlled externally; firmware cannot stop the pump. Raise alert and secure valves.
        Serial.println("🛑 Pump control is external - raising alert and securing system");

        // Close all loop valves
        if (rearLoopOpen) {
          controlRearLoopValve(false);
          Serial.println("🛑 Rear loop CLOSE");
        }
        if (engineLoopOpen) {
          controlEngineLoopValve(false);
          engineLoopMqttRequest = false;
          preferences.putBool("engLoopMqtt", false);
          Serial.println("🛑 Engine loop CLOSE");
        }
        if (frontLoopOpen) {
          controlFrontLoopValve(false);
          frontLoopMqttRequest = false;
          preferences.putBool("frtLoopMqtt", false);
          Serial.println("🛑 Front loop CLOSE");
        }
        if (greyWaterHeaterOn) {
          controlGreyWaterHeater(false);
          Serial.println("🛑 Grey water heater CLOSE");
        }
        if (mainWaterPumpOn) {
          controlMainWaterPump(false);
          Serial.println("🛑 Main water pump OFF");
        }

        // Publish alerts
        publishAlertRaw("High Humidity - possible leak");
        client.publish(Topic_Plumbing_Status, "LEAK_DETECTED_SHUTDOWN", true);

        Serial.println("\nSystem locked. Send MQTT command to clear:");
        Serial.println("Topic: ClearHighHumidityCommand");
        Serial.println("Message: clear\n");
      }
    }
  }

  // Update tracking variables
  environmentHumidity = newHumidity;
  lastHumidity = newHumidity;
  lastHumidityCheck = currentTime;
}

void publishBME280() {
  if (!bmeAvailable) return;

  char tempStr[8];

  dtostrf(environmentTemp, 4, 2, tempStr);
  client.publish(Topic_Environment_Temp, tempStr);

  dtostrf(environmentHumidity, 4, 2, tempStr);
   client.publish(Topic_Environment_Humidity, tempStr);

  dtostrf(environmentPressure, 6, 2, tempStr);
  client.publish(Topic_Environment_Pressure, tempStr);
}

// ============================================================================
// VALVE CONTROL FUNCTIONS
// ============================================================================
void controlFreshWaterHeater(bool turnOn) {
  digitalWrite(FRESH_WATER_HEATER_PIN, turnOn ? LOW : HIGH);
  freshWaterHeaterOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_FreshWtrHeat_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("🔥 Fresh Water Heater: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlGreyWaterHeater(bool turnOn) {
  digitalWrite(GREY_WATER_HEATER_PIN, turnOn ? LOW : HIGH);
  greyWaterHeaterOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_GreyWtrHeat_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("🔥 Grey Water Heater: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlExhaustFan(bool turnOn) {
  digitalWrite(EXHAUST_FAN_PIN, turnOn ? LOW : HIGH);
  exhaustFanOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_ExhaustFan_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("💨 Exhaust Fan: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlRearLoopValve(bool open) {
  digitalWrite(REAR_LOOP_VALVE_PIN, open ? LOW : HIGH);
  rearLoopOpen = open;
  saveValveStatesToEEPROM();
  client.publish(Topic_RearLoop_Status, open ? "OPEN" : "CLOSE", true);
  Serial.print("🌊 Rear Loop: ");
  Serial.println(open ? "OPEN" : "CLOSE");
}

// ----------------------------------------------------------------------------
// Engine loop valve travel model (see ENGINE_LOOP_TRAVEL_MS)
// ----------------------------------------------------------------------------
// Modelled travel right now: 0 = fully closed, ENGINE_LOOP_TRAVEL_MS = fully open
unsigned long engineLoopTravel(unsigned long now) {
  unsigned long elapsed = now - engineLoopMarkAt;
  if (engineLoopOpen) {
    unsigned long remaining = ENGINE_LOOP_TRAVEL_MS - engineLoopTravelAtMark;
    return (elapsed >= remaining) ? ENGINE_LOOP_TRAVEL_MS : engineLoopTravelAtMark + elapsed;
  }
  return (elapsed >= engineLoopTravelAtMark) ? 0 : engineLoopTravelAtMark - elapsed;
}

ValveTravelState engineLoopTravelState(unsigned long travel) {
  if (engineLoopOpen) return (travel >= ENGINE_LOOP_TRAVEL_MS) ? VALVE_OPEN : VALVE_OPENING;
  return (travel == 0) ? VALVE_CLOSED : VALVE_CLOSING;
}

// Boot: where is the valve? A warm reset (OTA, restart, crash, watchdog) only
// drops the relay for a second or two, so resume from the last snapshot. After a
// power loss it may have moved for an unknown time, so assume closed and let it
// travel from there. No snapshot yet (first boot on this firmware): assume it
// finished moving whichever way the relay is set.
void initEngineLoopTravel() {
  esp_reset_reason_t why = esp_reset_reason();
  bool warm = (why == ESP_RST_SW || why == ESP_RST_PANIC || why == ESP_RST_INT_WDT ||
               why == ESP_RST_TASK_WDT || why == ESP_RST_WDT || why == ESP_RST_EXT);
  if (!warm) {
    engineLoopTravelAtMark = 0;
  } else if (preferences.isKey("elTravel")) {
    unsigned long saved = preferences.getULong("elTravel", 0);
    engineLoopTravelAtMark = (saved > ENGINE_LOOP_TRAVEL_MS) ? ENGINE_LOOP_TRAVEL_MS : saved;
  } else {
    engineLoopTravelAtMark = engineLoopOpen ? ENGINE_LOOP_TRAVEL_MS : 0;
  }
  engineLoopMarkAt = millis();
  Serial.printf("  Engine Loop valve model: %lu%% open, %s (%s reset)\n",
                engineLoopTravelAtMark * 100UL / ENGINE_LOOP_TRAVEL_MS,
                engineLoopOpen ? "opening/open" : "closing/closed", warm ? "warm" : "cold");
}

// Runs once a second, and immediately on a valve command
void updateEngineLoopValve(bool forcePublish) {
  unsigned long now = millis();
  unsigned long travel = engineLoopTravel(now);
  ValveTravelState state = engineLoopTravelState(travel);
  bool moving = (state == VALVE_OPENING || state == VALVE_CLOSING);

  // Re-mark at an end stop, so the elapsed count never runs long enough to wrap
  if (!moving) {
    engineLoopTravelAtMark = travel;
    engineLoopMarkAt = now;
  }

  // NVS snapshot on every state change (a reversal or reaching an end) and once
  // a minute while moving, so a reboot resumes within a minute of reality
  if ((int)state != engineLoopLastState || (moving && now - engineLoopLastSave >= ENGINE_LOOP_SAVE_MS)) {
    preferences.putULong("elTravel", travel);
    engineLoopLastSave = now;
  }
  engineLoopLastState = state;

  // Opening rounds down and closing rounds up, so 100 % only ever means OPEN
  // and 0 % only ever means CLOSED
  unsigned long scaled = travel * 100UL;
  int pct = (int)(engineLoopOpen ? scaled / ENGINE_LOOP_TRAVEL_MS
                                 : (scaled + ENGINE_LOOP_TRAVEL_MS - 1) / ENGINE_LOOP_TRAVEL_MS);

  bool stateChanged = ((int)state != engineLoopPublishedState);
  bool due = forcePublish || stateChanged ||
             (moving && pct != engineLoopPublishedPct && now - engineLoopLastPublish >= ENGINE_LOOP_PUBLISH_MS);
  if (due && client.connected()) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", pct);
    client.publish(Topic_EngineLoop_ValvePosition, buf, true);
    client.publish(Topic_EngineLoop_ValveState, VALVE_TRAVEL_NAMES[state], true);
    if (stateChanged) Serial.printf("🌊 Engine Loop valve %s (%d%%)\n", VALVE_TRAVEL_NAMES[state], pct);
    engineLoopPublishedState = state;
    engineLoopPublishedPct = pct;
    engineLoopLastPublish = now;
  }
}

void controlEngineLoopValve(bool open) {
  if (open != engineLoopOpen) {
    // Freeze the model where the valve is now; it travels back from there
    unsigned long now = millis();
    engineLoopTravelAtMark = engineLoopTravel(now);
    engineLoopMarkAt = now;
  }
  digitalWrite(ENGINE_LOOP_VALVE_PIN, open ? LOW : HIGH);
  engineLoopOpen = open;
  saveValveStatesToEEPROM();
  client.publish(Topic_EngineLoop_Status, open ? "OPEN" : "CLOSE", true);
  Serial.print("🌊 Engine Loop: ");
  Serial.println(open ? "OPEN" : "CLOSE");
  updateEngineLoopValve(false);
}

void controlFrontLoopValve(bool open) {
  digitalWrite(FRONT_LOOP_VALVE_PIN, open ? LOW : HIGH);
  frontLoopOpen = open;
  saveValveStatesToEEPROM();
  client.publish(Topic_FrontLoop_Status, open ? "OPEN" : "CLOSE", true);
  Serial.print("🌊 Front Loop: ");
  Serial.println(open ? "OPEN" : "CLOSE");
}

void controlMainWaterPump(bool turnOn) {
  // Normally closed relay: HIGH (relay OFF) = pump ON, LOW (relay ON) = pump OFF
  digitalWrite(MAIN_WATER_PUMP_PIN, turnOn ? HIGH : LOW);
  mainWaterPumpOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_MainPump_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("💧 Main Water Pump: ");
  Serial.println(turnOn ? "RUN" : "STOP");
}

void controlDHWSolinoid(bool open) {
  digitalWrite(DOMESTIC_HW_SOLINOID_PIN, open ? LOW : HIGH);
  DHWSolinoidOpen = open;
  saveValveStatesToEEPROM();
  client.publish(Topic_DomesticHW_Status, open ? "OPEN" : "CLOSE", true);
  Serial.print("🔥 Domestic Hot Water Loop: ");
  Serial.println(open ? "OPEN" : "CLOSE");
}

void controlDieselHeater(bool turnOn) {
  // Simple ON/OFF control
  // ACTIVE LOW RELAYS: LOW = relay energized (circuit ON), HIGH = relay off (circuit OFF)
  // Pin 1 (DIESEL_HEATER_PIN) = White wire (Circuit 1)
  // Pin 2 (DIESEL_HEATER_2_PIN) = Blue wire (Circuit 2)
  // Pin 3 (DIESEL_HEATER_3_PIN) = Yellow wire (Circuit 3)

  if (turnOn) {
    // ON: Pins 1&3 LOW, Pin 2 HIGH (circuits 1 & 3 ON, circuit 2 OFF)
    digitalWrite(DIESEL_HEATER_PIN, LOW);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, LOW);
  } else {
    // OFF: All pins HIGH (all relays off, circuits disabled)
    digitalWrite(DIESEL_HEATER_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, HIGH);
  }
  dieselHeaterOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_DieselHtr_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("🔥 Diesel Heater: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

void controlElectricHeater(bool turnOn) {
  // Electric heater on pin 37: HIGH = ON, LOW = OFF
  digitalWrite(ELECTRIC_HEATER_PIN, turnOn ? HIGH : LOW);
  electricHeaterOn = turnOn;
  saveValveStatesToEEPROM();
  client.publish(Topic_ElectricHtr_Status, turnOn ? "ON" : "OFF", true);
  Serial.print("⚡ Electric Heater: ");
  Serial.println(turnOn ? "ON" : "OFF");
}

// ============================================================================
// CABINET LOCK CONTROL
// ============================================================================
void lockCabinet() {
  if (!lockServo.attached()) {
    lockServo.setPeriodHertz(50);
    lockServo.attach(SERVO_PIN, 500, 2400);
  }

  lockServo.write(LOCKED_POSITION);
  currentLockState = STATE_LOCKED;
  delay(500);

  lockServo.detach();

  saveValveStatesToEEPROM();
  client.publish(Topic_CabLock_Status, "LOCKED", true);
  Serial.println("🔒 Cabinet: LOCKED");
}

void unlockCabinet() {
  if (!lockServo.attached()) {
    lockServo.setPeriodHertz(50);
    lockServo.attach(SERVO_PIN, 500, 2400);
  }

  lockServo.write(UNLOCKED_POSITION);
  currentLockState = STATE_UNLOCKED;
  delay(500);

  lockServo.detach();

  saveValveStatesToEEPROM();
  client.publish(Topic_CabLock_Status, "UNLOCKED", true);
  Serial.println("🔓 Cabinet: UNLOCKED");
}

void processLockCommand(String command) {
  if (command == "LOCK") {
    lockCabinet();
  } else if (command == "UNLOCK") {
    unlockCabinet();
  } else if (command == "TOGGLE") {
    if (currentLockState == STATE_LOCKED) {
      unlockCabinet();
    } else {
      lockCabinet();
    }
  } else if (command == "STATUS") {
    const char* status = (currentLockState == STATE_LOCKED) ? "LOCKED" : "UNLOCKED";
    client.publish(Topic_CabLock_Status, status, true);
  }
}

// ============================================================================
// CRITICAL AUTOMATION
// ============================================================================
void handleCriticalAutomation() {
  // Probe-loss fail-safe. A tank heater whose own probe has died has no control
  // input - the rules below can no longer decide to switch it off, so it would
  // stay energised indefinitely. Only applies in AUTO; MANUAL leaves the user
  // in charge. Governed by TEMP_PROBE_LOSS_FAILSAFE.
  if (TEMP_PROBE_LOSS_FAILSAFE && plumbingMode == "AUTO") {
    unsigned long now = millis();
    unsigned long freshLost = tempRoleState[TEMP_ROLE_FRESH].missingSince;
    unsigned long greyLost = tempRoleState[TEMP_ROLE_GREY].missingSince;

    if (freshWaterHeaterOn && freshLost != 0 &&
        (now - freshLost) >= TEMP_PROBE_LOSS_GRACE_MS) {
      Serial.println("⚠️ Fresh tank probe lost - switching heater OFF (fail-safe)");
      controlFreshWaterHeater(false);
    }
    if (greyWaterHeaterOn && greyLost != 0 &&
        (now - greyLost) >= TEMP_PROBE_LOSS_GRACE_MS) {
      Serial.println("⚠️ Grey tank probe lost - switching heater OFF (fail-safe)");
      controlGreyWaterHeater(false);
    }
  }

  // Fresh water tank heater automation (only when global plumbing mode is AUTO)
  if (plumbingMode == "AUTO") {
    if (freshWaterTemp > -999.0) {
      if (freshWaterTemp < freshWaterTempTarget && !freshWaterHeaterOn) {
        controlFreshWaterHeater(true);
      } else if (freshWaterTemp > (freshWaterTempTarget + 2.0) && freshWaterHeaterOn) {
        controlFreshWaterHeater(false);
      }
    }
  }

  // Grey water tank heater automation (only when global plumbing mode is AUTO)
  if (plumbingMode == "AUTO") {
    if (greyWaterTemp > -999.0) {
      if (greyWaterTemp < greyWaterTempTarget && !greyWaterHeaterOn) {
        controlGreyWaterHeater(true);
      } else if (greyWaterTemp > (greyWaterTempTarget + 2.0) && greyWaterHeaterOn) {
        controlGreyWaterHeater(false);
      }
    }
  }

  // Exhaust fan automation - temperature control using adjustable setpoint
  if (plumbingMode == "AUTO" && bmeAvailable) {
    if (environmentTemp > -999.0) {
      if (environmentTemp >= exhaustFanTempSet && !exhaustFanOn) {
        controlExhaustFan(true);
        Serial.print("🌡️ Exhaust fan ON - Temperature above ");
        Serial.print(exhaustFanTempSet);
        Serial.println("°F");
      }
      // Hysteresis of 2°F
      else if (environmentTemp < (exhaustFanTempSet - 3.0) && exhaustFanOn) {
        controlExhaustFan(false);
        Serial.print("🌡️ Exhaust fan OFF - Temperature below ");
        Serial.print(exhaustFanTempSet - 3.0);
        Serial.println("°F");
      }
    }
  }
}

void checkAlerts() {
  // Freeze warning
  bool freezeCondition = false;
  if (freshWaterTemp > -999.0 && freshWaterTemp < FREEZE_WARNING_TEMP) freezeCondition = true;
  if (greyWaterTemp > -999.0 && greyWaterTemp < FREEZE_WARNING_TEMP) freezeCondition = true;

  if (freezeCondition && !freezeWarningActive) {
    freezeWarningActive = true;
    publishAlertRaw("Tank Freeze Warning ACTIVE");
    Serial.println("⚠️ FREEZE WARNING!");
  } else if (!freezeCondition && freezeWarningActive) {
    freezeWarningActive = false;
    refreshAlertPublish();
  }

  // High temperature warning
  bool highTempCondition = false;
  if (hydronicTemp > HIGH_TEMP_WARNING) highTempCondition = true;

  if (highTempCondition && !highTempWarningActive) {
    highTempWarningActive = true;
    publishAlertRaw("High Hydronic Tank Temp ACTIVE");
    Serial.println("⚠️ HIGH TEMPERATURE WARNING!");
  } else if (!highTempCondition && highTempWarningActive) {
    highTempWarningActive = false;
    refreshAlertPublish();
  }

  // Missing temperature probes. Worth surfacing loudly: freeze protection has
  // no input at all for a position that is unassigned or off the bus.
  String missing = "";
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) {
    if (tempRoleState[r].assigned && tempRoleState[r].present && tempRoleState[r].valid) continue;
    if (missing.length() > 0) missing += ", ";
    missing += TEMP_ROLES[r].key;
  }
  bool probeFault = (missing.length() > 0);
  if (probeFault != tempProbeAlertActive || missing != tempProbeAlertText) {
    tempProbeAlertActive = probeFault;
    tempProbeAlertText = missing;
    if (probeFault) Serial.println("⚠️ Temp probe(s) missing: " + missing);
    refreshAlertPublish();
  }
}

// ============================================================================
// STATUS & HEALTH
// ============================================================================
void publishStatus() {
  // All valve states already published by control functions with retained flag
  // Publish main water pump and domestic HW loop initial states
  client.publish(Topic_MainPump_Status, mainWaterPumpOn ? "ON" : "OFF", true);
  client.publish(Topic_DomesticHW_Status, DHWSolinoidOpen ? "OPEN" : "CLOSE", true);

  // Publish plumbing mode and setpoints so HA sees them on reconnect
  client.publish(Topic_Plumbing_Mode, plumbingMode.c_str(), true);
  char fwStr[8]; dtostrf(freshWaterTempTarget, 4, 1, fwStr); client.publish(Topic_FreshWtrHeat_TempSet, fwStr, true);
  char gwStr[8]; dtostrf(greyWaterTempTarget, 4, 1, gwStr); client.publish(Topic_GreyWtrHeat_TempSet, gwStr, true);
  char fanStr[8]; dtostrf(exhaustFanTempSet, 4, 1, fanStr); client.publish(Topic_ExhaustFan_TempSet, fanStr, true);
}

// ============================================================================
// ALERT HELPERS
// Centralize alert publishing so Topic_Alert shows the latest active alert
// or "OKAY" when no alerts are active.
// ============================================================================
String lastAlert = "OKAY";

void publishAlertRaw(const String &msg) {
  lastAlert = msg;
  client.publish(Topic_Alert, lastAlert.c_str(), true);
}

void refreshAlertPublish() {
  // Priority: high humidity > high temp > freeze > probe fault > OKAY
  if (highHumidityAlarmActive) {
    publishAlertRaw("High Humidity - possible leak");
  } else if (highTempWarningActive) {
    publishAlertRaw("High Hydronic Tank Temp ACTIVE");
  } else if (freezeWarningActive) {
    publishAlertRaw("Tank Freeze Warning ACTIVE");
  } else if (tempProbeAlertActive) {
    publishAlertRaw("Temp probe missing: " + tempProbeAlertText);
  } else {
    publishAlertRaw("OKAY");
  }
}

void publishHealth() {
  char uptimeStr[16];
  unsigned long uptime = millis() / 1000;
  snprintf(uptimeStr, sizeof(uptimeStr), "%lu", uptime);
  client.publish(Topic_Plumbing_Uptime, uptimeStr);

  client.publish(Topic_Plumbing_Status, "ONLINE", true);
  client.publish(Topic_Firmware_Version, FIRMWARE_VERSION);
  publishNetworkInfo();

  Serial.print("📊 Uptime: ");
  Serial.print(uptime);
  Serial.println("s");
}

// ============================================================================
// CONFIGURATION PERSISTENCE
// ============================================================================
void saveConfigToPreferences() {
  preferences.putFloat("fwTarget", freshWaterTempTarget);
  preferences.putFloat("gwTarget", greyWaterTempTarget);
  preferences.putFloat("fanSet", exhaustFanTempSet);
  preferences.putFloat("flowK", flowKFactor);
  preferences.putString("plumbingMode", plumbingMode);
  Serial.println("💾 Configuration saved");
}

void loadConfigFromPreferences() {
  freshWaterTempTarget = preferences.getFloat("fwTarget", 40.0);
  greyWaterTempTarget = preferences.getFloat("gwTarget", 35.0);
  exhaustFanTempSet = preferences.getFloat("fanSet", 85.0);
  flowKFactor = preferences.getFloat("flowK", 450.0);
  plumbingMode = preferences.getString("plumbingMode", "AUTO");

  // Restore loop request states (MQTT commands from GIGA/thermostat)
  engineLoopMqttRequest = preferences.getBool("engLoopMqtt", false);
  frontLoopMqttRequest = preferences.getBool("frtLoopMqtt", false);

  Serial.println("💾 Configuration loaded");
  Serial.print("  Engine Loop MQTT Request: ");
  Serial.println(engineLoopMqttRequest ? "OPEN" : "CLOSE");
  Serial.print("  Front Loop MQTT Request: ");
  Serial.println(frontLoopMqttRequest ? "OPEN" : "CLOSE");
  Serial.print("  Plumbing Mode: ");
  Serial.println(plumbingMode);
  Serial.print("  Exhaust Fan Setpoint: ");
  Serial.print(exhaustFanTempSet);
  Serial.println(" °F");
  Serial.print("  Flow K-factor: ");
  Serial.print(flowKFactor, 1);
  Serial.println(" pulses/L");
}

// ============================================================================
// EEPROM STATE MANAGEMENT (Board is Authoritative)
// ============================================================================
// Deferred write: mark dirty and flush after debounce to reduce flash wear.
bool eepromDirty = false;
unsigned long eepromDirtyTime = 0;
const unsigned long EEPROM_DEBOUNCE_MS = 2000;  // Wait 2s after last change before writing

void saveValveStatesToEEPROM() {
  // Mark dirty - actual write happens in flushEEPROM() called from loop()
  eepromDirty = true;
  eepromDirtyTime = millis();
}

void flushEEPROM() {
  if (!eepromDirty) return;
  if (millis() - eepromDirtyTime < EEPROM_DEBOUNCE_MS) return;

  eepromDirty = false;
  preferences.putBool("freshHeat", freshWaterHeaterOn);
  preferences.putBool("greyHeat", greyWaterHeaterOn);
  preferences.putBool("exhaustFan", exhaustFanOn);
  preferences.putBool("rearLoop", rearLoopOpen);
  preferences.putBool("engineLoop", engineLoopOpen);
  preferences.putBool("frontLoop", frontLoopOpen);
  preferences.putBool("dieselHtr", dieselHeaterOn);
  preferences.putBool("electricHtr", electricHeaterOn);
  preferences.putBool("mainPump", mainWaterPumpOn);
  preferences.putBool("dhwSolenoid", DHWSolinoidOpen);
  preferences.putBool("cabLock", currentLockState == STATE_LOCKED);
  Serial.println("💾 Valve states saved to EEPROM");
}

void restoreValveStatesFromEEPROM() {
  Serial.println("\n🔄 Restoring valve states from EEPROM (AUTHORITATIVE)...");

  // Restore state variables from EEPROM
  freshWaterHeaterOn = preferences.getBool("freshHeat", false);
  greyWaterHeaterOn = preferences.getBool("greyHeat", false);
  exhaustFanOn = preferences.getBool("exhaustFan", false);
  rearLoopOpen = preferences.getBool("rearLoop", false);
  engineLoopOpen = preferences.getBool("engineLoop", false);
  frontLoopOpen = preferences.getBool("frontLoop", false);
  dieselHeaterOn = preferences.getBool("dieselHtr", false);
  electricHeaterOn = preferences.getBool("electricHtr", false);
  mainWaterPumpOn = preferences.getBool("mainPump", true);  // Default ON
  // The DHW solenoid is driven by the flow state machine, which boots IDLE. If
  // it were restored open (board reset mid-draw) nothing would close it until
  // the next tap was opened and shut again, so it always starts closed.
  DHWSolinoidOpen = false;
  bool savedLockState = preferences.getBool("cabLock", false);  // Default UNLOCKED

  // Apply restored states to physical hardware (active LOW relays)
  digitalWrite(FRESH_WATER_HEATER_PIN, freshWaterHeaterOn ? LOW : HIGH);
  digitalWrite(GREY_WATER_HEATER_PIN, greyWaterHeaterOn ? LOW : HIGH);
  digitalWrite(EXHAUST_FAN_PIN, exhaustFanOn ? LOW : HIGH);
  digitalWrite(REAR_LOOP_VALVE_PIN, rearLoopOpen ? LOW : HIGH);
  digitalWrite(ENGINE_LOOP_VALVE_PIN, engineLoopOpen ? LOW : HIGH);
  digitalWrite(FRONT_LOOP_VALVE_PIN, frontLoopOpen ? LOW : HIGH);
  digitalWrite(MAIN_WATER_PUMP_PIN, mainWaterPumpOn ? HIGH : LOW);
  digitalWrite(DOMESTIC_HW_SOLINOID_PIN, DHWSolinoidOpen ? LOW : HIGH);

  // Electric heater: HIGH = ON, LOW = OFF
  digitalWrite(ELECTRIC_HEATER_PIN, electricHeaterOn ? HIGH : LOW);

  // Restore diesel heater state (simple on/off)
  if (dieselHeaterOn) {
    // ON: Pins 1&3 LOW, Pin 2 HIGH
    digitalWrite(DIESEL_HEATER_PIN, LOW);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, LOW);
  } else {
    // OFF: All pins HIGH
    digitalWrite(DIESEL_HEATER_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_2_PIN, HIGH);
    digitalWrite(DIESEL_HEATER_3_PIN, HIGH);
  }

  // Drive the cabinet lock straight to its saved position (no unlock-first)
  currentLockState = savedLockState ? STATE_LOCKED : STATE_UNLOCKED;
  lockServo.setPeriodHertz(50);
  lockServo.attach(SERVO_PIN, 500, 2400);
  lockServo.write(savedLockState ? LOCKED_POSITION : UNLOCKED_POSITION);
  delay(500);
  lockServo.detach();

  Serial.println("  ✓ Hardware relays restored from EEPROM");
  Serial.println("  Fresh Water Heater: " + String(freshWaterHeaterOn ? "ON" : "OFF"));
  Serial.println("  Grey Water Heater: " + String(greyWaterHeaterOn ? "ON" : "OFF"));
  Serial.println("  Exhaust Fan: " + String(exhaustFanOn ? "ON" : "OFF"));
  Serial.println("  Rear Loop: " + String(rearLoopOpen ? "OPEN" : "CLOSE"));
  Serial.println("  Engine Loop: " + String(engineLoopOpen ? "OPEN" : "CLOSE"));
  Serial.println("  Front Loop: " + String(frontLoopOpen ? "OPEN" : "CLOSE"));
  Serial.println("  Diesel Heater: " + String(dieselHeaterOn ? "ON" : "OFF"));
  Serial.println("  Electric Heater: " + String(electricHeaterOn ? "ON" : "OFF"));
  Serial.println("  Main Water Pump: " + String(mainWaterPumpOn ? "ON" : "OFF"));
  Serial.println("  Domestic HW: " + String(DHWSolinoidOpen ? "OPEN" : "CLOSE"));
  Serial.println("  Cabinet Lock: " + String(currentLockState == STATE_LOCKED ? "LOCKED" : "UNLOCKED"));
  initEngineLoopTravel();
  Serial.println("✓ Board is now AUTHORITATIVE source\n");
}

void publishAllStatesToBroker() {
  Serial.println("📡 Publishing authoritative states to MQTT broker...");

  // Publish all current states with retained flag
  client.publish(Topic_FreshWtrHeat_Status, freshWaterHeaterOn ? "ON" : "OFF", true);
  client.publish(Topic_GreyWtrHeat_Status, greyWaterHeaterOn ? "ON" : "OFF", true);
  client.publish(Topic_ExhaustFan_Status, exhaustFanOn ? "ON" : "OFF", true);
  client.publish(Topic_RearLoop_Status, rearLoopOpen ? "OPEN" : "CLOSE", true);
  client.publish(Topic_EngineLoop_Status, engineLoopOpen ? "OPEN" : "CLOSE", true);
  updateEngineLoopValve(true);   // modelled travel: EngineLoopValveState / Position
  client.publish(Topic_FrontLoop_Status, frontLoopOpen ? "OPEN" : "CLOSE", true);
  client.publish(Topic_MainPump_Status, mainWaterPumpOn ? "ON" : "OFF", true);
  client.publish(Topic_DomesticHW_Status, DHWSolinoidOpen ? "OPEN" : "CLOSE", true);
  client.publish(Topic_DieselHtr_Status, dieselHeaterOn ? "ON" : "OFF", true);
  client.publish(Topic_ElectricHtr_Status, electricHeaterOn ? "ON" : "OFF", true);

  // DS18B20 harness state
  for (int r = 0; r < TEMP_ROLE_COUNT; r++) publishTempPresence(r, true);
  publishTempCounts(true);
  publishTempScan();
  client.publish(Topic_TempIdentify_State, tempIdentifyState.c_str(), true);
  { char kbuf[12]; dtostrf(flowKFactor, 6, 1, kbuf); client.publish(Topic_Flow_KFactor, kbuf, true); }
  publishFlowRate(true);

  Serial.println("✓ Broker synchronized with board's authoritative state");
}
