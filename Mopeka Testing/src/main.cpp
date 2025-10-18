#include <Arduino.h>
#include <ArduinoBLE.h>

// Mopeka Pro Check identifiers for ChesterTheBus RV tank monitoring
#define MOPEKA_MANUFACTURER_ID_1 0x000D  // Standard Mopeka ID
#define MOPEKA_MANUFACTURER_ID_2 0x0059  // Alternative ID (Pro Universal Check)
#define MOPEKA_SERVICE_UUID "fee5"       // Service advertised by Mopeka sensors

// Structure to hold tank data for ChesterTheBus RV Control System
// Tanks: Fresh Water, Grey Water, Propane, Diesel
struct TankData {
  String sensorName;
  String address;
  float levelPercent;
  float temperatureC;
  int batteryPercent;
  float batteryVoltage;
  int rssi;
  unsigned long lastUpdate;
  uint16_t rawDistance;  // Store raw distance for debugging/calibration
};

// Array to store data from multiple sensors (Fresh, Grey, Propane, Diesel)
TankData tanks[4];
int tankCount = 0;

// Tank calibration settings - adjust these for each tank on ChesterTheBus
struct TankCalibration {
  String macAddress;
  const char* tankName;
  float tankHeightMM;  // Full tank height in mm
};

// Calibration data for ChesterTheBus RV tanks
// Update MAC addresses and heights after testing each sensor
TankCalibration tankCalibrations[] = {
  {"e0:ca:26:b0:c5:f2", "Fresh Water", 610.0},   // 24 inches - CALIBRATE
  {"00:00:00:00:00:00", "Grey Water", 610.0},    // 24 inches - CALIBRATE
  {"00:00:00:00:00:00", "Propane", 305.0},       // 12 inches - CALIBRATE
  {"00:00:00:00:00:00", "Diesel", 508.0}         // 20 inches - CALIBRATE
};

// Forward declarations
bool isMopekaSensor(BLEDevice peripheral);
bool parseMopekaAdvertisement(BLEDevice peripheral, TankData& tankData);
void updateTankData(TankData& newData);
void displayAllTankData();
TankCalibration* getTankCalibration(String macAddress);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("Mopeka Pro Check Sensor - ChesterTheBus RV Control");
  Serial.println("Arduino GIGA R1 WiFi");
  Serial.println("Tank Level Monitoring: Fresh, Grey, Propane, Diesel");
  Serial.println("----------------------------------------------------");
  
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }
  
  Serial.println("BLE initialized successfully");
  Serial.println("Scanning for Mopeka Pro Universal Check sensors...");
  Serial.println();
  
  // Start scanning for BLE devices
  BLE.scan();
}

void loop() {
  // Check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();
  
  if (peripheral) {
    // Check if this is a Mopeka sensor
    if (isMopekaSensor(peripheral)) {
      // Parse advertisement data
      TankData tankData;
      if (parseMopekaAdvertisement(peripheral, tankData)) {
        updateTankData(tankData);
        displayAllTankData();
      }
    }
  }
  
  // Print status every 30 seconds
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 30000) {
    lastStatusPrint = millis();
    Serial.println("\n========== ChesterTheBus Tank Status ==========");
    displayAllTankData();
  }
}

bool isMopekaSensor(BLEDevice peripheral) {
  // Check by name first
  String localName = peripheral.localName();
  if (localName.indexOf("Mopeka") >= 0 || localName.indexOf("MOPEKA") >= 0 || 
      localName.indexOf("Pro Check") >= 0 || localName.indexOf("PRO CHECK") >= 0) {
    return true;
  }
  
  // Check for Mopeka service UUID
  if (peripheral.hasAdvertisedServiceUuid()) {
    for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
      String serviceUuid = peripheral.advertisedServiceUuid(i);
      if (serviceUuid.indexOf(MOPEKA_SERVICE_UUID) >= 0) {
        return true;
      }
    }
  }
  
  // Check manufacturer data for known Mopeka IDs
  if (peripheral.hasManufacturerData()) {
    int dataLength = peripheral.manufacturerDataLength();
    if (dataLength >= 2) {
      uint8_t manufacturerData[dataLength];
      peripheral.manufacturerData(manufacturerData, dataLength);
      
      // Check company identifier (first 2 bytes, little-endian)
      uint16_t companyId = manufacturerData[0] | (manufacturerData[1] << 8);
      if (companyId == MOPEKA_MANUFACTURER_ID_1 || companyId == MOPEKA_MANUFACTURER_ID_2) {
        return true;
      }
    }
  }
  
  return false;
}

TankCalibration* getTankCalibration(String macAddress) {
  for (int i = 0; i < 4; i++) {
    if (tankCalibrations[i].macAddress.equalsIgnoreCase(macAddress)) {
      return &tankCalibrations[i];
    }
  }
  return nullptr;
}

bool parseMopekaAdvertisement(BLEDevice peripheral, TankData& tankData) {
  tankData.sensorName = peripheral.localName();
  if (tankData.sensorName.length() == 0) {
    tankData.sensorName = "Mopeka Pro Universal";
  }
  tankData.address = peripheral.address();
  tankData.rssi = peripheral.rssi();
  tankData.lastUpdate = millis();
  
  // Get manufacturer data
  if (!peripheral.hasManufacturerData()) {
    return false;
  }
  
  int dataLength = peripheral.manufacturerDataLength();
  uint8_t manufacturerData[dataLength];
  peripheral.manufacturerData(manufacturerData, dataLength);
  
  if (dataLength < 12) {
    return false;
  }
  
  // Get company ID
  uint16_t companyId = manufacturerData[0] | (manufacturerData[1] << 8);
  
  if (companyId == MOPEKA_MANUFACTURER_ID_2) {  // 0x0059 format
    // Mopeka Pro Universal Check format:
    // Byte 0-1: Company ID (0x59 0x00)
    // Byte 2: Sync/Quality byte
    // Byte 3: Temperature (raw value / 4 = °C)
    // Byte 4: Battery voltage and flags
    // Byte 5-6: Reserved (0x00 0x00)
    // Byte 7-9: MAC address last 3 bytes
    // Byte 10-11: Distance reading (little-endian)
    
    // Temperature from byte 3 (VERIFIED: byte3 / 4 = °C)
    tankData.temperatureC = manufacturerData[3] / 4.0;
    
    // Battery voltage analysis - testing different interpretations
    Serial.println("\n--- Battery Voltage Analysis ---");
    
    uint8_t byte2 = manufacturerData[2];
    uint8_t byte4 = manufacturerData[4];
    uint8_t byte11 = manufacturerData[11];
    
    // Test voltage interpretations (CR2032: 2.0V-3.0V range)
    // Try different scaling factors
    
    Serial.println("Testing byte 2 (0x" + String(byte2, HEX) + " = " + String(byte2) + "):");
    float v1 = byte2 * 0.02;  // Common: 0.02V per unit
    float v2 = byte2 * 0.01;  // 0.01V per unit
    float v3 = byte2 / 10.0;  // Direct decimal (12 = 1.2V - too low)
    float v4 = (byte2 + 20) * 0.01;  // Offset encoding (12 + 20 = 32 = 3.2V)
    
    Serial.print("  0.02V scale: ");
    Serial.print(v1, 2);
    Serial.print("V");
    if (v1 >= 2.8 && v1 <= 3.1) Serial.print(" <<<< MATCH CR2032 full range!");
    Serial.println();
    
    Serial.print("  0.01V scale: ");
    Serial.print(v2, 2);
    Serial.print("V");
    if (v2 >= 2.8 && v2 <= 3.1) Serial.print(" <<<< MATCH CR2032 full range!");
    Serial.println();
    
    Serial.print("  /10 scale: ");
    Serial.print(v3, 2);
    Serial.print("V");
    if (v3 >= 2.8 && v3 <= 3.1) Serial.print(" <<<< MATCH CR2032 full range!");
    Serial.println();
    
    Serial.print("  Offset +20, 0.01V: ");
    Serial.print(v4, 2);
    Serial.print("V");
    if (v4 >= 2.8 && v4 <= 3.1) Serial.print(" <<<< MATCH CR2032 full range!");
    Serial.println();
    
    Serial.println("\nTesting byte 4 (0x" + String(byte4, HEX) + " = " + String(byte4) + "):");
    float v5 = byte4 * 0.02;
    float v6 = byte4 * 0.01;
    float v7 = (byte4 >> 4) * 0.3;  // Upper nibble (3) * 0.3 = 0.9V (too low)
    float v8 = byte4 * 0.05;  // 0.05V scale (62 * 0.05 = 3.1V - close!)
    
    Serial.print("  0.02V scale: ");
    Serial.print(v5, 2);
    Serial.print("V");
    if (v5 >= 2.8 && v5 <= 3.1) Serial.print(" <<<< MATCH CR2032 full range!");
    Serial.println();
    
    Serial.print("  0.01V scale: ");
    Serial.print(v6, 2);
    Serial.print("V");
    if (v6 >= 2.8 && v6 <= 3.1) Serial.print(" <<<< MATCH CR2032 full range!");
    Serial.println();
    
    Serial.print("  Upper nibble * 0.3V: ");
    Serial.print(v7, 2);
    Serial.print("V");
    if (v7 >= 2.8 && v7 <= 3.1) Serial.print(" <<<< MATCH CR2032 full range!");
    Serial.println();
    
    Serial.print("  0.05V scale: ");
    Serial.print(v8, 2);
    Serial.print("V");
    if (v8 >= 2.8 && v8 <= 3.1) Serial.print(" <<<< MATCH CR2032 full range!");
    Serial.println();
    
    Serial.println("\nTesting byte 11 (0x" + String(byte11, HEX) + " = " + String(byte11) + "):");
    float v9 = byte11 * 0.02;
    float v10 = byte11 * 0.01;
    
    Serial.print("  0.02V scale: ");
    Serial.print(v9, 2);
    Serial.print("V");
    if (v9 >= 2.8 && v9 <= 3.1) Serial.print(" <<<< MATCH CR2032 full range!");
    Serial.println();
    
    Serial.print("  0.01V scale: ");
    Serial.print(v10, 2);
    Serial.print("V");
    if (v10 >= 2.8 && v10 <= 3.1) Serial.print(" <<<< MATCH CR2032 full range!");
    Serial.println();
    
    // Default to byte 4 with 0.05V scaling (closest to 3.0V for 100% battery)
    tankData.batteryVoltage = byte4 * 0.05;
    
    // Convert voltage to percentage (CR2032: 3.0V = 100%, 2.0V = 0%)
    tankData.batteryPercent = constrain((tankData.batteryVoltage - 2.0) * 100.0, 0, 100);
    
    Serial.print("\nUsing: Byte 4 with 0.05V scale = ");
    Serial.print(tankData.batteryVoltage, 2);
    Serial.print("V (");
    Serial.print(tankData.batteryPercent);
    Serial.println("%)");
    Serial.println("⚠ Verify with app's battery voltage if available");
    
    // Distance reading from bytes 10-11 (little-endian)
    uint16_t distanceRaw = manufacturerData[10] | (manufacturerData[11] << 8);
    tankData.rawDistance = distanceRaw;
    
    // Get tank calibration for this sensor
    TankCalibration* calibration = getTankCalibration(tankData.address);
    
    if (calibration != nullptr) {
      tankData.sensorName = calibration->tankName;
      float tankHeightMM = calibration->tankHeightMM;
      
      // The sensor measures distance from top to liquid surface
      float distanceMM = distanceRaw;
      
      tankData.levelPercent = ((tankHeightMM - distanceMM) / tankHeightMM) * 100.0;
      tankData.levelPercent = constrain(tankData.levelPercent, 0.0, 100.0);
      
      Serial.print("\nTank: ");
      Serial.print(calibration->tankName);
      Serial.print(" | Raw Distance: ");
      Serial.print(distanceRaw);
      Serial.print(" | Level: ");
      Serial.print(tankData.levelPercent, 1);
      Serial.println("%");
    } else {
      tankData.levelPercent = 0.0;
      Serial.println("\n⚠ Uncalibrated sensor - add MAC to tankCalibrations array");
    }
    
    Serial.println("=============================\n");
    return true;
  }
  
  return false;
}

void updateTankData(TankData& newData) {
  // Find if we already have this sensor (by MAC address)
  for (int i = 0; i < tankCount; i++) {
    if (tanks[i].address == newData.address) {
      tanks[i] = newData;
      return;
    }
  }
  
  // New sensor - add if we have space for ChesterTheBus tanks
  if (tankCount < 4) {
    tanks[tankCount] = newData;
    tankCount++;
  }
}

void displayAllTankData() {
  if (tankCount == 0) {
    Serial.println("No sensor data available");
    Serial.println("Ensure Mopeka sensors are powered and within range");
    return;
  }
  
  for (int i = 0; i < tankCount; i++) {
    Serial.println("=========================================");
    Serial.print("Tank: ");
    Serial.println(tanks[i].sensorName);
    Serial.print("  MAC: ");
    Serial.println(tanks[i].address);
    Serial.print("  Level: ");
    Serial.print(tanks[i].levelPercent, 1);
    Serial.print("% | Raw Distance: ");
    Serial.println(tanks[i].rawDistance);
    Serial.print("  Temp: ");
    Serial.print(tanks[i].temperatureC, 1);
    Serial.print("°C (");
    Serial.print(tanks[i].temperatureC * 9.0 / 5.0 + 32.0, 1);
    Serial.println("°F)");
    Serial.print("  Battery: ");
    Serial.print(tanks[i].batteryPercent);
    Serial.print("% (");
    Serial.print(tanks[i].batteryVoltage, 2);
    Serial.print("V) | Signal: ");
    Serial.print(tanks[i].rssi);
    Serial.println(" dBm");
    Serial.print("  Last Update: ");
    Serial.print((millis() - tanks[i].lastUpdate) / 1000);
    Serial.println("s ago");
  }
  Serial.println("=========================================\n");
}