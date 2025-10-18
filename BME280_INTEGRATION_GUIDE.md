# Adding BME280 to D1 Plumbing System

## Current Situation
You have **all 9 usable pins on the D1 Mini occupied**:
- D0, D2-D8: Relays and Servo (9 devices)
- D1: DS18B20 OneWire sensors

## ✅ SOLUTION: I2C Can Share Pins!

Good news! **BME280 uses I2C and can share pins** with existing devices. Here's how:

## I2C on D1 Mini

| Pin | GPIO | Default I2C | Your Current Use | Can Share? |
|-----|------|-------------|------------------|------------|
| D1 | GPIO5 | **SCL** (Clock) | DS18B20 OneWire | ✅ YES |
| D2 | GPIO4 | **SDA** (Data) | Grey Water Heater Relay | ✅ YES |

### Why This Works:

1. **D1 (SCL + OneWire)**:
   - OneWire uses timing-based protocol
   - I2C uses clock-based protocol
   - They can coexist if you're careful with timing
   - Use OneWire first, then I2C, in your code

2. **D2 (SDA + Relay Output)**:
   - Relay uses OUTPUT mode (3.3V high, 0V low)
   - I2C SDA uses INPUT_PULLUP mode for receiving data
   - **No conflict** - they work on different modes
   - Just ensure relay doesn't toggle during I2C communication

## Implementation

### Hardware Wiring

**BME280 Connections:**
```
BME280    →    D1 Mini
------         --------
VCC       →    3.3V
GND       →    GND
SDA       →    D2 (GPIO4)
SCL       →    D1 (GPIO5)
```

**Note:** No pull-up resistors needed - D1 Mini has internal pull-ups.

### Software Changes

#### 1. Add Library to `platformio.ini`

```ini
[env:d1_mini]
platform = espressif8266
board = d1_mini
framework = arduino
lib_deps =
    PubSubClient
    DallasTemperature
    OneWire
    Servo
    Adafruit BME280 Library    ; ADD THIS LINE
    Adafruit Unified Sensor    ; ADD THIS LINE (required dependency)
```

#### 2. Update `main.cpp` Header

Add these includes after the existing ones:
```cpp
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>
#include <ArduinoOTA.h>
#include <Wire.h>                  // ADD THIS
#include <Adafruit_Sensor.h>       // ADD THIS
#include <Adafruit_BME280.h>       // ADD THIS
```

#### 3. Add BME280 Configuration

After the DS18B20 setup section:
```cpp
// ----------------- BME280 SETUP -----------------
#define BME280_ADDRESS 0x76  // or 0x77 (check your module)
Adafruit_BME280 bme;
bool bme280Available = false;
```

#### 4. Add MQTT Topics

```cpp
// BME280 sensor topics
const char* Topic_BME_Temp = "PlumbingBMETemperature";
const char* Topic_BME_Humidity = "PlumbingBMEHumidity";
const char* Topic_BME_Pressure = "PlumbingBMEPressure";
```

#### 5. Initialize BME280 in `setup()`

Add this AFTER the DS18B20 initialization:
```cpp
// Initialize BME280 temperature/humidity sensor
Serial.println("Initializing BME280 sensor...");
Wire.begin(D2, D1);  // SDA=D2 (GPIO4), SCL=D1 (GPIO5)

if (bme.begin(BME280_ADDRESS)) {
  bme280Available = true;
  Serial.println("✓ BME280 sensor detected!");
  Serial.print("  I2C Address: 0x");
  Serial.println(BME280_ADDRESS, HEX);
  Serial.println("  Measuring: Temperature, Humidity, Pressure");
} else {
  Serial.println("⚠ WARNING: BME280 sensor not detected!");
  Serial.println("  Check wiring and I2C address (0x76 or 0x77)");
  bme280Available = false;
}
```

#### 6. Add BME280 Reading Function

```cpp
// Variables for BME280 readings
float bmeTemperature = 0.0;
float bmeHumidity = 0.0;
float bmePressure = 0.0;

// Add this function with the other read functions
void readBME280() {
  if (!bme280Available) return;

  bmeTemperature = bme.readTemperature() * 9.0/5.0 + 32.0;  // Convert to Fahrenheit
  bmeHumidity = bme.readHumidity();
  bmePressure = bme.readPressure() / 100.0F;  // Convert Pa to hPa

  // Check for valid readings
  if (isnan(bmeTemperature) || isnan(bmeHumidity) || isnan(bmePressure)) {
    Serial.println("⚠ BME280 reading failed!");
    return;
  }
}

void publishBME280() {
  if (!bme280Available) return;

  char tempStr[8];

  dtostrf(bmeTemperature, 4, 2, tempStr);
  client.publish(Topic_BME_Temp, tempStr);

  dtostrf(bmeHumidity, 4, 2, tempStr);
  client.publish(Topic_BME_Humidity, tempStr);

  dtostrf(bmePressure, 6, 2, tempStr);
  client.publish(Topic_BME_Pressure, tempStr);
}
```

#### 7. Update `loop()` to Read BME280

In your main loop, add BME280 reading alongside DS18B20:
```cpp
// Read temperature sensors periodically
if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
  lastSensorRead = currentMillis;
  readTemperatures();        // DS18B20 sensors
  readBME280();              // ADD THIS LINE
  publishTemperatures();     // DS18B20 data
  publishBME280();           // ADD THIS LINE
  handleCriticalAutomation();
  checkAlerts();
}
```

## Timing Considerations

### Potential Conflicts

**Problem:** Both DS18B20 (on D1) and BME280 (using D1 for SCL) share the same pin.

**Solution:** Order matters! Always read in this sequence:
1. DS18B20 first (OneWire is time-critical)
2. Small delay (10-50ms)
3. BME280 second (I2C is more forgiving)

```cpp
void readSensors() {
  // Read DS18B20 first
  readTemperatures();

  // Small delay to let OneWire settle
  delay(20);

  // Then read BME280
  readBME280();
}
```

## Alternative: I2C on Different Pins

If you experience conflicts, you can use **software I2C** on unused pins:

**But wait, you have NO unused pins!** So this won't work either.

## Best Alternative: I2C GPIO Expander

If sharing pins doesn't work reliably, add a **PCF8574 or MCP23017 I2C GPIO Expander**:

### PCF8574 (8 extra GPIO pins)
- Connects to I2C (D1/D2)
- Gives you 8 more digital I/O pins
- Costs ~$1
- Move some relays to the expander
- Free up a pin for dedicated OneWire

### Wiring:
```
PCF8574    →    D1 Mini
-------         --------
VCC        →    3.3V
GND        →    GND
SDA        →    D2 (GPIO4)
SCL        →    D1 (GPIO5)
A0-A2      →    GND (address 0x20)
P0-P7      →    Your relays (8 pins)
```

### Code Example:
```cpp
#include <PCF8574.h>

PCF8574 expander(0x20);  // I2C address

void setup() {
  expander.begin();
  expander.pinMode(P0, OUTPUT);  // Fresh water heater
  expander.pinMode(P1, OUTPUT);  // Grey water heater
  // etc...
}

void controlFreshWaterHeater(bool turnOn) {
  expander.digitalWrite(P0, turnOn ? HIGH : LOW);
}
```

## Testing BME280

After uploading, check Serial Monitor:

**Success:**
```
Initializing BME280 sensor...
✓ BME280 sensor detected!
  I2C Address: 0x76
  Measuring: Temperature, Humidity, Pressure
```

**If Not Found:**
- Check wiring (especially SDA/SCL)
- Try address 0x77 instead of 0x76
- Verify BME280 has power (3.3V)
- Use I2C scanner sketch to find address

## I2C Scanner Code

If BME280 isn't detected, run this to find it:
```cpp
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(D2, D1);  // SDA, SCL

  Serial.println("I2C Scanner");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
    }
  }
}

void loop() {}
```

## Summary

✅ **YES, you can add BME280!**
- Use D1 (SCL) and D2 (SDA) for I2C
- These pins are shared but compatible
- Add minimal code changes
- No additional hardware needed

If you experience issues:
- Consider PCF8574 GPIO expander
- Move some relays to the expander
- Gives you 8 extra pins for future expansion

---

**Need help with the code integration? Let me know and I can add all the changes to your main.cpp file!**
