# Water Flow Sensor Integration Guide

## Overview
Hall Effect water flow sensor (YF-S201 or similar) integrated into the D1 Plumbing System board to monitor water flow and enable hot water automation on a separate board.

---

## Hardware Specifications

### Flow Sensor: YF-S201 (or compatible)
- **Type**: Hall Effect Turbine Flow Sensor
- **Range**: 1-30 L/min
- **Output**: Digital pulses (square wave)
- **Calibration Factor**: ~450 pulses per liter (may vary by model)
- **Operating Voltage**: 5-18V DC
- **Output Voltage**: 5V logic (compatible with 3.3V ESP8266 via INPUT_PULLUP)

### Wiring
```
Flow Sensor → D1 Mini Plumbing Board
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
RED (VCC)    → 5V or VIN pin
BLACK (GND)  → GND
YELLOW (SIG) → D8 (GPIO15)
```

**Note**: D8 on ESP8266 supports interrupts, making it ideal for pulse counting.

---

## Pin Assignment

### D1 Plumbing Board - Final Configuration
```
D0 = Fresh Water Heater (relay)
D1 = DS18B20 OneWire (6 temperature sensors)
D2 = Grey Water Heater (relay)
D3 = Exhaust Fan (relay)
D4 = Diesel Heater #2 (relay)
D5 = Rear Loop Valve (relay)
D6 = Engine Loop Valve (relay)
D7 = Diesel Heater #1 (relay)
D8 = Water Flow Sensor (interrupt pin)
```

---

## How It Works

### 1. Pulse Counting
- Flow sensor outputs **pulses** as water flows through turbine
- Each pulse represents a fixed volume of water
- ESP8266 uses **hardware interrupt** on D8 to count pulses
- Interrupt Service Routine (ISR) increments counter on every rising edge

### 2. Flow Rate Calculation
Calculated every 1 second:
```
Flow Rate (L/min) = (pulses per second × 60) / pulses_per_liter
```

Default calibration: **450 pulses per liter**

### 3. Volume Tracking
```
Total Volume (L) = total_pulses / pulses_per_liter
```

---

## Software Implementation

### Key Variables
```cpp
volatile unsigned long pulseCount = 0;     // Pulse counter (ISR)
float flowRate = 0.0;                      // L/min
float totalVolume = 0.0;                   // Total liters
const float PULSES_PER_LITER = 450.0;      // Calibration factor
const float FLOW_THRESHOLD = 0.5;          // L/min trigger threshold
```

### Interrupt Service Routine (ISR)
```cpp
void IRAM_ATTR flowSensorISR() {
  pulseCount++;  // Called on each pulse rising edge
}
```

### Flow Rate Calculation
```cpp
void calculateFlowRate() {
  // Called every 1 second
  // Calculates flow rate from pulse frequency
  // Updates totalVolume counter
}
```

---

## MQTT Topics

### Published by D1 Plumbing Board:
- **`PlumbingFlowRate`** - Current flow rate in L/min (float)
- **`FlowSensorStatus`** - "FLOWING" or "STOPPED" (string)

### Consumed by Separate Board:
The separate board (with hot water solenoid) subscribes to:
- **`FlowSensorStatus`** - Opens solenoid when "FLOWING", closes when "STOPPED"

---

## Hot Water Automation Logic

### Separate Board Automation:
```
IF FlowSensorStatus == "FLOWING" AND flow >= 0.5 L/min:
    Open Hot Water Solenoid
ELSE:
    Close Hot Water Solenoid
```

This ensures:
- ✅ Hot water only flows when cold water is flowing
- ✅ Prevents pump dry-run
- ✅ Automatic on-demand hot water
- ✅ Water conservation

---

## Calibration

### Finding Your Calibration Factor

**Method 1: Measure Known Volume**
1. Run water through sensor into container
2. Measure exact volume collected (e.g., 5 liters)
3. Read `pulseCount` from Serial Monitor
4. Calculate: `PULSES_PER_LITER = pulseCount / volume`

**Method 2: Use Manufacturer Spec**
- YF-S201: ~450 pulses/liter
- YF-B1: ~2000 pulses/liter
- Check datasheet for your specific model

### Update Calibration in Code:
```cpp
const float PULSES_PER_LITER = 450.0;  // Adjust this value
```

---

## Troubleshooting

### No Flow Detected
- ✅ Check wiring (VCC, GND, SIG)
- ✅ Verify 5V power supply to sensor
- ✅ Monitor `pulseCount` in Serial output
- ✅ Ensure water is flowing through sensor
- ✅ Check turbine not stuck/blocked

### Inaccurate Flow Rate
- ✅ Calibrate `PULSES_PER_LITER` factor
- ✅ Verify voltage levels (sensor may need 5V VCC)
- ✅ Check for air bubbles in system
- ✅ Ensure proper sensor orientation (arrow points flow direction)

### Erratic Readings
- ✅ Add 100Ω resistor between signal and GND (pull-down)
- ✅ Use shielded cable if long wire runs
- ✅ Check for electrical noise from relays
- ✅ Verify stable power supply

### Flow Threshold Issues
Adjust threshold in code:
```cpp
const float FLOW_THRESHOLD = 0.5;  // L/min (increase if too sensitive)
```

---

## Testing

### Serial Monitor Output (115200 baud):
```
✓ Flow sensor interrupt attached on D8
💧 Flow: 2.45 L/min | Total: 12.34 L
💧 Flow: 3.12 L/min | Total: 15.46 L
```

### MQTT Test Commands:
Monitor topics in MQTT client:
```bash
mosquitto_sub -h 192.168.8.1 -t PlumbingFlowRate
mosquitto_sub -h 192.168.8.1 -t FlowSensorStatus
```

---

## Integration with Separate Board

The hot water solenoid control has been moved to a separate D1 Mini board that will also include:
- BME280 temperature/humidity/pressure sensor
- Hot water solenoid relay
- Water flow sensor automation logic
- Optional: Cabinet lock servo

This board subscribes to `FlowSensorStatus` and opens the hot water solenoid when water is flowing.

See separate board documentation for implementation details.

---

## Safety Considerations

### Electrical
- ⚠️ Use proper electrical enclosure (IP65+ rating for plumbing area)
- ⚠️ Keep 5V/3.3V electronics away from 120V/240V AC
- ⚠️ Use strain relief on all cables
- ⚠️ Ground relay module properly

### Plumbing
- ⚠️ Install sensor with correct flow direction (arrow on body)
- ⚠️ Use food-grade sensor for potable water
- ⚠️ Install after any filters (prevent debris damage)
- ⚠️ Consider bypass valve for sensor maintenance
- ⚠️ Use Teflon tape on threaded connections

### Software
- ⚠️ Flow automation continues even if WiFi/MQTT down
- ⚠️ Hot water solenoid failsafe on separate board
- ⚠️ Monitor for sensor failures (0 pulses = broken turbine)

---

## Future Enhancements

### Possible Additions:
- Water usage tracking per day/week/month
- Low flow alerts (leak detection)
- High flow alerts (burst pipe detection)
- Integration with fresh/grey water tank levels
- Water conservation metrics
- Multiple flow sensors (hot vs cold lines)

---

## Technical Notes

### Why D8?
- GPIO15 (D8) supports **hardware interrupts** (required for accurate pulse counting)
- Boot state is important: D8 must be pulled LOW during boot (already true with INPUT_PULLUP)
- Alternative pins: D1, D2, D5, D6, D7 (but all used for other functions)

### Why Separate Board for Hot Water?
- **Pin availability**: D1 Plumbing board is at capacity
- **Modularity**: Flow sensor stays with plumbing sensors, hot water control is separate
- **Reliability**: Decoupled systems - one board failure doesn't affect both
- **Future expansion**: Separate board can add BME280, cabinet locks, etc.

### ISR Constraints
- ISR must be **fast** (only increments counter)
- Marked `IRAM_ATTR` to run from RAM (faster, avoids flash cache delays)
- No Serial.print() or WiFi operations in ISR
- Volatile variable used for thread-safe counter access

---

## Appendix: Alternative Flow Sensors

### Compatible Models:
- **YF-S201** - 1-30 L/min (most common, 450 pulses/L)
- **YF-B1** - 0.3-6 L/min (higher resolution, 2000 pulses/L)
- **YF-B7** - 2-200 L/min (larger diameter, 150 pulses/L)
- **YF-DN50** - Industrial grade, DN50 (2") pipe size

### Selection Criteria:
- Max flow rate (match your system)
- Pipe size compatibility
- Operating pressure rating
- Food-grade certification (for potable water)
- Temperature rating

---

*Last Updated: October 18, 2025*
*Firmware Version: v1.2.0*
