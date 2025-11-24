# Automatic Pump Control Implementation

## Overview
The hot water pump now features automatic operation that starts and stops based on actual system demand. The pump automatically turns on when water flow is detected or when any heating-related valve opens, and turns off when no demand exists.

## Features Implemented

### 1. Flow Sensor Integration
**Hardware:** YF-S201 Hall Effect flow sensor on GPIO17
- Interrupt-driven pulse counting
- Flow rate calculation in liters per minute
- Noise filtering (minimum 10 pulses/second threshold)
- 3-second timeout (flow considered stopped if no pulses for 3 seconds)
- Flow detection triggers automatic pump start

### 2. Valve-Based Triggering
The pump automatically starts when ANY of these heating-related systems activates:
- **Grey Water Heater** - Tank heating loop
- **Rear Loop Valve** - Rear floor heating
- **Engine Loop Valve** - Engine bay heating
- **Front Loop Valve** - Front cabin heating

### 3. Intelligent Pump Management
**Automatic Start Conditions:**
- Water flow detected (someone turned on a faucet/shower)
- Any heating valve opens (heat demand)

**Automatic Stop Conditions:**
- No water flow detected for 3+ seconds, AND
- All heating valves are closed
- Pump only stops if NOTHING else is on

### 4. Operating Modes

#### AUTO Mode (Default)
```
Pump Mode: AUTO
├─ Starts automatically when:
│  ├─ Flow sensor detects water usage
│  └─ Any heating valve opens
└─ Stops automatically when:
   ├─ No flow for 3+ seconds
   └─ All heating valves closed
```

#### MANUAL Mode
```
Pump Mode: MANUAL
├─ User controls via MQTT commands
├─ "ON" command: Force pump on
├─ "OFF" command: Force pump off
└─ Ignores flow sensor and valve states
```

## MQTT Control

### Commands
**Topic:** `HotWaterPumpCommand`

| Command | Action | Mode Switch |
|---------|--------|-------------|
| `AUTO` | Enable automatic control | → AUTO |
| `ON` | Force pump on | → MANUAL |
| `OFF` | Force pump off | → MANUAL |

### Status Messages
**Topic:** `HotWaterPumpStatus`
- `RUNNING` - Pump is on
- `STOPPED` - Pump is off

### Flow Sensor Data
**Topics:**
- `HotWaterFlowRate` - Flow rate in L/min
- `HotWaterFlowStatus` - `FLOWING` or `STOPPED`

## PID Setpoint Management

The pump's PID controller automatically adjusts pressure setpoints based on active valves:

**Base Pressure:** 4.0 PSI (when pump running but no valves open)

**Pressure Boosts:**
- Engine Loop: +4.0 PSI (highest demand)
- Front Loop: +2.0 PSI
- Rear Loop: +1.6 PSI
- Grey Tank Loop: +1.6 PSI

**Example:** If Engine Loop and Rear Loop are both open:
- Setpoint = 4.0 (base) + 4.0 (engine) = 8.0 PSI
- System uses highest individual boost, not cumulative

## Code Implementation

### Key Variables
```cpp
// Flow Sensor
volatile unsigned long flowPulseCount = 0;
volatile unsigned long lastPulseTime = 0;
bool flowDetected = false;
float flowRate = 0.0;  // L/min

// Pump Control
String pumpMode = "AUTO";  // AUTO or MANUAL
bool pumpAutoStarted = false;  // Track auto vs manual start
```

### Flow Sensor ISR
```cpp
void IRAM_ATTR flowSensorISR() {
  flowPulseCount++;
  recentPulseCount++;
  lastPulseTime = millis();
}
```

### Auto Pump Logic
```cpp
bool shouldPumpBeRunning() {
  return flowDetected || greyWaterHeaterOn ||
         rearLoopOpen || engineLoopOpen || frontLoopOpen;
}

void handleAutoPump() {
  if (pumpMode != "AUTO") return;

  bool pumpShouldRun = shouldPumpBeRunning();

  if (pumpShouldRun && !pumpRunning) {
    // Auto start pump
    pumpRunning = true;
    pumpAutoStarted = true;
    controlPump(60.0);  // Startup speed
  }
  else if (!pumpShouldRun && pumpRunning && pumpAutoStarted) {
    // Auto stop pump (only if auto-started)
    pumpRunning = false;
    pumpAutoStarted = false;
    controlPump(0);
  }
}
```

## Serial Monitor Output

### Flow Detection
```
💦 Water flow: DETECTED
   Flow rate: 2.5 L/min (112 pulses)
⚙️ AUTO START: Pump starting because flow detected
💧 Hot Water Pump: RUNNING
```

### Valve Opening
```
🌊 Engine Loop: OPEN
⚙️ AUTO START: Pump starting because engine-loop
🔧 PID Setpoint: 8.0 PSI (base + engine loop boost)
```

### Auto Stop
```
💦 Water flow: STOPPED
   Recent pulses: 3 (threshold: 10)
🌊 Front Loop: CLOSED
⚙️ AUTO STOP: Pump stopping (no valves open, no flow)
💧 Hot Water Pump: STOPPED
```

### Manual Override
```
MQTT RX [HotWaterPumpCommand]: ON
💧 Hot Water Pump: MANUAL START
```

## Usage Examples

### Example 1: Morning Shower
1. User turns on hot water faucet
2. Flow sensor detects flow (>10 pulses/sec)
3. Pump auto-starts at 60% (startup boost)
4. PID adjusts to maintain 4.0 PSI (base setpoint)
5. User finishes shower, turns off faucet
6. No flow detected for 3 seconds
7. No heating valves are open
8. Pump auto-stops

### Example 2: Floor Heating
1. Thermostat activates, sends trigger to GPIO13
2. Engine loop valve opens
3. Pump auto-starts
4. PID adjusts to maintain 8.0 PSI (4.0 base + 4.0 engine boost)
5. Thermostat satisfied, deactivates trigger
6. Engine loop valve closes
7. No other valves open, no flow detected
8. Pump auto-stops

### Example 3: Multiple Demands
1. Grey water heater turns on (valve opens)
2. Pump auto-starts, setpoint = 5.6 PSI (4.0 + 1.6)
3. User turns on faucet (flow detected)
4. Setpoint stays at 5.6 PSI (grey loop boost)
5. User finishes, flow stops
6. Grey heater still on, pump continues running
7. Grey water reaches temperature, heater turns off
8. No demand remaining, pump auto-stops

### Example 4: Manual Override
1. System in AUTO mode, pump off
2. User sends MQTT command "ON"
3. Pump switches to MANUAL mode, starts
4. Pump stays on regardless of valves or flow
5. User sends MQTT command "AUTO"
6. Pump switches back to AUTO mode
7. If no demand, pump auto-stops immediately

## Benefits

### Energy Efficiency
- Pump only runs when actually needed
- No wasted electricity or wear when demand is zero
- Extends pump lifespan

### User Convenience
- No manual pump control required
- Instant hot water response
- Seamless operation

### Safety
- PID control prevents over-pressure
- Alarm system monitors safety limits
- 5-second startup grace period

### Flexibility
- Can switch between AUTO and MANUAL modes
- Manual override available when needed
- All modes preserve safety features

## Troubleshooting

### Pump Won't Auto-Start
**Check:**
- Mode is set to AUTO: `mosquitto_pub -h 192.168.8.1 -t HotWaterPumpCommand -m "AUTO"`
- Flow sensor is connected to GPIO17
- Valves are actually opening (check relay status)

### Pump Won't Auto-Stop
**Check:**
- Verify no valves are stuck open
- Check flow sensor isn't generating false pulses
- Ensure 3-second timeout has passed since last flow

### False Flow Detection
**Symptom:** Pump starts when no water is flowing
**Solution:** Increase `MIN_PULSES_FOR_FLOW` threshold (currently 10)

### Pump Cycles On/Off Rapidly
**Symptom:** Pump starts and stops every few seconds
**Possible Causes:**
- Flow sensor near threshold (dripping faucet)
- Valve bouncing (relay chatter)
**Solution:** Increase `FLOW_TIMEOUT` from 3 to 5 seconds

## Configuration Constants

```cpp
// In main_esp32s2.cpp, adjust these values if needed:

const unsigned long FLOW_CALC_INTERVAL = 1000;        // Flow calculation period
const unsigned long FLOW_TIMEOUT = 3000;               // No-flow timeout (3 sec)
const unsigned long MIN_PULSES_FOR_FLOW = 10;          // Pulse threshold per second

const float BASE_PRESSURE_SETPOINT = 4.0;              // Base pressure (PSI)
const float ENGINE_LOOP_PSI_BOOST = 4.0;               // Engine loop boost
const float FRONT_LOOP_PSI_BOOST = 2.0;                // Front loop boost
const float REAR_LOOP_PSI_BOOST = 1.6;                 // Rear loop boost
const float GREY_LOOP_PSI_BOOST = 1.6;                 // Grey tank boost
```

## Testing Checklist

- [ ] Upload updated code to ESP32-S2
- [ ] Verify pump starts at 10kHz PWM frequency
- [ ] Test flow sensor by running water
- [ ] Verify pump auto-starts when flow detected
- [ ] Verify pump auto-stops 3 seconds after flow stops
- [ ] Test each valve triggers pump start
- [ ] Verify pressure setpoint adjusts with valve states
- [ ] Test manual override (ON/OFF commands)
- [ ] Test AUTO mode restoration
- [ ] Verify pump stays running with multiple demands
- [ ] Confirm pump only stops when all demands clear

## Future Enhancements

**Potential additions:**
- Adjustable timers via MQTT
- Flow rate-based pressure adjustment
- Usage statistics (gallons/day)
- Predictive start (anticipate demand)
- Integration with home assistant automations
