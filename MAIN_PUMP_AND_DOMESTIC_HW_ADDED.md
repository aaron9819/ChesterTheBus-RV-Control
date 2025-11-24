# Main Water Pump & Domestic Hot Water Loop Controls Added

## Overview
Added two new relay-controlled systems to the plumbing page:
1. **Main Water Pump** - Controls city water pressure pump (Pin 18)
2. **Domestic Hot Water Loop** - Controls thermo-actuator for hot water circulation (Pin 10)

## Changes Summary

### ESP32-S2 Plumbing Board (`Plumbing/src/main_esp32s2.cpp`)

#### Pin Assignments
```cpp
#define MAIN_WATER_PUMP_PIN 18    // Main water pump relay (Active LOW)
#define DOMESTIC_HW_LOOP_PIN 10   // Domestic hot water loop relay (Active LOW)
```

#### MQTT Topics
```cpp
const char* Topic_MainPump_Command = "MainWaterPumpCommand";
const char* Topic_MainPump_Status = "MainWaterPumpStatus";
const char* Topic_DomesticHW_Command = "DomesticHotWaterCommand";
const char* Topic_DomesticHW_Status = "DomesticHotWaterStatus";
```

#### State Variables
```cpp
bool mainWaterPumpOn = false;
bool domesticHWLoopOpen = false;
```

#### Control Functions
```cpp
void controlMainWaterPump(bool turnOn) {
  mainWaterPumpOn = turnOn;
  digitalWrite(MAIN_WATER_PUMP_PIN, turnOn ? LOW : HIGH);  // Active LOW
  mqttClient.publish(Topic_MainPump_Status, turnOn ? "ON" : "OFF");
}

void controlDomesticHWLoop(bool open) {
  domesticHWLoopOpen = open;
  digitalWrite(DOMESTIC_HW_LOOP_PIN, open ? LOW : HIGH);  // Active LOW
  mqttClient.publish(Topic_DomesticHW_Status, open ? "OPEN" : "CLOSED");
}
```

#### MQTT Integration
- Subscribed to command topics in `subscribeToTopics()`
- Added callback handlers in `mqttCallback()` to process commands
- State persistence via EEPROM (addresses 20 & 21)
- States published on boot and after changes

### GIGA R1 Display (`GIGA_R1_TheBrain/src/main.cpp`)

#### MQTT Topics
```cpp
const char* topic_main_pump_cmd = "MainWaterPumpCommand";
const char* topic_main_pump_status = "MainWaterPumpStatus";
const char* topic_domestic_hw_cmd = "DomesticHotWaterCommand";
const char* topic_domestic_hw_status = "DomesticHotWaterStatus";
```

#### State Variables
```cpp
bool mainWaterPumpOn = false;
bool domesticHWLoopOpen = false;
```

#### UI Layout Changes
- Changed plumbing page from 2 columns x 5 rows to **3 columns x 4 rows**
- Button dimensions: 240px wide x 85px tall
- Column X positions: 20, 275, 530
- Row Y positions: 70, 170, 270, 370
- Connection status at line 462

#### Button Layout (3 Columns x 4 Rows)
```
Row 1: Fresh Water | Grey Water | Exhaust Fan
Row 2: Rear Loop | Engine Loop | Diesel Heater
Row 3: Front Loop | Hot Water Pump | Main Water Pump
Row 4: Domestic HW Loop | (empty) | (empty)
```

#### Touch Handlers
- Main Water Pump button: Toggles ON/OFF, sends MQTT command
- Domestic HW Loop button: Toggles OPEN/CLOSED, sends MQTT command
- Both update lastTouchTime and touchCurrentlyPressed flag

#### MQTT Callbacks
- Subscribed to both status topics
- Update state variables when status messages received
- Trigger UI redraw when on plumbing page

## Operation

### Main Water Pump
- **Purpose**: Controls the main water pump for city water connection
- **States**: ON / OFF
- **Control**: Manual button on plumbing page
- **Note**: Independent of automatic pump control system

### Domestic Hot Water Loop
- **Purpose**: Controls thermo-actuator for hot water circulation
- **States**: OPEN / CLOSED
- **Control**: Manual button on plumbing page
- **Note**: Independent of flow sensor and heating valves

## MQTT Communication Flow

### Command Flow
1. User touches button on GIGA display
2. GIGA publishes command: "ON"/"OFF" or "OPEN"/"CLOSE"
3. Plumbing board receives command via MQTT
4. Plumbing board executes relay control
5. Plumbing board publishes status confirmation

### Status Flow
1. Plumbing board changes state (boot, EEPROM restore, or command)
2. Plumbing board publishes status message
3. GIGA receives status update
4. GIGA updates state variable
5. GIGA redraws UI if on plumbing page

## Hardware Configuration

### Plumbing Board Relay Outputs
| Pin | Function | Active State | Default |
|-----|----------|--------------|---------|
| 18  | Main Water Pump | LOW (ON) | HIGH (OFF) |
| 10  | Domestic HW Loop | LOW (OPEN) | HIGH (CLOSED) |

Both relays are **Active LOW**:
- HIGH = Relay OFF = Device OFF/CLOSED
- LOW = Relay ON = Device ON/OPEN

## State Persistence
- States saved to EEPROM on plumbing board
- Main water pump state: Address 20
- Domestic HW loop state: Address 21
- Restored on boot with 100ms delay between restorations

## Testing Checklist
- [ ] Upload plumbing board code
- [ ] Upload GIGA display code
- [ ] Verify main water pump button toggles ON/OFF
- [ ] Verify domestic HW loop button toggles OPEN/CLOSED
- [ ] Confirm MQTT messages sent from GIGA
- [ ] Confirm relay activation on plumbing board
- [ ] Verify status updates displayed on GIGA
- [ ] Test state persistence (power cycle plumbing board)
- [ ] Verify states independent of auto pump logic

## Notes
- Both controls are completely independent of the automatic pump control system
- These are simple manual ON/OFF relay controls
- Main water pump is for city water connection scenarios
- Domestic HW loop circulates hot water through the system
- No interaction with flow sensor or heating valve logic
