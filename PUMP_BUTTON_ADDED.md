# Hot Water Pump Button Added to GIGA Display

## Overview
A hot water pump control button has been successfully added to the GIGA R1 plumbing page, allowing users to start and stop the hot water pump via the touchscreen interface.

## Changes Made

### 1. MQTT Topics (Lines 69-70)
```cpp
const char* topic_pump_cmd = "HotWaterPumpCommand";
const char* topic_pump_status = "HotWaterPumpStatus";
```
- Added command and status topics for pump control

### 2. State Variable (Line 210)
```cpp
bool pumpRunning = false;
```
- Tracks the current running state of the hot water pump

### 3. MQTT Subscription (Line 495)
```cpp
mqttClient.subscribe(topic_pump_status);
```
- Subscribes to pump status updates from the plumbing board

### 4. MQTT Callback Handler (Lines 600-611)
```cpp
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
```
- Handles incoming pump status messages
- Updates display when pump state changes
- Accepts both "RUNNING" and "ON" as active states

### 5. Button Display (Lines 1614-1623)
```cpp
// Hot Water Pump
uint16_t pumpColor = pumpRunning ? COLOR_UNLOCKED : COLOR_LOCKED;
display.fillRoundRect(rightX, row4Y, buttonWidth, buttonHeight, 10, pumpColor);
display.drawRoundRect(rightX, row4Y, buttonWidth, buttonHeight, 10, COLOR_TEXT_WHITE);
display.setTextSize(2);
display.setTextColor(COLOR_TEXT_BLACK);
display.setCursor(rightX + 20, row4Y + 25);
display.println("HOT WATER PUMP");
display.setCursor(rightX + 80, row4Y + 50);
display.println(pumpRunning ? "ON" : "OFF");
```
- Added button to row 4, right side (previously empty)
- Green background when running, gray when stopped
- Displays "HOT WATER PUMP" label and "ON"/"OFF" status

### 6. Touch Handler (Lines 1786-1792)
```cpp
// Check Hot Water Pump button
if (x >= rightX && x <= (rightX + buttonWidth) && y >= row4Y && y <= (row4Y + buttonHeight)) {
  sendPlumbingCommand(topic_pump_cmd, pumpRunning ? "OFF" : "ON");
  lastTouchTime = currentMillis;
  touchCurrentlyPressed = true;
  return;
}
```
- Detects touch on pump button area
- Sends "ON" when stopped, "OFF" when running
- Uses existing debounce logic

## Button Layout
The plumbing page now has a complete 4×2 grid:

**Row 1:**
- Fresh Water Heater | Grey Water Heater

**Row 2:**
- Exhaust Fan | Rear Loop Valve

**Row 3:**
- Engine Loop Valve | Diesel Heater

**Row 4:**
- Front Loop Valve | **Hot Water Pump** ← NEW!

## Integration with Plumbing Board
The GIGA display communicates with the ESP32-S2 plumbing board via MQTT:

1. **User presses pump button** → GIGA sends "ON" to `HotWaterPumpCommand`
2. **Plumbing board receives command** → Starts pump with PID control
3. **Plumbing board publishes status** → Sends "RUNNING" to `HotWaterPumpStatus`
4. **GIGA receives status** → Updates button color and text

## Testing Checklist
- [ ] Upload updated code to GIGA R1
- [ ] Verify pump button appears on plumbing page
- [ ] Press pump button - should send MQTT command
- [ ] Confirm plumbing board receives command and starts pump
- [ ] Verify button color changes from gray to green when pump starts
- [ ] Press pump button again - should send OFF command
- [ ] Confirm pump stops and button returns to gray

## Notes
- The pump controller on the ESP32-S2 board handles all safety logic (pressure limits, startup delays, etc.)
- The GIGA button simply sends ON/OFF commands and displays status
- Pump will automatically modulate speed based on PID control regardless of button state
- Hardware triggers from thermostat continue to work independently of this UI control
