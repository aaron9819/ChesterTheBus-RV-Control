# PWM Force-Off Fix for Sticky Pump Controller

## Critical Problem
The pump PWM controller was "sticking" - once it received a PWM signal, it would continue running at that speed even after the ESP32 sent 0% PWM. The pump would only stop by pulling the fuse.

## Root Cause
The external PWM controller is **latching/sampling** the PWM signal. Even when `ledcWrite(channel, 0)` was called to set 0% duty cycle, the PWM hardware peripheral on the ESP32 may still be outputting a signal (even at 0%), and the external controller was holding the last non-zero value it saw.

## Solution: Aggressive Pin Control
When stopping the pump, we now:
1. Set PWM duty cycle to 0 via `ledcWrite()`
2. **Detach the LEDC peripheral** from the GPIO pin via `ledcDetachPin()`
3. **Reconfigure pin as digital output** via `pinMode(OUTPUT)`
4. **Force pin physically LOW** via `digitalWrite(LOW)`

This ensures the pin is driven to a solid LOW state with no PWM artifacts.

## Code Changes

### 1. Updated `controlPump()` Function
```cpp
void controlPump(float speedPercent) {
  pumpSpeed = constrain(speedPercent, 0.0, 100.0);

  int pwmValue = map((int)(pumpSpeed * 10), 0, 1000, 0, 255);
  pwmValue = constrain(pwmValue, 0, 255);

  // CRITICAL FIX: If stopping pump, detach PWM and force pin LOW
  if (pumpSpeed == 0.0) {
    ledcWrite(PUMP_PWM_CHANNEL, 0);        // Set duty cycle to 0
    ledcDetachPin(PUMP_PWM_PIN);            // Detach LEDC from pin
    pinMode(PUMP_PWM_PIN, OUTPUT);          // Configure as digital output
    digitalWrite(PUMP_PWM_PIN, LOW);        // Force pin LOW
    Serial.println("⚠️ PUMP STOP: PWM detached, pin forced LOW");
  } else {
    // If starting from 0, reattach PWM
    static bool wasZero = true;
    if (wasZero) {
      ledcAttachPin(PUMP_PWM_PIN, PUMP_PWM_CHANNEL);  // Reattach LEDC
      Serial.println("⚠️ PUMP START: PWM reattached");
      wasZero = false;
    }
    // Write PWM value
    ledcWrite(PUMP_PWM_CHANNEL, pwmValue);

    if (pumpSpeed == 0.0) wasZero = true;
  }

  // ... rest of function
}
```

### 2. Updated MQTT OFF Command Handler
```cpp
} else if (message == "OFF") {
  pumpRunning = false;

  // CRITICAL: Force pump to stop immediately with multiple methods
  controlPump(0);                         // Set speed to 0 via normal method
  ledcWrite(PUMP_PWM_CHANNEL, 0);         // Force PWM to 0
  ledcDetachPin(PUMP_PWM_PIN);            // Detach LEDC from pin
  pinMode(PUMP_PWM_PIN, OUTPUT);          // Set as digital output
  digitalWrite(PUMP_PWM_PIN, LOW);        // Force pin physically LOW

  Serial.println("💧 Hot Water Pump: STOPPED (PWM=0, pin forced LOW)");

  // Publish status immediately
  client.publish(Topic_Pump_Status, "STOPPED", true);
}
```

### 3. Updated PWM Watchdog
Now runs every 2 seconds and aggressively forces pin LOW when pump is off:
```cpp
// PWM Watchdog: Aggressively force PWM state every 2 seconds
if (currentMillis - lastPwmCheck >= 2000) {
  lastPwmCheck = currentMillis;

  if (!pumpRunning) {
    // AGGRESSIVELY force pin LOW when pump is off
    ledcWrite(PUMP_PWM_CHANNEL, 0);
    ledcDetachPin(PUMP_PWM_PIN);
    pinMode(PUMP_PWM_PIN, OUTPUT);
    digitalWrite(PUMP_PWM_PIN, LOW);
    Serial.println("🔍 PWM Watchdog: FORCING pump OFF (detached, pin LOW)");
  }
}
```

## How It Works

### Stopping the Pump (0% Speed):
1. **`ledcWrite(channel, 0)`** - Set LEDC duty cycle to 0%
2. **`ledcDetachPin(pin)`** - Disconnect LEDC peripheral from GPIO pin
3. **`pinMode(pin, OUTPUT)`** - Configure GPIO as standard digital output
4. **`digitalWrite(pin, LOW)`** - Drive pin to solid LOW state

This ensures:
- No PWM signal artifacts
- Pin is physically driven LOW
- External controller sees a clean OFF signal

### Starting the Pump (>0% Speed):
1. **`ledcAttachPin(pin, channel)`** - Reconnect LEDC peripheral to GPIO pin
2. **`ledcWrite(channel, value)`** - Set new PWM duty cycle

This ensures:
- LEDC takes control of the pin again
- PWM signal resumes with new duty cycle

### Watchdog Protection:
Every 2 seconds, the main loop checks pump state:
- If `!pumpRunning`: **Force pin LOW** (detach + digital LOW)
- If `pumpRunning`: Log current PWM value

This provides continuous verification that the pump is in the correct state.

## Why This is Necessary

Some PWM controllers (especially motor controllers and pump drivers):
- **Sample and hold** the PWM signal
- May have **input filtering/debouncing**
- Can **latch onto the last valid PWM duty cycle**
- Don't recognize 0% duty cycle as "OFF"

By detaching the LEDC peripheral and forcing the pin to digital LOW:
- The controller sees a true DC LOW signal (not 0% PWM)
- Any input filtering/sampling is reset
- The controller's latch is cleared

## Testing Checklist
- [ ] Upload new code to ESP32-S2
- [ ] Send MQTT command: "ON" → pump should start at 60%
- [ ] Verify pump speed modulates via PID
- [ ] Send MQTT command: "OFF" → pump should stop immediately
- [ ] Verify PWM pin is at 0V when pump is off (multimeter test)
- [ ] Verify pump stays off (no restart without command)
- [ ] Verify watchdog messages in serial monitor every 2 seconds

## Expected Serial Output

### When Pump Stops:
```
💧 Hot Water Pump: STOPPED (PWM=0, pin forced LOW)
⚠️ PUMP STOP: PWM detached, pin forced LOW
🔍 PWM Watchdog: FORCING pump OFF (detached, pin LOW)
```

### When Pump Starts:
```
💧 Hot Water Pump: STARTED
⚠️ PUMP START: PWM reattached
⚙️ Pump PWM UPDATE: 60.0% → PWM=153/255 (60.0% duty) on GPIO12 [Channel 3, 25000 Hz]
```

## Hardware Verification
If the pump still doesn't stop after this fix:
1. **Measure GPIO12 with multimeter** when pump is "off"
   - Should read 0V (or very close to 0V)
2. **Check PWM controller datasheet**
   - May need enable/disable pin separate from PWM input
   - May need minimum pulse width to recognize OFF state
3. **Check wiring**
   - Ensure GPIO12 connects directly to PWM controller input
   - Check for shorts or miswired connections
4. **PWM controller may be defective**
   - If pin is at 0V but pump still runs, controller is faulty

## Alternative Solutions If This Doesn't Work
1. **Add hardware relay** - Use ESP32 to control relay that cuts power to pump controller
2. **Use enable pin** - If controller has enable pin, use separate GPIO to enable/disable
3. **Replace PWM controller** - Some controllers have better PWM detection logic
