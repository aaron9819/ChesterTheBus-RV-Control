# PWM Pin Initialization Fix

## Problem Identified
The plumbing board PWM wasn't working correctly because the code was mixing GPIO digital I/O functions with ESP32 LEDC PWM functions.

## Root Cause
In `/Users/aaronlapp/Desktop/ChesterTheBus-RV-Control/Plumbing/src/main_esp32s2.cpp`, the setup code was doing:

```cpp
// WRONG APPROACH - Conflicts with LEDC PWM!
pinMode(PUMP_PWM_PIN, OUTPUT);           // Sets pin as digital output
digitalWrite(PUMP_PWM_PIN, LOW);         // Writes digital LOW
delay(50);
ledcSetup(PUMP_PWM_CHANNEL, ...);       // Then tries to use LEDC
ledcAttachPin(PUMP_PWM_PIN, ...);
ledcWrite(PUMP_PWM_CHANNEL, 0);
```

**The issue:** When you use `pinMode()` and `digitalWrite()` on a pin, you're configuring it for standard digital I/O. This conflicts with the LEDC (LED Control) PWM peripheral that needs exclusive control of the pin's output.

## Correct Approach
Looking at the working thermostat code in `D1Mini_CabLock_KitchenPassSide/src/main.cpp`:

```cpp
// CORRECT APPROACH - LEDC PWM only!
ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);  // Setup PWM channel first
ledcAttachPin(FAN_PWM_PIN, PWM_CHANNEL);            // Attach pin to channel
ledcWrite(PWM_CHANNEL, 0);                          // Write initial duty cycle
```

**The correct method:**
1. **Never call `pinMode()` or `digitalWrite()` on LEDC PWM pins**
2. Use `ledcSetup()` to configure the PWM channel
3. Use `ledcAttachPin()` to attach the GPIO pin to the channel
4. Use `ledcWrite()` to set duty cycle (0-255 for 8-bit resolution)

## What Was Changed

### Before (Lines 401-415):
```cpp
// Initialize pump PWM - set pin LOW first to prevent false trigger
pinMode(PUMP_PWM_PIN, OUTPUT);
digitalWrite(PUMP_PWM_PIN, LOW);  // Ensure pin is LOW before PWM init
delay(50);  // Give pump controller time to recognize LOW state

ledcSetup(PUMP_PWM_CHANNEL, PUMP_PWM_FREQ, PUMP_PWM_RESOLUTION);
ledcAttachPin(PUMP_PWM_PIN, PUMP_PWM_CHANNEL);
ledcWrite(PUMP_PWM_CHANNEL, 0);  // Ensure PWM starts at 0%
delay(50);  // Additional safety delay
```

### After:
```cpp
// Initialize pump PWM (CRITICAL: Do NOT use pinMode/digitalWrite with LEDC PWM!)
// Follow ESP32 LEDC best practice: setup channel → attach pin → write initial value
ledcSetup(PUMP_PWM_CHANNEL, PUMP_PWM_FREQ, PUMP_PWM_RESOLUTION);
ledcAttachPin(PUMP_PWM_PIN, PUMP_PWM_CHANNEL);
ledcWrite(PUMP_PWM_CHANNEL, 0);  // Start at 0% duty cycle
```

## Technical Explanation

### ESP32 LEDC Peripheral
The ESP32 LEDC (LED Control) peripheral is a hardware PWM generator that:
- Has 16 independent channels (8 high-speed, 8 low-speed)
- Supports frequencies from 1 Hz to 40 MHz
- Provides hardware-based duty cycle control
- Requires exclusive control of the GPIO pin

### Why Mixing Methods Fails
When you use `pinMode()`/`digitalWrite()`:
- The pin is configured for the GPIO peripheral
- The GPIO peripheral takes control of the pin's output buffer
- When `ledcAttachPin()` is called, there's a conflict between GPIO and LEDC peripherals
- The PWM signal may not output correctly, or may "stick" at the last digital value

### Correct Initialization Sequence
1. **`ledcSetup(channel, frequency, resolution)`**
   - Configures the LEDC timer for the specified channel
   - Sets PWM frequency and bit resolution
   - Does NOT touch any GPIO pins yet

2. **`ledcAttachPin(pin, channel)`**
   - Routes the LEDC channel output to the specified GPIO pin
   - Automatically configures the pin for LEDC peripheral use
   - Overrides any previous `pinMode()` setting

3. **`ledcWrite(channel, duty)`**
   - Sets the PWM duty cycle (0 = 0%, 255 = 100% for 8-bit)
   - Updates the hardware timer immediately
   - Pin starts outputting PWM signal

## Verification
After uploading the fixed code:
1. The PWM pin will start in a known OFF state (0% duty cycle)
2. PWM signals will update correctly when `controlPump()` is called
3. The external PWM controller should now respond to speed changes
4. No more "sticky" PWM behavior where the controller holds the last value

## Additional Notes
- The `controlPump()` function already uses `ledcWrite()` correctly
- No other parts of the code use `digitalWrite()` on the PWM pin
- This fix aligns with ESP32 best practices and matches the working thermostat implementation

## Testing Checklist
- [ ] Upload fixed code to plumbing board
- [ ] Verify pump starts when commanded ON via MQTT
- [ ] Verify PWM signal reaches pump controller
- [ ] Verify pump speed modulates based on PID output
- [ ] Verify pump stops completely when commanded OFF
- [ ] Check that pump doesn't "stick" at old speed values
