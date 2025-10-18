# Double Touch / Lock Toggle Issue - FIXED

## Problem Description
When tapping a single cabinet lock button on the GIGA R1 display, the lock would often toggle twice (lock → unlock → lock or unlock → lock → unlock). However, the "Lock All" and "Unlock All" buttons worked correctly without this issue.

## Root Causes Identified

### 1. **Rapid State Changes + Toggle Logic**
The individual lock buttons used **toggle logic** that changed behavior based on current state:
```cpp
// OLD CODE - PROBLEMATIC
if (lock.state == STATE_LOCKED) {
  sendLockCommand(i, "UNLOCK");
} else {
  sendLockCommand(i, "LOCK");
}
```

**The Problem:**
1. User taps button → sends "LOCK" command
2. Lock state changes to `STATE_WAITING`
3. D1 Mini responds → state changes to `STATE_LOCKED`
4. If touch lingers or bounces, code sees new state and toggles AGAIN
5. Result: Lock → Unlock (double action)

**Why "Lock All" didn't have this issue:**
The "Lock All" / "Unlock All" buttons send **explicit commands** (not toggles), so repeated touches just resend the same command harmlessly.

### 2. **Touch Debounce Too Short**
Original debounce of 300ms wasn't enough to prevent rapid touch events during state transitions.

### 3. **No Command Deduplication on D1 Mini**
If MQTT delivered the same command twice (network retry, QoS issues), the D1 Mini would execute both commands without filtering duplicates.

## Solutions Implemented

### Fix #1: Block Commands During WAITING State (GIGA R1)
**File:** `GIGA_R1_TheBrain/src/main.cpp`

Added check to ignore touches while lock is processing:
```cpp
// Ignore touch if lock is already waiting for a response
if (lock.state == STATE_WAITING) {
  Serial.print("Ignoring touch - ");
  Serial.print(lock.name);
  Serial.println(" is already waiting for response");
  lastTouchTime = currentMillis;
  return;
}
```

**Effect:** Prevents sending multiple commands before the first one completes.

### Fix #2: Increased Touch Debounce (GIGA R1)
**File:** `GIGA_R1_TheBrain/src/main.cpp`

```cpp
// Before: 300ms
const unsigned long TOUCH_DEBOUNCE_MS = 500;  // Increased to 500ms
```

**Effect:** More aggressive filtering of rapid touch events.

### Fix #3: Command Deduplication on D1 Mini (All Cabinet Locks)
**Files:**
- `D1Mini_CabLock_KitchenDriverSide/src/main.cpp`
- `D1Mini_CabLock_KitchenPassSide/src/main.cpp`
- `D1Mini_CabLock_RearDriverSide/src/main.cpp`

Added command cooldown tracking:
```cpp
// Command deduplication
String lastCommand = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_COOLDOWN_MS = 1000; // Ignore duplicates within 1 second
```

And filtering in `processCommand()`:
```cpp
void processCommand(String command) {
  unsigned long currentMillis = millis();

  // Ignore duplicate commands within cooldown period (except STATUS)
  if (command != "STATUS") {
    if (command == lastCommand && (currentMillis - lastCommandTime) < COMMAND_COOLDOWN_MS) {
      Serial.print("⚠️  Ignoring duplicate command within cooldown: ");
      Serial.println(command);
      return;
    }
    lastCommand = command;
    lastCommandTime = currentMillis;
  }

  // ... rest of command processing
}
```

**Effect:** D1 Mini ignores duplicate LOCK/UNLOCK commands received within 1 second, but still accepts STATUS requests.

## Testing the Fixes

### Expected Behavior After Fix:

1. **Single Lock Button Tap:**
   - ✅ Sends one command only
   - ✅ State changes to WAITING immediately
   - ✅ Further touches ignored until state resolves
   - ✅ Lock changes state once

2. **Rapid Multiple Taps:**
   - ✅ First tap processes normally
   - ✅ Subsequent taps within 500ms ignored (debounce)
   - ✅ No double-toggling

3. **Network Issues (Duplicate MQTT Messages):**
   - ✅ D1 Mini filters duplicate commands
   - ✅ Lock only responds to first command
   - ✅ Debug message in serial: "⚠️ Ignoring duplicate command within cooldown"

4. **Lock All / Unlock All:**
   - ✅ Continue working normally
   - ✅ Explicit commands don't have toggle issues

### Debug Output to Monitor:

**GIGA R1 Serial Monitor:**
```
Touch detected - Raw: X:123 Y:456 | Transformed: X:456 Y:357
Sending command to Kitchen Driver: LOCK
```

If working correctly, you should NOT see:
```
Sending command to Kitchen Driver: LOCK
Sending command to Kitchen Driver: UNLOCK  ← This should NOT appear
```

If WAITING state protection kicks in:
```
Ignoring touch - Kitchen Driver is already waiting for response
```

**D1 Mini Serial Monitor:**
```
MQTT message received on topic: CabLockKitchenDriverSideCommand - Message: LOCK
>>> LOCKING Cabinet: KitchenDriverSide
```

If duplicate filtering works:
```
⚠️  Ignoring duplicate command within cooldown: LOCK
```

## Files Modified

### GIGA R1 Brain:
- ✅ `GIGA_R1_TheBrain/src/main.cpp`
  - Added STATE_WAITING check before sending commands
  - Increased debounce from 300ms to 500ms

### Cabinet Lock Controllers (All 3):
- ✅ `D1Mini_CabLock_KitchenDriverSide/src/main.cpp`
- ✅ `D1Mini_CabLock_KitchenPassSide/src/main.cpp`
- ✅ `D1Mini_CabLock_RearDriverSide/src/main.cpp`
  - Added command deduplication with 1-second cooldown
  - STATUS commands exempt from cooldown

## Deployment

### 1. Upload to GIGA R1:
```bash
cd GIGA_R1_TheBrain
pio run --target upload
```

### 2. Upload to all Cabinet Locks:
```bash
# Kitchen Driver Side
cd D1Mini_CabLock_KitchenDriverSide
pio run --target upload

# Kitchen Pass Side
cd ../D1Mini_CabLock_KitchenPassSide
pio run --target upload

# Rear Driver Side
cd ../D1Mini_CabLock_RearDriverSide
pio run --target upload
```

Or use OTA:
```bash
pio run --target upload --upload-port CabLock-KitchenDriverSide.local
pio run --target upload --upload-port CabLock-KitchenPassSide.local
pio run --target upload --upload-port CabLock-RearDriverSide.local
```

## Additional Notes

### Why Not Remove Toggle Logic Entirely?
The toggle logic is actually user-friendly - it lets the user tap once to change state without thinking about whether they want LOCK or UNLOCK. The fixes preserve this UX while preventing the double-trigger bug.

### Alternative Approach (If Issues Persist)
If you still see double-triggering after these fixes, you could switch to **explicit Lock/Unlock buttons** instead of toggle buttons:
- Each lock gets TWO buttons: 🔒 Lock and 🔓 Unlock
- No toggle logic needed
- Clearer user intent
- More button real estate needed

### Cooldown Tuning
If 1 second feels too long (commands seem to be ignored), you can reduce:
```cpp
const unsigned long COMMAND_COOLDOWN_MS = 500; // Try 500ms instead of 1000ms
```

If still seeing doubles, increase:
```cpp
const unsigned long COMMAND_COOLDOWN_MS = 1500; // Try 1.5 seconds
```

---

**Issue Status:** ✅ RESOLVED
**Date Fixed:** October 7, 2025
**Tested:** Awaiting user confirmation after deployment
