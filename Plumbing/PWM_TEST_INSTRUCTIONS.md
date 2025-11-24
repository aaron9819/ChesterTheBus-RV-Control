# PWM Test Program Instructions

## Purpose
This test program isolates the PWM issue by running a simple up/down ramp on GPIO12. It will help determine if the problem is in the ESP32 code or the external PWM controller hardware.

## How to Use

### Step 1: Switch to Test Program
```bash
cd /Users/aaronlapp/Desktop/ChesterTheBus-RV-Control/Plumbing/src

# Backup the main program
mv main_esp32s2.cpp main_esp32s2.cpp.bak

# Activate test program
mv pwm_test.cpp.bak main.cpp
```

### Step 2: Upload Test Program
```bash
cd /Users/aaronlapp/Desktop/ChesterTheBus-RV-Control/Plumbing
pio run -t upload
```

### Step 3: Monitor Serial Output
```bash
pio device monitor
```

### Step 4: Watch Pump Behavior
The test will run 4 tests in sequence:

**TEST 1: Ramp UP (0% → 100%)**
- PWM increases from 0 to 255 in steps of 5
- Each step lasts 0.5 seconds
- **Watch pump**: Should gradually speed up

**TEST 2: Ramp DOWN (100% → 0%)**
- PWM decreases from 255 to 0 in steps of 5
- Each step lasts 0.5 seconds
- **Watch pump**: Should gradually slow down to stop
- **CRITICAL**: Does pump actually stop at 0%?

**TEST 3: Force Digital LOW**
- Sets PWM to 0
- Detaches LEDC peripheral
- Forces pin to digital OUTPUT LOW
- Holds for 10 seconds
- **CRITICAL**: If pump is STILL running, controller is defective!

**TEST 4: Quick Cycles**
- 5 cycles of: 60% for 3 sec → 0% + digital LOW for 3 sec
- **CRITICAL**: Does pump stop during each OFF cycle?

### Step 5: Restore Normal Program
```bash
cd /Users/aaronlapp/Desktop/ChesterTheBus-RV-Control/Plumbing/src

# Deactivate test program
mv main.cpp pwm_test.cpp.bak

# Restore main program
mv main_esp32s2.cpp.bak main_esp32s2.cpp
```

Then upload normal program:
```bash
cd /Users/aaronlapp/Desktop/ChesterTheBus-RV-Control/Plumbing
pio run -t upload
```

## What to Look For

### If pump STOPS correctly in TEST 2 (at 0% PWM):
✅ **PWM controller is working properly**
- Problem is likely in the main program logic
- Check when `controlPump(0)` is being called
- Check if `pumpRunning` flag is being set correctly

### If pump STOPS in TEST 3 (digital LOW only):
⚠️ **PWM controller doesn't recognize 0% PWM as OFF**
- Controller requires true DC LOW signal, not 0% PWM
- The aggressive detach/digital LOW fix should work
- May need to always use digital LOW for OFF state

### If pump NEVER STOPS (even in TEST 3):
❌ **PWM controller is defective or miswired**
- Pin is physically LOW but pump still runs
- Possible issues:
  1. **Wrong wire connected** - Not connected to PWM input
  2. **Controller needs enable pin** - Separate enable/disable signal required
  3. **Controller is damaged** - Replace PWM controller
  4. **Power issue** - Controller has power but ignoring PWM input

### If pump STOPS intermittently:
⚠️ **Timing or noise issue**
- Controller may have input filtering/debouncing
- May need longer LOW periods
- May need capacitor on PWM input line
- May need pull-down resistor on GPIO12

## Serial Output Example

```
============================================
  ESP32-S2 PWM TEST PROGRAM
  Testing GPIO12 PWM Output
============================================

Initializing PWM...
✓ PWM initialized on GPIO12 at 25000 Hz
✓ Starting at 0% duty cycle

Test sequence will begin in 3 seconds...

========================================
TEST SEQUENCE START
========================================

TEST 1: Ramping UP (0% → 100%)
  PWM = 0/255 (0.0%)
  PWM = 5/255 (2.0%)
  PWM = 10/255 (3.9%)
  ...
  PWM = 255/255 (100.0%)

✓ Reached 100% - Holding for 5 seconds...

TEST 2: Ramping DOWN (100% → 0%)
  PWM = 255/255 (100.0%)
  PWM = 250/255 (98.0%)
  ...
  PWM = 0/255 (0.0%)

✓ Reached 0% - Holding for 10 seconds...

TEST 3: Forcing pin to DIGITAL LOW
  Step 1: Setting PWM to 0
  Step 2: Detaching LEDC from pin
  Step 3: Setting pinMode to OUTPUT
  Step 4: Writing digital LOW

✓ Pin forced to digital LOW - Holding for 10 seconds...
  ⚠️  If pump is STILL running, the controller is defective!

TEST 4: Quick ON/OFF cycles (60% → 0%)
  Cycle 1: Reattaching PWM...
    → 60% PWM (153/255)
    → 0% PWM
    → Detach + Digital LOW
  ...

✓ Quick cycle test complete

========================================
ALL TESTS COMPLETE
========================================
Waiting 30 seconds before repeating...
```

## Hardware Verification

If pump doesn't stop in TEST 3, use a multimeter:

1. **Measure GPIO12 voltage during TEST 3**
   - Should read 0V (or very close, < 0.1V)
   - If reading > 0.5V, ESP32 pin is not LOW
   - If reading 0V but pump runs, controller is faulty

2. **Check PWM controller input**
   - Verify GPIO12 is connected to PWM input
   - Check for loose or broken wires
   - Check for shorts to other signals

3. **Check PWM controller enable pin (if present)**
   - Some controllers need separate enable signal
   - May be labeled EN, ENABLE, or similar
   - May default to ON when floating

## Next Steps Based on Results

### Pump stops in TEST 2 or TEST 3:
→ Fix is in the main program logic (check `pumpRunning` state management)

### Pump never stops:
→ Check wiring, then likely need:
- Hardware relay to cut power to controller
- Separate enable pin control
- New PWM controller

### Pump behavior is inconsistent:
→ May need:
- Pull-down resistor (1kΩ-10kΩ from GPIO12 to GND)
- Capacitor filtering (0.1µF on PWM input)
- Longer delay times in main program
