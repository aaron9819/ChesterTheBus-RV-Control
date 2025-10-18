# Cabinet Lock Control System - Setup Instructions

## Overview
This system provides wireless control of a servo-based cabinet lock using:
- **D1 Mini (ESP8266)**: Controls servo motor on pin D4
- **GIGA R1 WiFi + Display Shield**: Touch interface for lock control
- **Communication**: WiFi UDP protocol

## Hardware Setup

### D1 Mini (ESP8266) Wiring
- **Servo Signal Wire** → Pin D4
- **Servo VCC (Red)** → 5V (if available) or 3.3V
- **Servo GND (Brown/Black)** → GND

**Important**: Some servos require 5V. If D1 Mini doesn't provide enough power, use external 5V power supply with common ground.

### GIGA R1 WiFi
- Connect GIGA Display Shield to GIGA R1 board
- No additional wiring needed

## Software Configuration

### Step 1: WiFi Network Setup

You have two options:

#### Option A: Use Existing WiFi Network (Recommended for testing)
1. Open both `main.cpp` files
2. Change these lines to match your existing WiFi:
   ```cpp
   const char* ssid = "YourWiFiName";
   const char* password = "YourWiFiPassword";
   ```

#### Option B: Create Dedicated Network (Better for RV use)
1. Configure your GL.iNet router with:
   - SSID: `RV_Control_Network`
   - Password: `RVControl2024`
2. Leave the code as-is

### Step 2: Set D1 Mini IP Address

After first boot, the D1 Mini will display its IP address on Serial Monitor. You need to update the GIGA R1 code:

1. Flash D1 Mini first
2. Open Serial Monitor (115200 baud)
3. Note the IP address (e.g., 192.168.1.100)
4. Update GIGA R1 code with this IP:
   ```cpp
   IPAddress d1MiniIP(192, 168, 1, 100);  // Change to actual D1 Mini IP
   ```

### Step 3: Upload Code

#### For D1 Mini:
```bash
cd D1Mini_CabLock
pio run --target upload
pio device monitor
```

#### For GIGA R1:
```bash
cd GIGA_R1_TheBrain
pio run --target upload
pio device monitor
```

## Operation

### Lock States
- **UNLOCKED**: Servo at 45° position, button shows GREEN
- **LOCKED**: Servo at 135° position, button shows RED
- **WAITING**: Command sent, waiting for response, button shows YELLOW

### Using the System
1. Power on both devices
2. Wait for WiFi connection (GIGA display shows status)
3. Tap the button on GIGA display to toggle lock state
4. Button color changes based on lock status

### Serial Monitor Output

**D1 Mini shows:**
- WiFi connection status
- IP and MAC address
- Received commands
- Lock state changes

**GIGA R1 shows:**
- WiFi connection status
- Touch events
- Commands sent
- Responses received

## Troubleshooting

### D1 Mini won't connect to WiFi
- Verify WiFi credentials
- Check WiFi is 2.4GHz (ESP8266 doesn't support 5GHz)
- Try moving closer to router

### GIGA R1 can't communicate with D1 Mini
- Verify both devices on same network
- Check D1 Mini IP address is correct in GIGA code
- Use Serial Monitor to see if commands are being sent

### Servo doesn't move
- Check servo wiring
- Verify servo has adequate power supply
- Test with Serial Monitor to see if commands received

### Touch not working on GIGA
- Make sure Display Shield is properly seated
- Check for display initialization errors in Serial Monitor

## Advanced Configuration

### Change Lock Angles
Edit in `D1Mini_CabLock/src/main.cpp`:
```cpp
const int LOCKED_POSITION = 135;    // Adjust as needed (0-180)
const int UNLOCKED_POSITION = 45;   // Adjust as needed (0-180)
```

### Change Button Colors
Edit in `GIGA_R1_TheBrain/src/main.cpp`:
```cpp
lockButton.colorUnlocked = 0x07E0;  // RGB565 format
lockButton.colorLocked = 0xF800;
```

### Change UDP Ports
If ports conflict, update in both files:
```cpp
const unsigned int localUdpPort = 4210;  // D1 Mini
const unsigned int localUdpPort = 4211;  // GIGA R1
```

## Next Steps

Consider adding:
- Multiple cabinet locks (use different UDP ports)
- Status LED on D1 Mini
- Authentication/encryption for commands
- Integration with Home Assistant
- Battery backup monitoring
- Auto-reconnect logic

## Support Commands

The D1 Mini accepts these UDP commands:
- `LOCK` - Lock the cabinet
- `UNLOCK` - Unlock the cabinet
- `TOGGLE` - Toggle current state
- `STATUS` - Request current status

## Pin Reference

### D1 Mini Pin Mapping
- D4 = GPIO2 (Servo control)
- Other pins available for expansion

### GIGA R1 Resources
- Display: 480x800 touchscreen
- Plenty of GPIO available for additional sensors/relays
