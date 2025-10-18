# GIGA R1 WiFi - Main Controller

## Description
Main control interface using GIGA R1 WiFi with Display Shield. Provides touch interface for controlling cabinet locks and other RV systems.

## Hardware
- **Board**: Arduino GIGA R1 WiFi
- **Display**: GIGA Display Shield (480x800 touchscreen)

## Features
- Touch-based UI with color-coded buttons
- WiFi UDP communication with D1 Mini devices
- Visual status feedback
- Command timeout handling
- Automatic status polling

## Display Interface
- **Title**: Shows "Cabinet Lock"
- **Status**: Current lock state with color coding
  - GREEN = Unlocked
  - RED = Locked
  - YELLOW = Waiting for response
- **Button**: Touch to toggle lock state
- **Footer**: IP addresses for debugging

## Configuration
Edit in `src/main.cpp`:
```cpp
const char* ssid = "RV_Control_Network";
const char* password = "RVControl2024";
IPAddress d1MiniIP(192, 168, 1, 100);  // Update with actual D1 Mini IP
```

## Building and Uploading
```bash
pio run --target upload
pio device monitor
```

## Serial Output
Monitor at 115200 baud to see:
- Display initialization
- WiFi connection status
- Touch events
- UDP communication
- State changes

## Future Expansion
This is the "brain" of the RV control system. Additional features will be added:
- Multiple device control
- Tank monitoring
- Temperature sensors
- Leveling system
- Integration with Victron Energy via Ekrano GX
