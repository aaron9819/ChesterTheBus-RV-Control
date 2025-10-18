# D1 Mini Cabinet Lock Controller

## Description
WiFi-enabled servo controller for cabinet lock mechanism. Receives commands via UDP and controls servo motor.

## Hardware
- **Board**: D1 Mini (ESP8266)
- **Servo**: Connected to pin D4
- **Power**: USB or 5V external

## Features
- WiFi UDP communication
- Servo position control (45° and 135°)
- Command acknowledgment
- Status reporting
- Serial debugging output

## Commands Accepted
- `LOCK` - Move servo to locked position (135°)
- `UNLOCK` - Move servo to unlocked position (45°)
- `TOGGLE` - Toggle between locked/unlocked
- `STATUS` - Report current lock state

## Configuration
Edit in `src/main.cpp`:
```cpp
const char* ssid = "RV_Control_Network";
const char* password = "RVControl2024";
const int SERVO_PIN = D4;
const int LOCKED_POSITION = 135;
const int UNLOCKED_POSITION = 45;
```

## Building and Uploading
```bash
pio run --target upload
pio device monitor
```

## Serial Output
Monitor at 115200 baud to see:
- WiFi connection status
- IP address assignment
- Received commands
- Lock state changes
