# Plumbing System Control Page - User Guide

## Overview
A new page has been added to the GIGA R1 display for manual control of the plumbing system relays. This allows you to test and manually override any relay for troubleshooting and maintenance.

## Navigation
The plumbing page can be accessed via the navigation buttons at the top of the screen:
- **LOCKS** - Cabinet lock control page
- **THERMOSTAT** - Rear thermostat control page
- **PLUMBING** - Plumbing system relay control page (NEW)

## Relay Controls

### Layout
The page displays 6 relay controls in a 3x2 grid:

**Row 1:**
- **Fresh Water Heater** (Left) - Tank heater for fresh water
- **Grey Water Heater** (Right) - Tank heater for grey water

**Row 2:**
- **Exhaust Fan** (Left) - Plumbing cabinet ventilation fan
- **Rear Loop Valve** (Right) - Hydronic heating rear loop valve

**Row 3:**
- **Engine Loop Valve** (Left) - Hydronic heating engine loop valve
- **Diesel Heater** (Right) - Diesel heater control (3-state)

### Button Colors

**Standard Toggle Buttons:**
- **GREEN** = Relay is ON/OPEN
- **RED** = Relay is OFF/CLOSED

**Diesel Heater Button:**
- **GRAY** = OFF
- **YELLOW** = MID power
- **RED** = HIGH power

### How to Use

**Toggle Buttons (5 relays):**
Simply tap the button to toggle between ON/OFF or OPEN/CLOSED. The button color and text will update to reflect the current state.

**Diesel Heater Button:**
Tap the button to cycle through the three states:
1. OFF → MID
2. MID → HIGH
3. HIGH → OFF

The button displays the current state (OFF/MID/HIGH) in large text.

## MQTT Topics

The page publishes commands to and receives status updates from these MQTT topics:

### Command Topics (Published by GIGA):
- `FreshWaterHeatCommand` - ON/OFF
- `GreyWaterHeatCommand` - ON/OFF
- `ExhaustFanCommand` - ON/OFF
- `RearLoopCommand` - OPEN/CLOSE
- `EngineLoopCommand` - OPEN/CLOSE
- `DieselHeaterCommand` - OFF/MID/HIGH

### Status Topics (Subscribed by GIGA):
- `FreshWaterHeatStatus`
- `GreyWaterHeatStatus`
- `ExhaustFanStatus`
- `RearLoopStatus`
- `EngineLoopStatus`
- `DieselHeaterStatus`

## Important Notes

1. **Manual Control Only**: This page provides manual override control. It does NOT show or control automatic modes (AUTO/MANUAL). The D1 Plumbing board manages automatic operation based on temperature sensors.

2. **Real-Time Status**: Button states update in real-time based on MQTT status messages from the D1 Plumbing board, so you'll see changes whether you made them or the automatic system did.

3. **Safety**: Be careful when manually controlling heaters and valves. The automatic system has safety logic built-in, but manual control bypasses those protections.

4. **Diesel Heater States**:
   - **OFF**: All 3 relays are inactive
   - **MID**: Partial power mode
   - **HIGH**: Full power mode

## Testing Procedure

To test a relay:
1. Navigate to the Plumbing page
2. Tap the relay button you want to test
3. Listen/observe for the relay click/activation
4. Check the actual device (heater, fan, valve) responds
5. Tap again to turn off
6. Verify the device deactivates

## Troubleshooting

**Button doesn't respond:**
- Check MQTT connection status at bottom of screen
- Verify D1 Plumbing board is online and connected

**Status doesn't update:**
- The D1 Plumbing board publishes status changes
- Check that the D1 board is running and connected to MQTT

**Relay activates but device doesn't work:**
- Could indicate wiring issue or faulty relay
- Could indicate the device itself has failed
- Check power supply to the relay board

## Future Enhancements

Possible future additions:
- Display current sensor readings (temperatures, etc.)
- Show AUTO/MANUAL mode indicators
- Add configuration controls for temperature setpoints
- Display relay activation history/logs
