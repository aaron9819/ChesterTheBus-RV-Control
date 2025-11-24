# Plumbing Page - 3 Column x 4 Row Layout

## Overview
The plumbing control page has been reorganized from a 2-column x 5-row layout to a more efficient **3-column x 4-row** layout. This provides better use of the 800x480 screen real estate.

## Layout Specifications

### Button Dimensions
- **Width**: 240 pixels
- **Height**: 85 pixels
- **Border Radius**: 10px
- **Spacing**: 15px between columns

### Column Positions
| Column | X Position |
|--------|------------|
| 1 (Left) | 20px |
| 2 (Center) | 275px |
| 3 (Right) | 530px |

### Row Positions
| Row | Y Position | Controls |
|-----|------------|----------|
| 1 | 70px | Fresh Water, Grey Water, Exhaust Fan |
| 2 | 170px | Rear Loop, Engine Loop, Diesel Heater |
| 3 | 270px | Front Loop, Hot Water Pump, Main Water Pump |
| 4 | 370px | Domestic HW Loop, (empty), (empty) |

## Button Grid

```
┌─────────────────────────────────────────────────────────────────────┐
│                    PLUMBING SYSTEM                                  │
├───────────────┬───────────────┬───────────────┐
│  FRESH WATER  │  GREY WATER   │  EXHAUST FAN  │  Row 1 (Y: 70)
│      ON       │      OFF      │      ON       │
├───────────────┼───────────────┼───────────────┤
│   REAR LOOP   │  ENGINE LOOP  │ DIESEL HEATER │  Row 2 (Y: 170)
│      ON       │      OFF      │     HIGH      │
├───────────────┼───────────────┼───────────────┤
│  FRONT LOOP   │ HOT WATER PUMP│  MAIN WATER   │  Row 3 (Y: 270)
│      ON       │      ON       │     PUMP      │
├───────────────┼───────────────┼───────────────┤
│ DOMESTIC HW   │               │               │  Row 4 (Y: 370)
│     OPEN      │               │               │
└───────────────┴───────────────┴───────────────┘
     Col 1          Col 2          Col 3
    (X: 20)       (X: 275)       (X: 530)
```

## Control Mapping

### Row 1 - Tank Heaters & Fan
1. **Fresh Water Heater** (Col 1)
   - States: ON / OFF
   - Topic: `FreshWaterHeatCommand/Status`

2. **Grey Water Heater** (Col 2)
   - States: ON / OFF
   - Topic: `GreyWaterHeatCommand/Status`

3. **Exhaust Fan** (Col 3)
   - States: ON / OFF
   - Topic: `ExhaustFanCommand/Status`

### Row 2 - Loop Valves & Heater
4. **Rear Loop Valve** (Col 1)
   - States: OPEN / CLOSED (displays ON/OFF)
   - Topic: `RearLoopCommand/Status`

5. **Engine Loop Valve** (Col 2)
   - States: OPEN / CLOSED (displays ON/OFF)
   - Topic: `EngineLoopCommand/Status`

6. **Diesel Heater** (Col 3)
   - States: OFF → MID → HIGH (cycles)
   - Topic: `DieselHeaterCommand/Status`
   - Color: Gray (OFF), Yellow (MID), Red (HIGH)

### Row 3 - Pumps & Valves
7. **Front Loop Valve** (Col 1)
   - States: ON / OFF
   - Topic: `FrontLoopCommand/Status`

8. **Hot Water Pump** (Col 2)
   - States: ON / OFF
   - Topic: `PumpCommand/Status`
   - Note: Connected to automatic flow sensor control

9. **Main Water Pump** (Col 3)
   - States: ON / OFF
   - Topic: `MainWaterPumpCommand/Status`
   - Note: For city water pressure

### Row 4 - Domestic Systems
10. **Domestic Hot Water Loop** (Col 1)
    - States: OPEN / CLOSED
    - Topic: `DomesticHotWaterCommand/Status`
    - Note: Thermo-actuator for hot water circulation

11. **Empty** (Col 2) - Reserved for future expansion

12. **Empty** (Col 3) - Reserved for future expansion

## Touch Detection

Touch handlers use coordinate detection:
```cpp
if (x >= colX && x <= (colX + buttonWidth) &&
    y >= rowY && y <= (rowY + buttonHeight)) {
  // Handle button press
}
```

All touch handlers:
- Update `lastTouchTime`
- Set `touchCurrentlyPressed = true`
- Send MQTT command via `sendPlumbingCommand()`
- Return immediately after handling

## Color Coding

| State | Color Code | Hex Value | Usage |
|-------|------------|-----------|-------|
| ON/OPEN | Green | `COLOR_UNLOCKED` | Active state |
| OFF/CLOSED | Gray | `COLOR_LOCKED` | Inactive state |
| DIESEL OFF | Gray | `COLOR_UNKNOWN` | Off state |
| DIESEL MID | Yellow | `0xFFE0` | Mid power |
| DIESEL HIGH | Red | `0xF800` | High power |

## Screen Layout

```
┌────────────────────────────────────────────────────────┐
│  Header: "PLUMBING SYSTEM" (Y: 15)                     │ 800x60
├─────┬──────┬──────┬────────┬──────────┬───────────────┤
│LOCKS│THERMO│PLUMB │  NAV   │          │               │ Y: 10-50
├─────┴──────┴──────┴────────┴──────────┴───────────────┤
│                                                        │
│  [Button Grid: 3 columns x 4 rows]                    │ Y: 70-455
│                                                        │
│                                                        │
├────────────────────────────────────────────────────────┤
│ IP: 192.168.8.100 | MQTT: Connected | FW: v1.0.0     │ Y: 462
└────────────────────────────────────────────────────────┘
```

## Benefits of 3-Column Layout

1. **Better Space Utilization**: Uses full 800px width more efficiently
2. **Larger Buttons**: 240px wide vs 360px provides better touch targets with less wasted space
3. **Room for Expansion**: 2 empty button slots available in row 4
4. **Visual Balance**: 3 columns create more symmetrical appearance
5. **Reduced Scrolling**: All controls visible in 4 rows instead of 5

## Implementation Notes

- All button rendering uses identical structure for consistency
- Text positioning calculated relative to button top-left corner
- Touch detection matches button positions exactly
- Connection status positioned below button grid
- Layout constants defined once at function start for easy modification

## Future Expansion

Row 4 has 2 empty slots (columns 2 & 3) that can accommodate:
- Additional pumps or valves
- System status indicators
- Quick access to other pages
- Temperature displays
- Flow rate indicators
- Or any other controls as needed
