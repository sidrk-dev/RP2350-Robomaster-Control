# Multi-Motor Control - Quick Reference

## All Motors Now Support Position Control!

All 8 motors now have the same PID tuning and can be controlled independently for position or speed.

## Command Format Changes

### Position Control
**Old (Motor 1 only):**
```
P 90
```

**New (All motors):**
```
P 1 90      # Motor 1 to 90 degrees
P 2 180     # Motor 2 to 180 degrees
P 3 270     # Motor 3 to 270 degrees
```

### Calibration
**Old (Motor 1 only):**
```
CAL 0
```

**New (All motors):**
```
CAL 1 0     # Set Motor 1 current position to 0°
CAL 2 90    # Set Motor 2 current position to 90°
CAL 3 180   # Set Motor 3 current position to 180°
```

### PID Tuning
**Old (Motor 1 only):**
```
AP 5 1.0 0.0
SP 0.3 0 0
SHOW
```

**New (All motors):**
```
AP 1 5 1.0 0.0    # Set Motor 1 angle PID
AP 2 5 1.0 0.0    # Set Motor 2 angle PID
SP 1 0.3 0 0      # Set Motor 1 speed PID
SP 2 0.3 0 0      # Set Motor 2 speed PID
SHOW 1            # Show Motor 1 PID values
SHOW 2            # Show Motor 2 PID values
```

### Speed Control (unchanged)
```
M 1 500     # Motor 1 at 500 RPM
M 2 -300    # Motor 2 at -300 RPM
```

## Complete Command List

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| **P** | `P <id> <angle>` | `P 1 90` | Move motor to absolute angle (0-360°) |
| **M** | `M <id> <speed>` | `M 2 500` | Set motor to continuous speed (RPM) |
| **CAL** | `CAL <id> <angle>` | `CAL 1 0` | Calibrate current position to angle |
| **AP** | `AP <id> <kp> <ki> <kd>` | `AP 1 5 1 0` | Set angle PID for motor |
| **SP** | `SP <id> <kp> <ki> <kd>` | `SP 1 0.3 0 0` | Set speed PID for motor |
| **SHOW** | `SHOW <id>` | `SHOW 1` | Display PID values for motor |
| **S** | `S` | `S` | Stop ALL motors |
| **H** | `H` | `H` | Show help |

## Default PID Values (All Motors)

```
Angle PID: Kp=5.0, Ki=1.0, Kd=0.0, OutLimit=500 RPM, IntLimit=5.0
Speed PID: Kp=0.3, Ki=0.0, Kd=0.0, OutLimit=20 A, IntLimit=5.0
```

These are tuned for M3508 motors (19:1 gear ratio).

## Status Display

The serial monitor now shows status for ALL motors in position control mode:

```
M1 POS:90.2 Tot:90.2 RPM:0.5 A:0.12 | M3 POS:180.1 Tot:180.1 RPM:-1.2 A:0.08 |
```

- **POS** - Current position (0-360°)
- **Tot** - Total angle (can exceed 360°)
- **RPM** - Current speed
- **A** - Current in Amps

## Example Multi-Motor Workflow

### Setup Multiple Motors
```
S                   # Stop all
CAL 1 0             # Calibrate motor 1 to 0°
CAL 2 0             # Calibrate motor 2 to 0°
CAL 3 0             # Calibrate motor 3 to 0°
```

### Coordinated Movement
```
P 1 90              # Motor 1 to 90°
P 2 180             # Motor 2 to 180°
P 3 270             # Motor 3 to 270°
```

### Individual Tuning
```
SHOW 1              # Check Motor 1 PID
AP 1 7 1 0.1        # Increase Motor 1 responsiveness
SHOW 2              # Check Motor 2 PID
AP 2 3 0.5 0.2      # Make Motor 2 more damped
```

### Return to Zero
```
P 1 0
P 2 0
P 3 0
```

## LED Indicator

The built-in LED turns ON when **any** motor is in position control mode.

## Notes

- All motors share the same default PID values at startup
- You can tune each motor independently during runtime
- PID values reset to defaults on power cycle
- Position tracking works across multiple rotations
- Shortest path is automatically calculated for position commands
