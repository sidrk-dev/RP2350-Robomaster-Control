# Automatic Homing Feature

## Overview
The homing feature automatically finds physical limits by detecting when a motor stalls (hits a hard stop). This is useful for calibrating absolute position without manual intervention.

## How It Works

### Stall Detection
The system monitors two conditions:
1. **High Current**: Motor current exceeds threshold (default: 2A)
2. **No Movement**: Motor position changes less than 0.5° in the detection window

When both conditions are met for 500ms, the motor is considered stalled.

### Homing Sequence
1. Motor moves slowly in reverse (-10 RPM by default)
2. System monitors current and position
3. When stall is detected, motor stops
4. Current position is set to 0°
5. PID states are reset

## Command

### HOME <id>
Starts automatic homing for the specified motor.

```bash
HOME 2      # Home Motor 2 to physical limit
```

### Parameters (hardcoded, can be adjusted in code)
- **Speed**: -10 RPM (slow reverse)
- **Stall Current**: 2.0 A
- **Stall Time**: 500 ms (confirmation period)
- **Timeout**: 30 seconds

## Usage Example

```bash
# Home Motor 2
HOME 2

# Output:
# Starting homing sequence for Motor 2
# Motor will move slowly until physical limit detected...
# Homing started: -10 RPM, stall threshold 2 A
# 
# ... motor moves slowly ...
# 
# Motor 2 homing complete! Position set to 0 degrees.
# Stall detected at 2.15 A
```

## After Homing

Once homed, you can use position control normally:

```bash
HOME 2      # Find zero position
P 2 90      # Move to 90° from zero
P 2 180     # Move to 180° from zero
P 2 0       # Return to zero
```

## Safety Features

### Timeout
If no limit is found within 30 seconds, homing stops automatically:
```
Motor 2 homing TIMEOUT! No limit found in 30s.
```

### Emergency Stop
You can cancel homing at any time:
```bash
S           # Stops ALL motors including homing
```

## Adjusting Parameters

To change homing parameters for different motors, edit the `startHoming()` call in `goo.ino`:

```cpp
// Current (line ~254):
startHoming(idx, -10.0, 2.0);
//              speed  current threshold

// Examples:
startHoming(idx, -5.0, 1.5);   // Slower, more sensitive
startHoming(idx, -20.0, 3.0);  // Faster, less sensitive
startHoming(idx, 10.0, 2.0);   // Forward direction
```

### Speed (RPM)
- **Negative**: Moves in reverse (typical for homing)
- **Positive**: Moves forward
- **Magnitude**: 5-20 RPM recommended
  - Too slow: Takes forever
  - Too fast: May damage mechanism

### Current Threshold (Amps)
- **Lower (1-1.5A)**: More sensitive, may false-trigger
- **Higher (2-3A)**: Less sensitive, more reliable
- **Typical**: 2.0A works well for most applications

### Stall Time (ms)
Hardcoded to 500ms in `homingState[i].stallTime`. To change:

```cpp
// In startHoming() function:
homingState[motorIndex].stallTime = 1000; // 1 second confirmation
```

## Troubleshooting

### Motor doesn't stop at limit
- **Increase current threshold**: Limit may not generate enough resistance
- **Check wiring**: Ensure motor is properly connected
- **Check limit**: Physical stop may be too weak

### False stall detection
- **Decrease current threshold**: Motor may be drawing high current normally
- **Increase stall time**: Give more time to confirm stall
- **Check for binding**: Mechanism may have friction issues

### Motor moves wrong direction
- Change speed sign: `-10.0` → `10.0`
- Or reverse motor direction in setup

### Timeout occurs
- **Increase timeout**: Edit `updateHoming()` function, change `30000` to higher value
- **Check mechanism**: Ensure there's actually a physical limit
- **Check speed**: Motor may be moving too slowly

## Implementation Details

### Files Modified
1. **goo.ino**: 
   - Added `homingState[]` structure
   - Added `HOME` command handler
   - Added `updateHoming()` call in loop
   - Added `startHoming()` prototype

2. **homing.ino** (new file):
   - `startHoming()`: Initiates homing sequence
   - `updateHoming()`: Monitors and completes homing

### State Machine
```
IDLE → HOME command → HOMING → Stall Detected → COMPLETE → IDLE
                         ↓
                      Timeout → IDLE
```

### Data Flow
```
CAN Feedback → actualCurrent, totalAngle
                      ↓
              updateHoming() checks:
              - Current > threshold?
              - Position not changing?
              - Time > stall time?
                      ↓
                   YES → Set position to 0
                   NO  → Continue monitoring
```

## Example Workflow

### Multi-Motor Homing
```bash
# Home all motors sequentially
HOME 1
# Wait for completion...

HOME 2
# Wait for completion...

HOME 3
# Wait for completion...

# Now all motors are at known zero positions
P 1 90
P 2 180
P 3 270
```

### Homing with External Gearing
```bash
# Set gear ratios first
GEAR 1 5
GEAR 2 5
GEAR 3 5

# Then home
HOME 1
HOME 2
HOME 3

# Position control now works on final output
P 1 90    # 90° on final output shaft
```

## Advanced: Custom Homing Direction

To home in forward direction instead of reverse:

```cpp
// In HOME command handler (around line 254):
startHoming(idx, 10.0, 2.0);  // Positive speed = forward
```

## Notes

- Homing is **non-blocking** - you can home multiple motors simultaneously
- Status updates print to serial when homing completes
- Homing state is cleared on `S` (stop) command
- PID states are reset after homing to prevent jumps
- Current position becomes the new zero reference

## Safety Recommendations

1. **Test with low current threshold first** to avoid damage
2. **Use slow speeds** (5-10 RPM) for initial testing
3. **Have emergency stop ready** (`S` command)
4. **Ensure physical limits are robust** enough to handle repeated contact
5. **Monitor serial output** during first homing attempts
6. **Consider adding soft limits** in your application code

## Future Enhancements

Possible improvements (not yet implemented):
- Configurable parameters via serial commands
- Multiple homing strategies (limit switch, encoder index, etc.)
- Homing in both directions to find center
- Automatic retry on timeout
- Homing status query command
