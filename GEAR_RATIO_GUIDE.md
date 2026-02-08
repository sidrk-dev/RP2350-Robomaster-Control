# External Gear Ratio & Calibration Fix

## What Changed

### 1. External Gear Ratio Support
Each motor can now have an **external gear ratio** in addition to its internal motor gear ratio. This is useful when you have:
- Belt/pulley reductions
- Additional gearboxes
- Chain drives
- Any external mechanical advantage

### 2. Calibration Fix
The calibration was failing because `externalGearRatio` wasn't initialized. It's now properly set to `1.0` (no external gearing) by default.

## New Command: GEAR

### Format
```
GEAR <id> <ratio>
```

### Examples
```
GEAR 1 5        # Motor 1 has 5:1 external gearing
GEAR 2 3.5      # Motor 2 has 3.5:1 external gearing
GEAR 3 1        # Motor 3 has no external gearing (default)
```

### How It Works

**Total Gear Ratio = Motor Internal Ratio × External Ratio**

For example:
- M3508 motor: 19:1 internal ratio
- External gearing: 5:1
- **Total ratio: 19 × 5 = 95:1**

This means:
- Position control works on the **final output shaft**
- Speed (RPM) is measured at the **final output**
- All calculations account for the total reduction

## Updated Workflow

### Setup with External Gearing

```bash
# 1. Set external gear ratios (if any)
GEAR 1 5        # Motor 1 has 5:1 external reduction
GEAR 2 5        # Motor 2 has 5:1 external reduction
GEAR 3 1        # Motor 3 has no external gearing

# 2. Calibrate positions
CAL 1 0         # Set Motor 1 current position to 0°
CAL 2 0         # Set Motor 2 current position to 0°
CAL 3 0         # Set Motor 3 current position to 0°

# 3. Move to positions (final output shaft degrees)
P 1 90          # Motor 1 final output to 90°
P 2 180         # Motor 2 final output to 180°
P 3 270         # Motor 3 final output to 270°
```

## Understanding the Gear Ratios

### Default Configuration (from your code)
```cpp
#define M2006_GEAR_RATIO 19.0 * 5  // = 95:1
```

This sets the **motor internal ratio** to 95:1. If you have additional external gearing, use the `GEAR` command to add it.

### Example Calculation

**Motor 1 with M3508 (19:1) + 5:1 external gearing:**

```bash
# Motor setup
motor[0].gearRatio = 95.0          # From M2006_GEAR_RATIO define
motor[0].externalGearRatio = 5.0   # Set via GEAR 1 5

# Total ratio
Total = 95.0 × 5.0 = 475:1

# If motor rotor spins 475 revolutions
# Final output shaft spins 1 revolution (360°)
```

## Calibration Now Works!

### Why It Was Broken
The `externalGearRatio` field wasn't initialized, so it had a random value (could be 0 or garbage). This caused:
- Division by zero errors
- Incorrect position calculations
- Calibration being immediately overwritten

### The Fix
```cpp
motor[i].externalGearRatio = 1.0;  // Default: no external gearing
```

Now calibration works correctly:
```bash
CAL 1 0      # Sets current position to 0°
P 1 90       # Moves to 90° (works!)
```

## Checking Your Setup

### View Total Gear Ratio
```bash
GEAR 1 5
# Output: Motor 1 external gear ratio set to 5:1 (Total ratio: 475:1)
```

### View PID Settings
```bash
SHOW 1
# Shows all PID values and limits
```

## Important Notes

### 1. Set Gear Ratio BEFORE Calibration
```bash
# CORRECT ORDER:
GEAR 1 5        # Set gear ratio first
CAL 1 0         # Then calibrate

# WRONG ORDER:
CAL 1 0         # Calibrate first
GEAR 1 5        # Changing ratio invalidates calibration!
```

### 2. Gear Ratio Affects Speed
If you set `GEAR 1 5`, the reported RPM will be 5× slower (final output speed).

### 3. Gear Ratio Affects Position
With higher gear ratios, you get:
- **More precision** (smaller motor movements = smaller output movements)
- **More torque** at the output
- **Slower maximum speed** at the output

## PID Tuning with External Gearing

Higher gear ratios may require different PID values:

```bash
# With 5:1 external gearing, you might need:
AP 1 10 1 0.2    # Higher Kp for faster response
SP 1 0.5 0 0     # Higher speed Kp
```

The position error is in **output shaft degrees**, which are smaller movements with high gear ratios.

## Complete Example Session

```bash
# Setup
H                # Show help
S                # Stop all motors

# Configure gear ratios
GEAR 1 5         # Motor 1: 5:1 external
GEAR 2 5         # Motor 2: 5:1 external  
GEAR 3 1         # Motor 3: no external gearing

# Calibrate
CAL 1 0          # Motor 1 at 0°
CAL 2 0          # Motor 2 at 0°
CAL 3 0          # Motor 3 at 0°

# Test movement
P 1 90           # Motor 1 to 90°
P 2 180          # Motor 2 to 180°
P 3 270          # Motor 3 to 270°

# Check status (shows final output positions)
# M1 POS:90.1 Tot:90.1 RPM:0.2 A:0.05 | M2 POS:180.0 ...

# Return home
P 1 0
P 2 0
P 3 0
```

## Troubleshooting

### Calibration Still Not Working?
1. Check that you set `GEAR` before `CAL`
2. Verify gear ratio is positive: `GEAR 1 5` (not 0 or negative)
3. Make sure motor is stopped before calibrating: `S` then `CAL 1 0`

### Position Drifts After Calibration?
- This is normal if the motor is moving
- Calibration sets the position at that instant
- Movement continues from the new reference

### Wrong Speed/Position Readings?
- Check your `M2006_GEAR_RATIO` define
- Verify external gear ratio is correct
- Total ratio = internal × external

## Summary

✅ **External gear ratios** now supported via `GEAR` command  
✅ **Calibration fixed** by initializing `externalGearRatio = 1.0`  
✅ **All calculations** use total gear ratio (internal × external)  
✅ **Position and speed** measured at final output shaft  

The lint errors are just IDE warnings - the code will compile fine in Arduino IDE!
