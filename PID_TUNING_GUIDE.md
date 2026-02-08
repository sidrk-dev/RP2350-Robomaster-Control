# PID Tuning Guide for RoboMaster Motor Position Control

## Overview
Your system uses **cascade PID control**:
- **Outer Loop (Position)**: Angle PID converts position error (degrees) → target speed (RPM)
- **Inner Loop (Speed)**: Speed PID converts speed error (RPM) → motor current (Amps)

## Current Problem: Motor Vibrating
This typically means the system is **over-responsive** or **oscillating**. Common causes:
1. **Kp too high** - System reacts too aggressively
2. **Kd too low** - Not enough damping
3. **Ki causing windup** - Integral term accumulating too much

## Quick Fix Commands

### View Current Settings
```
SHOW
```

### Emergency Stop
```
S
```

### Test Position Control
```
P 90
```

## Step-by-Step Tuning Process

### Phase 1: Stabilize Speed Loop First
The speed loop must be stable before tuning position control.

**Start with conservative values:**
```
SP 0.5 0 0
```

**Test with direct speed command:**
```
M 1 100
```

**Gradually increase Kp until you get good tracking:**
```
SP 1.0 0 0
SP 2.0 0 0
SP 3.0 0 0
```

**Signs of good speed control:**
- Motor reaches target RPM smoothly
- No oscillation or vibration
- Steady-state error is acceptable

**If oscillating:** Lower Kp
```
SP 0.5 0 0
```

**If sluggish:** Add a tiny bit of Ki (start very small!)
```
SP 2.0 0.1 0
```

**If noisy/jittery:** Add Kd for damping
```
SP 2.0 0.1 0.05
```

### Phase 2: Tune Position Loop
Once speed control is stable, tune the position loop.

**Start with very low Kp:**
```
AP 0.1 0 0
```

**Test position control:**
```
P 90
```

**Gradually increase Kp:**
```
AP 0.3 0 0
AP 0.5 0 0
AP 1.0 0 0
```

**Signs of good position control:**
- Motor moves smoothly to target
- Settles without overshoot
- No vibration at target position

**If vibrating at target:** Add Kd for damping
```
AP 0.5 0 0.2
AP 0.5 0 0.5
```

**If slow to reach target:** Increase Kp
```
AP 1.5 0 0.5
```

**If steady-state error (doesn't quite reach target):** Add tiny Ki
```
AP 1.0 0.05 0.5
```

## Recommended Starting Values

### For M2006 Motors (36:1 gear ratio)
```
SP 1.0 0.1 0      # Speed PID
AP 0.5 0 0.3      # Angle PID
```

### For M3508 Motors (19:1 gear ratio)
```
SP 2.0 0.2 0      # Speed PID
AP 1.0 0 0.5      # Angle PID
```

## Understanding the Parameters

### Kp (Proportional Gain)
- **What it does:** Immediate response to error
- **Too high:** Oscillation, vibration, overshoot
- **Too low:** Slow response, large steady-state error
- **Typical range:** 0.1 - 5.0

### Ki (Integral Gain)
- **What it does:** Eliminates steady-state error over time
- **Too high:** Overshoot, slow settling, instability
- **Too low:** Persistent steady-state error
- **Typical range:** 0 - 0.5 (start with 0!)
- **⚠️ Warning:** Can cause windup - use sparingly

### Kd (Derivative Gain)
- **What it does:** Damping, resists rapid changes
- **Too high:** Amplifies noise, jittery motion
- **Too low:** Overshoot, oscillation
- **Typical range:** 0 - 1.0

## Troubleshooting Guide

### Problem: Motor vibrates/oscillates at target position
**Cause:** Position loop Kp too high or Kd too low
**Fix:**
```
AP 0.3 0 0.5    # Lower Kp, add damping
```

### Problem: Motor overshoots target
**Cause:** Not enough damping
**Fix:**
```
AP 0.5 0 1.0    # Increase Kd
```

### Problem: Motor doesn't reach exact target
**Cause:** Steady-state error, need integral term
**Fix:**
```
AP 1.0 0.05 0.5  # Add small Ki
```

### Problem: Motor moves very slowly
**Cause:** Position Kp too low or speed limit too low
**Fix:**
```
AP 2.0 0 0.5    # Increase Kp
```
Or check that `outputLimit` (max RPM) is reasonable (default: 500 RPM)

### Problem: Motor makes grinding noise
**Cause:** Speed loop too aggressive or current limit too high
**Fix:**
```
SP 0.5 0 0      # Lower speed Kp
```

### Problem: Erratic behavior after running for a while
**Cause:** Integral windup
**Fix:**
```
AP 1.0 0 0.5    # Remove Ki from angle PID
SP 2.0 0 0      # Remove Ki from speed PID
```

## Advanced: Understanding Limits

### Output Limits
- **Angle PID outputLimit:** Maximum RPM command (default: 500)
  - Higher = faster motion
  - Lower = smoother, safer motion
  
- **Speed PID outputLimit:** Maximum current in Amps (default: 20)
  - M2006: Max ~10A continuous, 20A peak
  - M3508: Max ~20A continuous

### Integral Limits
- **integralLimit:** Maximum integral accumulation (default: 5.0)
  - Prevents integral windup
  - Should be ~25% of outputLimit

## Tuning Workflow Summary

1. **Stop everything:** `S`
2. **Check current values:** `SHOW`
3. **Start conservative:** `SP 0.5 0 0` and `AP 0.1 0 0`
4. **Test speed first:** `M 1 100`
5. **Tune speed loop** until stable
6. **Test position:** `P 90`
7. **Tune position loop** incrementally
8. **Add damping (Kd)** if oscillating
9. **Add integral (Ki)** only if needed for steady-state error
10. **Document your final values!**

## Example Tuning Session

```
# Start fresh
S
SHOW

# Conservative speed control
SP 0.5 0 0
M 1 100
# (observe - is it smooth?)

# Increase responsiveness
SP 1.0 0 0
M 1 100
# (better tracking?)

# Add tiny integral for steady-state
SP 1.0 0.1 0
M 1 100
# (reaches target exactly?)

# Now tune position - start low
AP 0.3 0 0
P 90
# (moves but slow?)

# Increase position gain
AP 0.8 0 0
P 180
# (faster but oscillates?)

# Add damping
AP 0.8 0 0.5
P 270
# (smooth and stable!)

# Final check
SHOW
# Document these values!
```

## Tips for Success

1. **Change ONE parameter at a time**
2. **Make small incremental changes** (0.1 - 0.5 steps)
3. **Wait for system to settle** before next change
4. **Start with Kp only**, add Kd second, Ki last
5. **Speed loop MUST be stable** before tuning position
6. **Lower values are safer** - start conservative
7. **Test multiple target positions** (0°, 90°, 180°, 270°)
8. **Watch the serial output** for actual values
9. **Document your final working values!**

## Safety Notes

- Always have `S` (stop) command ready
- Start with low gains to avoid damage
- Monitor motor temperature (shown in status)
- If motor gets hot (>60°C), reduce gains
- Current limit protects motor - don't exceed 20A for long periods

## Your Current Settings (from code)

```
Angle PID: Kp=1.0, Ki=0.0, Kd=0.5, OutLimit=500, IntLimit=5.0
Speed PID: Kp=1.0, Ki=0.5, Kd=0.0, OutLimit=20, IntLimit=5.0
```

**Recommendation for vibration issue:**
Try these more conservative values:
```
SP 0.5 0 0      # Lower speed Kp, remove Ki
AP 0.3 0 0.5    # Lower position Kp, keep damping
```

Then gradually increase from there!
