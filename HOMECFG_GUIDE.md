# Homing Parameters - Quick Reference

## New Command: HOMECFG

Set homing parameters on-the-fly for quick testing!

### Format
```
HOMECFG <speed> <current> <time>
```

### Parameters
- **speed**: Homing speed in RPM (negative = reverse direction)
- **current**: Stall current threshold in Amps
- **time**: Stall confirmation time in milliseconds (100-5000)

### Examples
```bash
# Default settings (initialized at startup)
# Speed: -5 RPM, Current: 1.75 A, Time: 300 ms

# More sensitive (lower current, faster detection)
HOMECFG -5 1.5 200

# Less sensitive (higher current, slower detection)
HOMECFG -5 2.5 500

# Slower speed, very sensitive
HOMECFG -3 1.0 400

# Faster speed, less sensitive
HOMECFG -10 2.0 300

# Forward direction instead of reverse
HOMECFG 5 1.75 300
```

## Complete Workflow

### 1. Set Homing Parameters
```bash
HOMECFG -5 1.75 300
# Output:
# Homing config updated:
#   Speed: -5 RPM
#   Current threshold: 1.75 A
#   Stall time: 300 ms
```

### 2. Run Homing
```bash
HOME 2
# Output:
# Starting homing sequence for Motor 2
# Motor will move slowly until physical limit detected...
# Settings: -5 RPM, 1.75 A, 300 ms
#
# ... motor moves ...
#
# Motor 2 homing complete! Position set to 0 degrees.
# Stall detected at 1.82 A
```

### 3. Adjust if Needed
```bash
# If it didn't detect the limit:
HOMECFG -5 1.5 200    # Lower threshold, faster detection
HOME 2

# If it false-triggered:
HOMECFG -5 2.0 400    # Higher threshold, longer confirmation
HOME 2
```

## Parameter Tuning Guide

### Speed (RPM)

**Too Slow (< -3 RPM)**
- ✅ Very gentle on mechanism
- ✅ More accurate detection
- ❌ Takes forever
- ❌ May timeout

**Good Range (-3 to -10 RPM)**
- ✅ Reasonable speed
- ✅ Reliable detection
- ✅ Won't damage mechanism

**Too Fast (> -15 RPM)**
- ❌ May damage mechanism
- ❌ Less accurate
- ❌ Harder to detect stall
- ✅ Quick homing

### Current Threshold (Amps)

**Too Low (< 1.0 A)**
- ❌ False triggers from friction
- ❌ May stop before limit
- ✅ Very sensitive

**Good Range (1.5 - 2.5 A)**
- ✅ Reliable detection
- ✅ Ignores normal friction
- ✅ Detects actual stall

**Too High (> 3.0 A)**
- ❌ May not detect soft limits
- ❌ Could damage mechanism
- ✅ Very robust

### Stall Time (ms)

**Too Short (< 200 ms)**
- ❌ False triggers
- ❌ Unreliable
- ✅ Fast response

**Good Range (200 - 500 ms)**
- ✅ Reliable confirmation
- ✅ Filters noise
- ✅ Good balance

**Too Long (> 1000 ms)**
- ❌ Slow to respond
- ❌ Motor pushes against limit longer
- ✅ Very reliable

## Troubleshooting

### Problem: Motor doesn't stop at limit
```bash
# Try lower current threshold
HOMECFG -5 1.5 300
HOME 2

# Or shorter confirmation time
HOMECFG -5 1.75 200
HOME 2
```

### Problem: Motor stops before reaching limit
```bash
# Try higher current threshold
HOMECFG -5 2.5 300
HOME 2

# Or longer confirmation time
HOMECFG -5 1.75 500
HOME 2
```

### Problem: Motor moves wrong direction
```bash
# Reverse the speed sign
HOMECFG 5 1.75 300    # Positive = forward
HOME 2
```

### Problem: Inconsistent results
```bash
# Increase confirmation time
HOMECFG -5 1.75 500
HOME 2

# Or slow down
HOMECFG -3 1.75 400
HOME 2
```

## Testing Procedure

1. **Start Conservative**
   ```bash
   HOMECFG -3 2.0 500    # Slow, high threshold, long time
   HOME 2
   ```

2. **Increase Speed**
   ```bash
   HOMECFG -5 2.0 500
   HOME 2
   ```

3. **Lower Threshold**
   ```bash
   HOMECFG -5 1.75 400
   HOME 2
   ```

4. **Optimize Time**
   ```bash
   HOMECFG -5 1.75 300
   HOME 2
   ```

5. **Final Test**
   ```bash
   # Run multiple times to verify consistency
   HOME 2
   # Wait for completion
   HOME 2
   # Wait for completion
   HOME 2
   ```

## Default Values

At startup, all motors are initialized with:
```cpp
Speed: -5.0 RPM
Current: 1.75 A
Time: 300 ms
```

These can be changed anytime with `HOMECFG`.

## Notes

- Settings apply to **all motors** (not per-motor)
- Settings are **not saved** - reset on power cycle
- Current threshold must be **positive**
- Stall time must be **100-5000 ms**
- Speed can be **positive or negative**

## Quick Reference Table

| Mechanism Type | Speed | Current | Time |
|----------------|-------|---------|------|
| Delicate | -3 | 1.5 | 400 |
| Normal | -5 | 1.75 | 300 |
| Robust | -10 | 2.5 | 200 |
| Heavy Load | -5 | 3.0 | 500 |
| Light Touch | -3 | 1.0 | 300 |

## Example Session

```bash
# Check help
H

# Set parameters for testing
HOMECFG -5 1.75 300

# Try homing Motor 2
HOME 2
# ... observe result ...

# Adjust if needed
HOMECFG -5 1.5 250
HOME 2
# ... better! ...

# Test consistency
HOME 2
HOME 2
HOME 2
# All successful!

# Now use position control
P 2 90
P 2 180
P 2 0
```
