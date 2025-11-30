# Edge Cases Analysis: Two Point Interpolation with Constant Acceleration

This document provides detailed analysis of edge cases and boundary conditions for the two-point interpolation algorithm.

## Table of Contents
1. [Overview](#overview)
2. [Case 1: Normal Operation with Short Distance](#case-1-normal-operation-with-short-distance)
3. [Case 2: Abnormal Input (v0 > vmax)](#case-2-abnormal-input-v0--vmax)
4. [Theoretical Minimum Distance](#theoretical-minimum-distance)
5. [Error Detection Mechanism](#error-detection-mechanism)

---

## Overview

The two-point interpolation algorithm handles trajectory planning with constant acceleration. This analysis examines behavior at boundary conditions, particularly when:
- Initial velocity is close to or exceeds vmax
- Distance is very short, approaching theoretical minimum

### Test Parameters Used
```python
p0 = 0.0  # Start position
v0 = 0.5 or 2.0  # Initial velocity (varies by case)
ve = 0.0  # Target velocity
acc_max = 2.0  # Maximum acceleration
dec_max = 2.0  # Maximum deceleration
vmax = 5.0 or 0.5  # Maximum velocity (varies by case)
```

---

## Case 1: Normal Operation with Short Distance

**Condition**: `v0 < vmax` (Normal input)

### Progressive Distance Reduction

| Distance (m) | dt01 (s) | Accel Phase % | Max Velocity (m/s) | Result |
|--------------|----------|---------------|---------------------|--------|
| 0.500 | 0.280330 | 34.58% | 1.061 | ✅ Normal |
| 0.200 | 0.112495 | 23.67% | 0.725 | ✅ Normal |
| 0.100 | 0.035044 | 10.95% | 0.570 | ✅ Short distance |
| 0.080 | 0.016927 | 5.96% | 0.534 | ✅ Very short |
| 0.065 | 0.002488 | 0.98% | 0.505 | ⚠️ Near critical |
| 0.063 | 0.000500 | 0.20% | 0.501 | ⚠️ Critical limit |
| 0.062 | N/A | N/A | N/A | ❌ **ERROR** |

### Key Observations

1. **Automatic Adaptation**: As distance decreases, the algorithm automatically reduces the acceleration phase duration (dt01)
2. **Smooth Transition**: The trajectory smoothly transitions from balanced accel/decel to nearly pure deceleration
3. **Approaching Pure Deceleration**: At pe=0.063m, dt01 is only 0.0005s (0.2% of total time), essentially pure deceleration
4. **Error Detection**: At pe=0.062m, the 2% tolerance threshold correctly detects insufficient distance

### Visualization

See: `case1_progressive_distances.png`

Four subplots showing:
- Normal short distance (pe=0.5m)
- Very short distance (pe=0.1m)
- Near critical (pe=0.065m)
- Critical limit (pe=0.063m)

Each showing position, velocity, and acceleration over time.

![Case 1: Progressive Distance Reduction](case1_progressive_distances.png)

---

## Case 2: Overspeed Input (v0 > vmax) - FIXED ✅

**Condition**: `|v0| > vmax` (Overspeed input requiring immediate deceleration)

**Status**: ✅ **FIXED** as of 2024-11-28

### Example: v0=2.0 m/s, vmax=0.5 m/s, pe=5.0 m

#### After Fix:
```
Phase 1: dt=1.500s, a=-1.0 m/s² ✅ (Correct deceleration!)
Phase 2: dt=6.000s, a=0.0 m/s²  (Constant velocity)
Phase 3: dt=0.500s, a=-1.0 m/s² (Final deceleration)

Velocities: [2.0, 0.5, 0.5] m/s
Positions:  [0.0, 1.875, 4.875] m ✅ (All correct!)
```

### Fix Applied (2024-11-28)

**Detection and Warning**:
```python
if sign * v0 < sign * v1:
    # Normal case: accelerate to vmax
    dt01 = np.fabs((v1 - v0) / acc)
    a_phase1 = acc
    p1 = p_integ(p0, v0, acc, dt01)
else:
    # Overspeed case: decelerate to vmax
    warnings.warn(
        f"Initial velocity ({abs(v0):.3f} m/s) exceeds vmax ({vmax:.3f} m/s). "
        f"Trajectory will start with deceleration to reach vmax. "
        f"Consider reducing initial velocity or increasing vmax.",
        UserWarning
    )
    dt01 = np.fabs((v1 - v0) / dec)
    a_phase1 = -dec
    p1 = p_integ(p0, v0, -dec, dt01)  # ✅ Now uses deceleration!
```

**Key Changes**:
1. ✅ Detect when `sign * v0 > vmax` (handles both forward and backward overspeed)
2. ✅ Use deceleration (`-dec`) instead of acceleration for Phase 1
3. ✅ Calculate position correctly with deceleration
4. ✅ Issue `UserWarning` to alert user of abnormal condition

### Results

| Metric | Before Fix | After Fix |
|--------|------------|-----------|
| Phase 1 acceleration | +1.0 m/s² ❌ | -1.0 m/s² ✅ |
| Position after Phase 1 | 4.125m ❌ | 1.875m ✅ |
| Phase 2 duration | 1.5s (compensating) | 6.0s (correct) |
| Final position | 5.0m ✅ | 5.0m ✅ |
| Intermediate trajectory | Wrong ❌ | Correct ✅ |

### Visualization

**Before and After Comparison**:

![Trajectory Comparison](trajectory_comparison.png)

Left: Normal case (v0 < vmax)
Right: Fixed overspeed case (v0 > vmax) - now with correct physics!
| User notification | None ❌ | Warning ✅ |

### Visualization

See: `trajectory_comparison.png`

Left column: Case 1 (v0 < vmax, normal)
Right column: Case 2 (v0 > vmax, overspeed - now fixed!)

The updated graph shows Case 2 now has correct physics throughout the entire trajectory.

---

## Theoretical Minimum Distance

### Formula

For pure deceleration from v0 to ve:

```
d_min = (v0² - ve²) / (2 * dec)
```

### Example Calculation

With v0=0.5 m/s, ve=0.0 m/s, dec=2.0 m/s²:

```
d_min = (0.5² - 0²) / (2 * 2.0)
      = 0.25 / 4.0
      = 0.0625 m
```

### Practical Minimum

The algorithm can handle distances down to approximately **101% of theoretical minimum**:
- Theoretical: 0.0625m
- Practical limit: 0.063m (with 2% tolerance)
- Below 0.062m: Error detected by tolerance check

---

## Error Detection Mechanism

### Tolerance Threshold

Defined in both implementations:
```cpp
constexpr double DECEL_DISTANCE_TOLERANCE = 0.02;  // 2%
```

### Detection Logic

When deceleration distance is within 2% of available distance:

```python
if abs(decel_distance - abs(dp)) < abs(dp) * DECEL_DISTANCE_TOLERANCE:
    raise ValueError(
        "Insufficient distance for trajectory planning: "
        "current velocity {v0} requires approximately {decel_distance} "
        "distance to reach target velocity {ve}, "
        "nearly equal to available distance {dp}. "
        "This typically occurs when the same goal is sent again during motion."
    )
```

### Three Error Levels

1. **Within 2% tolerance**: "Same goal sent again" message
2. **More than 2% shortage**: "Insufficient distance" with percentage
3. **General constraint violation**: Discriminant or no positive solution errors

---

## Test Commands

### Reproduce Case 1 Analysis
```bash
# Navigate to the project root directory
python3 << 'EOF'
from two_point_interpolation import TwoPointInterpolation

distances = [0.5, 0.1, 0.065, 0.063, 0.062]
for pe in distances:
    tpi = TwoPointInterpolation()
    tpi.init(p0=0.0, pe=pe, acc_max=2.0, vmax=5.0, v0=0.5, ve=0.0, dec_max=2.0)
    try:
        t = tpi.calc_trajectory()
        print(f"pe={pe:.3f}m: ✅ dt01={tpi.dt[0]:.6f}s, max_v={max(tpi.v):.3f} m/s")
    except ValueError as e:
        print(f"pe={pe:.3f}m: ❌ {str(e)[:60]}")
EOF
```

### Reproduce Case 2 (Now Fixed)
```bash
python3 << 'EOF'
from two_point_interpolation import TwoPointInterpolation
import warnings

# Enable all warnings
warnings.simplefilter('always')

tpi = TwoPointInterpolation()
tpi.init(p0=0.0, pe=5.0, acc_max=1.0, vmax=0.5, v0=2.0, ve=0.0, dec_max=1.0)
t = tpi.calc_trajectory()

print(f"v0={2.0} > vmax={0.5}")
print(f"Phase 1: dt={tpi.dt[0]:.3f}s, a={tpi.a[0]:.3f} m/s² ✅ (correct deceleration)")
print(f"  v: {tpi.v[0]:.3f} → {tpi.v[1]:.3f} m/s (decreasing)")
print(f"  p: {tpi.p[0]:.3f} → {tpi.p[1]:.3f} m (correct: 1.875m)")
print(f"\nNote: A UserWarning is issued when v0 > vmax")
EOF
```

---

## Related Files

- Implementation: `two_point_interpolation/constant_acc.py`
- C++ Implementation: `two_points_interpolation_cpp/include/two_point_interpolation/constant_acc.hpp`
- Tests: `tests/test_constant_acc.py`
- Graphs:
  - `trajectory_comparison.png` (Case 1 vs Case 2 - Updated with fix)
  - `case1_progressive_distances.png` (Progressive distance reduction)

---

## Version History

- **2024-11-28 (Update 2)**: Case 2 fix applied
  - **FIXED**: v0 > vmax now handled correctly
  - Phase 1 uses deceleration when needed
  - Correct position calculation throughout trajectory
  - Added UserWarning for overspeed condition
  - Updated graphs and documentation
  - All tests passing (17 tests)

- **2024-11-28 (Update 1)**: Bug fix for error detection
  - Added `std::fabs()` to dt01 calculation in C++
  - Fixed dt12 < 0 error detection
  - This was a partial fix, Case 2 physics still had issues

- **2024-11-28 (Initial)**: Initial analysis
  - Documented Case 1 behavior down to theoretical minimum
  - Identified Case 2 physics issue
  - Added tolerance-based error detection
  - Applied `std::fabs()` fix to dt01 calculation

---

## Future Work

1. Fix Case 2 (v0 > vmax) Phase 1 physics
2. Add input validation option for v0 vs vmax
3. Consider separate handling for "overspeed" initial conditions
4. Add more test cases for backward motion (negative velocities)
