# Constant Acceleration Trajectory Planning - Mathematical Derivation

This document explains the mathematical foundation of the constant acceleration trajectory planner.

## Problem Statement

Given:
- Initial state: position `p0`, velocity `v0`, time `t0`
- Final state: position `pe`, velocity `ve`
- Constraints: maximum acceleration `amax`, maximum deceleration `dmax`, maximum velocity `vmax`

Find: Time-optimal trajectory that satisfies all constraints.

## Trajectory Structure

### Case 0: vmax Not Reached

Two phases:
1. **Acceleration phase** (0 → t1): velocity changes from `v0` to `v1`
2. **Deceleration phase** (t1 → te): velocity changes from `v1` to `ve`

### Case 1: vmax Reached

Three phases:
1. **Acceleration phase** (0 → t1): velocity changes from `v0` to `vmax`
2. **Constant velocity phase** (t1 → t2): velocity stays at `vmax`
3. **Deceleration phase** (t2 → te): velocity changes from `vmax` to `ve`

## Mathematical Derivation (Case 0)

### Step 1: Motion Equations

**Acceleration phase:**
```
v1 = v0 + acc * t1
p1 = p0 + v0*t1 + 0.5*acc*t1²
```

**Deceleration phase:**
```
ve = v1 - dec * t2
pe = p1 + v1*t2 - 0.5*dec*t2²
```

Where:
- `acc` = signed acceleration (= amax for forward motion)
- `dec` = signed deceleration magnitude (= dmax for forward motion)

### Step 2: Eliminate t2

From velocity equation:
```
t2 = (v1 - ve) / dec
```

Substitute `v1 = v0 + acc*t1`:
```
t2 = (v0 + acc*t1 - ve) / dec
   = (v0 - ve)/dec + (acc/dec)*t1
```

### Step 3: Solve for t1

Define:
- `dp = pe - p0` (displacement)
- `ratio = acc / dec`

Substitute t2 into position equation and rearrange to get quadratic equation:
```
a_coeff * t1² + b_coeff * t1 + c_coeff = 0
```

Where:
```
a_coeff = 0.5 * acc * (1 + ratio)
b_coeff = v0 * (1 + ratio)
c_coeff = -dp + (v0² - ve²) / (2 * dec)
```

> **Note**: For detailed step-by-step algebraic derivation of these coefficients, see [QUADRATIC_COEFFICIENTS_DERIVATION.md](QUADRATIC_COEFFICIENTS_DERIVATION.md).

### Step 4: Quadratic Formula

```
discriminant = b_coeff² - 4*a_coeff*c_coeff

If discriminant > 0:
    t1 = (-b_coeff + √discriminant) / (2*a_coeff)
    v1 = v0 + acc*t1

    If |v1| < vmax:
        Use Case 0 (two phases)
    Else:
        Use Case 1 (three phases)
```

## Special Cases

### Symmetric (acc = dec)

When `amax = dmax`, the equations simplify:
```
ratio = 1
a_coeff = acc
b_coeff = 2*v0
c_coeff = -dp + (v0² - ve²)/(2*acc)
```

### Zero Displacement (dp = 0)

- If `dv = 0`: No movement needed (case -1)
- If `dv ≠ 0`: Impossible (raises error)

## Implementation Notes

### Variables in Code

- `acc`: Signed acceleration value
- `dec`: Signed deceleration value (magnitude)
- `ratio`: Acceleration to deceleration ratio
- `dt01`: Duration of acceleration phase
- `dt1e` or `dt2e`: Duration of deceleration phase

### Verification

The implementation ensures:
1. Velocity limits are respected: `|v| ≤ vmax`
2. Acceleration limits are respected: `|a| ≤ amax`
3. Deceleration limits are respected: `|a| ≤ dmax`
4. Final state is reached: `p(te) = pe`, `v(te) = ve`

## References

- Original constant acceleration implementation: `two_point_interpolation_constant_acc.py`
- Unit tests: `tests/test_asymmetric_interpolation.py`
- Examples: `examples/example_asymmetric_acc.py`
