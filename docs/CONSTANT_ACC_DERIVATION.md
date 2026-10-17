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

## Mathematical Derivation

### Step 1: Motion equations

Let `dp = pe - p0`, `sign = dp / |dp|`, and write the limits as signed values
pointing toward `pe`: `acc = sign * amax`, `dec = sign * dmax`. The peak velocity
`v1` is reached at the end of the acceleration phase.

**Acceleration phase** (`v0 -> v1` at `acc`):
```
v1 = v0 + acc * t1
distance = (v1² - v0²) / (2 * acc)
```

**Deceleration phase** (`v1 -> ve` at `-dec`):
```
ve = v1 - dec * t2
distance = (v1² - ve²) / (2 * dec)
```

Both distance formulas hold whatever the signs of `v0` and `ve`: a body that starts
moving away from `pe` slows down, stops and turns back within the acceleration
phase, and a body that must arrive moving away from `pe` passes it and turns back
within the deceleration phase.

### Step 2: Solve for the peak velocity v1

The two phases together cover the displacement:
```
|dp| = (v1² - v0²) / (2*amax) + (v1² - ve²) / (2*dmax)
```

so
```
v1² = (|dp| + v0²/(2*amax) + ve²/(2*dmax)) / (1/(2*amax) + 1/(2*dmax))
```

This fixes `|v1|`, not its sign. Only `v1` pointing toward `pe` is a trajectory:
```
v1 = sign * √(v1²)
t1 = (v1 - v0) / acc
t2 = (v1 - ve) / dec
```

### Step 3: Feasibility

```
t1 <= 0             -> the body already moves toward pe at least as fast as v1:
                       it cannot slow down to ve within |dp|
sign*(v1 - ve) < 0  -> ve is faster, toward pe, than any speed reachable within
                       |dp| (that is sqrt(v0² + 2*amax*|dp|)): ve is unreachable
                       without first moving away from pe
```

Both raise `ValueError`. Both are limits of this accelerate-then-decelerate
profile only: the reverse order (decelerate, then accelerate) would pass `pe` and
come back, or back away first to gain run-up distance. That is tracked in #9.

### Step 4: vmax

```
If |v1| < vmax:
    Case 0: the two phases above
Else:
    Case 1: accelerate (or brake, if |v0| > vmax) to vmax, cruise, decelerate to ve;
            the cruise covers what the two ramps leave of |dp|
```

### Why not solve for t1 (history, up to 1.3.1)

Up to 1.3.1 the implementation substituted `v1 = v0 + acc*t1` into the distance
equation and solved the resulting quadratic in `t1`, taking the smaller positive
root. Because the distance equation only contains `v1²`, the quadratic's two roots
are `v1 = +|v1|` and `v1 = -|v1|`. When `v0` points toward `pe` (or is zero) only
the correct one is positive. When `v0` points away from `pe` both are positive, and
the smaller one could be `v1 = -|v1|`: the acceleration phase ended still moving
away, the deceleration then drove further away, and `pe` was never reached.
Solving for `v1` directly and fixing its sign removes the spurious root.

## Special Cases

### Zero Displacement (dp = 0)

- If `dv = 0`: No movement needed (case -1)
- If `dv ≠ 0`: Impossible (raises error)

## Implementation Notes

### Variables in Code

- `acc`: Signed acceleration value
- `dec`: Signed deceleration value (magnitude)
- `v1`: Peak velocity at the end of the acceleration phase
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
