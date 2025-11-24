# Quadratic Coefficients Derivation - Detailed Steps

This document provides the detailed mathematical derivation of the quadratic equation coefficients used in Case 0 trajectory planning.

## Prerequisites

From the basic motion equations, we have:

**Acceleration phase (0 → t1):**
```
v1 = v0 + acc * t1
p1 = p0 + v0*t1 + 0.5*acc*t1²
```

**Deceleration phase (t1 → te):**
```
ve = v1 - dec * t2
pe = p1 + v1*t2 - 0.5*dec*t2²
```

Where:
- `dp = pe - p0` (total displacement)
- `ratio = acc / dec` (acceleration to deceleration ratio)

## Goal

We want to find t1 by eliminating t2 and rearranging into a standard quadratic form:
```
a_coeff * t1² + b_coeff * t1 + c_coeff = 0
```

## Detailed Derivation

### Step 1: Express t2 in terms of t1

From the velocity equation of deceleration phase:
```
ve = v1 - dec * t2
```

Solve for t2:
```
t2 = (v1 - ve) / dec
```

Substitute v1 = v0 + acc*t1:
```
t2 = (v0 + acc*t1 - ve) / dec
   = (v0 - ve)/dec + (acc/dec)*t1
   = (v0 - ve)/dec + ratio*t1
```

### Step 2: Express total displacement

The total displacement is:
```
dp = pe - p0 = (p1 - p0) + (pe - p1)
```

Substitute the position equations:
```
dp = [v0*t1 + 0.5*acc*t1²] + [v1*t2 - 0.5*dec*t2²]
```

### Step 3: Substitute v1 and t2

Substitute v1 = v0 + acc*t1:
```
dp = v0*t1 + 0.5*acc*t1² + (v0 + acc*t1)*t2 - 0.5*dec*t2²
```

Substitute t2 = (v0 - ve)/dec + ratio*t1:
```
dp = v0*t1 + 0.5*acc*t1²
     + (v0 + acc*t1)*[(v0 - ve)/dec + ratio*t1]
     - 0.5*dec*[(v0 - ve)/dec + ratio*t1]²
```

### Step 4: Expand the second term

```
(v0 + acc*t1)*[(v0 - ve)/dec + ratio*t1]
= v0*(v0 - ve)/dec + v0*ratio*t1 + acc*t1*(v0 - ve)/dec + acc*ratio*t1²
```

Using ratio = acc/dec:
```
= v0*(v0 - ve)/dec + v0*ratio*t1 + ratio*dec*t1*(v0 - ve)/dec + acc*ratio*t1²
= v0*(v0 - ve)/dec + v0*ratio*t1 + ratio*(v0 - ve)*t1 + acc*ratio*t1²
```

### Step 5: Expand the third term

```
-0.5*dec*[(v0 - ve)/dec + ratio*t1]²
= -0.5*dec*[(v0 - ve)²/dec² + 2*(v0 - ve)*ratio*t1/dec + ratio²*t1²]
= -0.5*(v0 - ve)²/dec - (v0 - ve)*ratio*t1 - 0.5*dec*ratio²*t1²
```

Using ratio = acc/dec:
```
= -0.5*(v0 - ve)²/dec - (v0 - ve)*ratio*t1 - 0.5*acc*ratio*t1²
```

### Step 6: Combine all terms

```
dp = v0*t1 + 0.5*acc*t1²
     + v0*(v0 - ve)/dec + v0*ratio*t1 + ratio*(v0 - ve)*t1 + acc*ratio*t1²
     - 0.5*(v0 - ve)²/dec - (v0 - ve)*ratio*t1 - 0.5*acc*ratio*t1²
```

Group by powers of t1:

**Constant terms:**
```
v0*(v0 - ve)/dec - 0.5*(v0 - ve)²/dec
= [v0*(v0 - ve) - 0.5*(v0 - ve)²] / dec
= [v0*(v0 - ve) - 0.5*(v0² - 2*v0*ve + ve²)] / dec
= [v0² - v0*ve - 0.5*v0² + v0*ve - 0.5*ve²] / dec
= [0.5*v0² - 0.5*ve²] / dec
= (v0² - ve²) / (2*dec)
```

**Linear terms (coefficient of t1):**
```
v0 + v0*ratio + ratio*(v0 - ve) - (v0 - ve)*ratio
= v0 + v0*ratio
= v0*(1 + ratio)
```

**Quadratic terms (coefficient of t1²):**
```
0.5*acc + acc*ratio - 0.5*acc*ratio
= 0.5*acc + 0.5*acc*ratio
= 0.5*acc*(1 + ratio)
```

### Step 7: Rearrange to standard form

```
dp = 0.5*acc*(1 + ratio)*t1² + v0*(1 + ratio)*t1 + (v0² - ve²)/(2*dec)
```

Move all terms to one side:
```
0 = 0.5*acc*(1 + ratio)*t1² + v0*(1 + ratio)*t1 + (v0² - ve²)/(2*dec) - dp
```

Rearrange:
```
0.5*acc*(1 + ratio)*t1² + v0*(1 + ratio)*t1 + [(v0² - ve²)/(2*dec) - dp] = 0
```

## Final Result

This gives us the quadratic equation:
```
a_coeff * t1² + b_coeff * t1 + c_coeff = 0
```

Where:
```
a_coeff = 0.5 * acc * (1 + ratio)
b_coeff = v0 * (1 + ratio)
c_coeff = (v0² - ve²) / (2 * dec) - dp
       = -dp + (v0² - ve²) / (2 * dec)
```

## Verification

### Symmetric Case (acc = dec, ratio = 1)

When acceleration equals deceleration:
```
a_coeff = 0.5 * acc * 2 = acc
b_coeff = v0 * 2 = 2*v0
c_coeff = -dp + (v0² - ve²)/(2*acc)
```

## Implementation Reference

These coefficients are implemented in `two_point_interpolation_constant_acc.py` in the `_calc_case0()` method:

```python
ratio = acc / dec
a_coeff = 0.5 * acc * (1.0 + ratio)
b_coeff = v0 * (1.0 + ratio)
c_coeff = -dp + (v0 * v0 - ve * ve) / (2.0 * dec)
```

## Related Documents

- [CONSTANT_ACC_DERIVATION.md](CONSTANT_ACC_DERIVATION.md): Overview of the trajectory planning algorithm
- [README.md](README.md): User guide and quick start
- [CHANGELOG.md](CHANGELOG.md): Project history and changes
