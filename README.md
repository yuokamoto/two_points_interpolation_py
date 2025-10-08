# Two Points Interpolation Python

Calculate optimal trajectory (position, velocity, acceleration) between two points with kinematic constraints.

## Overview

This library provides two trajectory planning algorithms:
1. **Constant Acceleration**: Generates smooth trajectories with acceleration limits
2. **Constant Jerk**: Generates even smoother trajectories with jerk (acceleration derivative) limits

## Features

- ✅ Position, velocity, and acceleration constraints
- ✅ Time-optimal trajectory generation
- ✅ Smooth motion profiles
- ✅ Error handling and input validation
- ✅ Type hints for better IDE support
- ✅ Unit tests included

## Installation

```bash
git clone https://github.com/yuokamoto/two_points_interpolation_py.git
cd two_points_interpolation_py
```

## Quick Start

### Constant Acceleration Example

```python
import two_point_interpolation_constant_acc as tpi

# Create interpolator
interp = tpi.TwoPointInterpolation()

# Set parameters: start_pos, end_pos, max_acc, max_vel, start_time, start_vel, end_vel
interp.init(p0=0.0, pe=100.0, amax=2.0, vmax=10.0, t0=0.0, v0=0.0, ve=0.0)

# Calculate trajectory
total_time = interp.calc_trajectory()

# Get position, velocity, acceleration at any time t
pos, vel, acc = interp.get_point(t=5.0)
```

### Constant Jerk Example

```python
import two_point_interpolation_constant_jerk as tpi

# Create interpolator  
interp = tpi.TwoPointInterpolation()

# Method 1: New API (compatible with constant_acc)
interp.init(p0=0.0, pe=100.0, amax=2.0, vmax=10.0, jmax=1.0, t0=0.0)

# Method 2: Original API
# max_values = [vmax, amax, jmax]
# interp.set(ps=0.0, pe=100.0, max=max_values)
# interp.set_initial_time(0.0)

# Calculate trajectory
total_time = interp.calc_trajectory()

# Get position, velocity, acceleration, jerk at any time t
pos, vel, acc, jerk = interp.get_point(t=5.0)
```

## Files

- `two_point_interpolation_constant_acc.py`: Constant acceleration trajectory planner
- `two_point_interpolation_constant_jerk.py`: Constant jerk trajectory planner  
- `two_point_interpolation_test_constant_acc.py`: Demo script for constant acceleration
- `two_point_interpolation_test_constant_jerk.py`: Demo script for constant jerk
- `test_two_point_interpolation.py`: Unit tests for both modules

## Running Tests

```bash
# Run unit tests
python test_two_point_interpolation.py

# Run demo scripts
python two_point_interpolation_test_constant_acc.py
python two_point_interpolation_test_constant_jerk.py
```

## Example result
### Constant ACC
#### case 0: not reach velocity limit
constraints: 
- p0 = 10, pe = 170.0
- v0 = -5, ve = 1
- amax = 1, vmax = 100
- t0 = 10

![alt text](images/acc_constant_0.png)

#### case 1: reach velocity limit
constraints: 
- p0 = 50, pe = -90.0
- v0 = -5, ve = 1
- amax = 1, vmax = 10
- t0 = 0

![alt text](images/acc_constant_1.png)
