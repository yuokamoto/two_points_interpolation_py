# Two Points Interpolation Python

Calculate optimal trajectory (position, velocity, acceleration) between two points with kinematic constraints.

## Overview

This library provides two trajectory planning algorithms:
1. **Constant Acceleration**: Generates smooth trajectories with acceleration limits
2. **Constant Jerk**: Generates even smoother trajectories with jerk (acceleration derivative) limits


## Installation

### From PyPI (when published)

```bash
pip install two-point-interpolation
```

### From Source

```bash
git clone https://github.com/yuokamoto/two_points_interpolation_py.git
cd two_points_interpolation_py
pip install -e .
```

### For Development

```bash
git clone https://github.com/yuokamoto/two_points_interpolation_py.git
cd two_points_interpolation_py
pip install -e ".[dev]"
```

## Quick Start

### Constant Acceleration

```python
from two_point_interpolation import TwoPointInterpolation

# Create interpolator
interp = TwoPointInterpolation()

# Set start, end, and constraints
interp.init(p0=0.0, pe=100.0, acc_max=2.0, vmax=10.0, dec_max=4.0)

# Calculate trajectory
total_time = interp.calc_trajectory()

# Get state at any time
pos, vel, acc = interp.get_point(t=5.0)
```

### Constant Jerk

```python
from two_point_interpolation.constant_jerk import TwoPointInterpolationJerk

interp = TwoPointInterpolationJerk()
interp.init(p0=0.0, pe=100.0, amax=2.0, vmax=10.0, jmax=1.0)

total_time = interp.calc_trajectory()
pos, vel, acc, jerk = interp.get_point(t=5.0)
```

**Note**: Constant jerk implementation is currently under review. See TODO section below.

## Examples

Run example scripts to see visualizations:

```bash
# Basic constant acceleration
python3 examples/example_constant_acc.py

# Constant jerk (smoother)
python3 examples/example_constant_jerk.py
```

## Testing

```bash
# Run all tests
python3 -m pytest tests/ -v
```

## Project Structure

```
two_points_interpolation_py/
├── two_point_interpolation/              # Main package
│   ├── __init__.py                       # Package exports
│   ├── constant_acc.py                   # Acceleration-based planning
│   └── constant_jerk.py                  # Jerk-based planning (TODO: needs review)
├── examples/                             # Example scripts
│   └── images/                           # Generated plots
├── tests/                                # Unit tests 
└── docs/                                 # Documentation
    ├── CONSTANT_ACC_DERIVATION.md        # Mathematical details
    ├── QUADRATIC_COEFFICIENTS_DERIVATION.md  # Quadratic solution derivation
    └── CHANGELOG.md                      # Version history
```

## Example Results

### Case 0: vmax not reached
![Case 0](https://raw.githubusercontent.com/yuokamoto/two_points_interpolation_py/main/examples/images/acc_constant_0.png)

**Parameters**: `t0=1.0, p0=5, pe=15, acc_max=2.0, dec_max=3.0, vmax=10.0, v0=0, ve=0`

Trajectory when the peak velocity is below vmax. Shows two phases: acceleration and deceleration.

### Case 1: vmax reached
![Case 1](https://raw.githubusercontent.com/yuokamoto/two_points_interpolation_py/main/examples/images/acc_constant_1.png)

**Parameters**: `t0=0, p0=0, pe=50, acc_max=2.0, dec_max=4.0, vmax=8.0, v0=2.0, ve=1.0`

Trajectory when vmax is reached. Shows three phases: acceleration, constant velocity, and deceleration.

## Documentation

- **Getting Started**: This README
- **Mathematical Derivation**: [docs/CONSTANT_ACC_DERIVATION.md](docs/CONSTANT_ACC_DERIVATION.md)
- **Detailed Coefficient Derivation**: [docs/QUADRATIC_COEFFICIENTS_DERIVATION.md](docs/QUADRATIC_COEFFICIENTS_DERIVATION.md)
- **Change History**: [docs/CHANGELOG.md](docs/CHANGELOG.md)

## TODO

### Constant Jerk Implementation Issues
The `constant_jerk` module requires significant improvements:

1. **API Contract Violation**: The `ve` (final velocity) parameter is accepted in `init()` but never used in calculations
2. **Debug Output**: `calc_trajectory()` prints debug information to stdout (should use logging or be removed)
3. **Time Boundary Bug**: Case 3 condition uses `4*t1+2*2*t2+t3` instead of correct `4*t1+2*t2+t3`, causing phase transition errors
4. **Missing Tests**: No test coverage for constant jerk functionality
5. **Support acc_max != dec_max**: Extend to support independent acceleration/deceleration limits

### Other
- Add comprehensive test suite for `constant_jerk` module
- Verify mathematical correctness of jerk-based trajectories

## Requirements

- Python 3.6+
- NumPy
- Matplotlib (for examples)

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

This software is provided "AS IS" without warranty of any kind.

