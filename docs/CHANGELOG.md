# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.1] - 2025-11-16

### Fixed
- **PyPI Packaging**: Fixed license metadata and README display on PyPI
  - Changed license specification from inline text to file reference for proper PyPI recognition
  - Updated README image URLs from relative paths to absolute GitHub raw URLs
  - Images now display correctly on PyPI/TestPyPI package page

## [0.1.0] - 2025-11-14

### Added
- **Pip Installable Package**: Now available as a standard Python package
  - Package name: `two-point-interpolation`
  - Install with: `pip install two-point-interpolation` (when published to PyPI)
  - Development install: `pip install -e .`
  - Modern packaging using `pyproject.toml`

- **Different Acceleration/Deceleration Support**: Independent max acceleration and deceleration values
  - Optional `dec_max` parameter (defaults to `acc_max`)
  - New API: `set_constraints(acc_max, vmax, dec_max=None)`
  - Mathematical derivation in `QUADRATIC_COEFFICIENTS_DERIVATION.md`

- **Comprehensive Test Coverage**: Parameterized tests with multiple scenarios
  - 12 test cases for Case 0 (vmax not reached)
  - 13 test cases for Case 1 (vmax reached)
  - Tests include: forward/backward, symmetric/asymmetric acc/dec, non-zero v0/ve
  - Total 36+ test scenarios executed via parameterized testing

- **Dual Export in Package**: Both interpolation methods available at top level
  - `TwoPointInterpolation` (default, constant acceleration)
  - `TwoPointInterpolationAcc` (explicit constant acceleration)
  - `TwoPointInterpolationJerk` (constant jerk)

### Changed
- **Package Structure**: Reorganized as proper Python package
  - Module moved to `two_point_interpolation/` directory
  - `constant_acc.py` and `constant_jerk.py` as submodules
  - `__init__.py` provides convenient imports
  - Old top-level files removed

- **API Parameter Naming**: Improved clarity and consistency
  - `amax` → `acc_max` (maximum acceleration)
  - `amax_decel` → `dec_max` (maximum deceleration)
  - Applied across all functions: `set_constraints()`, `init()`

- **Project Structure**: Better organization
  - Consolidated test files into `tests/test_constant_acc.py` (13 test functions, 36+ test cases)
  - Consolidated examples into `examples/example_constant_acc.py`
  - Documentation moved to `docs/` directory (except README.md)
  - Test files → `tests/` directory
  - Example files → `examples/` directory

- **Quadratic Solution Selection**: More robust handling of two solutions
  - Evaluates both roots of quadratic equation
  - Selects smallest positive solution (most efficient)
  - Better error handling for edge cases

- **Example Scripts**: Enhanced usability
  - Command-line options: `--show` (default on), `--no-show`, `--save` (default off)
  - Updated to use non-zero initial conditions (t0=1.0 for Case 0, v0=2.0, ve=1.0 for Case 1)
  - Better demonstration of general use cases

### Improved
- **Documentation**:
  - Simplified README (removed redundant "asymmetric" terminology)
  - Added installation instructions (PyPI, source, development modes)
  - Detailed mathematical derivation in separate document
  - Improved code comments explaining edge cases
  - Better error messages with actionable suggestions
  - Organized documentation into `docs/` directory

- **Test Quality**:
  - Physics-based tolerances for boundary continuity (dt*vmax, dt*amax)
  - Physical validity checks (total_time >= dp/vmax)
  - Helper functions for common assertions (`assert_final_state`, `assert_boundary_continuity`)
  - Boundary continuity automatically checked at all phase transitions
  - Eliminated redundant tests (24 tests consolidated to 13 functions)
  - `assertEqual` argument order fixed to (expected, actual)

- **Internal Code Quality**:
  - Simplified variable names (`acc`, `dec` instead of `a_accel`, `a_decel`)
  - Enhanced edge case detection (numerical errors, impossible trajectories)
  - Mathematical validation comments for dt12 < 0 check
  - Removed debug print statements from production code

### Fixed
- **Constraint Validation**: More comprehensive error checking
  - Validates negative values for acc_max, vmax, and dec_max
  - Validates zero values
  - Clear error messages for invalid constraints

## [0.0.1] - Initial Development

### Added
- Two-point interpolation with constant acceleration
- Two-point interpolation with constant jerk
- Basic test coverage and examples
