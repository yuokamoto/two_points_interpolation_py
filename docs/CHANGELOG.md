# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.3.0] - 2024-12-03

### Added
- **Angle Interpolation Support**: New `TwoAngleInterpolation` class for angle-aware trajectory planning
  - Automatic angle normalization to [-π, π] range
  - Shortest rotation path calculation (e.g., 350° to 10° goes through 0°, not 180°)
  - `normalize_angle()` utility function for angle normalization using atan2
  - Optional output normalization via `normalize_output` parameter in `get_point()`
- **Comprehensive Angle Testing**: 6 new test cases covering various angle interpolation scenarios
  - Shortest path verification with midpoint checks
  - Full rotation (360°) handling
  - Negative angle support
  - Boundary crossing tests

### Changed
- **API Consistency**: Parameter naming standardized to match parent class
  - Child class `init()` uses `acc_max` instead of `amax` for Liskov Substitution Principle compliance
  - Removed redundant `__init__()` method following Python best practices

### Fixed
- **Type Safety**: Removed `calc_trajectory()` override that violated Liskov Substitution Principle
  - Users now call `init()` followed by `calc_trajectory()` for angle interpolation
  - Eliminates need for `# type: ignore[override]` annotations

## [1.2.1] - 2024-11-30

### Fixed
- Fixed version inconsistency between `__init__.py` and `pyproject.toml` files
- Ensured proper version synchronization for PyPI package distribution

## [1.2.0] - 2024-11-30

### Added
- **Comprehensive YAML-Based Testing System**: Complete overhaul of testing infrastructure
  - 51 test cases across 9 categories with comprehensive coverage
  - Flexible error message validation with AND/OR conditions (`error_contains`, `error_contains_any`)
  - Test-type based routing (standard, error, constraint_validation, utility, comparison, boundary)
  - Backward compatibility for existing test configurations
- **Advanced Visualization & Debugging System**:
  - Complete rewrite of trajectory visualization for debugging support
  - Generate plots for all successful test cases (32 visualizable cases)
  - Multiple visualization modes: representative, category-specific, individual tests, all cases
  - High-quality plots with phase boundaries, constraints, and detailed trajectory information
- **Comprehensive Testing Documentation**:
  - Dedicated testing documentation (`tests/README.md`) with usage examples
  - Visual debugging guides and troubleshooting instructions
  - Test statistics and coverage information

### Changed
- **Simplified Code Architecture**:
  - Consolidated error message validation using `any()` for cleaner logic
  - Removed unnecessary enable/disable flags from TestVisualizer
  - Improved separation of concerns between standard and error test handling
  - Clean up legacy compatibility code for better maintainability
- **Enhanced CI Pipeline**: Updated to test new YAML system and visualization functionality
- **Documentation Structure**: Moved detailed improvements from README to CHANGELOG

### Improved
- **Error Message Handling**: More robust and flexible error message validation system
- **Test Case Management**: Easier addition and maintenance of test cases through YAML
- **Debug Workflow**: Visual trajectory debugging for failed test cases
- **Code Quality**: Better separation of concerns and cleaner implementation

## [1.1.0] - 2024-11-28

### Added
- **Enhanced Error Messages**: Comprehensive 3-tier error message system for trajectory planning failures
  - Same goal detection (within 2% tolerance) with contextual information
  - Clear distance shortage messages with percentage calculations
  - Improved constraint violation messages with specific parameter details
- **Overspeed Handling**: Robust handling of cases where initial velocity exceeds maximum velocity
  - Automatic detection of `|v0| > vmax` conditions
  - UserWarning issued with helpful information
  - Correct physics implementation for overspeed deceleration in Phase 1
- **YAML-Based Testing System**: Complete migration to data-driven testing
  - 51+ comprehensive test cases covering all functionality
  - Test categories: basic, case0, case1, performance, constraints, integration, boundaries, overspeed, errors, visualization
  - Easy test case maintenance and extension through YAML configuration
- **Test Visualization System**: Comprehensive trajectory plotting for analysis
  - Integrated TestVisualizer class for generating trajectory plots
  - Support for all test categories with detailed plots (position, velocity, acceleration)
  - Command-line tool for generating documentation plots
- **Edge Cases Documentation**: Detailed analysis of boundary conditions and minimum distance requirements
  - Mathematical analysis of deceleration distance calculations
  - Visual documentation with embedded trajectory plots
  - Test case reproduction instructions

### Fixed
- **Critical Physics Bug**: Fixed incorrect acceleration in overspeed conditions (v0 > vmax)
  - Phase 1 now correctly uses deceleration (-dec) instead of acceleration (+acc)
  - Prevents incorrect intermediate positions (e.g., 4.125m → 1.875m for forward overspeed)
  - Applies to both forward (v0 > vmax) and backward (v0 < -vmax) overspeed conditions
- **Implementation Consistency**: Resolved differences between Python and C++ implementations
  - Added missing `std::fabs()` to C++ dt01 calculation for consistency with Python `np.fabs()`
  - Ensures identical trajectory calculations across language implementations

### Changed
- **Error Message Improvement**: More informative and actionable error messages
  - Added deceleration distance tolerance threshold (2%) for distinguishing error types
  - Enhanced error messages include current speed, position range, and specific shortage amounts
  - Context-aware messages help users understand the root cause of trajectory failures
- **Test Architecture**: Complete redesign of testing system
  - Migrated from hardcoded test methods to YAML-driven test cases
  - Improved test maintainability and coverage reporting
  - Standardized test case format with parameters, expectations, and documentation
- **Code Organization**: Consolidated visualization functionality
  - Merged test visualization utilities into a single module
  - Removed redundant helper files and simplified project structure

### Technical Details

#### Error Detection Algorithm
```python
# New 3-tier error detection system
decel_distance = v0**2 / (2 * dec_max)
available_distance = abs(pe - p0)
percentage_diff = abs(decel_distance - available_distance) / decel_distance

if percentage_diff <= 0.02:  # Within 2%
    # "Same goal resent during motion" error
elif decel_distance > available_distance:  # >2% shortage
    # "Insufficient distance to decelerate" error
else:
    # General constraint violation error
```

#### Overspeed Physics Correction
```python
# Fixed overspeed handling in Phase 1
sign = 1 if pe > p0 else -1
if sign * v0 > vmax:
    warnings.warn(f"Initial velocity {v0} exceeds vmax {vmax}")
    # Use deceleration instead of acceleration for Phase 1
    phase1_acc = -dec_max if sign > 0 else dec_max
else:
    # Normal acceleration for Phase 1
    phase1_acc = acc_max if sign > 0 else -acc_max
```

### Migration Guide

#### For Users Updating from v0.1.x:
- No breaking API changes - all existing code continues to work
- Enhanced error messages provide more context for debugging
- New UserWarning for overspeed conditions (can be suppressed if needed)

#### For Developers:
- Test cases migrated to `tests/test_cases.yaml` - modify YAML for new test cases
- Use `pytest tests/test_yaml_cases.py -v` instead of old test files
- Visualization available via `python tests/generate_plots.py --category <category>`

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
