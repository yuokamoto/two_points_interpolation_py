# Testing Documentation

This document describes the comprehensive testing system for the two_point_interpolation library.

## Overview

The testing system is based on YAML-driven test cases that provide comprehensive coverage of all trajectory planning scenarios. This approach ensures maintainability, readability, and easy extension of test cases.

## Test Structure

### Test Categories

1. **basic_tests**: Fundamental functionality tests
2. **case0_tests**: Case 0 scenarios (vmax not reached)
3. **case1_tests**: Case 1 scenarios (vmax reached)
4. **performance_tests**: Performance comparison tests
5. **constraint_tests**: Constraint validation tests
6. **integration_tests**: Utility function tests
7. **boundary_tests**: Boundary condition tests
8. **overspeed_tests**: Overspeed warning tests
9. **error_tests**: Error condition tests

### Test Types

Tests are categorized by `test_type`:

- `standard`: Normal trajectory calculation tests
- `error`: Tests that expect errors during trajectory calculation
- `constraint_validation`: Tests that validate constraint parameters
- `no_init`: Tests that verify error when calc_trajectory is called without initialization
- `utility`: Tests for utility functions (v_integ, p_integ)
- `comparison`: Tests that compare multiple configurations
- `boundary`: Tests for boundary conditions

## Running Tests

### Quick Start

```bash
# Run all tests
python -m pytest tests/test_yaml_cases.py -v

# Run specific test category
python -m pytest tests/test_yaml_cases.py::YAMLBasedTestRunner::test_case0_scenarios -v

# Run tests with coverage
python -m pytest tests/test_yaml_cases.py --cov=two_point_interpolation
```

### Test Categories

```bash
# Basic functionality
python -m pytest tests/test_yaml_cases.py::YAMLBasedTestRunner::test_basic_functionality -v

# Case 0 scenarios (no cruise phase)
python -m pytest tests/test_yaml_cases.py::YAMLBasedTestRunner::test_case0_scenarios -v

# Case 1 scenarios (with cruise phase)
python -m pytest tests/test_yaml_cases.py::YAMLBasedTestRunner::test_case1_scenarios -v

# Performance comparisons
python -m pytest tests/test_yaml_cases.py::YAMLBasedTestRunner::test_performance_comparisons -v

# Error conditions
python -m pytest tests/test_yaml_cases.py::YAMLBasedTestRunner::test_error_conditions -v

# Overspeed conditions
python -m pytest tests/test_yaml_cases.py::YAMLBasedTestRunner::test_overspeed_conditions -v

# Boundary conditions
python -m pytest tests/test_yaml_cases.py::YAMLBasedTestRunner::test_boundary_conditions -v

# Constraint validation
python -m pytest tests/test_yaml_cases.py::YAMLBasedTestRunner::test_constraint_validation -v

# Integration functions
python -m pytest tests/test_yaml_cases.py::YAMLBasedTestRunner::test_integration_functions -v
```

## Test Visualization

The library includes a powerful visualization system for debugging and understanding trajectory behavior.

### Generating Plots

```bash
# Generate representative plots from each category
python tests/generate_plots.py

# Generate all visualizable test cases
python tests/generate_plots.py --all

# Generate plots for a specific category
python tests/generate_plots.py --category case1_tests

# Generate plot for a specific test case
python tests/generate_plots.py --case test_case1_forward_zero_velocities

# List available categories and visualizable test counts
python tests/generate_plots.py --list
```

### Visualization Features

- **Position, Velocity, Acceleration plots**: Complete trajectory analysis
- **Phase boundaries**: Visual markers for trajectory phase transitions
- **Constraint visualization**: Shows vmax, amax limits
- **Automatic file naming**: Safe filenames based on test case names
- **High-quality output**: 300 DPI PNG files suitable for documentation

### Debugging with Visualization

When a test fails, you can visualize the trajectory to understand what went wrong:

```bash
# If test_problematic_case fails
python tests/generate_plots.py --case test_problematic_case
```

This generates a detailed plot showing:
- Expected vs actual trajectory
- Constraint violations
- Phase transitions
- Numerical values

## Test Configuration

### YAML Test Cases

Test cases are defined in `tests/test_cases.yaml`. Each test case includes:

```yaml
- name: test_case_name
  description: "Human-readable description"
  test_type: standard  # or error, constraint_validation, etc.
  params:
    p0: 0.0
    pe: 10.0
    acc_max: 2.0
    vmax: 3.0
    dec_max: 2.0
    v0: 0.0
    ve: 0.0
  expected:
    case: 1
    total_time: 5.0
    warning: false
  notes: |
    Optional notes about the test case
```

### Error Message Validation

For error tests, the system supports flexible error message validation:

```yaml
expected:
  error: ValueError
  error_contains: "specific error text"  # Single string (must be present)
  # OR
  error_contains:
    - "text1"
    - "text2"  # All strings must be present (AND condition)
  # OR
  error_contains_any:
    - "option1"
    - "option2"  # Any string can be present (OR condition)
```

### Warning Validation

The system can validate warning behavior:

```yaml
expected:
  warning: true  # Should generate warning
  warning_contains: "expected warning text"
  # OR
  warning: false  # Should NOT generate warning
  # OR omit warning field for "warnings allowed"
```

## Test Data Management

### Adding New Test Cases

1. Add test case to appropriate section in `tests/test_cases.yaml`
2. Specify correct `test_type`
3. Run tests to verify
4. Generate visualization if needed

### Test Case Categories

- **51 total test cases** across 9 categories
- **32 visualizable cases** for trajectory analysis
- **Comprehensive coverage** of all trajectory scenarios
- **Physics validation** with boundary continuity checks

## Legacy Tests

Legacy test files are preserved for reference:
- `test_constant_acc.py.deprecated`: Original hardcoded tests
- `generate_plots_old.py`: Previous visualization implementation

## Best Practices

### Writing Test Cases

1. **Descriptive names**: Use clear, descriptive test case names
2. **Complete documentation**: Include description and notes
3. **Realistic parameters**: Use physically meaningful values
4. **Edge cases**: Test boundary conditions and error cases
5. **Expected results**: Always specify expected outcomes

### Debugging Failed Tests

1. **Read the error message**: Test failures include detailed context
2. **Visualize the trajectory**: Use `generate_plots.py` for visual debugging
3. **Check boundary conditions**: Verify continuity at phase transitions
4. **Validate physics**: Ensure parameters make physical sense

### Adding New Features

1. **Add test cases first**: Define expected behavior in YAML
2. **Implement feature**: Write the code to pass tests
3. **Verify visualization**: Ensure plots look correct
4. **Update documentation**: Add notes about new functionality

## Test Statistics

Current test coverage:

| Category | Test Cases | Visualizable | Purpose |
|----------|------------|--------------|---------|
| basic_tests | 3 | 2 | Core functionality |
| case0_tests | 12 | 12 | No cruise phase scenarios |
| case1_tests | 13 | 13 | With cruise phase scenarios |
| performance_tests | 2 | 0 | Performance comparisons |
| constraint_tests | 4 | 0 | Parameter validation |
| integration_tests | 2 | 0 | Utility functions |
| boundary_tests | 2 | 2 | Boundary conditions |
| overspeed_tests | 3 | 3 | Warning scenarios |
| error_tests | 5 | 0 | Error conditions |
| **Total** | **51** | **32** | **Complete coverage** |

## Continuous Integration

Tests are automatically run on:
- Every push to any branch
- Pull requests
- Scheduled daily runs

The CI system:
- Runs all test categories
- Generates coverage reports
- Validates code quality
- Ensures no regressions

## Troubleshooting

### Common Issues

1. **Import errors**: Ensure the library is properly installed
2. **Missing dependencies**: Install matplotlib for visualization
3. **File permissions**: Ensure write access to tests/plots/ directory
4. **Memory issues**: Use `--all` sparingly for large test suites

### Getting Help

- Check test output for detailed error messages
- Use visualization to understand trajectory behavior
- Review test case YAML for expected vs actual values
- Consult physics documentation for trajectory planning concepts
