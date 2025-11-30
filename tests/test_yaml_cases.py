#!/usr/bin/env python3
"""
Comprehensive YAML-based test runner for two_point_interpolation.

This replaces test_constant_acc.py with a data-driven approach using YAML test cases.
All test cases from the original file have been migrated to test_cases.yaml.
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import unittest
import warnings
from two_point_interpolation import TwoPointInterpolation, v_integ, p_integ
from test_case_loader import load_test_cases

EXCEPTION_MAP = {
    'ValueError': ValueError,
    'TypeError': TypeError,
    'AttributeError': AttributeError,
    'RuntimeError': RuntimeError,
    'NotImplementedError': NotImplementedError,
}


class YAMLBasedTestRunner(unittest.TestCase):
    """Test runner that executes all test cases from YAML configuration."""

    def setUp(self):
        """Load test cases from YAML."""
        self.test_cases = load_test_cases()

    def run_single_test_case(self, test_case):
        """
        Execute a single test case and verify expectations.

        Args:
            test_case: Test case dictionary from YAML
        """
        name = test_case['name']
        description = test_case['description']
        test_type = test_case.get('test_type', 'standard')

        with self.subTest(name=name, description=description):
            # Route to appropriate test handler based on test_type
            handler_map = {
                'constraint_validation': self._run_constraint_validation_test,
                'no_init': self._run_no_init_test,
                'utility': self._run_utility_test,
                'comparison': self._run_comparison_test,
                'boundary': self._run_boundary_test,
                'error': self._run_error_trajectory_test_wrapper,
                'standard': self._run_standard_trajectory_test
            }

            handler = handler_map.get(test_type, self._run_standard_trajectory_test)

            try:
                handler(test_case)
            except Exception as e:
                self.fail(f"Test {name} failed with exception: {e}")

    def _run_constraint_validation_test(self, test_case):
        """Handle constraint validation tests."""
        params = test_case.get('params', {})
        expected = test_case.get('expected', {})

        tpi = TwoPointInterpolation()
        error_type = EXCEPTION_MAP.get(expected['error'])
        if error_type is None:
            raise ValueError(f"Unknown exception type: {expected['error']}")

        with self.assertRaises(error_type):
            tpi.set_constraints(**params)

    def _run_no_init_test(self, test_case):
        """Handle tests that verify proper error when calc_trajectory is called without initialization."""
        expected = test_case.get('expected', {})

        tpi = TwoPointInterpolation()
        error_type = EXCEPTION_MAP.get(expected['error'])
        if error_type is None:
            raise ValueError(f"Unknown exception type: {expected['error']}")

        with self.assertRaises(error_type) as context:
            tpi.calc_trajectory()  # Should fail - no init called

        # Check error message if specified
        if 'error_contains' in expected:
            error_msg = str(context.exception)
            self.assertIn(expected['error_contains'], error_msg)

    def _run_utility_test(self, test_case):
        """Handle utility function tests."""
        params = test_case.get('params', {})
        expected = test_case.get('expected', {})
        func_name = test_case['function']

        if func_name == 'v_integ':
            result = v_integ(**params)
        elif func_name == 'p_integ':
            result = p_integ(**params)
        else:
            self.fail(f"Unknown utility function: {func_name}")

        self.assertEqual(expected['result'], result)

    def _run_boundary_test(self, test_case):
        """Handle boundary tests."""
        tpi = TwoPointInterpolation()
        self.run_boundary_test(test_case, tpi)

    def _run_error_trajectory_test_wrapper(self, test_case):
        """Wrapper for error trajectory tests."""
        tpi = TwoPointInterpolation()
        self._run_error_trajectory_test(test_case, tpi)

    def _run_standard_trajectory_test(self, test_case):
        """Handle standard trajectory tests (no errors expected)."""
        params = test_case.get('params', {})
        expected = test_case.get('expected', {})

        tpi = TwoPointInterpolation()

        # Normal initialization and trajectory calculation
        tpi.init(**params)

        # Check initialization properties if specified
        self._verify_initialization_properties(tpi, expected)

        # Calculate trajectory with appropriate warning handling
        total_time = self._calculate_trajectory_with_warnings(test_case, tpi)

        # Verify trajectory expectations (includes final state)
        self._verify_trajectory_expectations(test_case, tpi, total_time)

        # Physical validity checks (boundary continuity only)
        self._verify_boundary_continuity(test_case, tpi, total_time)

    def _run_error_trajectory_test(self, test_case, tpi):
        """Handle tests that expect errors during trajectory calculation."""
        params = test_case.get('params', {})
        expected = test_case.get('expected', {})

        # Initialize if parameters are provided
        if params:
            tpi.init(**params)

        # Should raise error during calc_trajectory
        error_type = EXCEPTION_MAP.get(expected['error'])
        if error_type is None:
            raise ValueError(f"Unknown exception type: {expected['error']}")

        with self.assertRaises(error_type) as context:
            tpi.calc_trajectory()

        # Check error message content
        self._verify_error_message(context.exception, expected)

    def _verify_error_message(self, exception, expected):
        """Verify error message contains expected text."""
        error_msg = str(exception)

        if 'error_contains' in expected:
            # Support both string and list for error_contains
            contains_list = expected['error_contains']
            if isinstance(contains_list, str):
                contains_list = [contains_list]  # Convert single string to list

            # All strings in the list must be present (AND condition)
            for text in contains_list:
                self.assertIn(text, error_msg)

        if 'error_contains_any' in expected:
            # At least one string from the list must be present (OR condition)
            self.assertTrue(
                any(text.lower() in error_msg.lower() for text in expected['error_contains_any']),
                f"None of {expected['error_contains_any']} found in: {error_msg}"
            )

    def _verify_initialization_properties(self, tpi, expected):
        """Verify initialization properties if specified."""
        if 'properties' in expected:
            for prop, value in expected['properties'].items():
                # Use assertEqual for exact values (like original test_init_method)
                self.assertEqual(value, getattr(tpi, prop))

        # Check initialization flags if init_success is True (like original test_init_method)
        if expected.get('init_success', False):
            self.assertTrue(tpi.point_setted, "point_setted flag should be True after init")
            self.assertTrue(tpi.constraints_setted, "constraints_setted flag should be True after init")
            self.assertTrue(tpi.initial_state_setted, "initial_state_setted flag should be True after init")

    def _calculate_trajectory_with_warnings(self, test_case, tpi):
        """Calculate trajectory with appropriate warning handling."""
        name = test_case['name']
        expected = test_case.get('expected', {})

        if expected.get('warning', False):
            # Should generate warning
            with warnings.catch_warnings(record=True) as w:
                warnings.simplefilter("always")
                total_time = tpi.calc_trajectory()

                self.assertEqual(len(w), 1)
                self.assertTrue(issubclass(w[0].category, UserWarning))
                if 'warning_contains' in expected:
                    self.assertIn(expected['warning_contains'], str(w[0].message))
        elif expected.get('warning') is False:
            # Should NOT generate warning
            with warnings.catch_warnings():
                warnings.simplefilter("error")  # Turn warnings into errors
                try:
                    total_time = tpi.calc_trajectory()
                except UserWarning:
                    self.fail(f"Unexpected warning in test {name}")
        else:
            # Standard case - warnings allowed
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                total_time = tpi.calc_trajectory()

        return total_time

    def _verify_trajectory_expectations(self, test_case, tpi, total_time):
        """Verify trajectory calculation results meet expectations."""
        name = test_case['name']
        expected = test_case.get('expected', {})

        # Check trajectory-specific expectations
        if 'case' in expected:
            self.assertEqual(expected['case'], tpi.case, f"Wrong case for {name}")

        if 'total_time' in expected:
            self.assertAlmostEqual(expected['total_time'], total_time, places=3)

        if 'total_time_min' in expected:
            self.assertGreater(total_time, expected['total_time_min'])

        if 'phase1_acceleration' in expected:
            self.assertAlmostEqual(expected['phase1_acceleration'], tpi.a[0], places=3)

        if 'p1' in expected:
            # Check intermediate position (p[1], not p[0] which is initial)
            self.assertAlmostEqual(expected['p1'], tpi.p[1], places=3)

        # Always check initial state (like original test_get_point_boundary_conditions)
        p_initial, v_initial, a_initial = tpi.get_point(0.0)
        self.assertAlmostEqual(tpi.p0, p_initial, places=5, msg=f"Initial position mismatch in {name}")
        self.assertAlmostEqual(tpi.v0, v_initial, places=5, msg=f"Initial velocity mismatch in {name}")

        # Always verify final state matches target (physical requirement)
        p_final, v_final, a_final = tpi.get_point(total_time)
        self.assertAlmostEqual(tpi.pe, p_final, places=3, msg=f"Did not reach target position in {name}")
        self.assertAlmostEqual(tpi.ve, v_final, places=3, msg=f"Did not reach target velocity in {name}")
        self.assertAlmostEqual(0.0, a_final, places=5, msg=f"Final acceleration should be zero in {name}")

    def _verify_boundary_continuity(self, test_case, tpi, total_time):
        """Verify boundary continuity at all phase transitions."""
        # Check boundary continuity only for non-zero trajectories
        if total_time > 0:
            self.check_boundary_continuity(tpi)

    def _run_comparison_test(self, test_case):
        """Run comparison tests (e.g., faster deceleration)."""
        params_set = test_case['params_set']
        expected_comparison = test_case['expected']['comparison']

        times = []
        tpi_instances = []

        for param_set in params_set:
            tpi = TwoPointInterpolation()
            tpi.init(**param_set['params'])
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                time = tpi.calc_trajectory()
            times.append(time)
            tpi_instances.append(tpi)

            # Check individual expectations
            if 'case' in param_set['expected']:
                self.assertEqual(param_set['expected']['case'], tpi.case)

        # Evaluate comparison
        if expected_comparison == "time[1] < time[0]":
            self.assertLess(times[1], times[0], "Second configuration should be faster")
        elif "abs(time[0] - time[1]) < 0.001" in expected_comparison:
            self.assertAlmostEqual(times[0], times[1], places=3, msg="Times should be nearly equal")

            # For nearly equal times, also verify trajectory similarity
            if len(tpi_instances) == 2:
                self._compare_trajectories(tpi_instances[0], tpi_instances[1], times[0])

    def _compare_trajectories(self, tpi1, tpi2, total_time, sample_points=5):
        """Compare two trajectories at multiple sample points.

        Args:
            tpi1: First TwoPointInterpolation instance
            tpi2: Second TwoPointInterpolation instance
            total_time: Total trajectory time to use for sampling
            sample_points: Number of sample points to compare (default: 5)
        """
        # Generate sample times (like original test_default_dec_max)
        sample_times = [total_time * i / (sample_points - 1) for i in range(sample_points)]

        for t in sample_times:
            p1, v1, a1 = tpi1.get_point(t)
            p2, v2, a2 = tpi2.get_point(t)

            # Compare position and velocity with appropriate tolerances
            self.assertAlmostEqual(p1, p2, places=3,
                                 msg=f"Position mismatch at t={t:.3f}: {p1} vs {p2}")
            self.assertAlmostEqual(v1, v2, places=3,
                                 msg=f"Velocity mismatch at t={t:.3f}: {v1} vs {v2}")
            # Note: acceleration comparison omitted as it can have numerical differences

    def run_boundary_test(self, test_case, tpi):
        """Run boundary condition tests."""
        params = test_case['params']
        expected = test_case['expected']
        test_time = test_case['test_time']

        tpi.init(**params)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            total_time = tpi.calc_trajectory()

        # Determine actual test time
        if test_time == 0.0:
            actual_time = 0.0
        elif test_time == "end+1.0":
            actual_time = total_time + 1.0
        else:
            actual_time = test_time

        p, v, a = tpi.get_point(actual_time)

        if 'position' in expected:
            self.assertAlmostEqual(expected['position'], p, places=5)
        if 'velocity' in expected:
            self.assertAlmostEqual(expected['velocity'], v, places=5)

    def check_boundary_continuity(self, tpi, eps=1e-6):
        """Check continuity at all phase boundaries."""
        # Physics-based tolerances
        p_tolerance = 1.1 * eps * tpi.vmax
        v_tolerance = 1.1 * eps * max(tpi.amax_accel, tpi.amax_decel)

        # Check continuity at each phase boundary
        cumulative_time = 0
        for i in range(len(tpi.dt) - 1):  # Loop through all boundaries between phases
            cumulative_time += tpi.dt[i]
            t_boundary = cumulative_time

            # Get values before, at, and after the boundary
            p_before, v_before, _ = tpi.get_point(t_boundary - eps)
            p_at, v_at, _ = tpi.get_point(t_boundary)
            p_after, v_after, _ = tpi.get_point(t_boundary + eps)

            # Check position continuity
            self.assertAlmostEqual(p_before, p_at, delta=p_tolerance,
                                 msg=f"Position discontinuity before boundary {i}")
            self.assertAlmostEqual(p_at, p_after, delta=p_tolerance,
                                 msg=f"Position discontinuity after boundary {i}")

            # Check velocity continuity
            self.assertAlmostEqual(v_before, v_at, delta=v_tolerance,
                                 msg=f"Velocity discontinuity before boundary {i}")
            self.assertAlmostEqual(v_at, v_after, delta=v_tolerance,
                                 msg=f"Velocity discontinuity after boundary {i}")

    # Test methods for each category
    def test_basic_functionality(self):
        """Test basic functionality cases."""
        for test_case in self.test_cases.get('basic_tests', []):
            self.run_single_test_case(test_case)

    def test_case0_scenarios(self):
        """Test Case 0 (vmax not reached) scenarios."""
        for test_case in self.test_cases.get('case0_tests', []):
            self.run_single_test_case(test_case)

    def test_case1_scenarios(self):
        """Test Case 1 (vmax reached) scenarios."""
        for test_case in self.test_cases.get('case1_tests', []):
            self.run_single_test_case(test_case)

    def test_performance_comparisons(self):
        """Test performance comparison scenarios."""
        for test_case in self.test_cases.get('performance_tests', []):
            self.run_single_test_case(test_case)

    def test_constraint_validation(self):
        """Test constraint validation scenarios."""
        for test_case in self.test_cases.get('constraint_tests', []):
            self.run_single_test_case(test_case)

    def test_integration_functions(self):
        """Test integration utility function scenarios."""
        for test_case in self.test_cases.get('integration_tests', []):
            self.run_single_test_case(test_case)

    def test_boundary_conditions(self):
        """Test boundary condition scenarios."""
        for test_case in self.test_cases.get('boundary_tests', []):
            self.run_single_test_case(test_case)

    def test_overspeed_conditions(self):
        """Test overspeed condition scenarios."""
        for test_case in self.test_cases.get('overspeed_tests', []):
            self.run_single_test_case(test_case)

    def test_error_conditions(self):
        """Test error condition scenarios."""
        for test_case in self.test_cases.get('error_tests', []):
            self.run_single_test_case(test_case)

    def test_all_yaml_cases_summary(self):
        """Summary test that counts and reports on all test cases."""
        total_tests = 0
        for category, test_cases in self.test_cases.items():
            total_tests += len(test_cases)

        print(f"\n✅ YAML Test Suite Summary:")
        print(f"   Total test cases: {total_tests}")
        for category, test_cases in self.test_cases.items():
            print(f"   - {category}: {len(test_cases)} cases")


if __name__ == '__main__':
    unittest.main(verbosity=2)
