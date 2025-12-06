#!/usr/bin/env python3
"""
Edge case tests for getter methods: get_duration() and get_end_time()

This module contains tests for getter-specific edge cases and error handling.
Most getter functionality is tested as part of the YAML test suite in test_yaml_cases.py.
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import unittest
from two_point_interpolation import TwoPointInterpolation


class TestGetterMethodsEdgeCases(unittest.TestCase):
    """Test getter methods edge cases and error handling."""

    def test_get_duration_before_calc_trajectory(self):
        """Test that get_duration() raises error before calc_trajectory()."""
        interp = TwoPointInterpolation()
        interp.set_initial(t0=0.0, p0=0.0, v0=0.0)
        interp.set_point(pe=10.0, ve=0.0)
        interp.set_constraints(vmax=2.0, acc_max=1.0)

        # Should raise RuntimeError before calling calc_trajectory()
        with self.assertRaises(RuntimeError) as context:
            interp.get_duration()

        self.assertIn("Trajectory not calculated", str(context.exception))

    def test_get_end_time_before_calc_trajectory(self):
        """Test that get_end_time() raises error before calc_trajectory()."""
        interp = TwoPointInterpolation()
        interp.set_initial(t0=0.0, p0=0.0, v0=0.0)
        interp.set_point(pe=10.0, ve=0.0)
        interp.set_constraints(vmax=2.0, acc_max=1.0)

        # Should raise RuntimeError before calling calc_trajectory()
        with self.assertRaises(RuntimeError) as context:
            interp.get_end_time()

        self.assertIn("Trajectory not calculated", str(context.exception))

    def test_getters_consistency_multiple_calls(self):
        """Test that getters return consistent values across multiple calls."""
        interp = TwoPointInterpolation()
        interp.set_initial(t0=1.0, p0=0.0, v0=0.0)
        interp.set_point(pe=10.0, ve=0.0)
        interp.set_constraints(vmax=2.0, acc_max=1.0)

        duration = interp.calc_trajectory()

        # Call getters multiple times and verify consistency
        for i in range(10):
            self.assertAlmostEqual(interp.get_duration(), duration, places=10,
                                 msg=f"get_duration() inconsistent on call {i+1}")
            self.assertAlmostEqual(interp.get_end_time(), 1.0 + duration, places=10,
                                 msg=f"get_end_time() inconsistent on call {i+1}")

    def test_getters_after_recalculation(self):
        """Test that getters update correctly after trajectory recalculation."""
        interp = TwoPointInterpolation()

        # First trajectory
        interp.set_initial(t0=0.0, p0=0.0, v0=0.0)
        interp.set_point(pe=10.0, ve=0.0)
        interp.set_constraints(vmax=2.0, acc_max=1.0)
        duration1 = interp.calc_trajectory()

        self.assertAlmostEqual(interp.get_duration(), duration1, places=10)
        self.assertAlmostEqual(interp.get_end_time(), 0.0 + duration1, places=10)

        # Second trajectory (different parameters)
        interp.set_initial(t0=5.0, p0=10.0, v0=1.0)
        interp.set_point(pe=30.0, ve=2.0)
        interp.set_constraints(vmax=5.0, acc_max=2.0)
        duration2 = interp.calc_trajectory()

        # Getters should now return values for the second trajectory
        self.assertAlmostEqual(interp.get_duration(), duration2, places=10)
        self.assertAlmostEqual(interp.get_end_time(), 5.0 + duration2, places=10)

        # Durations should be different
        self.assertNotAlmostEqual(duration1, duration2, places=3,
                                 msg="Durations should differ for different trajectories")


if __name__ == '__main__':
    unittest.main()
