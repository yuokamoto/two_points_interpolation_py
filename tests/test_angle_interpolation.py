#!/usr/bin/env python3
"""
Unit tests for TwoAngleInterpolation class.

This module contains tests specifically for angle interpolation functionality,
including shortest path calculation and angle normalization.
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import unittest
import numpy as np
from two_point_interpolation import TwoAngleInterpolation, normalize_angle


class TwoAngleInterpolationTest(unittest.TestCase):
    """Test cases for TwoAngleInterpolation class."""

    def _verify_shortest_path(self, p0_deg, pe_deg, expected_p0_normalized_deg,
                             expected_pe_normalized_deg, expected_displacement_deg,
                             expected_midpoint_range_deg):
        """
        Helper function to verify shortest path interpolation.

        Args:
            p0_deg: Initial angle in degrees
            pe_deg: Final angle in degrees
            expected_p0_normalized_deg: Expected normalized start angle in degrees
            expected_pe_normalized_deg: Expected normalized end angle in degrees
            expected_displacement_deg: Expected angular displacement in degrees
            expected_midpoint_range_deg: Tuple of (min, max) expected midpoint range in degrees
        """
        interp = TwoAngleInterpolation()

        # Convert degrees to radians
        p0_rad = p0_deg * np.pi / 180.0
        pe_rad = pe_deg * np.pi / 180.0

        # Initialize and calculate trajectory
        interp.init(p0=p0_rad, pe=pe_rad, acc_max=1.0, vmax=10.0, dec_max=1.0)
        t_total = interp.calc_trajectory()

        # Check start and end points
        p0_result, v0, a0 = interp.get_point(0.0)
        pe_result, ve, ae = interp.get_point(t_total)

        # Verify normalized start angle
        expected_p0_rad = expected_p0_normalized_deg * np.pi / 180.0
        self.assertAlmostEqual(p0_result, expected_p0_rad, places=6,
                              msg=f"Start should be {expected_p0_normalized_deg}°, got {p0_result*180/np.pi}°")

        # Verify normalized end angle
        expected_pe_rad = expected_pe_normalized_deg * np.pi / 180.0
        self.assertAlmostEqual(pe_result, expected_pe_rad, places=6,
                              msg=f"End should be {expected_pe_normalized_deg}°, got {pe_result*180/np.pi}°")

        # Check midpoint to verify shortest path
        t_mid = t_total / 2.0
        p_mid, v_mid, a_mid = interp.get_point(t_mid, normalize_output=True)

        # Verify midpoint is in expected range
        min_deg, max_deg = expected_midpoint_range_deg
        min_rad = min_deg * np.pi / 180.0
        max_rad = max_deg * np.pi / 180.0
        self.assertGreaterEqual(p_mid, min_rad - 0.1,
                               msg=f"Midpoint {p_mid*180/np.pi:.1f}° is below expected range [{min_deg}°, {max_deg}°]")
        self.assertLessEqual(p_mid, max_rad + 0.1,
                            msg=f"Midpoint {p_mid*180/np.pi:.1f}° is above expected range [{min_deg}°, {max_deg}°]")

        # Verify total angular displacement
        angular_displacement = pe_result - p0_result
        expected_displacement_rad = expected_displacement_deg * np.pi / 180.0
        self.assertAlmostEqual(angular_displacement, expected_displacement_rad, places=6,
                              msg=f"Displacement should be {expected_displacement_deg}°, got {angular_displacement*180/np.pi}°")

    def test_normalize_angle_function(self):
        """Test normalize_angle function."""
        # Test basic cases
        self.assertAlmostEqual(normalize_angle(0.0), 0.0, places=10)
        self.assertAlmostEqual(normalize_angle(np.pi), np.pi, places=10)

        # Test wrapping
        self.assertAlmostEqual(normalize_angle(2 * np.pi), 0.0, places=10)
        self.assertAlmostEqual(normalize_angle(-2 * np.pi), 0.0, places=10)

        # Test specific angles
        angle_350 = 350.0 * np.pi / 180.0
        angle_10 = 10.0 * np.pi / 180.0
        self.assertAlmostEqual(normalize_angle(angle_350), -10.0 * np.pi / 180.0, places=6)
        self.assertAlmostEqual(normalize_angle(angle_10), 10.0 * np.pi / 180.0, places=6)

    def test_shortest_path_350_to_10(self):
        """Test shortest path from 350° to 10° (should go 20° CCW through 0°)."""
        self._verify_shortest_path(
            p0_deg=350.0,
            pe_deg=10.0,
            expected_p0_normalized_deg=-10.0,
            expected_pe_normalized_deg=10.0,
            expected_displacement_deg=20.0,
            expected_midpoint_range_deg=(-10.0, 10.0)  # Should pass through ~0°
        )

    def test_shortest_path_10_to_350(self):
        """Test shortest path from 10° to 350° (should go 20° CW through 0°)."""
        self._verify_shortest_path(
            p0_deg=10.0,
            pe_deg=350.0,
            expected_p0_normalized_deg=10.0,
            expected_pe_normalized_deg=-10.0,
            expected_displacement_deg=-20.0,
            expected_midpoint_range_deg=(-10.0, 10.0)  # Should pass through ~0°
        )

    def test_shortest_path_0_to_270(self):
        """Test shortest path from 0° to 270° (should go 90° CW through -45°)."""
        self._verify_shortest_path(
            p0_deg=0.0,
            pe_deg=270.0,
            expected_p0_normalized_deg=0.0,
            expected_pe_normalized_deg=-90.0,
            expected_displacement_deg=-90.0,
            expected_midpoint_range_deg=(-90.0, 0.0)  # Should pass through ~-45°
        )

    def test_no_movement_360(self):
        """Test that 360° difference results in no movement (shortest path is 0°)."""
        interp = TwoAngleInterpolation()

        interp.init(p0=0.0, pe=2*np.pi, acc_max=1.0, vmax=10.0, dec_max=1.0)
        t_total = interp.calc_trajectory()

        p0, v0, a0 = interp.get_point(0.0)
        pe, ve, ae = interp.get_point(t_total)

        # Both should be 0° (no movement)
        self.assertAlmostEqual(p0, 0.0, places=6)
        self.assertAlmostEqual(pe, 0.0, places=6)

        # Total time should be 0 (or very small)
        self.assertLess(t_total, 0.01, msg="Total time should be near zero for no movement")

    def test_normalize_parameter(self):
        """Test that normalize_output parameter works correctly in get_point."""
        interp = TwoAngleInterpolation()

        # Setup a trajectory that crosses boundaries
        angle_350 = 350.0 * np.pi / 180.0
        angle_10 = 10.0 * np.pi / 180.0

        interp.init(p0=angle_350, pe=angle_10, acc_max=1.0, vmax=10.0, dec_max=1.0)
        t_total = interp.calc_trajectory()

        # Get point at middle of trajectory with normalization
        t_mid = t_total / 2.0
        p_normalized, v, a = interp.get_point(t_mid, normalize_output=True)

        # Angle should be in [-π, π] range
        self.assertGreaterEqual(p_normalized, -np.pi)
        self.assertLessEqual(p_normalized, np.pi)

        # Get same point without normalization
        p_raw, v2, a2 = interp.get_point(t_mid, normalize_output=False)

        # Velocity and acceleration should be identical
        self.assertAlmostEqual(v, v2, places=10)
        self.assertAlmostEqual(a, a2, places=10)

        # Raw angle might be outside [-π, π], but should match when normalized
        self.assertAlmostEqual(normalize_angle(p_raw), p_normalized, places=10)

    def test_getter_methods_with_angle_interpolation(self):
        """Test that getter methods work correctly with angle interpolation."""
        interp = TwoAngleInterpolation()
        t0 = 2.0

        interp.set_initial(t0=t0, p0=0.0, v0=0.0)
        interp.set_point(pe=3.14, ve=0.0)
        interp.set_constraints(vmax=2.0, acc_max=1.0)

        duration = interp.calc_trajectory()

        # Test both getters
        self.assertAlmostEqual(interp.get_duration(), duration, places=10)
        self.assertAlmostEqual(interp.get_end_time(), t0 + duration, places=10)

    def test_large_angle_wrapping(self):
        """Test angle wrapping with large angles (multiple rotations)."""
        interp = TwoAngleInterpolation()

        # 3π should wrap to π
        p0 = 3 * np.pi
        pe = 0.0

        interp.init(p0=p0, pe=pe, acc_max=1.0, vmax=10.0)
        t_total = interp.calc_trajectory()

        p0_result, _, _ = interp.get_point(0.0)
        pe_result, _, _ = interp.get_point(t_total)

        # Start should be normalized to π (not 3π)
        self.assertAlmostEqual(p0_result, np.pi, places=6)
        # End should be 0
        self.assertAlmostEqual(pe_result, 0.0, places=6)

    def test_negative_angle_handling(self):
        """Test handling of negative angles."""
        interp = TwoAngleInterpolation()

        # -π/2 to π/2 should be 180° rotation
        p0 = -np.pi / 2
        pe = np.pi / 2

        interp.init(p0=p0, pe=pe, acc_max=1.0, vmax=10.0)
        t_total = interp.calc_trajectory()

        p0_result, _, _ = interp.get_point(0.0)
        pe_result, _, _ = interp.get_point(t_total)

        self.assertAlmostEqual(p0_result, -np.pi / 2, places=6)
        self.assertAlmostEqual(pe_result, np.pi / 2, places=6)


if __name__ == '__main__':
    unittest.main()
