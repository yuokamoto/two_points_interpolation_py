#!/usr/bin/env python3
"""
Unit tests for two-point interpolation modules.
"""

import unittest
import numpy as np
import two_point_interpolation_constant_acc as tpi_acc
import two_point_interpolation_constant_jerk as tpi_jerk


class TestConstantAccInterpolation(unittest.TestCase):
    """Test cases for constant acceleration interpolation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.interp = tpi_acc.TwoPointInterpolation()
    
    def test_init_method(self):
        """Test the init method with valid parameters."""
        p0, pe = 10.0, 50.0
        amax, vmax = 2.0, 10.0
        t0, v0, ve = 0.0, 0.0, 0.0
        
        self.interp.init(p0, pe, amax, vmax, t0, v0, ve)
        
        self.assertEqual(self.interp.p0, p0)
        self.assertEqual(self.interp.pe, pe)
        self.assertEqual(self.interp.amax, amax)
        self.assertEqual(self.interp.vmax, vmax)
        self.assertTrue(self.interp.point_setted)
        self.assertTrue(self.interp.constraints_setted)
        self.assertTrue(self.interp.initial_state_setted)
    
    def test_invalid_constraints(self):
        """Test that invalid constraints raise appropriate errors."""
        with self.assertRaises(ValueError):
            self.interp.set_constraints(-1.0, 10.0)  # Negative amax
        
        with self.assertRaises(ValueError):
            self.interp.set_constraints(1.0, -10.0)  # Negative vmax
        
        with self.assertRaises(ValueError):
            self.interp.set_constraints(0.0, 10.0)  # Zero amax
    
    def test_calc_trajectory_without_setup(self):
        """Test that calc_trajectory raises error when not properly initialized."""
        with self.assertRaises(ValueError):
            self.interp.calc_trajectory()
    
    def test_simple_trajectory(self):
        """Test a simple trajectory calculation."""
        self.interp.init(0.0, 10.0, 2.0, 5.0, 0.0, 0.0, 0.0)
        te = self.interp.calc_trajectory()
        
        # Check that trajectory time is positive
        self.assertGreater(te, 0)
        
        # Check end position
        p_end, v_end, a_end = self.interp.get_point(te)
        self.assertAlmostEqual(p_end, 10.0, places=3)
        self.assertAlmostEqual(v_end, 0.0, places=3)
    
    def test_get_point_boundary_conditions(self):
        """Test get_point at boundary conditions."""
        self.interp.init(5.0, 15.0, 1.0, 10.0, 1.0, 2.0, 1.0)
        te = self.interp.calc_trajectory()
        
        # Test before start time
        p, v, a = self.interp.get_point(0.0)
        self.assertEqual(p, 5.0)
        self.assertEqual(v, 2.0)
        
        # Test after end time
        p, v, a = self.interp.get_point(1.0 + te + 1.0)
        self.assertEqual(p, 15.0)
        self.assertEqual(v, 1.0)
    
    def test_zero_displacement_same_velocity(self):
        """Test case where start and end positions are the same with same velocity."""
        self.interp.init(10.0, 10.0, 2.0, 5.0, 0.0, 2.0, 2.0)
        te = self.interp.calc_trajectory()
        
        # Should return zero time for no movement
        self.assertEqual(te, 0.0)
        
        # Position should remain constant
        p, v, a = self.interp.get_point(0.0)
        self.assertEqual(p, 10.0)
        self.assertEqual(v, 2.0)
        self.assertEqual(a, 0.0)
    
    def test_zero_displacement_different_velocity(self):
        """Test case where start and end positions are the same but velocities differ."""
        with self.assertRaises(ValueError) as context:
            self.interp.init(10.0, 10.0, 2.0, 5.0, 0.0, 1.0, 3.0)
            self.interp.calc_trajectory()
        
        self.assertIn("Cannot have different velocities at the same position", str(context.exception))


class TestConstantJerkInterpolation(unittest.TestCase):
    """Test cases for constant jerk interpolation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.interp = tpi_jerk.TwoPointInterpolation()
    
    def test_original_api(self):
        """Test the original API with array constraints."""
        ps, pe = 5.0, 100.0
        max_constraints = [5.0, 1.0, 0.98]  # [vmax, amax, jmax]
        
        self.interp.set(ps, pe, max_constraints)
        self.interp.set_initial_time(0.0)
        
        te = self.interp.calc_trajectory()
        self.assertGreater(te, 0)
    
    def test_new_api_compatibility(self):
        """Test the new API compatible with constant_acc."""
        p0, pe = 5.0, 100.0
        amax, vmax, jmax = 1.0, 5.0, 0.98
        
        self.interp.init(p0, pe, amax, vmax, jmax)
        te = self.interp.calc_trajectory()
        self.assertGreater(te, 0)
    
    def test_invalid_constraints_jerk(self):
        """Test that invalid constraints raise appropriate errors."""
        with self.assertRaises(ValueError):
            self.interp.set_constraints(amax=-1.0, vmax=5.0, jmax=1.0)
        
        with self.assertRaises(ValueError):
            self.interp.set_constraints(max_array=[5.0, -1.0, 1.0])
        
        with self.assertRaises(ValueError):
            self.interp.set_constraints(max_array=[5.0, 1.0])  # Wrong length
    
    def test_zero_displacement_jerk(self):
        """Test case where start and end positions are the same for jerk interpolation."""
        p0, pe = 10.0, 10.0
        amax, vmax, jmax = 1.0, 5.0, 0.98
        
        self.interp.init(p0, pe, amax, vmax, jmax)
        te = self.interp.calc_trajectory()
        
        # Should return zero time for no movement
        self.assertEqual(te, 0.0)
        
        # Position should remain constant
        p, v, a, j = self.interp.get_point(0.0)
        self.assertEqual(p, 10.0)
        self.assertEqual(v, 0.0)
        self.assertEqual(a, 0.0)
        self.assertEqual(j, 0.0)


class TestIntegrationFunctions(unittest.TestCase):
    """Test integration utility functions."""
    
    def test_v_integ(self):
        """Test velocity integration function."""
        result = tpi_acc.v_integ(10.0, 2.0, 5.0)
        expected = 10.0 + 2.0 * 5.0
        self.assertEqual(result, expected)
    
    def test_p_integ(self):
        """Test position integration function."""
        result = tpi_acc.p_integ(0.0, 10.0, 2.0, 5.0)
        expected = 0.0 + 10.0 * 5.0 + 0.5 * 2.0 * 5.0**2
        self.assertEqual(result, expected)


if __name__ == '__main__':
    unittest.main()
