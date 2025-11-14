#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import unittest
import numpy as np
from two_point_interpolation import TwoPointInterpolation, v_integ, p_integ


class TestConstantAccInterpolation(unittest.TestCase):
    """Test cases for constant acceleration interpolation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.interp = TwoPointInterpolation()
    
    def assert_final_state(self, tpi, total_time, expected_pe, expected_ve):
        """Helper function to check final position and velocity.
        
        Args:
            tpi: TwoPointInterpolation instance
            total_time: Total trajectory time
            expected_pe: Expected final position
            expected_ve: Expected final velocity
        """
        p_final, v_final, a_final = tpi.get_point(total_time)
        self.assertAlmostEqual(expected_pe, p_final, places=5)
        self.assertAlmostEqual(expected_ve, v_final, places=5)
        self.assertAlmostEqual(0.0, a_final, places=5)
    
    def assert_boundary_continuity(self, tpi, eps=1e-6):
        """Helper function to check continuity at all phase boundaries.
        
        Args:
            tpi: TwoPointInterpolation instance
            eps: Small time delta for checking continuity
        """
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
    
    def test_init_method(self):
        """Test the init method with valid parameters."""
        p0, pe = 10.0, 50.0
        acc_max, vmax = 2.0, 10.0
        t0, v0, ve = 0.0, 0.0, 0.0
        
        self.interp.init(p0, pe, acc_max, vmax, t0, v0, ve)
        
        self.assertEqual(p0, self.interp.p0)
        self.assertEqual(pe, self.interp.pe)
        self.assertEqual(acc_max, self.interp.amax_accel)
        self.assertEqual(acc_max, self.interp.amax_decel)  # Should default to acc_max
        self.assertEqual(vmax, self.interp.vmax)
        self.assertTrue(self.interp.point_setted)
        self.assertTrue(self.interp.constraints_setted)
        self.assertTrue(self.interp.initial_state_setted)
    
    def test_calc_trajectory_without_setup(self):
        """Test that calc_trajectory raises error when not properly initialized."""
        with self.assertRaises(ValueError):
            self.interp.calc_trajectory()
    
    def test_get_point_boundary_conditions(self):
        """Test get_point at boundary conditions."""
        self.interp.init(5.0, 15.0, 1.0, 10.0, 1.0, 2.0, 1.0)
        te = self.interp.calc_trajectory()
        
        # Test before start time
        p, v, a = self.interp.get_point(0.0)
        self.assertEqual(5.0, p)
        self.assertEqual(2.0, v)
        
        # Test after end time
        p, v, a = self.interp.get_point(1.0 + te + 1.0)
        self.assertEqual(15.0, p)
        self.assertEqual(1.0, v)
    
    def test_zero_displacement_different_velocity(self):
        """Test case where start and end positions are the same but velocities differ."""
        with self.assertRaises(ValueError) as context:
            self.interp.init(10.0, 10.0, 2.0, 5.0, 0.0, 1.0, 3.0)
            self.interp.calc_trajectory()
        
        self.assertIn("is not supported", str(context.exception))
    
    def test_case0(self):
        """Test Case 0: vmax not reached with multiple parameter combinations"""
        test_cases = [
            # (p0, pe, acc_max, dec_max, vmax, v0, ve, description)
            # Small displacement with high vmax ensures Case 0
            (0, 10, 2.0, 3.0, 20.0, 0, 0, "forward, zero v0/ve, asymmetric acc/dec"),
            (10, 0, 2.0, 3.0, 20.0, 0, 0, "backward, zero v0/ve, asymmetric acc/dec"),
            (0, 5, 1.5, 2.5, 15.0, 0, 0, "forward, zero v0/ve, different acc/dec"),
            (0, 8, 2.0, 2.0, 25.0, 0, 0, "forward, zero v0/ve, symmetric acc/dec"),
            (5, 15, 3.0, 4.0, 30.0, 0, 0, "forward, zero v0/ve, non-zero start position"),
            (20, 8, 2.5, 3.5, 28.0, 0, 0, "backward, zero v0/ve, non-zero positions"),
            # Non-zero v0 cases
            (0, 8, 2.0, 3.0, 20.0, 1.0, 0, "forward, non-zero v0"),
            (10, 2, 2.0, 3.0, 20.0, 0.5, 0, "backward, non-zero v0"),
            # Non-zero ve cases
            (0, 6, 2.0, 3.0, 18.0, 0, 0.5, "forward, non-zero ve"),
            (12, 4, 2.0, 3.0, 20.0, 0, 0.3, "backward, non-zero ve"),
            # Non-zero v0 and ve cases
            (0, 5, 2.0, 3.0, 18.0, 0.8, 0.4, "forward, non-zero v0 and ve"),
            (10, 5, 2.5, 3.5, 22.0, 0.6, 0.3, "backward, non-zero v0 and ve"),
        ]
        
        for p0, pe, acc_max, dec_max, vmax, v0, ve, description in test_cases:
            with self.subTest(description=description):
                tpi = TwoPointInterpolation()
                tpi.init(p0=p0, pe=pe, acc_max=acc_max, dec_max=dec_max, vmax=vmax, 
                        v0=v0, ve=ve)
                total_time = tpi.calc_trajectory()
                
                self.assertEqual(0, tpi.case, f"Expected Case 0 for {description}")
                self.assertGreater(total_time, 0)
                
                # Physical validity: total_time should be at least dp/vmax
                dp = abs(pe - p0)
                self.assertGreaterEqual(total_time, dp / vmax)
                
                # Check final state
                self.assert_final_state(tpi, total_time, expected_pe=pe, expected_ve=ve)
                
                # Check boundary continuity at all phase transitions
                self.assert_boundary_continuity(tpi)
    
    def test_case1(self):
        """Test Case 1: vmax reached with multiple parameter combinations"""
        test_cases = [
            # (p0, pe, acc_max, dec_max, vmax, v0, ve, description)
            # Large displacement with low vmax ensures Case 1
            (0, 50, 2.0, 4.0, 8.0, 0, 0, "forward, zero v0/ve, asymmetric acc/dec"),
            (50, 0, 2.0, 4.0, 8.0, 0, 0, "backward, zero v0/ve, asymmetric acc/dec"),
            (0, 60, 3.0, 3.0, 10.0, 0, 0, "forward, zero v0/ve, symmetric acc/dec"),
            (0, 80, 2.5, 5.0, 12.0, 0, 0, "forward, zero v0/ve, faster deceleration"),
            (0, 100, 4.0, 2.5, 12.0, 0, 0, "forward, zero v0/ve, faster acceleration"),
            (10, 90, 3.0, 4.5, 10.0, 0, 0, "forward, zero v0/ve, non-zero start position"),
            (100, 20, 2.8, 3.8, 9.0, 0, 0, "backward, zero v0/ve, non-zero positions"),
            # Non-zero v0 cases
            (0, 55, 2.0, 4.0, 8.0, 1.5, 0, "forward, non-zero v0"),
            (60, 0, 2.5, 4.0, 9.0, 1.0, 0, "backward, non-zero v0"),
            # Non-zero ve cases
            (0, 52, 2.0, 4.0, 8.5, 0, 1.2, "forward, non-zero ve"),
            (55, 0, 2.5, 4.0, 9.0, 0, 0.8, "backward, non-zero ve"),
            # Non-zero v0 and ve cases
            (0, 58, 2.0, 4.0, 9.0, 1.8, 1.0, "forward, non-zero v0 and ve"),
            (70, 10, 2.5, 3.5, 10.0, 1.5, 0.8, "backward, non-zero v0 and ve"),
        ]
        
        for p0, pe, acc_max, dec_max, vmax, v0, ve, description in test_cases:
            with self.subTest(description=description):
                tpi = TwoPointInterpolation()
                tpi.init(p0=p0, pe=pe, acc_max=acc_max, dec_max=dec_max, vmax=vmax, 
                        v0=v0, ve=ve)
                total_time = tpi.calc_trajectory()
                
                self.assertEqual(1, tpi.case, f"Expected Case 1 for {description}")
                self.assertGreater(total_time, 0)
                
                # Physical validity: total_time should be at least dp/vmax
                dp = abs(pe - p0)
                self.assertGreaterEqual(total_time, dp / vmax)
                
                # Check final state
                self.assert_final_state(tpi, total_time, expected_pe=pe, expected_ve=ve)
                
                # Check boundary continuity at all phase transitions
                self.assert_boundary_continuity(tpi)
    
    def test_faster_deceleration(self):
        """Test that faster deceleration reduces total time"""
        # Same acceleration, different deceleration
        tpi1 = TwoPointInterpolation()
        tpi1.init(p0=0, pe=30, acc_max=2.0, dec_max=2.0, vmax=10.0)
        time1 = tpi1.calc_trajectory()
        
        tpi2 = TwoPointInterpolation()
        tpi2.init(p0=0, pe=30, acc_max=2.0, dec_max=4.0, vmax=10.0)
        time2 = tpi2.calc_trajectory()
        
        # Faster deceleration should result in shorter time
        self.assertLess(time2, time1)
    
    def test_zero_displacement(self):
        """Test dp=0, dv=0 case"""
        tpi = TwoPointInterpolation()
        tpi.init(p0=5, pe=5, acc_max=2.0, dec_max=3.0, vmax=10.0, 
                 v0=0, ve=0)
        total_time = tpi.calc_trajectory()
        
        self.assertEqual(-1, tpi.case)
        self.assertEqual(0.0, total_time)
        
        # Should stay at same position
        p, v, a = tpi.get_point(0)
        self.assertEqual(5.0, p)
        self.assertEqual(0.0, v)
        self.assertEqual(0.0, a)
    
    def test_invalid_dp0_with_dv_nonzero(self):
        """Test that dp=0 with dv!=0 raises error"""
        tpi = TwoPointInterpolation()
        tpi.init(p0=5, pe=5, acc_max=2.0, dec_max=3.0, vmax=10.0, 
                 v0=0, ve=1.0)
        
        with self.assertRaises(ValueError):
            tpi.calc_trajectory()
    
    def test_constraints_validation(self):
        """Test constraint validation"""
        tpi = TwoPointInterpolation()
        
        # Negative acceleration
        with self.assertRaises(ValueError):
            tpi.set_constraints(acc_max=-1.0, vmax=2.0, dec_max=5.0)
        
        # Negative vmax
        with self.assertRaises(ValueError):
            tpi.set_constraints(acc_max=2.0, vmax=-1.0, dec_max=5.0)
        
        # Negative deceleration
        with self.assertRaises(ValueError):
            tpi.set_constraints(acc_max=2.0, vmax=2.0, dec_max=-5.0)
        
        # Zero values
        with self.assertRaises(ValueError):
            tpi.set_constraints(acc_max=0.0, vmax=2.0, dec_max=5.0)
    
    def test_default_dec_max(self):
        """Test that dec_max defaults to acc_max when not specified"""
        # Without dec_max
        tpi1 = TwoPointInterpolation()
        tpi1.init(p0=0, pe=20, acc_max=2.0, vmax=10.0, v0=0, ve=0)
        time1 = tpi1.calc_trajectory()
        
        # With explicit dec_max equal to acc_max
        tpi2 = TwoPointInterpolation()
        tpi2.init(p0=0, pe=20, acc_max=2.0, dec_max=2.0, vmax=10.0, 
                  v0=0, ve=0)
        time2 = tpi2.calc_trajectory()
        
        # Times should be approximately equal
        self.assertAlmostEqual(time1, time2, places=4)
        
        # Sample points should match at key times
        for t in [0, time1 * 0.25, time1 * 0.5, time1 * 0.75, time1]:
            p1, v1, a1 = tpi1.get_point(t)
            p2, v2, a2 = tpi2.get_point(t)
            
            self.assertAlmostEqual(p1, p2, places=3)
            self.assertAlmostEqual(v1, v2, places=3)


class TestIntegrationFunctions(unittest.TestCase):
    """Test integration utility functions."""
    
    def test_v_integ(self):
        """Test velocity integration function."""
        result = v_integ(10.0, 2.0, 5.0)
        expected = 10.0 + 2.0 * 5.0
        self.assertEqual(expected, result)
    
    def test_p_integ(self):
        """Test position integration function."""
        result = p_integ(0.0, 10.0, 2.0, 5.0)
        expected = 0.0 + 10.0 * 5.0 + 0.5 * 2.0 * 5.0**2
        self.assertEqual(expected, result)


if __name__ == '__main__':
    unittest.main()
