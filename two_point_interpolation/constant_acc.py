from typing import List, Tuple, Optional
import numpy as np
import warnings
'''
Two-point interpolation with asymmetric acceleration/deceleration

case==0 (not hit the vmax)
d^2x/dt^2

    ^
    |
    |
 amax|----------
    |         |
    |         |
 0--------------------------------->
     t0       | t1         te
              |
-dmax          -----------


case==1 (hit vmax)
d^2x/dt^2

    ^
    |
    |
 amax|----
    |   |
    |   |
 0--------------------------------->
     t0   t1    | t2         te
                |
-dmax            --------

amax: maximum acceleration
dmax: maximum deceleration (absolute value)
'''

# Tolerance threshold for deceleration distance comparison (2%)
DECEL_DISTANCE_TOLERANCE = 0.02


def v_integ(v0: float, a: float, dt: float) -> float:
    """Integrate velocity from acceleration."""
    return v0 + a * dt


def p_integ(p0: float, v0: float, a: float, dt: float) -> float:
    """Integrate position from velocity and acceleration."""
    return p0 + v0 * dt + 0.5 * a * dt**2


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to range [-π, π].

    Args:
        angle: Input angle in radians

    Returns:
        Normalized angle in range [-π, π]
    """
    # Use atan2 to normalize: atan2(sin(angle), cos(angle)) returns angle in [-π, π]
    result: float = np.arctan2(np.sin(angle), np.cos(angle))
    return result


class TwoPointInterpolation:
    """Two-point interpolation with symmetric or asymmetric
    acceleration/deceleration constraints."""

    def __init__(self) -> None:
        self.point_setted = False
        self.constraints_setted = False
        self.initial_state_setted = False
        self.trajectory_calced = False
        # Initialize attributes for type checker
        self.t0: float = 0.0
        self.p0: float = 0.0
        self.v0: float = 0.0
        self.pe: float = 0.0
        self.ve: float = 0.0
        self.vmax: float = 0.0
        self.amax_accel: float = 0.0
        self.amax_decel: float = 0.0
        self.dt: List[float] = []
        self.a: List[float] = []
        self.v: List[float] = []
        self.p: List[float] = []
        self.case: int = 0

    def set_initial(self, t0: float, p0: float, v0: float = 0) -> None:
        '''set initial state'''
        self.t0 = t0
        self.p0 = p0
        self.v0 = v0
        self.initial_state_setted = True

    def set_point(self, pe: float, ve: float = 0) -> None:
        '''set end state'''
        self.pe = pe
        self.ve = ve
        self.point_setted = True

    def set_constraints(self, acc_max: float, vmax: float, dec_max: Optional[float] = None) -> None:
        '''set constraints with symmetric or asymmetric acceleration/deceleration

        Args:
            acc_max: maximum acceleration (positive value)
            vmax: maximum velocity (positive value)
            dec_max: maximum deceleration (positive value).
                     If None, defaults to acc_max
        '''
        if acc_max <= 0:
            raise ValueError("acc_max must be positive")
        if vmax <= 0:
            raise ValueError("vmax must be positive")

        # Backward compatibility: if dec_max not specified, use symmetric acceleration
        if dec_max is None:
            dec_max = acc_max

        if dec_max <= 0:
            raise ValueError("dec_max must be positive")

        self.amax_accel = acc_max
        self.amax_decel = dec_max
        self.vmax = vmax
        self.constraints_setted = True

    def _raise_deceleration_error(self, v0: float, ve: float, dp: float,
                                  dec: float, sign: float, context: str) -> None:
        """Check deceleration feasibility and raise appropriate error.

        Args:
            v0: initial velocity
            ve: final velocity
            dp: position displacement
            dec: deceleration value (with sign)
            sign: direction sign
            context: error context ('discriminant' or 'no_positive_solution')
        """
        # Calculate minimum distance required to decelerate from v0 to ve
        # Using kinematic equation: d = (v0² - ve²) / (2 * dec)
        decel_distance = (v0**2 - ve**2) / (2 * abs(dec))

        # Check if moving toward target (sign * v0 > 0 means velocity and direction align)
        is_moving_toward_target = sign * v0 > 0

        if (is_moving_toward_target and
                abs(decel_distance - abs(dp)) < abs(dp) * DECEL_DISTANCE_TOLERANCE):
            # Within tolerance: deceleration distance is very close to available distance
            if context == 'discriminant':
                msg_prefix = "No valid trajectory found"
            else:  # no_positive_solution
                msg_prefix = "Insufficient distance for trajectory planning"

            raise ValueError(
                f"{msg_prefix}: "
                f"current velocity {abs(v0):.4f} requires approximately "
                f"{decel_distance:.4f} distance to reach target velocity {abs(ve):.4f}, "
                f"nearly equal to available distance {abs(dp):.4f}. "
                f"This leaves no room for trajectory planning. "
                f"This typically occurs when the same goal is sent again during motion. "
                f"Consider checking if the goal has changed before recalculating trajectory."
            )
        elif is_moving_toward_target and decel_distance > abs(dp):
            # Deceleration distance exceeds available distance (more than tolerance)
            # Guard against division by zero (though dp==0 is checked earlier in calc_trajectory)
            percentage = (abs(decel_distance - abs(dp))/abs(dp)*100) if abs(dp) > 1e-10 else 0.0
            raise ValueError(
                f"Insufficient distance to decelerate: "
                f"current velocity {abs(v0):.4f} requires {decel_distance:.4f} distance "
                f"to reach target velocity {abs(ve):.4f}, but only {abs(dp):.4f} available. "
                f"Shortage: {decel_distance - abs(dp):.4f} "
                f"({percentage:.2f}%). "
                f"Consider reducing initial velocity or increasing distance."
            )
        else:
            # General case
            if context == 'discriminant':
                raise ValueError(
                    f"No valid trajectory found (discriminant <= 0). "
                    f"The constraints might be too restrictive for the given end conditions. "
                    f"Distance: {abs(dp):.4f}, v0: {abs(v0):.4f}, ve: {abs(ve):.4f}, "
                    f"acc_max: {self.amax_accel:.4f}, dec_max: {self.amax_decel:.4f}, "
                    f"vmax: {self.vmax:.4f}"
                )
            else:  # no_positive_solution
                raise ValueError(
                    f"No positive time solution found for trajectory. "
                    f"Distance: {abs(dp):.4f}, v0: {abs(v0):.4f}, ve: {abs(ve):.4f}, "
                    f"acc_max: {self.amax_accel:.4f}, dec_max: {self.amax_decel:.4f}"
                )

    def init(self, p0: float, pe: float, acc_max: float, vmax: float,
             t0: float = 0, v0: float = 0, ve: float = 0, dec_max: Optional[float] = None) -> None:
        '''set point and constraints

        Args:
            p0: initial position
            pe: final position
            acc_max: maximum acceleration
            vmax: maximum velocity
            t0: initial time (default: 0)
            v0: initial velocity (default: 0)
            ve: final velocity (default: 0)
            dec_max: maximum deceleration. If None, defaults to acc_max
        '''
        self.set_initial(t0, p0, v0)
        self.set_point(pe, ve)
        self.set_constraints(acc_max, vmax, dec_max)

    def calc_trajectory(self) -> float:
        '''calc_trajectory'''
        if not self.point_setted:
            raise ValueError("End point not set. Call set_point() first.")
        if not self.constraints_setted:
            raise ValueError("Constraints not set. Call set_constraints() first.")
        if not self.initial_state_setted:
            raise ValueError("Initial state not set. Call set_initial() first.")

        vmax = self.vmax
        v0 = self.v0
        p0 = self.p0
        t0 = self.t0
        ve = self.ve
        pe = self.pe
        dp = pe - p0
        dv = ve - v0

        # Check if start and end positions are the same
        if dp == 0:
            if dv == 0:
                # No movement needed, trajectory is already complete
                self.dt = []
                self.a = []
                self.v = [v0]
                self.p = [p0]
                self.case = -1  # Special case for no movement
                self.trajectory_calced = True
                return 0.0
            else:
                raise ValueError(
                    "Changing velocity at the same position (dp=0, dv!=0) is not supported. "
                    "This library is designed for position interpolation.")

        self.dt = []
        self.a = []
        self.v = [v0]
        self.p = [p0]

        # Direction sign
        sign = dp / np.fabs(dp)
        acc = self.amax_accel * sign
        dec = self.amax_decel * sign

        # Calculate coefficients for quadratic equation: a_coeff*t1^2 + b_coeff*t1 + c_coeff = 0
        # where t1 is the duration of acceleration phase
        ratio = acc / dec
        a_coeff = 0.5 * acc * (1 + ratio)
        b_coeff = v0 * (1 + ratio)
        c_coeff = -dp + (v0**2 - ve**2) / (2 * dec)

        # Calculate discriminant
        discriminant = b_coeff**2 - 4 * a_coeff * c_coeff

        # Check for invalid trajectory before attempting to solve
        # Early-exit pattern: Check discriminant <= 0 first for better error reporting
        # (original logic had discriminant > 0, but inverting provides clearer error messages)
        if discriminant <= 0:
            # Discriminant <= 0: no valid solution
            self._raise_deceleration_error(v0, ve, dp, dec, sign, 'discriminant')

        # Valid solution exists (discriminant > 0)
        # Solve for t1 (acceleration duration)
        # Quadratic equation has two solutions, choose the positive one
        sqrt_disc = np.sqrt(discriminant)
        dt01_plus = (-b_coeff + sqrt_disc) / (2 * a_coeff)
        dt01_minus = (-b_coeff - sqrt_disc) / (2 * a_coeff)

        # Choose the positive solution
        if dt01_plus > 0 and dt01_minus > 0:
            # Both positive: choose the smaller one (more efficient)
            dt01 = min(dt01_plus, dt01_minus)
        elif dt01_plus > 0:
            dt01 = dt01_plus
        elif dt01_minus > 0:
            dt01 = dt01_minus
        else:
            # No positive solution: check if this is due to insufficient deceleration distance
            self._raise_deceleration_error(v0, ve, dp, dec, sign, 'no_positive_solution')

        v1 = v_integ(v0, acc, dt01)

        if np.fabs(v1) < vmax:
            # Case 0: vmax not reached
            self.case = 0
            p1 = p_integ(p0, v0, acc, dt01)

            # Deceleration duration
            dt1e = np.fabs((v1 - ve) / dec)

            self.dt.append(dt01)
            self.dt.append(dt1e)
            self.a.extend([acc, -dec])
            self.v.append(v1)
            self.p.append(p1)

        else:
            # Case 1: vmax reached
            self.case = 1

            # Phase 1: Acceleration or deceleration to reach vmax
            v1 = vmax * sign

            # Check if we need to accelerate or decelerate to reach vmax
            # sign * v0 < sign * v1: need to accelerate (v0 moving slower than v1 in direction)
            # sign * v0 > sign * v1: need to decelerate (v0 moving faster than v1 in direction)
            if sign * v0 < sign * v1:
                # Normal case: accelerate to vmax
                dt01 = np.fabs((v1 - v0) / acc)
                a_phase1 = acc
                p1 = p_integ(p0, v0, acc, dt01)
            else:
                # Overspeed case: decelerate to vmax
                warnings.warn(
                    f"Initial velocity ({np.fabs(v0):.3f} m/s) exceeds vmax ({vmax:.3f} m/s). "
                    f"Trajectory will start with deceleration to reach vmax. "
                    f"Consider reducing initial velocity or increasing vmax.",
                    UserWarning,
                    stacklevel=2
                )
                dt01 = np.fabs((v1 - v0) / dec)
                a_phase1 = -dec
                p1 = p_integ(p0, v0, -dec, dt01)

            self.dt.append(dt01)
            self.a.append(a_phase1)
            self.v.append(v1)
            self.p.append(p1)

            # Phase 3: Deceleration (vmax → ve)
            v2 = v1
            dt2e = np.fabs((v2 - ve) / dec)
            dp2e = p_integ(0, v2, -dec, dt2e)

            # Phase 2: Constant velocity (vmax maintained)
            dt12 = (pe - p1 - dp2e) / v1

            # Mathematical note: dt12 should always be >= 0 in theory
            # because Case 0's solution satisfies pe exactly, and limiting v1 to vmax
            # reduces the required distance. If dt12 < 0, it indicates:
            # - Numerical error (floating-point precision limits)
            # - Implementation bug
            # - Invalid input data
            if dt12 < 0:
                raise ValueError(
                    f"Invalid trajectory: cannot reach target with given constraints. "
                    f"Distance too short ({np.fabs(dp):.3f}) for vmax ({vmax:.3f}). "
                    f"Consider reducing vmax or increasing distance."
                )

            p2 = pe - dp2e

            self.dt.append(dt12)
            self.dt.append(dt2e)
            self.a.append(0.0)
            self.a.append(-dec)
            self.v.append(v2)
            self.p.append(p2)

        self.trajectory_calced = True
        return sum(self.dt)

    def get_point(self, t: float) -> Tuple[float, float, float]:
        '''get pos, vel, acc depends on time t'''

        # output
        a: float = 0.0
        v: float = 0.0
        p: float = 0.0

        tau = t - self.t0

        # Handle special case where no movement is needed (dp=0, dv=0)
        if hasattr(self, 'case') and self.case == -1:
            a = 0.0
            v = self.v0
            p = self.p0
            return p, v, a

        if tau < 0:
            a = 0.0
            v = self.v0
            p = self.p0
        elif tau >= sum(self.dt):
            a = 0.0
            v = self.ve
            p = self.pe
        else:
            a_in: float = 0.0
            v_in: float = 0.0
            p_in: float = 0.0
            t_in: float = tau
            for i in range(len(self.dt)):
                dt = sum(self.dt[:i])
                if tau <= dt + self.dt[i]:
                    t_in = tau - dt
                    a_in = self.a[i]
                    v_in = self.v[i]
                    p_in = self.p[i]
                    break

            a = a_in
            v = v_integ(v_in, a_in, t_in)
            p = p_integ(p_in, v_in, a_in, t_in)

        return p, v, a


class TwoAngleInterpolation(TwoPointInterpolation):
    """
    Two-point interpolation specialized for angles with normalization.

    This class automatically handles angle normalization and ensures
    the shortest rotation path between two angles.

    Example:
        From 350° to 10° will rotate 20° counter-clockwise (shortest path)
        rather than 340° clockwise.

    Usage:
        interp = TwoAngleInterpolation()
        interp.init(p0=350*np.pi/180, pe=10*np.pi/180,
                    acc_max=1.0, vmax=10.0, dec_max=1.0)
        total_time = interp.calc_trajectory()
        p, v, a = interp.get_point(t, normalize_output=True)
    """

    def init(self, p0: float, pe: float, acc_max: float, vmax: float,
             t0: float = 0.0, v0: float = 0.0, ve: float = 0.0,
             dec_max: Optional[float] = None) -> None:
        """
        Initialize trajectory parameters with angle normalization.

        This method does NOT calculate the trajectory. Call calc_trajectory() separately.

        Args:
            p0: Initial angle (radians)
            pe: Final angle (radians)
            acc_max: Maximum acceleration (positive value)
            vmax: Maximum velocity (positive value)
            t0: Initial time (default: 0.0)
            v0: Initial angular velocity (rad/s, default: 0.0)
            ve: Final angular velocity (rad/s, default: 0.0)
            dec_max: Maximum deceleration (positive value). If None, defaults to acc_max
        """
        # Normalize start and end angles to [-π, π]
        p0_normalized = normalize_angle(p0)
        pe_normalized = normalize_angle(pe)

        # Calculate shortest angular difference
        dp = normalize_angle(pe_normalized - p0_normalized)

        # Set trajectory parameters using parent class methods
        self.set_initial(t0, p0_normalized, v0)
        self.set_point(p0_normalized + dp, ve)
        self.set_constraints(acc_max, vmax, dec_max)

    def get_point(self, t: float, normalize_output: bool = True) -> Tuple[float, float, float]:
        """
        Get trajectory point at time t with optional normalization.

        Args:
            t: Time value
            normalize_output: If True, output angle is normalized to [-π, π] (default: True)

        Returns:
            Tuple of (angle, angular_velocity, angular_acceleration)
        """
        # Call parent class get_point which returns (p, v, a)
        p, v, a = super().get_point(t)

        # Normalize output angle if requested
        if normalize_output:
            p = normalize_angle(p)

        return p, v, a
