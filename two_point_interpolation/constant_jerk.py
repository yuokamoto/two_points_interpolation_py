import numpy as np
from typing import Tuple, List, Optional, Union
"""
Two-point interpolation with constant jerk constraints.

d^3x/dt^3

    ^
    |
    |
 max|--------            ----------
    |       |            |
    |       |            |
 --------------------------------->
            | t1         |3t1
            |            |
 min        --------------


* assumption v0 = 0
* jerk is constant
"""


class TwoPointInterpolation:
    """Two-point interpolation with constant jerk constraints."""

    def __init__(self):
        """Initialize the interpolation object."""
        self.point_setted = False
        self.constraints_setted = False
        self.initial_state_setted = False
        self.trajectory_calced = False

    def set_initial_time(self, time: float) -> None:
        """Set start time."""
        self.t0 = time

    def set_initial(self, t0: float, p0: float, v0: float = 0) -> None:
        """Set initial state."""
        self.t0 = t0
        self.p0 = p0
        self.v0 = v0
        self.initial_state_setted = True

    def set_point(self, ps: float, pe: Optional[float] = None) -> None:
        """Set start point and end point, or just end point if ps is end point."""
        if pe is None:
            # Compatible with constant_acc API where only end point is set
            self.pe = ps
            self.ve = 0  # Default end velocity
        else:
            self.ps = ps
            self.pe = pe
        self.point_setted = True

    def set_constraints(self, amax: Optional[float] = None,
                        vmax: Optional[float] = None,
                        jmax: Optional[float] = None,
                        max_array: Optional[List[float]] = None) -> None:
        """Set constraints - accepts either individual parameters or array."""
        if max_array is not None:
            # Original API compatibility
            if len(max_array) != 3:
                raise ValueError("max_array must contain [vmax, amax, jmax]")
            if any(x <= 0 for x in max_array):
                raise ValueError("All constraint values must be positive")
            self.vmax = max_array[0]
            self.amax = max_array[1]
            self.jmax = max_array[2]
        else:
            # New API compatible with constant_acc
            if amax is None or vmax is None or jmax is None:
                raise ValueError("amax, vmax, and jmax must all be provided")
            if amax <= 0 or vmax <= 0 or jmax <= 0:
                raise ValueError("All constraint values must be positive")
            self.amax = amax
            self.vmax = vmax
            self.jmax = jmax
        self.constraints_setted = True

    def set(self, ps: float, pe: float, max_constraints: List[float]) -> None:
        """Set point and constraints."""
        self.set_point(ps, pe)
        self.set_constraints(max_array=max_constraints)

    def init(self, p0: float, pe: float, amax: float, vmax: float, jmax: float,
             t0: float = 0, v0: float = 0, ve: float = 0) -> None:
        """Set point and constraints - compatible with constant_acc API."""
        self.set_initial(t0, p0, v0)
        self.set_point(pe)
        self.ve = ve
        self.set_constraints(amax, vmax, jmax)

    def calc_trajectory(self) -> float:
        """Calculate trajectory."""
        if not self.point_setted:
            raise ValueError("End point not set. Call set_point() first.")
        if not self.constraints_setted:
            raise ValueError("Constraints not set. Call set_constraints() first.")

        # Handle case where initial state is set vs using ps
        if hasattr(self, 'p0'):
            ps = self.p0
        else:
            ps = self.ps

        # Check if start and end positions are the same
        dp = self.pe - ps
        if dp == 0:
            # Handle case where no movement is needed
            if hasattr(self, 've') and hasattr(self, 'v0') and self.ve != self.v0:
                raise ValueError(
                    "Cannot have different velocities at the same position (dp=0, but dv!=0)")
            elif not hasattr(self, 've') or not hasattr(self, 'v0'):
                # Default case - no movement needed
                pass

            self.case = -1  # Special case for no movement
            self.te = 0.0
            self.t1 = 0.0  # Initialize t1 for consistency
            self.trajectory_calced = True
            return 0.0

        self.t1 = np.power(np.fabs(dp)/2.0/self.jmax, 1/3.0)
        self.te = 0.0

        if self.t1*self.jmax < self.amax:  # not hit acc limit
            if self.t1*self.jmax*self.t1 < self.vmax:  # not hit v limit
                self.case = 0
                self.te = 4*self.t1
            else:  # hit v limit
                self.case = 1
                self.t1 = np.sqrt(self.vmax/self.jmax)
                self.t2 = np.fabs(self.pe-ps)/self.vmax - 2.0*np.sqrt(self.vmax/self.jmax)
                self.te = 4*self.t1+self.t2
        else:  # hit acc limit
            self.t1 = self.amax/self.jmax
            self.t2 = - 1.5*self.t1 + \
                np.sqrt(4*np.fabs(self.pe-ps)/self.amax + 1.0/3.0*self.t1**2)/2.0
            if (self.t1+self.t2)*self.amax < self.vmax:  # not hit v limit
                self.case = 2
                self.te = 4*self.t1 + 2*self.t2
            else:  # hit v limit
                self.case = 3
                self.t1 = self.amax/self.jmax
                self.t2 = self.vmax/self.amax - self.t1
                self.t3 = np.fabs(self.pe-ps)/self.vmax - 2*self.t1 - self.t2
                self.te = 4*self.t1+2*self.t2+self.t3

        print("case", self.case, self.te)

        self.trajectory_calced = True

        return self.te

    def get_point(self, t: float) -> Tuple[float, float, float, float]:
        """Get pos, vel, acc, jerk depends on time t."""
        j = 0
        a = 0
        v = 0
        p = 0

        # Handle case where initial state is set vs using ps
        if hasattr(self, 'p0'):
            ps = self.p0
        else:
            ps = self.ps

        tau = t-self.t0
        jmax = self.jmax
        amax = self.amax
        vmax = self.vmax
        t1 = self.t1

        # Handle special case where no movement is needed (dp=0)
        if hasattr(self, 'case') and self.case == -1:
            j = 0.0
            a = 0.0
            if hasattr(self, 'v0'):
                v = self.v0
            else:
                v = 0.0
            p = ps
            return p, v, a, j

        # Ensure t1 is available for other cases
        if not hasattr(self, 't1'):
            raise ValueError("Trajectory not calculated. Call calc_trajectory() first.")

        if self.case == 0:
            if tau < 0:
                j = jmax
                if self.pe < ps:
                    j = -j
                a = 0.0
                v = 0.0
                p = ps
            elif tau < 4*t1:
                if tau < t1:
                    j = jmax
                    a = jmax*tau
                    v = 0.5*jmax*tau**2
                    p = 1.0/6.0*jmax*tau**3

                elif tau < 3*t1:
                    tau1 = tau - t1
                    j = -jmax
                    a = -jmax*tau1 + jmax*t1
                    v = -0.5*jmax*tau1**2 + jmax*t1*tau1 + 0.5*jmax*t1**2
                    p = -1.0/6.0*jmax*tau1**3 + 0.5*jmax*t1*tau1**2 + 0.5*jmax*t1**2*tau1 + 1.0/6.0*jmax*t1**3

                else:  # tau<4*t1:
                    tau3 = tau - 3*t1
                    j = jmax
                    a = jmax*tau3 - jmax*t1
                    v = 0.5*jmax*tau3**2 - jmax*t1*tau3 + 0.5*jmax*t1**2
                    p = 1.0/6.0*jmax*tau3**3 - 0.5*jmax*t1*tau3**2 + 0.5*jmax*t1**2*tau3 + 11.0/6.0*jmax*t1**3

                if self.pe < ps:
                    j = -j
                    a = -a
                    v = -v
                    p = -p

                p += ps

            else:  # tau > 4*t1:
                j = jmax
                if self.pe < ps:
                    j = -j
                a = 0.0
                v = 0.0
                p = self.pe

        elif self.case == 1:
            t2 = self.t2
            if tau < 0:
                j = jmax
                if self.pe < ps:
                    j = -j
                a = 0.0
                v = 0.0
                p = ps
            elif tau < 4*t1+t2:
                if tau < t1:
                    j = jmax
                    a = jmax*tau
                    v = 0.5*jmax*tau**2
                    p = 1.0/6.0*jmax*tau**3

                elif tau < 2*t1:
                    tau1 = tau - t1
                    j = -jmax
                    a = -jmax*tau1+jmax*t1
                    v = -0.5*jmax*tau1**2 + jmax*t1*tau1 + 0.5*jmax*t1**2
                    p = -1.0/6.0*jmax*tau1**3 + 0.5*jmax*t1*tau1**2 + 0.5*jmax*t1**2*tau1 + 1.0/6.0*jmax*t1**3

                elif tau < 2*t1+t2:
                    tau2 = tau - 2*t1
                    j = 0
                    a = 0
                    v = vmax
                    p = vmax*tau2+jmax*t1**3

                elif tau < 3*t1+t2:
                    tau3 = tau - 2*t1 - t2
                    j = -jmax
                    a = -jmax*tau3
                    v = -0.5*jmax*tau3**2+vmax
                    p = -1.0/6.0*jmax*tau3**3 + vmax*tau3 + vmax*t2 + jmax*t1**3

                else:
                    tau4 = tau - 3*t1 - t2
                    j = jmax
                    a = jmax*tau4 - jmax*t1
                    v = 0.5*jmax*tau4**2 - jmax*t1*tau4 - 0.5*jmax*t1**2+vmax
                    p = 1.0/6.0*jmax*tau4**3 - 0.5*jmax*t1*tau4**2 + 0.5 * \
                        jmax*t1**2*tau4 + vmax*t2 + vmax*t1 + 5.0/6.0*jmax*t1**3

                if self.pe < ps:
                    j = -j
                    a = -a
                    v = -v
                    p = -p

                p += ps

            else:  # tau > 4*t1:
                j = jmax
                if self.pe < ps:
                    j = -j
                a = 0.0
                v = 0.0
                p = self.pe

        elif self.case == 2:
            t2 = self.t2
            if tau < 0:
                j = jmax
                if self.pe < ps:
                    j = -j
                a = 0.0
                v = 0.0
                p = ps
            elif tau < 4*t1+2*t2:
                if tau < t1:
                    j = jmax
                    a = jmax*tau
                    v = 0.5*jmax*tau**2
                    p = 1.0/6.0*jmax*tau**3

                elif tau < t1+t2:
                    tau1 = tau - t1
                    j = 0
                    a = amax
                    v = amax*tau1 + 0.5*amax**2/jmax
                    p = 0.5*amax*tau1**2 + 0.5*amax**2/jmax*tau1 + 1.0/6.0*amax**3/jmax**2

                elif tau < 3*t1+t2:
                    tau2 = tau - t1 - t2
                    j = -jmax
                    a = -jmax*tau2 + amax
                    v = -0.5*jmax*tau2**2 + amax*tau2 + amax*t2 + 0.5*amax**2/jmax
                    p = -1.0/6.0*jmax*tau2**3 + 0.5*amax*tau2**2 + \
                        (amax*t2+0.5*amax**2/jmax)*tau2 + 0.5*amax*t2**2 + \
                        0.5*amax**2/jmax*t2 + 1.0/6.0*amax**3/jmax**2

                elif tau < 3*t1+2*t2:
                    tau3 = tau - 3*t1 - t2
                    j = 0
                    a = -amax
                    v = -amax*tau3 + amax*t2 + 0.5*amax**2/jmax
                    p = -0.5*amax*tau3**2 + (amax*t2+0.5*amax**2/jmax)*tau3 + \
                        0.5*amax*t2**2 + 2.5*amax**2/jmax*t2 + 2.0*amax**3/jmax**2

                else:
                    tau4 = tau - 3*t1 - 2*t2
                    j = jmax
                    a = jmax*tau4 - amax
                    v = 0.5*jmax*tau4**2 - amax*tau4 + 0.5*amax**2/jmax
                    p = 1.0/6.0*jmax*tau4**3 - 0.5*amax*tau4**2 + 0.5*amax**2/jmax * \
                        tau4 + amax*t2**2 + 3*amax**2/jmax*t2 + 2.0*amax**3/jmax**2

                if self.pe < ps:
                    j = -j
                    a = -a
                    v = -v
                    p = -p

                p += ps

            else:  # tau > 4*t1:
                j = jmax
                if self.pe < ps:
                    j = -j
                a = 0.0
                v = 0.0
                p = self.pe

        elif self.case == 3:
            t2 = self.t2
            t3 = self.t3
            if tau < 0:
                j = jmax
                if self.pe < ps:
                    j = -j
                a = 0.0
                v = 0.0
                p = ps
            elif tau < 4*t1+2*2*t2+t3:
                if tau < t1:
                    j = jmax
                    a = jmax*tau
                    v = 0.5*jmax*tau**2
                    p = 1.0/6.0*jmax*tau**3

                elif tau < t1+t2:
                    tau1 = tau - t1
                    j = 0
                    a = amax
                    v = amax*tau1 + 0.5*amax**2/jmax
                    p = 0.5*amax*tau1**2 + 0.5*amax**2/jmax*tau1 + 1.0/6.0*amax**3/jmax**2

                elif tau < 2*t1+t2:
                    tau2 = tau - t1 - t2
                    j = -jmax
                    a = -jmax*tau2 + amax
                    v = -0.5*jmax*tau2**2 + amax*tau2 + amax*t2 + 0.5*amax**2/jmax
                    p = -1.0/6.0*jmax*tau2**3 + 0.5*amax*tau2**2 + \
                        (amax*t2+0.5*amax**2/jmax)*tau2 + 0.5*amax*t2**2 + \
                        0.5*amax**2/jmax*t2 + 1.0/6.0*amax**3/jmax**2

                elif tau < 2*t1+t2+t3:
                    tau3 = tau - 2*t1 - t2
                    j = 0
                    a = 0
                    v = vmax
                    p = vmax*tau3 + vmax*t1 + 0.5*vmax*t2

                elif tau < 3*t1+t2+t3:
                    tau4 = tau - 2*t1 - t2 - t3
                    j = -jmax
                    a = -jmax*tau4
                    v = -0.5*jmax*tau4**2 + vmax
                    p = -1.0/6.0*jmax*tau4**3 + vmax*tau4 + vmax*(t1+0.5*t2+t3)

                elif tau < 3*t1+2*t2+t3:
                    tau5 = tau - 3*t1 - t2 - t3
                    j = 0
                    a = -amax
                    v = -amax*tau5 - 0.5*amax**2/jmax + vmax
                    p = -0.5*amax*tau5**2 + (vmax-0.5*amax**2/jmax)*tau5 + \
                        vmax*(2.0*t1+0.5*t2+t3) - 1.0/6.0*amax**3/jmax**2

                else:
                    tau6 = tau - 3*t1 - 2*t2 - t3
                    j = jmax
                    a = jmax*tau6 - amax
                    v = 0.5*jmax*tau6**2 - amax*tau6 + 0.5*amax**2/jmax
                    p = 1.0/6.0*jmax*tau6**3 - 0.5*amax*tau6**2 + 0.5*amax**2 / \
                        jmax*tau6 + vmax*(2.0*t1+t2+t3) - 1.0/6.0*amax**3/jmax**2

                if self.pe < ps:
                    j = -j
                    a = -a
                    v = -v
                    p = -p

                p += ps

            else:  # tau > 4*t1:
                j = jmax
                if self.pe < ps:
                    j = -j
                a = 0.0
                v = 0.0
                p = self.pe

        else:
            pass

        return p, v, a, j
