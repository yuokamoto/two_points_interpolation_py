"""Two-Point Interpolation Library.

This library provides trajectory generation with constant acceleration and constant jerk.
"""

__version__ = "0.1.0"

# Constant acceleration interpolation
from .constant_acc import TwoPointInterpolation as TwoPointInterpolationAcc
from .constant_acc import p_integ, v_integ

# Constant jerk interpolation
from .constant_jerk import TwoPointInterpolation as TwoPointInterpolationJerk

# Default to constant acceleration for backward compatibility
TwoPointInterpolation = TwoPointInterpolationAcc

__all__ = [
    # Constant acceleration
    "TwoPointInterpolation",  # Default (same as TwoPointInterpolationAcc)
    "TwoPointInterpolationAcc",
    "p_integ",
    "v_integ",
    # Constant jerk
    "TwoPointInterpolationJerk",
]
