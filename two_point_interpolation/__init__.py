# Copyright 2025 Yu Okamoto
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Two-Point Interpolation Library.

This library provides trajectory generation with constant acceleration and constant jerk.
"""

__version__ = "1.3.0"

# Constant acceleration interpolation
from .constant_acc import TwoPointInterpolation as TwoPointInterpolationAcc
from .constant_acc import TwoAngleInterpolation
from .constant_acc import normalize_angle
from .constant_acc import p_integ, v_integ

# Constant jerk interpolation
from .constant_jerk import TwoPointInterpolation as TwoPointInterpolationJerk

# Default to constant acceleration for backward compatibility
TwoPointInterpolation = TwoPointInterpolationAcc

__all__ = [
    # Constant acceleration
    "TwoPointInterpolation",  # Default (same as TwoPointInterpolationAcc)
    "TwoPointInterpolationAcc",
    "TwoAngleInterpolation",
    "normalize_angle",
    "p_integ",
    "v_integ",
    # Constant jerk
    "TwoPointInterpolationJerk",
]
