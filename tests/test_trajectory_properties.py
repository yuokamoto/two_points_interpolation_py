"""Properties every returned trajectory must have, checked over input grids.

Concrete cases live in test_cases.yaml (run by test_yaml_cases.py). This file
holds what a table of cases cannot express: sweeps over many inputs, and the
angle class, which the YAML runner does not cover.

get_point() returns exactly (pe, ve) from te on, which would hide a trajectory
that does not arrive there, so the end state is sampled just before te.
"""

import itertools
import warnings

import numpy as np
import pytest

from two_point_interpolation import TwoAngleInterpolation, TwoPointInterpolation

EPS = 1e-9


def _build(p0, pe, v0, acc, dec, vmax, ve=0.0):
    tpi = TwoPointInterpolation()
    tpi.init(p0=p0, pe=pe, acc_max=acc, vmax=vmax, v0=v0, ve=ve, dec_max=dec)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)  # v0 > vmax warns by design
        tpi.calc_trajectory()
    return tpi


def _assert_reaches_target(tpi, pe, ve):
    p, v, _ = tpi.get_point(tpi.get_end_time() - EPS)
    assert p == pytest.approx(pe, abs=1e-6)
    assert v == pytest.approx(ve, abs=1e-6)


GRID = list(
    itertools.product(
        [-3.0, -1.0, -0.2, 0.0, 0.2, 1.0, 3.0],  # v0
        [-2.0, -0.3, 0.3, 2.0],  # dp
        [0.5, 2.0],  # acc
        [0.5, 2.0],  # dec
        [1.0, 4.0],  # vmax
        [0.0, 0.3],  # |ve|, signed toward the target below
    )
)

# The end velocity may also point away from the target: the body passes pe,
# turns around in the deceleration phase and arrives back at pe moving away.
GRID_VE_SIGNED = [row + (ve_sign,) for row in GRID if row[5] > 0 for ve_sign in (1.0, -1.0)]


def _check_returned_trajectory(v0, dp, acc, dec, vmax, ve):
    """Refusing is allowed; a returned trajectory must be valid."""
    try:
        tpi = _build(0.0, dp, v0, acc, dec, vmax, ve)
    except ValueError:
        return
    _assert_reaches_target(tpi, dp, ve)
    assert all(dt >= -1e-12 for dt in tpi.dt)
    limit = max(acc, dec) + 1e-9
    assert all(abs(a) <= limit for a in tpi.a)
    speed_cap = max(vmax, abs(v0)) + 1e-9  # an initial overspeed is braked, never exceeded
    for t in np.linspace(tpi.t0, tpi.get_end_time() - EPS, 50):
        assert abs(tpi.get_point(t)[1]) <= speed_cap


@pytest.mark.parametrize("v0, dp, acc, dec, vmax, ve_mag", GRID)
def test_every_returned_trajectory_is_valid(v0, dp, acc, dec, vmax, ve_mag):
    _check_returned_trajectory(v0, dp, acc, dec, vmax, ve_mag * np.sign(dp))


@pytest.mark.parametrize("v0, dp, acc, dec, vmax, ve_mag, ve_sign", GRID_VE_SIGNED)
def test_every_returned_trajectory_is_valid_for_either_end_direction(v0, dp, acc, dec, vmax, ve_mag, ve_sign):
    _check_returned_trajectory(v0, dp, acc, dec, vmax, ve_sign * ve_mag * np.sign(dp))


@pytest.mark.parametrize("direction", [1.0, -1.0])
@pytest.mark.parametrize("v0_toward", [0.0, -1.0, 0.5])
@pytest.mark.parametrize("dp_mag, acc", [(15.0, 0.3), (1.0, 2.0), (7.3, 0.7)])
def test_an_acceleration_only_arrival_is_accepted(direction, v0_toward, dp_mag, acc):
    """ve exactly the speed reached by accelerating over the whole distance:
    the square root can land a rounding error below ve, which must not count
    as unreachable. v0_toward < 0 starts moving away from the target."""
    v0 = v0_toward * direction
    ve = direction * np.sqrt(v0_toward**2 + 2 * acc * dp_mag)
    tpi = _build(0.0, direction * dp_mag, v0, acc, 0.5, 100.0, ve)
    _assert_reaches_target(tpi, direction * dp_mag, ve)
    assert tpi.dt[-1] == pytest.approx(0.0, abs=1e-6)  # no deceleration phase


def test_an_angle_turning_away_from_its_target_still_reaches_it():
    """The angle class goes through the same solver: an angular velocity away
    from the shortest rotation to the target must still end at the target."""
    tpi = TwoAngleInterpolation()
    # 350 deg to 10 deg is +20 deg the short way; start turning at -1 rad/s.
    pe = np.deg2rad(10.0)
    tpi.init(p0=np.deg2rad(350.0), pe=pe, acc_max=1.0, vmax=3.0, v0=-1.0, dec_max=1.0)
    tpi.calc_trajectory()
    assert tpi.v[1] > 0  # the peak is toward the target
    p, v, _ = tpi.get_point(tpi.get_end_time() - EPS)
    assert p == pytest.approx(pe, abs=1e-6)
    assert v == pytest.approx(0.0, abs=1e-6)
