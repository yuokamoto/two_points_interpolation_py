"""Trajectories that start moving away from the target.

The peak velocity v1 of the acceleration phase satisfies

    |dp| = (v1^2 - v0^2) / (2 * amax_accel) + (v1^2 - ve^2) / (2 * amax_decel)

which fixes |v1| but not its sign. Up to 1.3.1 the solver used the equivalent
quadratic in the acceleration time t1, whose two roots correspond to v1 = +|v1|
and v1 = -|v1|. When v0 points away from pe both roots are positive, and taking
the smaller one picked v1 = -|v1|: the body was still moving away when the
deceleration phase started, so it never reached pe. get_point() clamps to pe
at te, which hid the jump; these tests therefore sample just before te.
"""

import itertools
import warnings

import numpy as np
import pytest

from two_point_interpolation import TwoPointInterpolation

EPS = 1e-9


def _build(p0, pe, v0, acc, dec, vmax, ve=0.0):
    tpi = TwoPointInterpolation()
    tpi.init(p0=p0, pe=pe, acc_max=acc, vmax=vmax, v0=v0, ve=ve, dec_max=dec)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)  # v0 > vmax warns by design
        tpi.calc_trajectory()
    return tpi


def _end_state(tpi):
    """Position and velocity just before te, where get_point() stops clamping."""
    p, v, _ = tpi.get_point(tpi.get_end_time() - EPS)
    return p, v


def _assert_reaches_target(tpi, pe, ve):
    p, v = _end_state(tpi)
    assert p == pytest.approx(pe, abs=1e-6)
    assert v == pytest.approx(ve, abs=1e-6)


def _old_dt01(p0, pe, v0, acc_max, dec_max, ve=0.0):
    """The 1.3.1 root selection, kept to compare against."""
    dp = pe - p0
    sign = dp / abs(dp)
    acc = acc_max * sign
    dec = dec_max * sign
    ratio = acc / dec
    a_coeff = 0.5 * acc * (1 + ratio)
    b_coeff = v0 * (1 + ratio)
    c_coeff = -dp + (v0**2 - ve**2) / (2 * dec)
    disc = b_coeff**2 - 4 * a_coeff * c_coeff
    if disc <= 0:
        return None
    roots = [(-b_coeff + np.sqrt(disc)) / (2 * a_coeff), (-b_coeff - np.sqrt(disc)) / (2 * a_coeff)]
    positive = [r for r in roots if r > 0]
    return min(positive) if positive else None


@pytest.mark.parametrize(
    "v0, acc, dec, vmax, ve",
    [
        (-2.0, 1.0, 1.0, 3.0, 0.0),  # symmetric, vmax not reached
        (-2.0, 1.0, 2.0, 3.0, 0.0),  # decel stronger than accel
        (-2.0, 2.0, 1.0, 3.0, 0.0),  # accel stronger than decel
        (-2.0, 1.0, 1.0, 1.5, 0.0),  # vmax reached
        (-4.0, 1.0, 1.0, 1.5, 0.0),  # |v0| above vmax, moving away
        (-2.0, 1.0, 1.0, 3.0, 0.5),  # non-zero end velocity
        (-2.0, 1.0, 2.0, 1.5, 0.5),  # asymmetric, vmax reached, non-zero end velocity
    ],
)
@pytest.mark.parametrize("direction", [1.0, -1.0])
def test_moving_away_still_reaches_the_target(v0, acc, dec, vmax, ve, direction):
    pe = 1.0 * direction
    tpi = _build(0.0, pe, v0 * direction, acc, dec, vmax, ve * direction)
    _assert_reaches_target(tpi, pe, ve * direction)


def test_the_reported_case():
    """v0 = -2 toward pe = 1 at 1 m/s^2: 1.3.1 ran off to p = -5 before te."""
    tpi = _build(0.0, 1.0, -2.0, 1.0, 1.0, 3.0)
    assert tpi.v[1] == pytest.approx(np.sqrt(3.0))  # toward pe, not away
    lowest = min(tpi.get_point(t)[0] for t in np.linspace(0.0, tpi.get_end_time(), 200))
    assert lowest == pytest.approx(-2.0, abs=1e-3)  # stops after v0^2 / (2 * acc), then returns
    _assert_reaches_target(tpi, 1.0, 0.0)


def test_toward_the_target_without_room_to_stop_still_raises():
    with pytest.raises(ValueError):
        _build(0.0, 1.0, 3.0, 1.0, 1.0, 5.0)


@pytest.mark.parametrize(
    "dp, v0, acc, dec, vmax, ve",
    [
        (0.1, 0.0, 1.0, 1.0, 5.0, 2.0),  # from rest: v^2 reaches only 0.2 in 0.1
        (0.3, -0.5, 0.5, 0.5, 1.5, 1.0),  # moving away first: v^2 reaches only 0.55
    ],
)
def test_an_end_velocity_out_of_reach_raises(dp, v0, acc, dec, vmax, ve):
    """1.3.1 returned a trajectory that missed pe for these."""
    with pytest.raises(ValueError):
        _build(0.0, dp, v0, acc, dec, vmax, ve)


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


@pytest.mark.parametrize("v0, dp, acc, dec, vmax, ve_mag", GRID)
def test_every_returned_trajectory_is_valid(v0, dp, acc, dec, vmax, ve_mag):
    ve = ve_mag * np.sign(dp)
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
def test_cases_1_3_1_got_right_are_unchanged(v0, dp, acc, dec, vmax, ve_mag):
    ve = ve_mag * np.sign(dp)
    old = _old_dt01(0.0, dp, v0, acc, dec, ve)
    if old is None:
        return
    old_v1 = v0 + acc * np.sign(dp) * old
    if np.sign(old_v1) != np.sign(dp):
        return  # the spurious root: covered by the tests above
    try:
        tpi = _build(0.0, dp, v0, acc, dec, vmax, ve)
    except ValueError:
        pytest.fail("a trajectory 1.3.1 built correctly is now refused")
    if abs(old_v1) < vmax:
        assert tpi.dt[0] == pytest.approx(old, rel=1e-9, abs=1e-12)
    else:
        assert tpi.v[1] == pytest.approx(vmax * np.sign(dp))
