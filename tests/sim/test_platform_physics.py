"""Static force model: equilibrium, symmetry, and the S20 refusing to move against an overload."""
import math

import pytest

from sim.drive.s20 import S20
from sim.platform_physics import GRAVITY, leg_forces
from stewart import geometry as G


@pytest.mark.parametrize("pose", [(0, 0, 0, 0, 0, 0), (10, -5, 12, 6, -4, 8), (-20, 15, -25, -10, 5, -12)])
def test_equilibrium(pose):
    g = G.load()
    mass = 3.0
    f = leg_forces(g, pose, mass, 20.0)
    vecs = g.leg_vectors(pose)
    fz = sum(fi * v[2] / math.sqrt(sum(c * c for c in v)) for fi, v in zip(f, vecs))
    fx = sum(fi * v[0] / math.sqrt(sum(c * c for c in v)) for fi, v in zip(f, vecs))
    assert fz == pytest.approx(mass * GRAVITY, rel=1e-6)
    assert fx == pytest.approx(0.0, abs=1e-6)
    assert all(fi > 0 for fi in f)  # a hanging payload compresses every leg in this workspace


def test_neutral_is_symmetric():
    f = leg_forces(G.load(), (0, 0, 0, 0, 0, 0), 2.0, 20.0)
    assert max(f) - min(f) < 1e-6


def test_s20_skips_steps_against_overload():
    leg = S20(position_mm=50.0, max_force_n=60.0, load_n=70.0)
    assert leg.move(0.01, 0.001) == 0.0 and leg.overloaded and leg.stalled  # extending against 70 N: refused
    assert leg.move(-0.01, 0.001) == pytest.approx(-0.01)                   # retracting with the load: fine
    leg.load_n = 50.0
    assert leg.move(0.001, 0.001) == pytest.approx(0.001)                   # 1 mm/s: 57 N available
    assert leg.move(0.01, 0.001) == 0.0                                     # 10 mm/s: 42 N available
