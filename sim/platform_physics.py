"""Static force model of the Stewart platform for the emulator: payload + platform weight -> axial leg forces.

Equilibrium of the top plate: sum_i f_i u_i = F and sum_i r_i x (f_i u_i) = M, with u_i the unit vector of leg i
from base to platform joint, r_i the platform joint position relative to the centre of mass, F = (0, 0, m g) the
force the legs must provide (opposing gravity), M = 0 about the centre of mass. f_i > 0 = the leg pushes
(compression). Quasi-static: no inertia (the platform moves at <= 10 mm/s).
"""
from __future__ import annotations

import math

from stewart import geometry as G

GRAVITY = 9.81


def leg_forces(geom: G.Geometry, pose: G.Pose, mass_kg: float, com_above_mm: float) -> list[float]:
    x, y, z, roll, pitch, yaw = pose
    rm = G.rotation(roll, pitch, yaw)
    t = (x, y, z + geom.neutral_z_mm)
    com = tuple(t[k] + rm[k][2] * com_above_mm for k in range(3))
    cols = []
    for b, p in zip(geom.base, geom.platform):
        q = [t[k] + sum(rm[k][j] * p[j] for j in range(3)) for k in range(3)]
        v = [q[k] - b[k] for k in range(3)]
        n = math.sqrt(sum(c * c for c in v))
        u = [c / n for c in v]
        r = [(q[k] - com[k]) / 1000.0 for k in range(3)]  # m
        m = [r[1] * u[2] - r[2] * u[1], r[2] * u[0] - r[0] * u[2], r[0] * u[1] - r[1] * u[0]]
        cols.append(u + m)
    a = [[cols[i][row] for i in range(6)] for row in range(6)]
    rhs = [0.0, 0.0, mass_kg * GRAVITY, 0.0, 0.0, 0.0]
    return G.solve(a, rhs)
