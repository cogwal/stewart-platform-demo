"""Stewart platform geometry: joint layout, inverse and forward kinematics.

Reference implementation in Python. The C master has its own implementation (master/core/sp_kin.c)
built from the same config (config/platform.yaml, via the generated header); tests cross-check both.

Pose = (x, y, z, roll, pitch, yaw) in mm and degrees, relative to the neutral pose (platform level,
centred, every leg at neutral_stroke_mm). Rotation R = Rz(yaw) * Ry(pitch) * Rx(roll), applied about
the platform frame origin. Stroke = leg length - closed length (0 = retracted end stop = home).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

CONFIG_FILE = Path(__file__).resolve().parent.parent / "config" / "platform.yaml"
AXES = ("x", "y", "z", "roll", "pitch", "yaw")

Vec = tuple[float, float, float]
Pose = tuple[float, float, float, float, float, float]


def circle_joints(radius: float, half_angle_deg: float, z: float) -> list[Vec]:
    out = []
    for i in range(6):
        sign = -1.0 if i % 2 == 0 else 1.0
        a = math.radians(120.0 * (i // 2) + sign * half_angle_deg)
        out.append((radius * math.cos(a), radius * math.sin(a), z))
    return out


def rotation(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list[list[float]]:
    r, p, y = (math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def solve(a: list[list[float]], b: list[float]) -> list[float]:
    """Gaussian elimination with partial pivoting (6x6, no numpy dependency)."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            raise ValueError("singular Jacobian")
        m[col], m[piv] = m[piv], m[col]
        for r in range(col + 1, n):
            f = m[r][col] / m[col][col]
            for c in range(col, n + 1):
                m[r][c] -= f * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


@dataclass
class Geometry:
    base: list[Vec]
    platform: list[Vec]
    closed_length_mm: float
    stroke_mm: float
    neutral_stroke_mm: float
    limits: dict[str, float]
    margin_mm: float
    soft_limits_mm: tuple[float, float]
    neutral_z_mm: float = 0.0

    def __post_init__(self):
        self.neutral_z_mm = self._neutral_height()

    # --- kinematics -------------------------------------------------------------------------
    def leg_vectors(self, pose: Pose) -> list[Vec]:
        x, y, z, roll, pitch, yaw = pose
        rm = rotation(roll, pitch, yaw)
        t = (x, y, z + self.neutral_z_mm)
        out = []
        for b, p in zip(self.base, self.platform):
            q = [t[k] + sum(rm[k][j] * p[j] for j in range(3)) for k in range(3)]
            out.append((q[0] - b[0], q[1] - b[1], q[2] - b[2]))
        return out

    def lengths(self, pose: Pose) -> list[float]:
        return [math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) for v in self.leg_vectors(pose)]

    def strokes(self, pose: Pose) -> list[float]:
        """Inverse kinematics: leg stroke positions (mm from the retracted end stop)."""
        return [length - self.closed_length_mm for length in self.lengths(pose)]

    def forward(self, strokes: list[float], guess: Pose = (0, 0, 0, 0, 0, 0), tol: float = 1e-6,
                max_iter: int = 30) -> Pose:
        """Forward kinematics by Newton-Raphson on the IK residual (numeric Jacobian)."""
        pose = list(guess)
        for _ in range(max_iter):
            res = [s - t for s, t in zip(self.strokes(tuple(pose)), strokes)]
            if max(abs(r) for r in res) < tol:
                return tuple(pose)
            jac = [[0.0] * 6 for _ in range(6)]
            for k in range(6):
                h = 1e-4
                p2 = pose[:]
                p2[k] += h
                s2 = self.strokes(tuple(p2))
                for i in range(6):
                    jac[i][k] = (s2[i] - strokes[i] - res[i]) / h
            step = solve(jac, [-r for r in res])
            pose = [p + d for p, d in zip(pose, step)]
        raise ValueError("forward kinematics did not converge")

    def _neutral_height(self) -> float:
        target = self.closed_length_mm + self.neutral_stroke_mm
        b, p = self.base[0], self.platform[0]
        dx, dy = p[0] - b[0], p[1] - b[1]
        horiz2 = dx * dx + dy * dy
        if target ** 2 <= horiz2:
            raise ValueError("legs too short for this joint layout")
        return math.sqrt(target ** 2 - horiz2) + b[2] - p[2]

    # --- workspace --------------------------------------------------------------------------
    def leg_range(self) -> tuple[float, float]:
        lo, hi = self.soft_limits_mm
        return lo + self.margin_mm, hi - self.margin_mm

    def reachable(self, pose: Pose) -> bool:
        lo, hi = self.leg_range()
        return all(lo <= s <= hi for s in self.strokes(pose))

    def clamp_axes(self, pose: Pose) -> Pose:
        out = []
        for axis, v in zip(AXES, pose):
            lim = self.limits[f"{axis}_mm" if axis in "xyz" else f"{axis}_deg"]
            out.append(min(max(v, -lim), lim))
        return tuple(out)

    def limit(self, pose: Pose, iterations: int = 20) -> Pose:
        """Clamp to the axis limits, then scale toward neutral until every leg is in range."""
        pose = self.clamp_axes(pose)
        if self.reachable(pose):
            return pose
        lo, hi = 0.0, 1.0
        for _ in range(iterations):
            mid = (lo + hi) / 2
            if self.reachable(tuple(mid * v for v in pose)):
                lo = mid
            else:
                hi = mid
        return tuple(lo * v for v in pose)


def load(path: Path = CONFIG_FILE, contract=None) -> Geometry:
    cfg = yaml.safe_load(path.read_text())
    if contract is None:
        import contract as C
        contract = C.load()
    if contract["actuator"]["stroke_mm"] != cfg["leg"]["stroke_mm"]:
        raise ValueError("platform.yaml leg.stroke_mm differs from the contract actuator stroke")
    base = [tuple(map(float, j)) for j in cfg["base_joints_mm"]] if "base_joints_mm" in cfg else \
        circle_joints(cfg["base"]["radius_mm"], cfg["base"]["half_angle_deg"], cfg["base"]["z_mm"])
    plat = [tuple(map(float, j)) for j in cfg["platform_joints_mm"]] if "platform_joints_mm" in cfg else \
        circle_joints(cfg["platform"]["radius_mm"], cfg["platform"]["half_angle_deg"], cfg["platform"]["z_mm"])
    m = cfg["motion"]
    return Geometry(
        base=base,
        platform=plat,
        closed_length_mm=float(cfg["leg"]["closed_length_mm"]),
        stroke_mm=float(cfg["leg"]["stroke_mm"]),
        neutral_stroke_mm=float(cfg["neutral_stroke_mm"]),
        limits={k: float(v) for k, v in m["limits"].items()},
        margin_mm=float(m["workspace_margin_mm"]),
        soft_limits_mm=tuple(contract["actuator"]["soft_limits_mm"]),
    )


def config(path: Path = CONFIG_FILE) -> dict:
    return yaml.safe_load(path.read_text())
