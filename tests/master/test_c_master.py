"""C master: generated files in sync, C unit tests, C kinematics == Python reference."""
import random
import subprocess
from pathlib import Path

import pytest

from contract import gen_c
from stewart import geometry as G

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build" / "master"


def make(target: str) -> subprocess.CompletedProcess:
    return subprocess.run(["make", "-C", str(ROOT / "master"), target], capture_output=True, text=True)


def test_generated_files_up_to_date():
    for name, text in gen_c.generate().items():
        assert (gen_c.OUT_DIR / name).read_text() == text, f"{name} is stale: run `make gen`"


def test_c_unit_tests():
    r = make("test")
    print(r.stdout[-3000:])
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]


@pytest.fixture(scope="module")
def kin_dump():
    r = make("test")
    assert (BUILD / "kin_dump").exists(), r.stderr
    return BUILD / "kin_dump"


def test_c_ik_matches_python(kin_dump):
    g = G.load()
    rng = random.Random(7)
    poses = [(0, 0, 0, 0, 0, 0), (25, 0, 0, 0, 0, 0), (0, 0, 30, 0, 0, 0), (0, 0, 0, 0, 0, 15),
             (25, 25, 30, 12, 12, 15), (-25, 10, -30, -12, 5, -15), (0, 0, 0, 40, 40, 0)]
    poses += [tuple(rng.uniform(-1, 1) * lim for lim in (30, 30, 35, 15, 15, 18)) for _ in range(200)]
    stdin = "\n".join(" ".join(f"{v:.6f}" for v in p) for p in poses) + "\n"
    out = subprocess.run([str(kin_dump)], input=stdin, capture_output=True, text=True, check=True).stdout
    for pose, line in zip(poses, out.splitlines()):
        strokes_c, limited_c, fk = ([float(v) for v in part.split()] for part in line.split("|"))
        assert strokes_c == pytest.approx(g.strokes(pose), abs=5e-3)
        lo, hi = g.leg_range()
        assert all(lo - 0.01 <= s <= hi + 0.01 for s in g.strokes(tuple(limited_c)))
        assert limited_c == pytest.approx(g.limit(pose), abs=0.05, rel=0.01)
        assert fk[0] == 1, f"C FK did not converge for {limited_c}"
        assert fk[1:] == pytest.approx(limited_c, abs=0.02)  # FK(IK(p)) == p
