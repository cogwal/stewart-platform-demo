"""Short soak run (tools/soak.py) in the test suite: random poses and faults for SOAK_SECONDS (default 60).
Run longer by hand: python tools/soak.py --minutes 15."""
import os
import random
from pathlib import Path

import pytest

from sim.bench import SimBench

IFACE = os.environ.get("CAN_IFACE", "vcan0")
SECONDS = float(os.environ.get("SOAK_SECONDS", "60"))
pytestmark = pytest.mark.skipif(not Path(f"/sys/class/net/{IFACE}").exists() or SECONDS <= 0, reason=f"no {IFACE}")


def test_short_soak(tmp_path):
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
    from soak import Soak

    bench = SimBench(tmp_path, IFACE, nodes=(51, 52, 53, 54, 55, 56), master=123, port_base=17005)
    soak = Soak(bench, random.Random(int(os.environ.get("SOAK_SEED", "11"))))
    try:
        soak.run(SECONDS)
    finally:
        bench.close()
    assert not soak.failures, soak.failures
