"""spmaster (C master on CANopenLinux) against six emulated drives on vcan0. Skipped without vcan0.

Own node ids, master id and UDP ports, so it can share the bus with a running `make sim`."""
import os
import time
from pathlib import Path

import pytest

from sim.bench import SimBench
from stewart import geometry as G

IFACE = os.environ.get("CAN_IFACE", "vcan0")
pytestmark = pytest.mark.skipif(not Path(f"/sys/class/net/{IFACE}").exists(), reason=f"no {IFACE}")

class Bench(SimBench):
    """Test-friendly names on top of the shared bench."""

    def send(self, msg: dict, port: int | None = None) -> None:
        (self.fault if port == CONTROL else self.command)(msg)


CONTROL = "control"


@pytest.fixture
def bench(tmp_path):
    b = Bench(tmp_path, IFACE)
    try:
        yield b
    finally:
        b.close()


def test_home_follow_pose_stop_resume_reboot(bench):
    g = G.load()
    st = bench.wait_phase("run", 30)
    assert st["homed"] and len(st["legs"]) == 6
    assert all(l["boot"] == "ready" for l in st["legs"])

    bench.send({"type": "pose", "z": 5, "roll": 4, "pitch": -3, "yaw": 2})
    time.sleep(10)
    expected = g.strokes((0, 0, 5, 4, -3, 2))
    assert bench.truth() == pytest.approx(expected, abs=0.05)
    st = bench.status()
    assert [l["actual_mm"] for l in st["legs"]] == pytest.approx(expected, abs=0.05)

    bench.send({"type": "stop"})
    bench.wait_phase("stopped", 2)
    bench.send({"type": "resume"})
    bench.wait_phase("run", 3)

    bench.send({"type": "power_cycle", "node": 23}, CONTROL)  # e.g. a loose power connector
    st = bench.wait_phase("fault", 3)
    assert not st["homed"] and "rebooted" in st["reason"]
    bench.send({"type": "home"})
    bench.wait_phase("homing", 5)
    bench.wait_phase("run", 40)


def test_heartbeat_glitch_then_resume_and_enn(bench):
    bench.wait_phase("run", 30)
    # Drive heartbeat lost for longer than the master's consumer time: stop, but keep homing (the drive
    # did not reboot, its step counter is still valid).
    bench.send({"type": "mute_hb", "node": 24, "s": 0.6}, CONTROL)
    st = bench.wait_phase("stopped", 2)
    assert st["homed"] and "heartbeat lost" in st["reason"]
    time.sleep(1.0)
    st = bench.status()
    assert all(l["boot"] == "ready" for l in st["legs"])
    bench.send({"type": "resume"})
    bench.wait_phase("run", 3)
    # ENN (enable input, e.g. e-stop chain) drops on one drive: EMCY 5441h, stop and lose homing.
    bench.send({"type": "enn", "node": 22, "on": False}, CONTROL)
    st = bench.wait_phase("fault", 2)
    assert not st["homed"] and "0x5441" in st["reason"]
    bench.send({"type": "enn", "node": 22, "on": True}, CONTROL)
    bench.send({"type": "home"})
    bench.wait_phase("run", 40)
