"""spmaster (C master on CANopenLinux) against six emulated drives on vcan0. Skipped without vcan0.

Own node ids, master id and UDP ports, so it can share the bus with a running `make sim`."""
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

from stewart import geometry as G

ROOT = Path(__file__).resolve().parents[2]
IFACE = os.environ.get("CAN_IFACE", "vcan0")
pytestmark = pytest.mark.skipif(not Path(f"/sys/class/net/{IFACE}").exists(), reason=f"no {IFACE}")

NODES = [21, 22, 23, 24, 25, 26]
MASTER = 126
CMD, STATUS, TRUTH, CONTROL = 15005, 15006, 15007, 15008


class Bench:
    def __init__(self, tmp: Path):
        assert subprocess.run(["make", "-C", str(ROOT / "master"), "spmaster"], capture_output=True).returncode == 0
        self.status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_sock.bind(("127.0.0.1", STATUS))
        self.status_sock.settimeout(0.5)
        self.truth_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.truth_sock.bind(("127.0.0.1", TRUTH))
        self.truth_sock.settimeout(0.5)
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.drive_log = open(tmp / "drive.log", "w")
        self.master_log = open(tmp / "master.log", "w")
        self.drive = subprocess.Popen(
            [sys.executable, "-m", "sim.drive", "--iface", IFACE, "--nodes", ",".join(map(str, NODES)),
             "--rod-mm", "4", "--rod-spread-mm", "1", "--truth-port", str(TRUTH), "--control-port", str(CONTROL)],
            cwd=ROOT, stdout=self.drive_log, stderr=subprocess.STDOUT)
        time.sleep(1.0)
        storage = tmp / "storage"
        storage.mkdir()
        env = dict(os.environ, SP_NODES=",".join(map(str, NODES)), SP_LEGS="6", SP_CMD_PORT=str(CMD),
                   SP_STATUS_PORT=str(STATUS))
        self.master = subprocess.Popen([str(ROOT / "build/master/spmaster"), "-i", str(MASTER), "-s", f"{storage}/", IFACE],
                                       cwd=ROOT, env=env, stdout=self.master_log, stderr=subprocess.STDOUT)
        self.tmp = tmp

    @staticmethod
    def latest(sock: socket.socket) -> dict:
        """A fresh datagram: drop everything queued (the queue may be full of old ones), then wait."""
        sock.setblocking(False)
        try:
            while True:
                sock.recv(65536)
        except BlockingIOError:
            pass
        finally:
            sock.settimeout(1.0)
        return json.loads(sock.recv(65536))

    def status(self) -> dict:
        return self.latest(self.status_sock)

    def truth(self) -> list[float]:
        return [d["rod_mm"] for d in self.latest(self.truth_sock)["drives"]]

    def send(self, msg: dict, port: int = CMD) -> None:
        self.out.sendto(json.dumps(msg).encode(), ("127.0.0.1", port))

    def wait_phase(self, phase: str, timeout: float) -> dict:
        deadline = time.monotonic() + timeout
        st = {}
        while time.monotonic() < deadline:
            try:
                st = self.status()
            except socket.timeout:
                continue
            if st["phase"] == phase:
                return st
        raise AssertionError(f"no phase {phase} within {timeout} s (last: {st.get('phase')}: {st.get('reason')})\n"
                             + (self.tmp / "master.log").read_text()[-3000:])

    def close(self):
        for p in (self.master, self.drive):
            p.terminate()
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        for s in (self.status_sock, self.truth_sock, self.out):
            s.close()


@pytest.fixture
def bench(tmp_path):
    b = Bench(tmp_path)
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
