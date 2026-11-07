"""The drive probe (tools/drive_probe.py) runs against the emulator: passive checks only (fast)."""
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
IFACE = os.environ.get("CAN_IFACE", "vcan0")
pytestmark = pytest.mark.skipif(not Path(f"/sys/class/net/{IFACE}").exists(), reason=f"no {IFACE}")


def test_probe_passive_against_emulator(tmp_path):
    drive = subprocess.Popen([sys.executable, "-m", "sim.drive", "--iface", IFACE, "--nodes", "41",
                              "--truth-port", "35007", "--control-port", "35008"], cwd=ROOT,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        import time
        time.sleep(1.0)
        out = tmp_path / "probe.json"
        r = subprocess.run([sys.executable, "tools/drive_probe.py", "--iface", IFACE, "--node", "41", "--master", "124",
                            "--out", str(out)], cwd=ROOT, capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, r.stdout + r.stderr
        import json
        report = json.loads(out.read_text())
        assert report["sdo_aborts"] == []
        assert report["measurements"]["write_2005_in_operation_enabled"]["value"].startswith("abort")
        same = subprocess.run([sys.executable, "tools/drive_probe.py", "--compare", str(out), str(out)], cwd=ROOT)
        assert same.returncode == 0
    finally:
        drive.terminate()
        drive.wait(timeout=5)
