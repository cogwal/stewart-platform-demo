"""Simulation bench: emulated drives + spmaster (C master) as subprocesses with their own node ids, master id
and UDP ports, so it can share a bus with a running `make sim`. Used by the end-to-end tests and tools/soak.py."""
from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


class SimBench:
    def __init__(self, tmp: Path, iface: str = "vcan0", nodes=(21, 22, 23, 24, 25, 26), master: int = 126,
                 port_base: int = 15005, rod_mm: float = 4.0):
        self.nodes, self.iface = list(nodes), iface
        self.cmd, self.status_port, self.truth_port, self.control = port_base, port_base + 1, port_base + 2, port_base + 3
        r = subprocess.run(["make", "-C", str(ROOT / "master"), "spmaster"], capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError(r.stdout + r.stderr)
        self.status_sock = self._bind(self.status_port)
        self.truth_sock = self._bind(self.truth_port)
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tmp.mkdir(parents=True, exist_ok=True)
        self.tmp = tmp
        self.drive_log = open(tmp / "drive.log", "w")
        self.master_log = open(tmp / "master.log", "w")
        self.drive = subprocess.Popen(
            [sys.executable, "-m", "sim.drive", "--iface", iface, "--nodes", ",".join(map(str, self.nodes)),
             "--rod-mm", str(rod_mm), "--rod-spread-mm", "1", "--truth-port", str(self.truth_port),
             "--control-port", str(self.control)],
            cwd=ROOT, stdout=self.drive_log, stderr=subprocess.STDOUT)
        time.sleep(1.0)
        storage = tmp / "storage"
        storage.mkdir(exist_ok=True)
        env = dict(os.environ, SP_NODES=",".join(map(str, self.nodes)), SP_LEGS=str(len(self.nodes)),
                   SP_CMD_PORT=str(self.cmd), SP_STATUS_PORT=str(self.status_port))
        self.master = subprocess.Popen([str(ROOT / "build/master/spmaster"), "-i", str(master), "-s", f"{storage}/", iface],
                                       cwd=ROOT, env=env, stdout=self.master_log, stderr=subprocess.STDOUT)

    @staticmethod
    def _bind(port: int) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", port))
        s.settimeout(1.0)
        return s

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

    def command(self, msg: dict) -> None:
        self.out.sendto(json.dumps(msg).encode(), ("127.0.0.1", self.cmd))

    def fault(self, msg: dict) -> None:
        self.out.sendto(json.dumps(msg).encode(), ("127.0.0.1", self.control))

    def wait_phase(self, phase: str, timeout: float) -> dict:
        deadline = time.monotonic() + timeout
        st: dict = {}
        while time.monotonic() < deadline:
            try:
                st = self.status()
            except socket.timeout:
                continue
            if st["phase"] == phase:
                return st
        raise AssertionError(f"no phase {phase} within {timeout} s (last: {st.get('phase')}: {st.get('reason')})\n"
                             + (self.tmp / "master.log").read_text()[-3000:])

    def close(self) -> None:
        for p in (self.master, self.drive):
            p.terminate()
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        for s in (self.status_sock, self.truth_sock, self.out):
            s.close()
        self.drive_log.close()
        self.master_log.close()
