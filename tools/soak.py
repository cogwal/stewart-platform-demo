"""Soak test: the C master against six emulated drives, random operator input and random faults for a while.
After every disturbance it recovers the way an operator would (Resume if homing was kept, else Home) and checks:
  - the master gets back to `run` in time, all drives `ready`
  - the legs the master believes in (bus) match the simulated rods (truth) within 0.1 mm, unless a slip
    (lost steps the master can't see) happened since the last homing
  - commanded poses are reached

python tools/soak.py [--minutes 10] [--seed 1] [--iface vcan0] [--out build/soak.json]
"""
from __future__ import annotations

import argparse
import json
import random
import sys
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sim.bench import SimBench  # noqa: E402
from stewart import geometry as G  # noqa: E402

TOL_MM = 0.1


class Soak:
    def __init__(self, bench: SimBench, rng: random.Random):
        self.b, self.rng, self.g = bench, rng, G.load()
        self.counts: Counter = Counter()
        self.failures: list[dict] = []
        self.slipped = False
        self.events: list[str] = []

    def log(self, text: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {text}"
        self.events.append(line)
        print(line, flush=True)

    def fail(self, what: str, st: dict | None = None) -> None:
        self.failures.append({"what": what, "phase": st and st.get("phase"), "reason": st and st.get("reason"),
                              "events": self.events[-8:]})
        self.log(f"FAIL {what}")

    def recover(self) -> bool:
        st = self.b.status()
        if st["phase"] == "stopped" and st["homed"]:
            self.b.command({"type": "resume"})
            timeout = 5
        elif st["phase"] != "run":
            self.b.command({"type": "home"})
            self.slipped = False
            timeout = 60
        else:
            return True
        try:
            self.b.wait_phase("run", timeout)
        except AssertionError:
            st = self.b.status()
            if st["phase"] == "stopped" and st["homed"]:  # e.g. a heartbeat glitch during recovery
                self.b.command({"type": "resume"})
                try:
                    self.b.wait_phase("run", 5)
                    return True
                except AssertionError:
                    pass
            self.fail(f"no recovery to run (from {st['phase']})", st)
            return False
        self.counts["recoveries"] += 1
        self.b.command({"type": "source", "source": "pose"})  # stand still for the consistency check
        return True

    def check_consistent(self, label: str, moving_ok: bool = False) -> None:
        """Compare bus and truth at standstill (while moving, the two are sampled at different times)."""
        if self.slipped:
            return
        prev, still = None, 0
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not moving_ok:
            bus = [leg["actual_mm"] for leg in self.b.status()["legs"]]
            if prev is not None and max(abs(a - b) for a, b in zip(bus, prev)) < 0.002:
                still += 1
                if still >= 3:
                    break
            else:
                still = 0
            prev = bus
        st, truth = self.b.status(), self.b.truth()
        bus = [leg["actual_mm"] for leg in st["legs"]]
        worst = max(abs(a - b) for a, b in zip(bus, truth))
        tol = 1.0 if moving_ok else TOL_MM  # moving: 10 mm/s x up to 100 ms sampling skew
        if st["phase"] == "run" and worst > tol:
            self.fail(f"{label}: bus vs truth differ by {worst:.3f} mm", st)

    def random_pose(self) -> tuple:
        lim = self.g.limits
        raw = tuple(self.rng.uniform(-1, 1) * lim[k] for k in ("x_mm", "y_mm", "z_mm", "roll_deg", "pitch_deg", "yaw_deg"))
        return self.g.limit(raw)

    def settle_pose(self, pose: tuple) -> None:
        self.b.command({"type": "source", "source": "pose"})
        self.b.command(dict(zip(("type", "x", "y", "z", "roll", "pitch", "yaw"), ("pose",) + tuple(pose))))
        deadline = time.monotonic() + 15
        expected = self.g.strokes(pose)
        while time.monotonic() < deadline:
            st = self.b.status()
            if st["phase"] != "run":
                return
            bus = [leg["actual_mm"] for leg in st["legs"]]
            if max(abs(a - b) for a, b in zip(bus, expected)) < 0.05:
                self.check_consistent("pose")
                return
        self.fail(f"pose {tuple(round(v, 1) for v in pose)} not reached", self.b.status())

    def node(self) -> int:
        return self.rng.choice(self.b.nodes)

    # --- actions ----------------------------------------------------------------------------
    def a_pose(self):
        self.settle_pose(self.random_pose())

    def a_demo(self):
        self.b.command({"type": "source", "source": "demo"})
        time.sleep(self.rng.uniform(3, 8))
        self.check_consistent("demo", moving_ok=True)

    def a_joystick(self):
        self.b.command({"type": "source", "source": "joystick"})
        for _ in range(20):
            self.b.command({"type": "joystick", "x": self.rng.uniform(-1, 1), "y": self.rng.uniform(-1, 1)})
            time.sleep(0.1)
        self.b.command({"type": "joystick", "x": 0, "y": 0})
        time.sleep(2)
        self.check_consistent("joystick")

    def a_stop_resume(self):
        self.b.command({"type": "stop"})
        self.expect("stopped", 2)
        self.recover()
        self.check_consistent("stop/resume")

    def a_mute_hb(self):
        self.b.fault({"type": "mute_hb", "node": self.node(), "s": 0.6})
        self.expect("stopped", 2)
        time.sleep(0.8)
        self.recover()
        self.check_consistent("heartbeat glitch")

    def a_power_cycle(self):
        self.b.fault({"type": "power_cycle", "node": self.node()})
        self.expect("fault", 3)
        self.recover()
        self.check_consistent("after reboot + home")

    def a_enn(self):
        n = self.node()
        self.b.fault({"type": "enn", "node": n, "on": False})
        self.expect("fault", 2)
        self.b.fault({"type": "enn", "node": n, "on": True})
        self.recover()
        self.check_consistent("after ENN + home")

    def a_jam(self):
        i = self.rng.randrange(len(self.b.nodes))
        truth = self.b.truth()
        self.b.fault({"type": "jam", "node": self.b.nodes[i], "mm": truth[i] + 2.0})
        self.b.command({"type": "source", "source": "legs"})
        legs = list(truth)
        legs[i] = min(truth[i] + 15.0, 95.0)
        self.b.command({"type": "legs", "mm": legs})
        self.expect("fault", 6)
        self.b.fault({"type": "jam", "node": self.b.nodes[i], "mm": None})
        self.b.command({"type": "source", "source": "pose"})
        self.recover()
        self.check_consistent("after jam + home")

    def a_slip(self):
        self.b.fault({"type": "slip", "node": self.node(), "mm": -1.5})
        self.slipped = True  # the master can't see this until the next homing
        time.sleep(0.5)

    def a_home(self):
        self.b.command({"type": "home"})
        self.slipped = False
        self.expect("run", 60)
        self.check_consistent("home")

    def expect(self, phase: str, timeout: float) -> None:
        try:
            self.b.wait_phase(phase, timeout)
        except AssertionError:
            self.fail(f"expected phase {phase}", self.b.status())

    def run(self, seconds: float) -> None:
        actions = [(self.a_pose, 30), (self.a_demo, 8), (self.a_joystick, 8), (self.a_stop_resume, 8),
                   (self.a_mute_hb, 8), (self.a_power_cycle, 6), (self.a_enn, 6), (self.a_jam, 6), (self.a_slip, 4),
                   (self.a_home, 3)]
        fns, weights = zip(*actions)
        self.b.wait_phase("run", 60)
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            fn = self.rng.choices(fns, weights)[0]
            name = fn.__name__[2:]
            self.counts[name] += 1
            self.log(f"{name}")
            fn()
            if self.b.status()["phase"] != "run":
                self.recover()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--minutes", type=float, default=10)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--out", type=Path, default=ROOT / "build" / "soak.json")
    ap.add_argument("--port-base", type=int, default=16005)
    args = ap.parse_args()
    bench = SimBench(ROOT / "build" / "soak", args.iface, nodes=(31, 32, 33, 34, 35, 36), master=125,
                     port_base=args.port_base)
    soak = Soak(bench, random.Random(args.seed))
    try:
        soak.run(args.minutes * 60)
    finally:
        bench.close()
    summary = {"minutes": args.minutes, "seed": args.seed, "counts": dict(soak.counts), "failures": soak.failures}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2))
    print(json.dumps({"counts": summary["counts"], "failures": len(soak.failures)}, indent=2))
    return 1 if soak.failures else 0


if __name__ == "__main__":
    sys.exit(main())
