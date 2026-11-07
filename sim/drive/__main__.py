"""Run one or more emulated TMCM-1270 drives on a CAN interface.

python -m sim.drive [--iface vcan0] [--nodes 1,2,3,4,5,6] [--rod-mm 37]

UDP side channels (localhost, JSON), which no real drive has:
  truth   -> :5007  every 50 ms {"drives": [snapshot, ...]}: real rod positions, stall state
  control <- :5008  fault injection, e.g. {"type": "power_cycle", "node": 3}
            types: power_cycle, power_off, enn {"on": bool}, jam {"mm": float|null},
                   mute_hb {"s": float}, slip {"mm": float} (rod moves without the step counter)
Force model: with six drives and a payload (config/platform.yaml simulation:, or SIM_PAYLOAD_KG) each leg gets its
static axial force; legs pushing beyond the S20's force limit skip steps.
"""
import argparse
import json
import logging
import os
import signal
import socket
import threading
import time

import canopen

from sim.drive.can_node import CanDrive
from sim.drive.s20 import S20
from sim.platform_physics import leg_forces
from stewart import geometry as G

log = logging.getLogger("sim.drive")


def apply_fault(drives: dict[int, CanDrive], msg: dict) -> str:
    kind, node = msg.get("type"), int(msg.get("node", 0))
    targets = list(drives.values()) if node == 0 else [drives[node]] if node in drives else []
    if not targets:
        return f"no drive {node}"
    for d in targets:
        if kind == "power_cycle":
            d.power_cycle()
        elif kind == "power_off":
            d.power_off()
        elif kind == "enn":
            with d.lock:
                d.core.set_enn(bool(msg.get("on", True)))
        elif kind == "jam":
            mm = msg.get("mm")
            d.plant.jam_mm = None if mm is None else float(mm)
        elif kind == "mute_hb":
            d.mute_heartbeat_until = time.monotonic() + float(msg.get("s", 1.0))
        elif kind == "slip":
            with d.lock:
                d.plant.move(float(msg.get("mm", 0.0)))
        else:
            return f"unknown fault {kind}"
    return f"{kind} on {'all' if node == 0 else node}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--nodes", default="1", help="comma-separated node IDs")
    ap.add_argument("--rod-mm", type=float, default=37.0, help="rod position at power-up (unknown to the drive)")
    ap.add_argument("--rod-spread-mm", type=float, default=0.0, help="add this much per node to --rod-mm")
    ap.add_argument("--truth-port", type=int, default=5007)
    ap.add_argument("--control-port", type=int, default=5008)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(asctime)s %(name)s %(message)s")
    logging.getLogger("canopen").setLevel(logging.WARNING)

    network = canopen.Network()
    network.connect(interface="socketcan", channel=args.iface)
    nodes = [int(n) for n in args.nodes.split(",")]
    drives = {n: CanDrive(network, n, plant=S20(position_mm=args.rod_mm + k * args.rod_spread_mm))
              for k, n in enumerate(nodes)}
    for d in drives.values():
        d.start()
    log.info("emulating TMCM-1270 node(s) %s on %s", args.nodes, args.iface)

    done = threading.Event()

    def publish_truth():
        out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while not done.wait(0.05):
            snap = {"drives": [d.snapshot() for d in drives.values()]}
            try:
                out.sendto(json.dumps(snap).encode(), ("127.0.0.1", args.truth_port))
            except OSError:
                pass

    def control():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", args.control_port))
        sock.settimeout(0.2)
        while not done.is_set():
            try:
                data, _ = sock.recvfrom(4096)
                log.warning("fault injection: %s", apply_fault(drives, json.loads(data)))
            except socket.timeout:
                pass
            except (ValueError, KeyError, TypeError) as e:
                log.warning("bad control message: %s", e)

    def physics():
        """Payload -> leg forces from the real (truth) pose; only for a full platform with a payload configured."""
        cfg = G.config().get("simulation", {})
        payload = float(os.environ.get("SIM_PAYLOAD_KG", cfg.get("payload_kg", 0.0)))
        mass = payload + cfg.get("platform_mass_kg", 0.0)
        if len(drives) != 6 or payload <= 0:
            return
        geom = G.load()
        legs = list(drives.values())
        for d in legs:
            d.plant.max_force_n = cfg.get("s20_max_force_n", 60.0)
            d.plant.derate_at_10mm_s = cfg.get("s20_force_derate_at_10mm_s", 0.3)
        log.info("force model on: %.2f kg incl. platform, S20 max %.0f N", mass, legs[0].plant.max_force_n)
        guess = (0.0,) * 6
        while not done.wait(0.02):
            try:
                guess = geom.forward([d.plant.position_mm for d in legs], guess, tol=1e-3, max_iter=10)
                forces = leg_forces(geom, guess, mass, cfg.get("com_above_platform_mm", 20.0))
            except ValueError:
                guess = (0.0,) * 6
                continue
            for d, f in zip(legs, forces):
                d.plant.load_n = f

    threading.Thread(target=publish_truth, daemon=True).start()
    threading.Thread(target=physics, daemon=True).start()
    threading.Thread(target=control, daemon=True).start()
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    signal.signal(signal.SIGINT, lambda *_: done.set())
    done.wait()
    for d in drives.values():
        d.stop()
    network.disconnect()


if __name__ == "__main__":
    main()
