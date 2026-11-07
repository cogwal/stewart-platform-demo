"""Run one or more emulated TMCM-1270 drives on a CAN interface.

python -m sim.drive [--iface vcan0] [--nodes 1] [--rod-mm 37]
"""
import argparse
import json
import logging
import signal
import socket
import threading

import canopen

from sim.drive.can_node import CanDrive
from sim.drive.s20 import S20


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--nodes", default="1", help="comma-separated node IDs")
    ap.add_argument("--rod-mm", type=float, default=37.0, help="rod position at power-up (unknown to the drive)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(asctime)s %(name)s %(message)s")
    logging.getLogger("canopen").setLevel(logging.WARNING)

    network = canopen.Network()
    network.connect(interface="socketcan", channel=args.iface)
    drives = [CanDrive(network, int(n), plant=S20(position_mm=args.rod_mm)) for n in args.nodes.split(",")]
    for d in drives:
        d.start()
    logging.info("emulating TMCM-1270 node(s) %s on %s", args.nodes, args.iface)

    done = threading.Event()

    def publish_truth():
        """Simulated rod position for the web view (:5007). No real drive can report this."""
        out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while not done.wait(0.05):
            out.sendto(json.dumps(drives[0].snapshot()).encode(), ("127.0.0.1", 5007))

    threading.Thread(target=publish_truth, daemon=True).start()
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    signal.signal(signal.SIGINT, lambda *_: done.set())
    done.wait()
    for d in drives:
        d.stop()
    network.disconnect()


if __name__ == "__main__":
    main()
