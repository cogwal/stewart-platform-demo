"""Run the Python test master: boot, configure and home the drive, then follow UDP setpoints.

python -m sim.master [--iface vcan0] [--node 1] [--no-home]
"""
import argparse
import logging
import threading

import canopen

from sim.master.testmaster import MasterError, PyMaster, serve


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--no-home", action="store_true", help="wait for a 'home' command instead of homing at start")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    logging.getLogger("canopen").setLevel(logging.WARNING)

    network = canopen.Network()
    network.connect(interface="socketcan", channel=args.iface)
    master = PyMaster(network, args.node)
    master.start()

    if not args.no_home:
        def startup():
            try:
                master.boot()
                master.home()
            except MasterError as e:
                logging.error("%s", e)
                master.phase = f"error: {e}"
        threading.Thread(target=startup, daemon=True).start()
    serve(master)


if __name__ == "__main__":
    main()
