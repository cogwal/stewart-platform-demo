"""Give a TMCM-1270 a new CANopen node ID (commissioning, one drive on the bus at a time).

python tools/set_node_id.py --iface can0 --from 1 --to 3 [--dry-run]

Firmware manual: 2705h = node ID (1..127), then the store signature "save" (0x65766173) to 2706h sub 1,
then a reset. Afterwards the tool waits for the boot-up frame from the new ID and reads 1018h there.
Not tested against a real drive yet (docs/commissioning.md, step 7): keep a candump running.
"""
from __future__ import annotations

import argparse
import sys
import time

import canopen

SAVE = 0x65766173


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--from", dest="old", type=int, default=1)
    ap.add_argument("--to", dest="new", type=int, required=True)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    if not 1 <= args.new <= 127:
        print("node ID must be 1..127", file=sys.stderr)
        return 2

    net = canopen.Network()
    net.connect(interface="socketcan", channel=args.iface)
    try:
        node = canopen.RemoteNode(args.old, canopen.ObjectDictionary())
        net.add_node(node)
        vendor = int.from_bytes(node.sdo.upload(0x1018, 1), "little")
        product = int.from_bytes(node.sdo.upload(0x1018, 2), "little")
        current = int.from_bytes(node.sdo.upload(0x2705, 0), "little")
        print(f"node {args.old}: vendor 0x{vendor:X}, product {product}, 2705h = {current}")
        if vendor != 0x286 or product != 1270:
            print("not a TMCM-1270: stopping", file=sys.stderr)
            return 1
        if args.dry_run:
            print(f"dry run: would write 2705h = {args.new}, 2706h.1 = 0x{SAVE:08X}, then reset")
            return 0
        node.sdo.download(0x2705, 0, args.new.to_bytes(1, "little"))
        node.sdo.download(0x2706, 1, SAVE.to_bytes(4, "little"))
        booted = []
        net.subscribe(0x700 + args.new, lambda cid, data, ts: booted.append(bytes(data)))
        node.nmt.send_command(0x81)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not any(b[:1] == b"\x00" for b in booted):
            time.sleep(0.01)
        if not booted:
            print(f"no boot-up from node {args.new} within 5 s", file=sys.stderr)
            return 1
        new = canopen.RemoteNode(args.new, canopen.ObjectDictionary())
        net.add_node(new)
        product = int.from_bytes(new.sdo.upload(0x1018, 2), "little")
        print(f"node {args.new} answers (product {product}). Label the drive.")
        return 0
    finally:
        net.disconnect()


if __name__ == "__main__":
    sys.exit(main())
