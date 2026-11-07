#!/usr/bin/env python3
"""Smoke test for a CANopen node on a SocketCAN interface.

Checks, in order: NMT reset -> boot-up message, SDO upload (device name, vendor ID),
SDO download (producer heartbeat time), NMT start -> heartbeats in OPERATIONAL.

Usage: check_canopen_node.py [--iface vcan0] [--node 4]
Exit code 0 on success, 1 on the first failed check.
"""
import argparse
import struct
import sys
import threading
import time

import canopen

NMT_START = 0x01
NMT_RESET_COMM = 0x82
STATE_OPERATIONAL = 0x05
HEARTBEAT_MS = 100


def fail(msg: str) -> None:
    print(f"FAIL: {msg}")
    sys.exit(1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--node", type=int, default=4)
    ap.add_argument(
        "--save-params",
        action="store_true",
        help="only write the 1010h 'save' signature (creates storage files on a first run) and exit",
    )
    args = ap.parse_args()

    network = canopen.Network()
    network.connect(interface="socketcan", channel=args.iface)
    try:
        node = canopen.RemoteNode(args.node, canopen.ObjectDictionary())
        network.add_node(node)

        if args.save_params:
            node.sdo.download(0x1010, 1, struct.pack("<I", 0x65766173))  # "save"
            print(f"OK   1010h.1 save parameters on node {args.node}")
            return

        frames: list[tuple[float, bytes]] = []
        lock = threading.Lock()

        def on_nmt_error_control(can_id: int, data: bytearray, timestamp: float) -> None:
            with lock:
                frames.append((time.monotonic(), bytes(data)))

        network.subscribe(0x700 + args.node, on_nmt_error_control)

        # 1. NMT reset communication -> boot-up message (0x700+id, data 00)
        node.nmt.send_command(NMT_RESET_COMM)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with lock:
                if any(d == b"\x00" for _, d in frames):
                    break
            time.sleep(0.01)
        else:
            fail(f"no boot-up message from node {args.node} on {args.iface}")
        print(f"OK   boot-up from node {args.node} after NMT reset communication")

        # 2. SDO uploads
        device_type = struct.unpack("<I", node.sdo.upload(0x1000, 0))[0]
        vendor = struct.unpack("<I", node.sdo.upload(0x1018, 1))[0]
        product = struct.unpack("<I", node.sdo.upload(0x1018, 2))[0]
        print(
            f"OK   SDO upload: 1000h device type = 0x{device_type:08X}, "
            f"1018h vendor = 0x{vendor:08X}, product = {product}"
        )
        try:  # optional object, not in every OD (e.g. not in the CANopenNode demo)
            name = node.sdo.upload(0x1008, 0).decode(errors="replace").strip("\x00")
            print(f"OK   SDO upload: 1008h device name = {name!r}")
        except canopen.SdoAbortedError:
            print("--   1008h device name not present (optional)")

        # 3. SDO download: producer heartbeat time
        node.sdo.download(0x1017, 0, struct.pack("<H", HEARTBEAT_MS))
        readback = struct.unpack("<H", node.sdo.upload(0x1017, 0))[0]
        if readback != HEARTBEAT_MS:
            fail(f"1017h readback {readback} != {HEARTBEAT_MS}")
        print(f"OK   SDO download: 1017h producer heartbeat = {readback} ms")

        # 4. NMT start -> heartbeats with state OPERATIONAL
        with lock:
            frames.clear()
        node.nmt.send_command(NMT_START)
        time.sleep(1.05)
        with lock:
            beats = [d for _, d in frames if len(d) == 1]
        operational = [d for d in beats if d[0] == STATE_OPERATIONAL]
        if len(operational) < 8:
            fail(f"expected ~10 OPERATIONAL heartbeats in 1 s, got {len(operational)} ({beats})")
        print(f"OK   {len(operational)} heartbeats in 1 s, state OPERATIONAL (0x05)")
        print("PASS")
    finally:
        network.disconnect()


if __name__ == "__main__":
    main()
