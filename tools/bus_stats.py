"""Listen on a CAN interface and report timing and load: SYNC period and jitter, heartbeat gaps per
node, PDO rates, EMCYs, estimated bus load. Works the same on vcan (simulation) and can0 (bench).

python tools/bus_stats.py [--iface vcan0] [--seconds 10] [--bitrate 1000000] [--json]
"""
from __future__ import annotations

import argparse
import json
import statistics
import time
from collections import defaultdict

import can

FUNC = {0x000: "NMT", 0x080: "SYNC/EMCY", 0x180: "TPDO1", 0x200: "RPDO1", 0x280: "TPDO2", 0x300: "RPDO2",
        0x380: "TPDO3", 0x400: "RPDO3", 0x480: "TPDO4", 0x500: "RPDO4", 0x580: "SDO tx", 0x600: "SDO rx",
        0x700: "HB"}


def frame_bits(dlc: int) -> float:
    """Standard frame on the wire incl. worst-case-ish stuffing (about 20 %) and interframe space."""
    return (47 + 8 * dlc) * 1.2


def collect(iface: str, seconds: float) -> dict:
    bus = can.Bus(interface="socketcan", channel=iface)
    times: dict[str, list[float]] = defaultdict(list)
    counts: dict[str, int] = defaultdict(int)
    emcy: list[tuple[float, int, str]] = []
    bits = 0.0
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            msg = bus.recv(timeout=0.1)
            if msg is None or msg.is_error_frame:
                continue
            ts = msg.timestamp
            cid, node = msg.arbitration_id, msg.arbitration_id & 0x7F
            base = cid & 0x780
            bits += frame_bits(msg.dlc)
            if cid == 0x080:
                key = "SYNC"
            elif base == 0x080:
                key = f"EMCY {node}"
                emcy.append((ts, node, bytes(msg.data).hex()))
            elif cid == 0x000:
                key = "NMT"
            else:
                key = f"{FUNC.get(base, hex(base))} {node}"
            counts[key] += 1
            times[key].append(ts)
    finally:
        bus.shutdown()
    elapsed = time.monotonic() - t0
    return {"elapsed": elapsed, "counts": counts, "times": times, "emcy": emcy, "bits": bits}


def gaps(ts: list[float]) -> list[float]:
    return [1000 * (b - a) for a, b in zip(ts, ts[1:])]


def report(data: dict, bitrate: int) -> dict:
    out = {"seconds": round(data["elapsed"], 2), "bus_load_pct": round(100 * data["bits"] / data["elapsed"] / bitrate, 2),
           "frames": sum(data["counts"].values()), "streams": {}}
    for key in sorted(data["times"]):
        g = gaps(data["times"][key])
        entry = {"count": data["counts"][key], "rate_hz": round(data["counts"][key] / data["elapsed"], 1)}
        if len(g) >= 2:
            entry.update(mean_ms=round(statistics.mean(g), 2), max_ms=round(max(g), 2), min_ms=round(min(g), 2),
                         stdev_ms=round(statistics.pstdev(g), 2))
        out["streams"][key] = entry
    out["emcy"] = [{"node": n, "data": d} for _, n, d in data["emcy"]]
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    r = report(collect(args.iface, args.seconds), args.bitrate)
    if args.json:
        print(json.dumps(r, indent=2))
        return
    print(f"{r['frames']} frames in {r['seconds']} s, estimated bus load {r['bus_load_pct']} % at {args.bitrate} bit/s")
    print(f"{'stream':<12} {'count':>6} {'Hz':>7} {'mean ms':>8} {'min':>7} {'max':>7} {'stdev':>6}")
    for key, e in r["streams"].items():
        print(f"{key:<12} {e['count']:>6} {e['rate_hz']:>7} {e.get('mean_ms', ''):>8} {e.get('min_ms', ''):>7} "
              f"{e.get('max_ms', ''):>7} {e.get('stdev_ms', ''):>6}")
    for e in r["emcy"]:
        print(f"EMCY node {e['node']}: {e['data']}")


if __name__ == "__main__":
    main()
