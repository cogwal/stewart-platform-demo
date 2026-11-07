"""Web view: Stewart platform in 3D, operator inputs, per-leg status, fault injection.

Reads every drive's state from the CAN bus (heartbeat, TPDO1 statusword/position, EMCY), so it works
the same against the emulator or real drives. Optional local sources over UDP:
  :5006  master status (C master or Python test master)
  :5007  emulator truth: real rod positions, which no real drive can report
Operator input goes to the master's command port (:5005); fault injection to the emulator (:5008).

Forward kinematics turns leg positions into platform poses twice: from the bus (what the master
believes) and from the truth (where the platform really is). They differ when steps are lost.

python -m sim.web [--iface vcan0] [--nodes 1,2,3,4,5,6] [--port 8080] [--record build/session.jsonl]
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import socket
import struct
import threading
import time
from pathlib import Path

import can
from aiohttp import WSMsgType, web

import contract as C
from stewart import geometry as G

log = logging.getLogger(__name__)
STATIC = Path(__file__).parent / "static"
MASTER_CMD = ("127.0.0.1", 5005)
DRIVE_CTL = ("127.0.0.1", 5008)
STATUS_PORT, TRUTH_PORT = 5006, 5007
NMT_NAMES = {0x00: "boot-up", 0x04: "stopped", 0x05: "operational", 0x7F: "pre-operational"}
MASTER_COMMANDS = {"setpoint", "home", "stop", "resume", "pose", "joystick", "imu", "legs", "source"}
FAULTS = {"power_cycle", "power_off", "enn", "jam", "mute_hb", "slip"}


class Recorder:
    """One JSON line per event, shared monotonic + wall timestamps: CAN frames, master status, simulator truth.
    Replaces separate candump / status logs when the timing between them matters (bench, tuning, debugging)."""

    def __init__(self, path: Path | None):
        self.f = open(path, "a", buffering=1) if path else None
        self.lock = threading.Lock()

    def write(self, kind: str, payload) -> None:
        if self.f is None:
            return
        line = json.dumps({"t": round(time.monotonic(), 6), "wall": round(time.time(), 6), "kind": kind, "data": payload})
        with self.lock:
            self.f.write(line + "\n")


class Drive:
    def __init__(self, node: int):
        self.node = node
        self.nmt: int | None = None
        self.last_heartbeat = 0.0
        self.statusword: int | None = None
        self.counts: int | None = None


class BusView(can.Listener):
    """Everything the bus tells us about the drives."""

    def __init__(self, nodes: list[int], contract: C.Contract, recorder: Recorder | None = None):
        self.c = contract
        self.rec = recorder
        self.drives = {n: Drive(n) for n in nodes}
        self.emcy: list[dict] = []
        self.frames = 0
        self.sync_times: list[float] = []

    def on_message_received(self, msg: can.Message) -> None:
        self.frames += 1
        cid, data = msg.arbitration_id, bytes(msg.data)
        if self.rec is not None and self.rec.f is not None:
            self.rec.write("can", {"id": cid, "data": data.hex(), "ts": msg.timestamp})
        if cid == 0x080 and not data:
            self.sync_times = (self.sync_times + [time.time()])[-50:]
            return
        node, base = cid & 0x7F, cid & 0x780
        d = self.drives.get(node)
        if d is None:
            return
        if base == 0x700 and data:
            d.nmt, d.last_heartbeat = data[0] & 0x7F, time.time()
        elif base == 0x180 and len(data) == 6:
            d.statusword, d.counts = struct.unpack("<Hi", data)
        elif base == 0x080 and len(data) >= 3:
            code, _ = struct.unpack_from("<HB", data)
            byte1 = data[3] if len(data) > 3 else 0
            self.emcy = (self.emcy + [{"t": time.time(), "node": node, "code": f"0x{code:04X}", "byte1": byte1,
                                       "class": self.c.emcy_class(code, byte1)}])[-12:]

    def state_name(self, sw: int | None) -> str | None:
        if sw is None:
            return None
        for name, (mask, value) in self.c["cia402"]["statusword"].items():
            if sw & mask == value:
                return name.replace("_", " ")
        return "unknown"

    def snapshot(self) -> list[dict]:
        timeout = self.c["heartbeat"]["master_consumes_drive_ms"] / 1000
        out = []
        for d in self.drives.values():
            alive = time.time() - d.last_heartbeat < timeout
            out.append({
                "node": d.node,
                "nmt": NMT_NAMES.get(d.nmt, d.nmt) if alive else "silent",
                "statusword": None if d.statusword is None else f"0x{d.statusword:04X}",
                "state": self.state_name(d.statusword),
                "position_mm": None if d.counts is None else round(d.counts / self.c.counts_per_mm, 3),
            })
        return out

    def sync_period_ms(self) -> float | None:
        t = self.sync_times
        if len(t) < 2 or time.time() - t[-1] > 0.5:
            return None
        return round(1000 * (t[-1] - t[0]) / (len(t) - 1), 2)


class UdpInbox(asyncio.DatagramProtocol):
    def __init__(self, kind: str = "", recorder: Recorder | None = None):
        self.latest: dict | None = None
        self.received = 0.0
        self.kind, self.rec = kind, recorder

    def datagram_received(self, data, addr):
        try:
            self.latest, self.received = json.loads(data), time.time()
        except ValueError:
            return
        if self.rec is not None:
            self.rec.write(self.kind, self.latest)

    def fresh(self) -> dict | None:
        return self.latest if time.time() - self.received < 1.0 else None


class PoseTracker:
    """Forward kinematics with a warm start; None when not all six legs are known."""

    def __init__(self, geom: G.Geometry):
        self.g = geom
        self.guess = (0.0,) * 6

    def update(self, strokes: list[float | None]) -> list[float] | None:
        if len(strokes) != 6 or any(s is None for s in strokes):
            return None
        try:
            pose = self.g.forward(strokes, self.guess, tol=1e-4, max_iter=15)
        except ValueError:
            self.guess = (0.0,) * 6
            return None
        self.guess = pose
        return [round(v, 3) for v in pose]


async def make_app(iface: str, nodes: list[int], record: Path | None = None) -> web.Application:
    c = C.load()
    geom = G.load(contract=c)
    rec = Recorder(record)
    view = BusView(nodes, c, rec)
    bus = can.Bus(interface="socketcan", channel=iface)
    notifier = can.Notifier(bus, [view])
    loop = asyncio.get_running_loop()
    master_status, truth = UdpInbox("master", rec), UdpInbox("truth", rec)
    await loop.create_datagram_endpoint(lambda: master_status, local_addr=("127.0.0.1", STATUS_PORT))
    await loop.create_datagram_endpoint(lambda: truth, local_addr=("127.0.0.1", TRUTH_PORT))
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bus_pose, truth_pose = PoseTracker(geom), PoseTracker(geom)
    pcfg = G.config()

    config = {
        "type": "config",
        "iface": iface,
        "nodes": nodes,
        "stroke_mm": c["actuator"]["stroke_mm"],
        "soft_limits_mm": c["actuator"]["soft_limits_mm"],
        "closed_length_mm": geom.closed_length_mm,
        "neutral_z_mm": geom.neutral_z_mm,
        "neutral_stroke_mm": geom.neutral_stroke_mm,
        "base": geom.base,
        "platform": geom.platform,
        "limits": geom.limits,
        "geometry_version": pcfg["geometry_version"],
        "contract_version": c["contract_version"],
    }

    def state() -> dict:
        drives = view.snapshot()
        tr = truth.fresh()
        truth_mm = None
        if tr and "drives" in tr:
            by_node = {d["node"]: d for d in tr["drives"]}
            truth_mm = [by_node.get(n, {}).get("rod_mm") for n in nodes]
            for d in drives:
                t = by_node.get(d["node"])
                d["truth_mm"] = t["rod_mm"] if t else None
                d["stalled"] = t["stalled"] if t else None
                d["force_n"] = t.get("force_n") if t else None
                d["overloaded"] = t.get("overloaded") if t else None
        return {
            "type": "state",
            "drives": drives,
            "master": master_status.fresh(),
            "emcy": view.emcy,
            "frames": view.frames,
            "sync_ms": view.sync_period_ms(),
            "pose_bus": bus_pose.update([d["position_mm"] for d in drives]) if len(nodes) == 6 else None,
            "pose_truth": truth_pose.update(truth_mm) if truth_mm and len(nodes) == 6 else None,
        }

    async def index(_):
        return web.FileResponse(STATIC / "index.html")

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        await ws.send_json(config)

        async def push():
            while not ws.closed:
                await ws.send_json(state())
                await asyncio.sleep(0.05)

        pusher = asyncio.create_task(push())
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    cmd = json.loads(msg.data)
                except ValueError:
                    continue
                kind = cmd.get("type")
                rec.write("operator", cmd)
                if kind in MASTER_COMMANDS:
                    out.sendto(json.dumps(cmd).encode(), MASTER_CMD)
                elif kind == "fault" and cmd.get("fault") in FAULTS:
                    fault = dict(cmd, type=cmd["fault"])
                    fault.pop("fault")
                    out.sendto(json.dumps(fault).encode(), DRIVE_CTL)
        finally:
            pusher.cancel()
        return ws

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/static", STATIC)

    async def cleanup(_):
        notifier.stop()
        bus.shutdown()

    app.on_cleanup.append(cleanup)
    return app


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--nodes", default=",".join(map(str, C.load().node_ids)))
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--record", type=Path, help="append CAN frames, master status, truth and operator input as JSON lines")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    logging.getLogger("aiohttp.access").setLevel(logging.WARNING)
    nodes = [int(n) for n in args.nodes.split(",")]
    web.run_app(make_app(args.iface, nodes, args.record), host=args.host, port=args.port, print=lambda *_: None)


if __name__ == "__main__":
    main()
