"""Web view: slider + leg view in the browser.

Reads the drive's state from the CAN bus (heartbeat, TPDO1 statusword/position, EMCY), so it works
the same against the emulator or a real drive. Adds two optional local sources over UDP: the master's
status (:5006) and the emulator's simulated rod position, the "truth" no real drive can report (:5007).
Operator input from the browser goes to the master's command port (:5005).

python -m sim.web [--iface vcan0] [--node 1] [--port 8080]
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import socket
import struct
import time
from pathlib import Path

import can
from aiohttp import WSMsgType, web

import contract as C

log = logging.getLogger(__name__)
STATIC = Path(__file__).parent / "static"
MASTER_CMD = ("127.0.0.1", 5005)
STATUS_PORT, TRUTH_PORT = 5006, 5007
NMT_NAMES = {0x00: "boot-up", 0x04: "stopped", 0x05: "operational", 0x7F: "pre-operational"}


class BusView(can.Listener):
    """Everything the bus tells us about one drive."""

    def __init__(self, node: int, contract: C.Contract):
        self.node, self.c = node, contract
        self.nmt = None
        self.last_heartbeat = 0.0
        self.statusword = None
        self.counts = None
        self.emcy: list[dict] = []
        self.frames = 0

    def on_message_received(self, msg: can.Message) -> None:
        self.frames += 1
        cid, data = msg.arbitration_id, bytes(msg.data)
        if cid == 0x700 + self.node and data:
            self.nmt, self.last_heartbeat = data[0] & 0x7F, time.time()
        elif cid == 0x180 + self.node and len(data) == 6:
            self.statusword, self.counts = struct.unpack("<Hi", data)
        elif cid == 0x080 + self.node and len(data) >= 3:
            code, register = struct.unpack_from("<HB", data)
            byte1 = data[3] if len(data) > 3 else 0
            self.emcy = (self.emcy + [{"t": time.time(), "code": f"0x{code:04X}", "byte1": byte1,
                                       "class": self.c.emcy_class(code, byte1)}])[-8:]

    def state_name(self) -> str | None:
        if self.statusword is None:
            return None
        for name, (mask, value) in self.c["cia402"]["statusword"].items():
            if self.statusword & mask == value:
                return name.replace("_", " ")
        return "unknown"

    def snapshot(self) -> dict:
        alive = time.time() - self.last_heartbeat < self.c["heartbeat"]["master_consumes_drive_ms"] / 1000
        return {
            "node": self.node,
            "nmt": NMT_NAMES.get(self.nmt, self.nmt) if alive else "silent",
            "statusword": None if self.statusword is None else f"0x{self.statusword:04X}",
            "state": self.state_name(),
            "position_mm": None if self.counts is None else round(self.counts / self.c.counts_per_mm, 3),
            "emcy": self.emcy,
        }


class UdpInbox(asyncio.DatagramProtocol):
    def __init__(self):
        self.latest: dict | None = None
        self.received = 0.0

    def datagram_received(self, data, addr):
        try:
            self.latest, self.received = json.loads(data), time.time()
        except ValueError:
            pass

    def fresh(self) -> dict | None:
        return self.latest if time.time() - self.received < 1.0 else None


async def make_app(iface: str, node: int) -> web.Application:
    c = C.load()
    view = BusView(node, c)
    bus = can.Bus(interface="socketcan", channel=iface)
    notifier = can.Notifier(bus, [view])
    loop = asyncio.get_running_loop()
    master_status = UdpInbox()
    truth = UdpInbox()
    await loop.create_datagram_endpoint(lambda: master_status, local_addr=("127.0.0.1", STATUS_PORT))
    await loop.create_datagram_endpoint(lambda: truth, local_addr=("127.0.0.1", TRUTH_PORT))
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    config = {
        "stroke_mm": c["actuator"]["stroke_mm"],
        "soft_limits_mm": c["actuator"]["soft_limits_mm"],
        "closed_length_mm": 174.8,
        "iface": iface,
        "node": node,
    }

    async def index(_):
        return web.FileResponse(STATIC / "index.html")

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        await ws.send_json({"type": "config", **config})

        async def push():
            while not ws.closed:
                await ws.send_json({"type": "state", "bus": view.snapshot(), "master": master_status.fresh(),
                                    "truth": truth.fresh(), "frames": view.frames})
                await asyncio.sleep(0.05)

        pusher = asyncio.create_task(push())
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    cmd = json.loads(msg.data)
                    if cmd.get("type") in ("setpoint", "home", "stop", "resume"):
                        cmd_sock.sendto(json.dumps(cmd).encode(), MASTER_CMD)
        finally:
            pusher.cancel()
        return ws

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)

    async def cleanup(_):
        notifier.stop()
        bus.shutdown()

    app.on_cleanup.append(cleanup)
    return app


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    logging.getLogger("aiohttp.access").setLevel(logging.WARNING)
    web.run_app(make_app(args.iface, args.node), host=args.host, port=args.port, print=lambda *_: None)


if __name__ == "__main__":
    main()
