"""Python test master for one drive, following the contract. Stand-in until the C master (S2b/S3)
takes over the same role and the same UDP operator interface.

Operator interface (UDP, JSON, localhost):
  commands  -> :5005   {"type": "setpoint", "mm": 42.0} | {"type": "home"} | {"type": "stop"} | {"type": "resume"}
  status    <- :5006   {"source": "master", "phase": ..., "homed": ..., "setpoint_mm": ..., "target_mm": ...}
"""
from __future__ import annotations

import json
import logging
import math
import socket
import struct
import threading
import time

import canopen

import contract as C

log = logging.getLogger(__name__)

CMD_PORT, STATUS_PORT = 5005, 5006


class MasterError(RuntimeError):
    pass


class PyMaster:
    def __init__(self, network: canopen.Network, node_id: int, contract: C.Contract | None = None):
        self.c = contract or C.load()
        self.net = network
        self.id = node_id
        od = canopen.import_od(str(self.c.path.parent / self.c["device"]["eds"]), node_id=node_id)
        self.node = canopen.RemoteNode(node_id, od)
        self.node.sdo.RESPONSE_TIMEOUT = self.c["timeouts_ms"]["sdo"] / 1000
        network.add_node(self.node)

        cia = self.c["cia402"]
        self.cw, self.states, self.modes = cia["controlword"], cia["statusword"], cia["modes"]
        self.period = self.c["bus"]["sync"]["period_ms"] / 1000
        self.lock = threading.RLock()

        self.statusword = 0
        self.position = 0
        self.nmt_state: int | None = None
        self.last_heartbeat = 0.0
        self.boot_event = threading.Event()
        self.emcys: list[tuple[float, int, int, str]] = []
        self.phase = "starting"
        self.homed = False
        self.mode = 0

        self.csp_active = False
        self.setpoint_mm = 0.0
        self.x_mm = 0.0
        self.v_mm_s = 0.0
        self._running = False

        network.subscribe(0x700 + node_id, self._on_heartbeat)
        network.subscribe(0x180 + node_id, self._on_tpdo1)
        network.subscribe(0x280 + node_id, self._on_tpdo2)
        network.subscribe(0x080 + node_id, self._on_emcy)

    # --- CAN receive ------------------------------------------------------------------------
    def _on_heartbeat(self, can_id, data, ts):
        state = data[0] & 0x7F
        if state == 0x00:
            if self.nmt_state not in (None, 0x00) and self.phase not in ("booting",):
                log.warning("drive %d rebooted (boot-up while %s)", self.id, self.phase)
                self._lose_homing("drive rebooted: press Home")
            self.boot_event.set()
        self.nmt_state = state
        self.last_heartbeat = time.monotonic()

    def _on_tpdo1(self, can_id, data, ts):
        if len(data) == 6:
            self.statusword, self.position = struct.unpack("<Hi", bytes(data))

    def _on_tpdo2(self, can_id, data, ts):
        if len(data) == 3:
            self.statusword, mode = struct.unpack("<Hb", bytes(data))

    def _on_emcy(self, can_id, data, ts):
        code, register = struct.unpack_from("<HB", bytes(data))
        byte1 = data[3] if len(data) > 3 else 0
        cls = self.c.emcy_class(code, byte1)
        self.emcys.append((time.time(), code, byte1, cls))
        log.info("EMCY 0x%04X/%d -> %s", code, byte1, cls)
        if cls in ("info", "warn") or (cls == "homing_stall" and self.phase == "homing"):
            return
        if cls in ("stop_all_rehome", "stall", "homing_stall"):
            self._lose_homing(f"EMCY 0x{code:04X}: press Home")
        else:
            self.csp_active = False
            self.phase = f"stopped: EMCY 0x{code:04X} ({cls})"

    def _lose_homing(self, why: str) -> None:
        self.csp_active = False
        self.homed = False
        self.phase = why

    # --- state helpers ----------------------------------------------------------------------
    def in_state(self, name: str) -> bool:
        mask, value = self.states[name]
        return self.statusword & mask == value

    def wait(self, cond, timeout: float, what: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cond():
                return
            time.sleep(0.002)
        raise MasterError(f"timeout waiting for {what} (statusword 0x{self.statusword:04X})")

    def wait_state(self, name: str) -> None:
        self.wait(lambda: self.in_state(name), self.c["timeouts_ms"]["state_transition"] / 1000, name)

    def sdo_read(self, index: int, sub: int = 0) -> int:
        size, signed = self.c.size_signed(index, sub)
        return int.from_bytes(self.node.sdo.upload(index, sub), "little", signed=signed)

    def sdo_write(self, index: int, sub: int, value: int) -> None:
        size, signed = self.c.size_signed(index, sub)
        self.node.sdo.download(index, sub, value.to_bytes(size, "little", signed=signed))

    # --- RPDOs ------------------------------------------------------------------------------
    def rpdo1(self, cw: int, target: int) -> None:
        self.net.send_message(0x200 + self.id, struct.pack("<Hi", cw, target))

    def rpdo2(self, cw: int, mode: int) -> None:
        self.mode = mode
        self.net.send_message(0x300 + self.id, struct.pack("<Hb", cw, mode))

    def rpdo3(self, cw: int, velocity: int) -> None:
        self.net.send_message(0x400 + self.id, struct.pack("<Hi", cw, velocity))

    def enable(self, mode: int) -> None:
        if self.in_state("fault"):
            self.rpdo2(0x0000, mode)
            self.rpdo2(self.cw["fault_reset"], mode)
            self.wait_state("switch_on_disabled")
        self.rpdo2(self.cw["shutdown"], mode)
        self.wait_state("ready_to_switch_on")
        self.rpdo2(self.cw["switch_on"], mode)
        self.wait_state("switched_on")
        self.rpdo2(self.cw["enable_operation"], mode)
        self.wait_state("operation_enabled")

    # --- boot -------------------------------------------------------------------------------
    def boot(self) -> None:
        self.phase = "booting"
        self.csp_active = False
        self.homed = False
        self.boot_event.clear()
        self.node.nmt.send_command(0x81)  # reset node: known state
        if not self.boot_event.wait(self.c["timeouts_ms"]["boot_up"] / 1000):
            raise MasterError(f"no boot-up from node {self.id}")

        ident = self.c["device"]["identity"]
        vendor, product, rev = self.sdo_read(0x1018, 1), self.sdo_read(0x1018, 2), self.sdo_read(0x1018, 3)
        if vendor != ident["vendor_id"] or product != ident["product_code"]:
            raise MasterError(f"unexpected device vendor 0x{vendor:X} product {product}")
        if rev and rev < ident["min_revision"]:
            log.warning("drive firmware revision 0x%X older than contract 0x%X", rev, ident["min_revision"])

        self.statusword = self.sdo_read(0x6041)
        if self.in_state("fault"):
            self.sdo_write(0x6040, 0, 0)
            self.sdo_write(0x6040, 0, self.cw["fault_reset"])
        elif not self.in_state("switch_on_disabled"):
            self.sdo_write(0x6040, 0, self.cw["disable_voltage"])

        seq = self.c.startup_sequence(self.id)
        for w in seq:
            self.node.sdo.download(w.index, w.sub, w.to_bytes())
        log.info("node %d configured (%d SDO writes)", self.id, len(seq))
        self.node.nmt.send_command(0x01)
        self.wait(lambda: self.nmt_state == 0x05, 1.0, "NMT operational")
        self.phase = "configured"

    # --- homing (contract: retract into the end stop, stall or timeout, then method 35) ------
    def home(self) -> None:
        h, cur = self.c["homing"], self.c["current"]
        self.phase = "homing"
        self.csp_active = False
        self.homed = False
        self.sdo_write(0x2003, 0, cur["homing"])
        self.enable(self.modes["profile_velocity"])
        n_emcy = len(self.emcys)
        self.rpdo3(self.cw["enable_operation"], -self.c.mm_s_to_pps(h["velocity_mm_s"]))
        start = time.monotonic()
        reason = "timeout (blind overtravel)"
        while time.monotonic() - start < h["timeout_s"]:
            time.sleep(h["load_poll_ms"] / 1000)
            if any(e[1] == 0x3230 for e in self.emcys[n_emcy:]):
                reason = "stallGuard EMCY 3230h"
                break
            if self.in_state("fault"):
                reason = "drive fault"
                break
            if time.monotonic() - start > 0.3 and self.sdo_read(0x2101) <= h["load_stall_threshold"]:
                reason = "stallGuard load"
                break
        self.rpdo3(self.cw["enable_operation"], 0)
        log.info("end stop reached: %s after %.1f s", reason, time.monotonic() - start)
        time.sleep(3 * self.period)  # let TPDO1 deliver the statusword after a stall fault

        self.enable(self.modes["homing"])  # includes fault reset if the stall faulted the drive
        self.rpdo2(self.cw["enable_operation"] | self.cw["new_set_point"], self.modes["homing"])
        self.wait(lambda: self.statusword & 0x1000, 1.0, "homing attained")
        self.rpdo2(self.cw["enable_operation"], self.modes["homing"])
        self.sdo_write(0x2003, 0, cur["run"])

        park = self.c.mm_to_counts(h["park_mm"])
        self.enable(self.modes["profile_position"])
        self.rpdo1(self.cw["enable_operation"], park)
        self.rpdo1(self.cw["enable_operation"] | self.cw["new_set_point"], park)
        self.wait(lambda: self.statusword & 0x1000, 1.0, "set-point acknowledged")
        self.rpdo1(self.cw["enable_operation"], park)
        prof = self.c["profile"]
        travel_s = h["park_mm"] / prof["velocity_mm_s"] + 2 * prof["velocity_mm_s"] / prof["acceleration_mm_s2"]
        self.wait(lambda: self.statusword & 0x0400 and self.position == park, travel_s + 2.0, "park position")
        self._start_csp(h["park_mm"])
        self.homed = True
        self.phase = "following setpoint"

    def _start_csp(self, at_mm: float) -> None:
        self.x_mm = self.setpoint_mm = at_mm
        self.v_mm_s = 0.0
        self.enable(self.modes["cyclic_sync_position"])
        self.csp_active = True

    # --- operator commands ------------------------------------------------------------------
    def set_setpoint(self, mm: float) -> None:
        lo, hi = self.c["actuator"]["soft_limits_mm"]
        self.setpoint_mm = min(max(float(mm), lo), hi)

    def quick_stop(self) -> None:
        self.csp_active = False
        self.rpdo1(self.cw["quick_stop"], self.position)
        self.phase = "stopped by operator"

    def resume(self) -> None:
        if not self.homed:
            raise MasterError("not homed")
        self._start_csp(self.position / self.c.counts_per_mm)
        self.phase = "following setpoint"

    # --- cyclic part: SYNC + csp target every period ----------------------------------------
    def _step_trajectory(self) -> None:
        vmax, amax, dt = self.c["csp"]["max_velocity_mm_s"], self.c["csp"]["max_acceleration_mm_s2"], self.period
        err = self.setpoint_mm - self.x_mm
        v_cmd = math.copysign(min(vmax, math.sqrt(2 * amax * abs(err))), err)
        self.v_mm_s = min(max(v_cmd, self.v_mm_s - amax * dt), self.v_mm_s + amax * dt)
        self.x_mm += self.v_mm_s * dt
        if abs(self.setpoint_mm - self.x_mm) < 1e-4 and abs(self.v_mm_s) < amax * dt:
            self.x_mm, self.v_mm_s = self.setpoint_mm, 0.0

    def _cycle(self) -> None:
        next_t = time.monotonic()
        while self._running:
            next_t += self.period
            self.net.send_message(0x080, b"")  # SYNC
            if self.csp_active:
                self._step_trajectory()
                self.rpdo1(self.cw["enable_operation"], self.c.mm_to_counts(self.x_mm))
            hb_timeout = self.c["heartbeat"]["master_consumes_drive_ms"] / 1000
            if self.homed and time.monotonic() - self.last_heartbeat > hb_timeout:
                log.warning("drive %d heartbeat lost", self.id)
                self._lose_homing("drive lost (heartbeat): press Home")
            time.sleep(max(0.0, next_t - time.monotonic()))

    def start(self) -> None:
        self._running = True
        hb = self.c["heartbeat"]["master_producer_ms"] / 1000
        self._hb_task = self.net.send_periodic(0x700 + self.c["bus"]["master_node_id"], [0x05], hb)
        threading.Thread(target=self._cycle, name="master-cycle", daemon=True).start()

    def stop(self) -> None:
        """Stop SYNC and the master heartbeat (idempotent)."""
        self._running = False
        task, self._hb_task = getattr(self, "_hb_task", None), None
        if task is not None:
            task.stop()

    def status(self) -> dict:
        return {
            "source": "master",
            "node": self.id,
            "phase": self.phase,
            "homed": self.homed,
            "csp": self.csp_active,
            "setpoint_mm": round(self.setpoint_mm, 3),
            "target_mm": round(self.x_mm, 3),
            "emcy": [{"t": t, "code": f"0x{code:04X}", "byte1": b1, "class": cls} for t, code, b1, cls in self.emcys[-8:]],
        }


def serve(master: PyMaster, status_host: str = "127.0.0.1") -> None:
    """UDP operator interface. Commands are run in a worker thread so homing doesn't block."""
    cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd.bind(("127.0.0.1", CMD_PORT))
    cmd.settimeout(0.1)
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    busy = threading.Lock()

    def run(fn, *args):
        def worker():
            if not busy.acquire(blocking=False):
                return
            try:
                fn(*args)
            except MasterError as e:
                log.error("%s", e)
                master.phase = f"error: {e}"
            finally:
                busy.release()
        threading.Thread(target=worker, daemon=True).start()

    last_status = 0.0
    while True:
        try:
            data, _ = cmd.recvfrom(4096)
            msg = json.loads(data)
            kind = msg.get("type")
            if kind == "setpoint":
                master.set_setpoint(msg["mm"])
            elif kind == "home":
                run(lambda: (master.boot(), master.home()))
            elif kind == "stop":
                master.quick_stop()
            elif kind == "resume":
                run(master.resume)
        except socket.timeout:
            pass
        except (ValueError, KeyError) as e:
            log.warning("bad command: %s", e)
        if time.monotonic() - last_status >= 0.1:
            last_status = time.monotonic()
            out.sendto(json.dumps(master.status()).encode(), (status_host, STATUS_PORT))
