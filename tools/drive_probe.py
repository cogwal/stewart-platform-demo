"""Characterise one TMCM-1270 (real or emulated) and write a JSON report. Run the same probe on the
bench and in the simulation, then --compare the reports: the differences are the emulator's tuning
list (docs/agents/sim-tuning.md). Commissioning procedure: docs/commissioning.md.

  python tools/drive_probe.py --iface can0 --node 1 --out build/probe-bench.json [--allow-motion]
  python tools/drive_probe.py --compare build/probe-sim.json build/probe-bench.json

Without --allow-motion only passive checks run (identity, object defaults, SDO behaviour, NMT,
heartbeat, state machine). With it, the actuator moves at most --travel-mm (default 10) away from
where it stands, positive direction first: unloaded actuator, clear travel, e-stop at hand.
--endstop additionally drives into the retracted end stop at homing speed to measure stallGuard.

The probe acts as master node 127 (or --master); nothing else may be master on the bus meanwhile.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import canopen  # noqa: E402

import contract as C  # noqa: E402
from sim.master.testmaster import PyMaster  # noqa: E402

# Objects whose power-on values we want to know (EDS defaults and the overlay are guesses until read).
READ_BACK = [
    (0x1000, 0), (0x1008, 0), (0x1009, 0), (0x100A, 0), (0x1017, 0), (0x1018, 1), (0x1018, 2), (0x1018, 3),
    (0x1029, 1), (0x2000, 0), (0x2003, 0), (0x2004, 0), (0x2005, 0), (0x2089, 0), (0x208C, 0), (0x2093, 0),
    (0x209E, 0), (0x20A4, 0), (0x20B0, 0), (0x2101, 0), (0x605A, 0), (0x6060, 0), (0x6061, 0), (0x607C, 0),
    (0x607D, 1), (0x607D, 2), (0x6081, 0), (0x6083, 0), (0x6084, 0), (0x6085, 0), (0x6098, 0), (0x60C2, 1),
    (0x60C2, 2), (0x1400, 1), (0x1401, 1), (0x1402, 1), (0x1403, 1), (0x1800, 1), (0x1800, 2), (0x1801, 1),
    (0x1802, 1), (0x1803, 1), (0x1A00, 0), (0x1A00, 1), (0x1A00, 2),
]

# What the comparison treats as "same": absolute or relative tolerance per measurement.
TOLERANCE = {"_default_rel": 0.15, "_default_abs_ms": 3.0}


class Probe:
    def __init__(self, iface: str, node: int, master: int):
        self.c = C.load()
        self.c.data["bus"]["master_node_id"] = master
        self.net = canopen.Network()
        self.net.connect(interface="socketcan", channel=iface)
        self.m = PyMaster(self.net, node, contract=self.c)
        self.node = node
        self.r: dict = {"node": node, "iface": iface, "contract": self.c["contract_version"], "t": time.time(),
                        "checks": {}, "measurements": {}, "objects": {}, "sdo_aborts": [], "emcy": []}

    # --- helpers ----------------------------------------------------------------------------
    def check(self, name: str, ok: bool, detail="") -> None:
        self.r["checks"][name] = {"ok": bool(ok), "detail": detail}
        print(f"  {'ok  ' if ok else 'FAIL'} {name} {detail}")

    def measure(self, name: str, value, unit: str = "") -> None:
        self.r["measurements"][name] = {"value": value, "unit": unit}
        print(f"  ..   {name} = {value} {unit}")

    def wait(self, cond, timeout: float) -> float | None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if cond():
                return time.monotonic() - t0
            time.sleep(0.001)
        return None

    def read(self, index: int, sub: int = 0):
        try:
            return self.m.sdo_read(index, sub)
        except canopen.SdoAbortedError as e:
            return f"abort 0x{e.code:08X}"
        except (canopen.SdoCommunicationError, KeyError, ValueError) as e:
            return f"error {e}"

    # --- passive ----------------------------------------------------------------------------
    def nmt_and_identity(self) -> None:
        print("NMT, boot-up, identity")
        self.m.start()
        self.m.boot_event.clear()
        t0 = time.monotonic()
        self.m.node.nmt.send_command(0x81)
        ok = self.m.boot_event.wait(5.0)
        self.check("boot-up after NMT reset node", ok)
        if ok:
            self.measure("bootup_after_reset_ms", round(1000 * (time.monotonic() - t0), 1), "ms")
        first_hb = self.wait(lambda: self.m.nmt_state == 0x7F, 3.0)
        # None = no heartbeat before 1017h is configured (power-on default 0)
        self.measure("first_heartbeat_after_bootup_ms", None if first_hb is None else round(1000 * first_hb, 1), "ms")
        ident = self.c["device"]["identity"]
        vendor, product, rev = self.read(0x1018, 1), self.read(0x1018, 2), self.read(0x1018, 3)
        self.check("vendor id", vendor == ident["vendor_id"], f"0x{vendor:X}" if isinstance(vendor, int) else vendor)
        self.check("product code", product == ident["product_code"], str(product))
        self.measure("revision", f"0x{rev:08X}" if isinstance(rev, int) else rev)

    def objects(self) -> None:
        print("object values after reset (vs EDS default / overlay)")
        for idx, sub in READ_BACK:
            v = self.read(idx, sub)
            key = f"{idx:04X}.{sub}"
            try:
                var = self.c.variable(idx, sub)
                expected = self.c.defaults_overlay.get((idx, sub), var.default)
            except KeyError:
                expected = None
            self.r["objects"][key] = {"value": v, "expected": expected}
            if expected is not None and isinstance(v, int) and v != expected:
                print(f"  diff {key}: read {v}, EDS/overlay {expected}")

    def startup_sequence(self) -> None:
        print("contract startup sequence (all writes, aborts recorded)")
        sw = self.read(0x6041)
        if isinstance(sw, int) and sw & 0x4F != 0x40:
            self.m.sdo_write(0x6040, 0, 0x80 if sw & 0x4F == 0x08 else 0)
        aborts = []
        for w in self.c.startup_sequence(self.node):
            try:
                self.m.node.sdo.download(w.index, w.sub, w.to_bytes())
            except canopen.SdoAbortedError as e:
                aborts.append({"object": f"{w.index:04X}.{w.sub}", "value": w.value, "abort": f"0x{e.code:08X}"})
        self.r["sdo_aborts"] = aborts
        self.check("startup sequence accepted", not aborts, f"{len(aborts)} aborts")

    def sdo_state_rules(self) -> None:
        print("SDO rules: 2005h only in SWITCH ON DISABLED")
        self.m.node.nmt.send_command(0x01)
        self.wait(lambda: self.m.nmt_state == 0x05, 1.0)
        self.m.enable(self.c["cia402"]["modes"]["profile_position"])
        try:
            self.m.sdo_write(0x2005, 0, self.c["switches"])
            result = "accepted"
        except canopen.SdoAbortedError as e:
            result = f"abort 0x{e.code:08X}"
        self.measure("write_2005_in_operation_enabled", result)
        self.m.rpdo2(0, self.m.mode)
        self.wait(lambda: self.m.in_state("switch_on_disabled"), 0.5)

    def state_machine_timing(self) -> None:
        print("CiA 402 transition latency (RPDO2 -> statusword via TPDO2)")
        cw, mode = self.c["cia402"]["controlword"], self.c["cia402"]["modes"]["profile_position"]
        lat = {}
        for name, word, state in (("shutdown", cw["shutdown"], "ready_to_switch_on"),
                                  ("switch_on", cw["switch_on"], "switched_on"),
                                  ("enable_operation", cw["enable_operation"], "operation_enabled"),
                                  ("disable_voltage", cw["disable_voltage"], "switch_on_disabled")):
            self.m.rpdo2(word, mode)
            dt = self.wait(lambda: self.m.in_state(state), 0.5)
            lat[name] = None if dt is None else round(1000 * dt, 2)
        for k, v in lat.items():
            self.measure(f"transition_{k}_ms", v, "ms")

    def heartbeat_timing(self) -> None:
        print("heartbeat period (2 s)")
        stamps = []
        self.net.subscribe(0x700 + self.node, lambda *_: stamps.append(time.monotonic()))
        time.sleep(2.0)
        gaps = [1000 * (b - a) for a, b in zip(stamps, stamps[1:])]
        if gaps:
            self.measure("heartbeat_period_mean_ms", round(statistics.mean(gaps), 2), "ms")
            self.measure("heartbeat_period_max_ms", round(max(gaps), 2), "ms")

    def tpdo_timing(self) -> None:
        print("TPDO1 period with SYNC every period (2 s)")
        stamps = []
        self.net.subscribe(0x180 + self.node, lambda *_: stamps.append(time.monotonic()))
        time.sleep(2.0)
        gaps = [1000 * (b - a) for a, b in zip(stamps, stamps[1:])]
        if gaps:
            self.measure("tpdo1_period_mean_ms", round(statistics.mean(gaps), 3), "ms")
            self.measure("tpdo1_period_max_ms", round(max(gaps), 3), "ms")
        self.measure("tpdo1_per_sync", round(len(stamps) / (2.0 / self.m.period), 2))

    def heartbeat_loss(self) -> None:
        print("master heartbeat loss -> drive reaction")
        n0 = len(self.m.emcys)
        self.m.stop()  # stops master heartbeat and SYNC
        t0 = time.monotonic()
        dt_emcy = self.wait(lambda: any(e[1] == 0x8130 for e in self.m.emcys[n0:]), 2.0)
        dt_stop = self.wait(lambda: self.m.nmt_state == 0x04, 2.0)
        self.measure("hb_loss_to_emcy_8130_ms", None if dt_emcy is None else round(1000 * dt_emcy, 1), "ms")
        self.measure("hb_loss_to_nmt_stopped_ms", None if dt_stop is None else round(1000 * (time.monotonic() - t0), 1), "ms")
        self.measure("statusword_after_hb_loss", f"0x{self.read(0x6041):04X}" if isinstance(self.read(0x6041), int) else None)
        self.m.start()

    # --- motion -----------------------------------------------------------------------------
    def motion(self, travel_mm: float) -> None:
        print(f"motion (max {travel_mm} mm from the current position)")
        self.m.boot()  # clean state, contract config, OPERATIONAL
        cpm = self.c.counts_per_mm
        modes, cw = self.c["cia402"]["modes"], self.c["cia402"]["controlword"]
        # The step counter starts at 0 after boot; work around it in relative terms.
        # Soft limits would reject targets below 0.5 mm, so disable them for the probe.
        self.m.rpdo2(0, modes["profile_position"])
        self.m.sdo_write(0x607D, 1, -10 * cpm * 100)
        self.m.sdo_write(0x607D, 2, 10 * cpm * 100)

        # pp move: duration vs trapezoid theory
        prof = self.c["profile"]
        self.m.enable(modes["profile_position"])
        start = self.m.position
        target = start + round(travel_mm * cpm)
        self.m.rpdo1(cw["enable_operation"], target)
        t0 = time.monotonic()
        self.m.rpdo1(cw["enable_operation"] | cw["new_set_point"], target)
        ack = self.wait(lambda: self.m.statusword & 0x1000, 1.0)
        self.m.rpdo1(cw["enable_operation"], target)
        done = self.wait(lambda: self.m.statusword & 0x0400 and self.m.position == target, 10.0)
        v, a = prof["velocity_mm_s"], prof["acceleration_mm_s2"]
        t_acc = v / a
        theory = (2 * t_acc + (travel_mm - v * t_acc) / v) if travel_mm > v * t_acc else 2 * math.sqrt(travel_mm / a)
        self.measure("pp_setpoint_ack_ms", None if ack is None else round(1000 * ack, 1), "ms")
        self.measure("pp_move_s", None if done is None else round(time.monotonic() - t0, 3), "s")
        self.measure("pp_move_theory_s", round(theory, 3), "s")

        # pv: velocity accuracy over 1 s, back toward the start
        self.m.enable(modes["profile_velocity"])
        v_cmd = -min(4.0, travel_mm / 2.0)
        self.m.rpdo3(cw["enable_operation"], self.c.mm_s_to_pps(v_cmd))
        time.sleep(0.5)
        p0, t0 = self.m.position, time.monotonic()
        time.sleep(1.0)
        p1, t1 = self.m.position, time.monotonic()
        loads = []
        for _ in range(5):
            val = self.read(0x2101)
            if isinstance(val, int):
                loads.append(val)
        self.m.rpdo3(cw["enable_operation"], 0)
        self.measure("pv_velocity_mm_s", round((p1 - p0) / cpm / (t1 - t0), 3), "mm/s")
        self.measure("pv_commanded_mm_s", v_cmd, "mm/s")
        self.measure("stallguard_load_moving", loads)
        time.sleep(0.3)
        standstill = [self.read(0x2101) for _ in range(3)]
        self.measure("stallguard_load_standstill", standstill)

        # csp: stream a 1 Hz sine of +-2 mm around the current position, measure tracking
        self.m.enable(modes["cyclic_sync_position"])
        base = self.m.position
        errs = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            t = time.monotonic() - t0
            tgt = base + round(2.0 * cpm * math.sin(2 * math.pi * t))
            self.m.rpdo1(cw["enable_operation"], tgt)
            errs.append((tgt - self.m.position) / cpm)
            time.sleep(self.m.period)
        self.measure("csp_tracking_rms_mm", round(math.sqrt(sum(e * e for e in errs[20:]) / max(1, len(errs[20:]))), 4), "mm")

        # quick stop from pv at 4 mm/s: stopping distance vs 6085h
        self.m.enable(modes["profile_velocity"])
        self.m.rpdo3(cw["enable_operation"], self.c.mm_s_to_pps(4.0))
        time.sleep(0.6)
        p0 = self.m.position
        self.m.rpdo1(cw["quick_stop"], self.m.position)
        self.wait(lambda: self.m.in_state("switch_on_disabled"), 1.0)
        time.sleep(0.2)
        qs = (self.m.position - p0) / cpm
        qd = prof["quick_stop_deceleration_mm_s2"]
        self.measure("quick_stop_distance_mm", round(qs, 3), "mm")
        self.measure("quick_stop_distance_theory_mm", round(4.0 ** 2 / (2 * qd), 3), "mm")
        self.m.rpdo2(0, modes["profile_position"])

    def endstop(self) -> None:
        print("stallGuard at the retracted end stop (homing as the master does it)")
        n0 = len(self.m.emcys)
        t0 = time.monotonic()
        self.m.boot()
        self.m.home()
        self.measure("homing_s", round(time.monotonic() - t0, 2), "s")
        self.measure("homing_emcys", [f"0x{e[1]:04X}" for e in self.m.emcys[n0:]])
        self.check("homed", self.m.homed)

    def run(self, allow_motion: bool, travel: float, endstop: bool) -> dict:
        try:
            self.nmt_and_identity()
            self.objects()
            self.startup_sequence()
            self.sdo_state_rules()
            self.state_machine_timing()
            self.heartbeat_timing()
            self.tpdo_timing()
            self.heartbeat_loss()
            if allow_motion:
                self.motion(travel)
                if endstop:
                    self.endstop()
        finally:
            self.r["emcy"] = [{"code": f"0x{e[1]:04X}", "byte1": e[2], "class": e[3]} for e in self.m.emcys]
            self.m.stop()
            self.net.disconnect()
        return self.r


def compare(a_path: Path, b_path: Path) -> int:
    a, b = json.loads(a_path.read_text()), json.loads(b_path.read_text())
    print(f"A = {a_path}  B = {b_path}")
    diffs = 0
    for section in ("checks", "measurements", "objects"):
        keys = sorted(set(a.get(section, {})) | set(b.get(section, {})))
        for k in keys:
            va = a.get(section, {}).get(k, {})
            vb = b.get(section, {}).get(k, {})
            xa, xb = va.get("value", va.get("ok")), vb.get("value", vb.get("ok"))
            same = xa == xb
            if not same and isinstance(xa, (int, float)) and isinstance(xb, (int, float)) and not isinstance(xa, bool):
                unit = va.get("unit", "")
                tol = TOLERANCE["_default_abs_ms"] if unit == "ms" else 0.0
                same = abs(xa - xb) <= max(tol, TOLERANCE["_default_rel"] * max(abs(xa), abs(xb)))
            if not same:
                diffs += 1
                print(f"  {section}/{k}: A={xa}  B={xb}")
    if a.get("sdo_aborts") != b.get("sdo_aborts"):
        diffs += 1
        print(f"  sdo_aborts: A={a.get('sdo_aborts')}  B={b.get('sdo_aborts')}")
    print(f"{diffs} differences")
    return 1 if diffs else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--master", type=int, default=127)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--allow-motion", action="store_true")
    ap.add_argument("--travel-mm", type=float, default=10.0)
    ap.add_argument("--endstop", action="store_true")
    ap.add_argument("--compare", nargs=2, type=Path, metavar=("A", "B"))
    args = ap.parse_args()
    if args.compare:
        return compare(*args.compare)
    report = Probe(args.iface, args.node, args.master).run(args.allow_motion, args.travel_mm, args.endstop)
    text = json.dumps(report, indent=2, default=str)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
        print(f"report written to {args.out}")
    failed = [k for k, v in report["checks"].items() if not v["ok"]]
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
