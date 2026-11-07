"""Loader for the drive contract (contract/drive-contract.yaml).

Single source of truth for both sides of the bus: the drive emulator uses it to know what a
correctly configured drive looks like, and the C master's configuration header is generated
from it (S2b). Sizes and signedness of every object come from the vendor EDS.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from pathlib import Path
from typing import Any

import canopen
import yaml
from canopen.objectdictionary import datatypes as dt

CONTRACT_DIR = Path(__file__).resolve().parent
CONTRACT_FILE = CONTRACT_DIR / "drive-contract.yaml"

PDO_DISABLED = 0x80000000
PDO_NO_RTR = 0x40000000  # manual: must always be set

_SIZES = {
    dt.BOOLEAN: (1, False),
    dt.INTEGER8: (1, True),
    dt.INTEGER16: (2, True),
    dt.INTEGER32: (4, True),
    dt.UNSIGNED8: (1, False),
    dt.UNSIGNED16: (2, False),
    dt.UNSIGNED32: (4, False),
}


@dataclass(frozen=True)
class SdoWrite:
    index: int
    sub: int
    value: int
    size: int  # bytes
    signed: bool
    note: str

    def to_bytes(self) -> bytes:
        return self.value.to_bytes(self.size, "little", signed=self.signed)

    def __str__(self) -> str:
        return f"{self.index:04X}h.{self.sub} = {self.value} ({self.size} B)  # {self.note}"


@dataclass(frozen=True)
class PdoEntry:
    index: int
    sub: int
    bits: int

    @property
    def mapping_value(self) -> int:
        return (self.index << 16) | (self.sub << 8) | self.bits


class Contract:
    def __init__(self, path: Path = CONTRACT_FILE):
        self.path = path
        self.data: dict[str, Any] = yaml.safe_load(path.read_text())

    def __getitem__(self, key: str) -> Any:
        return self.data[key]

    # --- referenced files -------------------------------------------------------------------
    @cached_property
    def od(self) -> canopen.ObjectDictionary:
        return canopen.import_od(str(self.path.parent / self.data["device"]["eds"]))

    @cached_property
    def defaults_overlay(self) -> dict[tuple[int, int], int]:
        raw = yaml.safe_load((self.path.parent / self.data["device"]["defaults_overlay"]).read_text())
        out = {}
        for key, value in raw.items():
            idx, _, sub = str(key).partition(".")
            out[(int(idx, 16), int(sub) if sub else 0)] = int(value)
        return out

    # --- EDS helpers ------------------------------------------------------------------------
    def variable(self, index: int, sub: int = 0) -> canopen.objectdictionary.ODVariable:
        obj = self.od[index]
        if isinstance(obj, canopen.objectdictionary.ODVariable):
            if sub != 0:
                raise KeyError(f"{index:04X}h is a variable, has no sub {sub}")
            return obj
        return obj[sub]

    def size_signed(self, index: int, sub: int = 0) -> tuple[int, bool]:
        var = self.variable(index, sub)
        if var.data_type not in _SIZES:
            raise ValueError(f"{index:04X}h.{sub}: unsupported data type 0x{var.data_type:04X}")
        return _SIZES[var.data_type]

    def write(self, index: int, sub: int, value: int, note: str) -> SdoWrite:
        size, signed = self.size_signed(index, sub)
        return SdoWrite(index, sub, int(value), size, signed, note)

    # --- units ------------------------------------------------------------------------------
    @property
    def counts_per_mm(self) -> int:
        u = self.data["units"]
        expected = u["full_steps_per_mm"] * (1 << u["microstep_exponent"])
        if u["counts_per_mm"] != expected:
            raise ValueError(f"units.counts_per_mm {u['counts_per_mm']} != {expected}")
        return expected

    def mm_to_counts(self, mm: float) -> int:
        return round(mm * self.counts_per_mm)

    def mm_s_to_pps(self, mm_s: float) -> int:
        return round(mm_s * self.counts_per_mm)

    # --- bus --------------------------------------------------------------------------------
    @property
    def node_ids(self) -> list[int]:
        return list(self.data["bus"]["drive_node_ids"])

    def pdo_entries(self, mapping: list[list[int]]) -> list[PdoEntry]:
        return [PdoEntry(i, s, self.size_signed(i, s)[0] * 8) for i, s in mapping]

    # --- startup sequence -------------------------------------------------------------------
    def startup_sequence(self, node_id: int) -> list[SdoWrite]:
        """Ordered SDO writes the master performs for one drive, in NMT PRE-OPERATIONAL with the
        drive in SWITCH_ON_DISABLED (required for 2005h and 208Ch)."""
        c, w = self.data, self.write
        hb, cur, prof, hom = c["heartbeat"], c["current"], c["profile"], c["homing"]
        lo, hi = c["actuator"]["soft_limits_mm"]
        period = c["bus"]["sync"]["period_ms"]
        seq = [
            w(0x1029, 1, 0, "comm error -> NMT stopped (drive faults)"),
            w(0x1017, 0, hb["drive_producer_ms"], "producer heartbeat"),
            w(0x1016, 1, (c["bus"]["master_node_id"] << 16) | hb["drive_consumes_master_ms"],
              "consume master heartbeat"),
            w(0x2005, 0, c["switches"], "no limit/home switches wired"),
            w(0x2000, 0, c["units"]["microstep_exponent"], "microsteps 2^n"),
            w(0x208C, 0, c["units"]["velocity_dimension"], "velocity unit PPS"),
            w(0x2093, 0, c["chopper"]["mode"], "spreadCycle"),
            w(0x20B0, 0, c["chopper"]["stealthchop_threshold"], "stealthChop off"),
            w(0x2003, 0, cur["run"], "run current"),
            w(0x2004, 0, cur["standby"], "standby current"),
            w(0x2089, 0, cur["standby_delay_10ms"], "standby delay"),
            w(0x607D, 1, self.mm_to_counts(lo), f"min soft limit {lo} mm"),
            w(0x607D, 2, self.mm_to_counts(hi), f"max soft limit {hi} mm"),
            w(0x6081, 0, self.mm_s_to_pps(prof["velocity_mm_s"]), "pp profile velocity"),
            w(0x6083, 0, self.mm_s_to_pps(prof["acceleration_mm_s2"]), "pp acceleration"),
            w(0x6084, 0, self.mm_s_to_pps(prof["deceleration_mm_s2"]), "pp deceleration"),
            w(0x6085, 0, self.mm_s_to_pps(prof["quick_stop_deceleration_mm_s2"]), "quick stop decel"),
            w(0x605A, 0, prof["quick_stop_option"], "quick stop option"),
            w(0x60C2, 1, period, f"csp interpolation period {period} ms"),
            w(0x60C2, 2, -3, "interpolation index 10^-3 s"),
            w(0x6098, 0, hom["method"], "homing method"),
            w(0x209E, 0, hom["stall_threshold"], "stallGuard threshold"),
            w(0x20A4, 0, self.mm_s_to_pps(hom["stop_on_stall_above_mm_s"]), "stop on stall above"),
        ]
        seq += self.pdo_sequence(node_id)
        return seq

    def pdo_sequence(self, node_id: int) -> list[SdoWrite]:
        out: list[SdoWrite] = []
        for kind, comm_base, map_base in (("rpdo", 0x1400, 0x1600), ("tpdo", 0x1800, 0x1A00)):
            for p in self.data["pdo"][kind]:
                comm, mapping = comm_base + p["num"] - 1, map_base + p["num"] - 1
                cob = p["base"] + node_id
                tag = f"{kind.upper()}{p['num']}"
                out.append(self.write(comm, 1, cob | PDO_DISABLED | PDO_NO_RTR, f"{tag} disable"))
                if not p.get("enabled", True):
                    continue
                entries = self.pdo_entries(p["map"])
                out.append(self.write(mapping, 0, 0, f"{tag} clear mapping"))
                for k, e in enumerate(entries, start=1):
                    out.append(self.write(mapping, k, e.mapping_value, f"{tag} map {e.index:04X}h.{e.sub}"))
                out.append(self.write(mapping, 0, len(entries), f"{tag} mapping count"))
                out.append(self.write(comm, 2, p["transmission"], f"{tag} transmission type"))
                if kind == "tpdo":
                    out.append(self.write(comm, 3, p.get("inhibit_100us", 0), f"{tag} inhibit time"))
                    out.append(self.write(comm, 5, p.get("event_timer_ms", 0), f"{tag} event timer"))
                out.append(self.write(comm, 1, cob | PDO_NO_RTR, f"{tag} enable, COB-ID 0x{cob:03X}"))
        return out

    # --- EMCY -------------------------------------------------------------------------------
    def emcy_class(self, code: int, byte1: int | None = None) -> str:
        fallback = None
        for e in self.data["emcy"]:
            if e["code"] != code:
                continue
            if "byte1" in e:
                if e["byte1"] == byte1:
                    return e["class"]
            else:
                fallback = e["class"]
        if fallback is None:
            return "stop_all"  # unknown codes are treated conservatively
        return fallback


def load(path: Path = CONTRACT_FILE) -> Contract:
    return Contract(path)
