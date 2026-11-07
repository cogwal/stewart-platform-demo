"""Checks that the drive contract is consistent with the vendor EDS, the firmware manual's limits
and basic CANopen bus rules. Runs without a CAN bus."""
import math

import pytest

import contract as C

c = C.load()
ALL_NODES = [1, 2, 3, 4, 5, 6]  # check the 6-drive future too, not just v0
WRITABLE = {"rw", "rww", "wo"}
READABLE = {"ro", "rw", "rwr", "rww", "const"}


@pytest.fixture(scope="module", params=ALL_NODES)
def sequence(request):
    return request.param, c.startup_sequence(request.param)


# --- SDO startup sequence --------------------------------------------------------------------
def test_every_write_targets_a_writable_eds_object(sequence):
    _, seq = sequence
    for w in seq:
        var = c.variable(w.index, w.sub)  # KeyError if missing from the EDS
        assert var.access_type in WRITABLE, f"{w}: access {var.access_type}"


def test_values_fit_their_eds_type(sequence):
    _, seq = sequence
    for w in seq:
        bits = w.size * 8
        lo, hi = (-(1 << (bits - 1)), (1 << (bits - 1)) - 1) if w.signed else (0, (1 << bits) - 1)
        assert lo <= w.value <= hi, str(w)
        w.to_bytes()


MANUAL_RANGES = {  # (index, sub): (min, max) from the firmware manual
    (0x2000, 0): (0, 8),
    (0x2003, 0): (0, 255),
    (0x2004, 0): (0, 255),
    (0x2005, 0): (0, 63),
    (0x2089, 0): (0, 400),
    (0x2093, 0): (0, 1),
    (0x209E, 0): (-63, 63),
    (0x20B0, 0): (0, 16777215),
    (0x605A, 0): (1, 6),
    (0x60C2, 2): (-3, 3),
}


def test_values_within_manual_ranges():
    for w in c.startup_sequence(1):
        if (w.index, w.sub) in MANUAL_RANGES:
            lo, hi = MANUAL_RANGES[(w.index, w.sub)]
            assert lo <= w.value <= hi, str(w)
    assert {w.value for w in c.startup_sequence(1) if w.index == 0x208C} <= {0, 181}
    assert {w.value for w in c.startup_sequence(1) if w.index == 0x605A} <= {1, 2, 5, 6}


def test_current_stays_within_s20_limit():
    # 0.6 A per phase taken as peak -> 0.424 A RMS; drive full scale 1.2 A RMS (worst case of HW vs FW manual)
    limit = math.floor(0.6 / math.sqrt(2) / 1.2 * 255)
    cur = c["current"]
    for key in ("run", "homing", "standby"):
        assert cur[key] <= limit, f"current.{key}={cur[key]} > {limit}"
    assert cur["standby"] <= cur["homing"] <= cur["run"]


# --- PDOs ------------------------------------------------------------------------------------
def all_pdos():
    for kind in ("rpdo", "tpdo"):
        for p in c["pdo"][kind]:
            yield kind, p


def test_pdo_mapping_limits():
    for kind, p in all_pdos():
        if not p.get("enabled", True):
            continue
        entries = c.pdo_entries(p["map"])
        assert len(entries) <= 3, f"{kind}{p['num']}: drive allows max 3 entries"
        assert sum(e.bits for e in entries) <= 64, f"{kind}{p['num']}: more than 8 bytes"
        for e in entries:
            var = c.variable(e.index, e.sub)
            assert var.pdo_mappable, f"{kind}{p['num']}: {e.index:04X}h not PDO-mappable in the EDS"
            allowed = WRITABLE if kind == "rpdo" else READABLE
            assert var.access_type in allowed, f"{kind}{p['num']}: {e.index:04X}h access {var.access_type}"


def test_pdo_transmission_types_supported_by_drive():
    for kind, p in all_pdos():
        if p.get("enabled", True):
            t = p["transmission"]
            assert t in (0xFE, 0xFF) or 1 <= t <= 240, f"{kind}{p['num']}: type {t}"


def test_pdo_numbers_and_bases_match_cia301_defaults():
    defaults = {"rpdo": [0x200, 0x300, 0x400, 0x500], "tpdo": [0x180, 0x280, 0x380, 0x480]}
    for kind, bases in defaults.items():
        pdos = c["pdo"][kind]
        assert [p["num"] for p in pdos] == [1, 2, 3, 4]
        assert [p["base"] for p in pdos] == bases


def test_no_cob_id_collisions_on_a_six_drive_bus():
    master = c["bus"]["master_node_id"]
    used: dict[int, str] = {0x000: "NMT", c["bus"]["sync"]["cob_id"]: "SYNC"}

    def claim(cob, what):
        assert cob not in used, f"COB-ID 0x{cob:03X}: {what} collides with {used[cob]}"
        used[cob] = what

    for node in ALL_NODES + [master]:
        claim(0x080 + node, f"EMCY {node}")
        claim(0x580 + node, f"SDO tx {node}")
        claim(0x600 + node, f"SDO rx {node}")
        claim(0x700 + node, f"heartbeat {node}")
    for node in ALL_NODES:
        for kind, p in all_pdos():
            if p.get("enabled", True):
                claim(p["base"] + node, f"{kind}{p['num']} node {node}")


def test_pdo_sequence_follows_manual_procedure():
    """Disable, clear count, map, set count, then enable (manual §3.1.19)."""
    seq = c.pdo_sequence(1)
    for kind, comm_base, map_base in (("rpdo", 0x1400, 0x1600), ("tpdo", 0x1800, 0x1A00)):
        for p in c["pdo"][kind]:
            comm, mapping = comm_base + p["num"] - 1, map_base + p["num"] - 1
            steps = [w for w in seq if w.index in (comm, mapping)]
            assert steps[0].index == comm and steps[0].sub == 1 and steps[0].value & C.PDO_DISABLED
            assert all(w.value & C.PDO_NO_RTR for w in steps if w.index == comm and w.sub == 1)
            if p.get("enabled", True):
                assert steps[1].index == mapping and steps[1].sub == 0 and steps[1].value == 0
                assert steps[-1].index == comm and steps[-1].sub == 1
                assert not steps[-1].value & C.PDO_DISABLED
                assert (steps[-1].value & 0x7FF) == p["base"] + 1
            else:
                assert len(steps) == 1


def frame_bits(data_bytes: int) -> float:
    """Classic CAN, 11-bit ID: 47 bits overhead + data, plus ~20 % bit stuffing."""
    return (47 + 8 * data_bytes) * 1.2


def test_bus_load_for_six_drives_fits_budget():
    period_s = c["bus"]["sync"]["period_ms"] / 1000
    per_cycle = frame_bits(0)  # SYNC
    for kind, p in all_pdos():
        if p.get("enabled", True) and (
            (kind == "tpdo" and 1 <= p["transmission"] <= 240) or (kind == "rpdo" and p["num"] == 1)
        ):
            size = sum(e.bits for e in c.pdo_entries(p["map"])) // 8
            per_cycle += frame_bits(size) * len(ALL_NODES)
    heartbeats = (len(ALL_NODES) + 1) * frame_bits(1) / (c["heartbeat"]["drive_producer_ms"] / 1000)
    load = (per_cycle / period_s + heartbeats) / c["bus"]["bitrate"]
    assert load < 0.30, f"bus load {load:.0%} for 6 drives"


# --- timing, units, identity -----------------------------------------------------------------
def test_heartbeat_consumers_tolerate_producer_jitter():
    hb = c["heartbeat"]
    assert hb["drive_consumes_master_ms"] >= 2 * hb["master_producer_ms"]
    assert hb["master_consumes_drive_ms"] >= 2 * hb["drive_producer_ms"]
    assert hb["drive_consumes_master_ms"] <= 0xFFFF


def test_csp_period_matches_sync():
    seq = {(w.index, w.sub): w.value for w in c.startup_sequence(1)}
    assert seq[(0x60C2, 1)] * 10 ** seq[(0x60C2, 2)] == pytest.approx(c["bus"]["sync"]["period_ms"] / 1000)


def test_units_and_limits():
    assert c.counts_per_mm == 25600
    lo, hi = c["actuator"]["soft_limits_mm"]
    assert 0 < lo < hi < c["actuator"]["stroke_mm"]
    assert c.mm_to_counts(c["actuator"]["stroke_mm"]) < 2**31


def test_homing_timeout_guarantees_overtravel():
    h = c["homing"]
    assert h["timeout_s"] * h["velocity_mm_s"] >= 1.05 * c["actuator"]["stroke_mm"]
    assert h["stop_on_stall_above_mm_s"] < h["velocity_mm_s"], "stallGuard stop must be active at homing speed"
    assert h["method"] == 35


def test_identity_matches_eds():
    ident = c["device"]["identity"]
    assert c.variable(0x1018, 1).default == ident["vendor_id"]
    assert c.variable(0x1018, 2).default == ident["product_code"]
    assert c.od.device_information.revision_number == ident["min_revision"]


def test_eds_supports_the_modes_we_use():
    supported = c.variable(0x6502).default
    bit = {1: 0, 3: 2, 6: 5, 8: 7}
    for name, mode in c["cia402"]["modes"].items():
        assert supported & (1 << bit[mode]), f"{name} ({mode}) not in 6502h=0x{supported:X}"


# --- defaults overlay ------------------------------------------------------------------------
def test_defaults_overlay_targets_existing_objects():
    for (index, sub), value in c.defaults_overlay.items():
        var = c.variable(index, sub)
        size, signed = c.size_signed(index, sub)
        value.to_bytes(size, "little", signed=signed)
        if var.default is not None and (index, sub) != (0x60C2, 2):  # EDS stores -3 as 0xFD
            assert var.default == value, f"{index:04X}h.{sub}: overlay {value} != EDS {var.default}"


# --- EMCY ------------------------------------------------------------------------------------
CLASSES = {"info", "warn", "homing_stall", "stall", "stop_all", "stop_all_rehome", "config_error"}


def test_emcy_table():
    seen = set()
    for e in c["emcy"]:
        assert e["class"] in CLASSES, e
        key = (e["code"], e.get("byte1"))
        assert key not in seen, f"duplicate EMCY entry {key}"
        seen.add(key)
    assert c.emcy_class(0x4310, 1) == "warn"
    assert c.emcy_class(0x4310, 2) == "stop_all_rehome"
    assert c.emcy_class(0x3230) == "homing_stall"
    assert c.emcy_class(0x1234) == "stop_all"  # unknown codes are conservative
