"""Drive emulator core without CAN, with fake time."""
import canopen
import pytest
from canopen.sdo.exceptions import SdoAbortedError

import contract as C
from sim.drive.core import MODE_CSP, MODE_HM, MODE_PP, MODE_PV, DriveCore, Quirks, State
from sim.drive.s20 import S20
from sim.od_store import ObjectStore

c = C.load()
CPM = c.counts_per_mm
DT = 0.001


def make(position_mm=37.0, quirks=None):
    od = canopen.import_od(str(c.path.parent / c["device"]["eds"]), node_id=1)
    emcys = []
    core = DriveCore(ObjectStore(od, c.defaults_overlay), S20(position_mm=position_mm), CPM,
                     emcy=lambda code, b1: emcys.append((code, b1)), quirks=quirks)
    return core, emcys


def run(core, seconds):
    for _ in range(round(seconds / DT)):
        core.tick(DT)


def configure(core):
    for w in c.startup_sequence(1):
        core.write(w.index, w.sub, w.value)


def enable(core, mode):
    core.write(0x6060, 0, mode)
    for cw in (0x06, 0x07, 0x0F):
        core.write(0x6040, 0, cw)
    assert core.state is State.OPERATION_ENABLED


# --- power-on and contract -------------------------------------------------------------------
def test_power_on_defaults_match_manual():
    core, _ = make()
    assert core.state is State.SWITCH_ON_DISABLED
    assert core.get(0x2003) == 128 and core.get(0x2005) == 0 and core.get(0x6081) == 0
    assert core.get(0x60C2, 2) == -3
    assert core.get(0x6041) & 0x4F == 0x40


def test_whole_contract_startup_sequence_is_accepted():
    core, emcys = make()
    configure(core)
    assert core.get(0x2005) == 0x13 and core.get(0x2003) == 88
    assert core.get(0x1600, 0) == 2 and core.get(0x1A02, 0) == 2  # TPDO3 mapping untouched, just disabled
    assert core.get(0x1802, 1) & 0x80000000
    assert emcys == []


def test_sod_only_objects_rejected_when_enabled():
    core, _ = make()
    configure(core)
    enable(core, MODE_PP)
    with pytest.raises(SdoAbortedError) as e:
        core.write(0x2005, 0, 3)
    assert e.value.code == 0x08000022


def test_pdo_mapping_rules():
    core, _ = make()
    with pytest.raises(SdoAbortedError):  # PDO still enabled
        core.write(0x1600, 0, 0)
    core.write(0x1400, 1, 0xC0000201)
    core.write(0x1600, 0, 0)
    with pytest.raises(SdoAbortedError):  # 2101h not mappable
        core.write(0x1600, 1, 0x21010010)
    with pytest.raises(SdoAbortedError):  # max 3 entries
        core.write(0x1600, 0, 4)


# --- state machine ---------------------------------------------------------------------------
def test_enable_and_disable_sequence():
    core, _ = make()
    configure(core)
    core.write(0x6040, 0, 0x06)
    assert core.state is State.READY_TO_SWITCH_ON and core.get(0x6041) & 0x6F == 0x21
    core.write(0x6040, 0, 0x07)
    assert core.state is State.SWITCHED_ON and core.get(0x6041) & 0x6F == 0x23
    core.write(0x6040, 0, 0x0F)
    assert core.state is State.OPERATION_ENABLED and core.get(0x6041) & 0x6F == 0x27
    core.write(0x6040, 0, 0x00)
    assert core.state is State.SWITCH_ON_DISABLED


def test_nmt_stopped_faults_enabled_axis_and_fault_reset_recovers():
    core, emcys = make()
    configure(core)
    enable(core, MODE_PP)
    core.on_nmt_stopped()
    assert core.state is State.FAULT and core.get(0x6041) & 0x4F == 0x08
    core.write(0x6040, 0, 0x00)
    core.write(0x6040, 0, 0x80)
    assert core.state is State.SWITCH_ON_DISABLED
    assert emcys[-1] == (0x0000, 0)


# --- quirks ----------------------------------------------------------------------------------
def test_unwired_switches_trip_when_2005_left_at_default():
    core, emcys = make()
    for w in c.startup_sequence(1):
        if w.index != 0x2005:
            core.write(w.index, w.sub, w.value)
    enable(core, MODE_PV)
    core.write(0x60FF, 0, -51200)
    run(core, 0.1)
    assert core.state in (State.FAULT, State.FAULT_REACTION_ACTIVE)
    assert (0xFF01, 4) in emcys


def test_pp_with_default_profile_never_moves():
    core, _ = make()
    core.write(0x2005, 0, 0x13)
    enable(core, MODE_PP)
    core.write(0x607A, 0, 10 * CPM)
    core.write(0x6040, 0, 0x1F)
    run(core, 1.0)
    assert core.position == 0 and not core.get(0x6041) & 0x0400


# --- modes -----------------------------------------------------------------------------------
def test_pp_move_reaches_target_with_handshake():
    core, _ = make(position_mm=10.0)
    configure(core)
    core.write(0x607D, 1, -(10**9))  # counter starts at 0, allow negative for this test
    enable(core, MODE_PP)
    core.write(0x607A, 0, 5 * CPM)
    core.write(0x6040, 0, 0x1F)
    assert core.get(0x6041) & 0x1000  # set-point acknowledged
    core.write(0x6040, 0, 0x0F)
    run(core, 0.5)
    assert not core.get(0x6041) & 0x0400  # still moving
    run(core, 2.0)  # 5 mm at 5 mm/s + ramps
    assert core.get(0x6041) & 0x0400
    assert core.get(0x6064) == 5 * CPM
    assert core.plant.position_mm == pytest.approx(15.0, abs=1e-6)


def test_target_outside_soft_limits_is_rejected():
    core, emcys = make()
    configure(core)
    enable(core, MODE_PP)
    core.write(0x607A, 0, 200 * CPM)
    core.write(0x6040, 0, 0x1F)
    run(core, 0.2)
    assert core.position == 0
    assert (0xFF01, 1) in emcys


def test_csp_follows_streamed_targets():
    core, _ = make(position_mm=50.0)
    configure(core)
    core.write(0x607D, 1, -(10**9))
    enable(core, MODE_CSP)
    period = c["bus"]["sync"]["period_ms"] / 1000
    step = 0.05 * CPM  # 0.05 mm per 10 ms = 5 mm/s
    for k in range(1, 101):
        core.write(0x607A, 0, round(k * step))
        run(core, period)
    assert core.get(0x6064) == round(100 * step)
    assert core.plant.position_mm == pytest.approx(55.0, abs=1e-3)
    assert core.get(0x6041) & 0x1000


def test_homing_against_end_stop_then_method_35():
    core, emcys = make(position_mm=0.5)
    configure(core)
    enable(core, MODE_PV)
    core.write(0x60FF, 0, -c.mm_s_to_pps(c["homing"]["velocity_mm_s"]))
    run(core, 1.0)
    assert core.plant.at_retracted_stop
    assert (0x3230, 0) in emcys  # stop on stall
    assert core.state is State.FAULT  # Quirks.stall_faults
    core.write(0x6040, 0, 0x80)
    enable(core, MODE_HM)
    core.write(0x6040, 0, 0x1F)
    assert core.get(0x6064) == 0 and core.get(0x6041) & 0x1000  # homing attained


def test_stall_without_fault_quirk_just_stops():
    core, emcys = make(position_mm=0.2, quirks=Quirks(stall_faults=False))
    configure(core)
    enable(core, MODE_PV)
    core.write(0x60FF, 0, -102400)
    run(core, 0.5)
    assert core.state is State.OPERATION_ENABLED and core.velocity == 0
    assert (0x3230, 0) in emcys


def test_steps_past_the_end_stop_are_lost():
    """Open loop: below the stall-stop speed the counter keeps going while the rod stays put."""
    core, _ = make(position_mm=1.0)
    configure(core)
    core.write(0x20A4, 0, 0)  # stop-on-stall off
    enable(core, MODE_PV)
    core.write(0x60FF, 0, -c.mm_s_to_pps(4.0))
    run(core, 1.0)
    assert core.plant.position_mm == 0.0
    assert core.get(0x6064) < -2 * CPM  # counter moved ~4 mm minus ramp, rod only 1 mm


def test_enn_missing_faults_the_drive():
    core, emcys = make()
    configure(core)
    enable(core, MODE_CSP)
    core.set_enn(False)
    assert (0x5441, 0) in emcys
    run(core, 0.1)
    assert core.state is State.FAULT


def test_too_high_current_warns_after_a_while():
    core, emcys = make()
    configure(core)
    core.write(0x2003, 0, 128)
    enable(core, MODE_CSP)
    run(core, 10.5)
    assert (0x4310, 1) in emcys
