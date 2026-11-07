"""End to end on a real SocketCAN interface: drive emulator and test master in one process, each with
its own CAN socket. Skipped when no vcan0 is available."""
import os
import time
from pathlib import Path

import canopen
import pytest

import contract as C
from sim.drive.can_node import NMT_STOPPED, CanDrive
from sim.drive.core import State
from sim.drive.s20 import S20
from sim.master.testmaster import PyMaster

IFACE = os.environ.get("CAN_IFACE", "vcan0")
pytestmark = pytest.mark.skipif(not Path(f"/sys/class/net/{IFACE}").exists(), reason=f"no {IFACE}")

# Own node IDs, so the tests can share the bus with a running `make sim` (drive 1, master 127).
NODE = 21
c = C.load()
c.data["bus"]["master_node_id"] = 126


@pytest.fixture
def bench():
    drive_net, master_net = canopen.Network(), canopen.Network()
    drive_net.connect(interface="socketcan", channel=IFACE)
    master_net.connect(interface="socketcan", channel=IFACE)
    drive = CanDrive(drive_net, NODE, contract=c, plant=S20(position_mm=3.0))
    drive.start()
    master = PyMaster(master_net, NODE, contract=c)
    master.start()
    try:
        yield drive, master
    finally:
        master.stop()
        drive.stop()
        drive_net.disconnect()
        master_net.disconnect()


def wait_for(cond, timeout, what):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return
        time.sleep(0.01)
    raise AssertionError(f"timeout: {what}")


def test_boot_home_park_and_follow_setpoint(bench):
    drive, master = bench
    master.boot()
    assert drive.store.get(0x2003) == c["current"]["run"]
    assert drive.store.get(0x1A00, 2) == 0x60640020  # TPDO1 remapped per contract

    master.home()
    park = c["homing"]["park_mm"]
    assert master.homed
    assert master.position == c.mm_to_counts(park)
    assert drive.plant.position_mm == pytest.approx(park, abs=0.01)  # zero = the real end stop
    assert any(e[1] == 0x3230 for e in master.emcys)  # end stop found by stallGuard

    master.set_setpoint(20.0)
    wait_for(lambda: abs(master.position - c.mm_to_counts(20.0)) < 5, 6.0, "csp move to 20 mm")
    assert drive.plant.position_mm == pytest.approx(20.0, abs=0.01)
    assert drive.core.state is State.OPERATION_ENABLED

    master.set_setpoint(500.0)  # clamped to the soft limit by the master
    assert master.setpoint_mm == c["actuator"]["soft_limits_mm"][1]


def test_drive_faults_when_master_heartbeat_stops(bench):
    drive, master = bench
    master.boot()
    master.home()
    master.stop()  # master heartbeat and SYNC stop
    wait_for(lambda: drive.core.state is State.FAULT, 1.0, "drive FAULT after heartbeat loss")
    assert drive.nmt_state == NMT_STOPPED
    assert drive.core.velocity == 0


def test_drive_reboot_is_detected_by_master(bench):
    drive, master = bench
    master.boot()
    master.home()
    drive.power_cycle()  # e.g. hard e-stop cut drive power
    wait_for(lambda: not master.homed, 1.0, "master notices the reboot")
    assert "rebooted" in master.phase
