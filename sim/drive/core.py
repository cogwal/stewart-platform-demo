"""TMCM-1270 drive behaviour without CAN: CiA 402 state machine, operating modes (pp, pv, hm, csp),
open-loop motion coupled to the S20 model, stallGuard, and the firmware quirks from the manual.

The CAN adapter (can_node.py) feeds SDO/RPDO writes into `write()` and NMT events into
`on_nmt_stopped()`, calls `tick()` periodically, and reads values for SDO/TPDOs from `store`.
Everything here is deterministic given the sequence of calls, so it can be tested with fake time.

Behaviour the manual does not pin down is marked ASSUMPTION; it is chosen to be strict (the
master must cope with the harsher case) and is checked against the real drive in S5.
"""
from __future__ import annotations

import enum
import logging
from dataclasses import dataclass
from typing import Callable

from canopen.sdo.exceptions import SdoAbortedError

from sim.drive.s20 import S20
from sim.od_store import ObjectStore

log = logging.getLogger(__name__)

# SDO abort codes
ABORT_STATE = 0x08000022        # data cannot be transferred because of the present device state
ABORT_RANGE = 0x06090030        # value range exceeded
ABORT_NOT_MAPPABLE = 0x06040041
ABORT_PDO_LENGTH = 0x06040042
ABORT_READ_ONLY = 0x06010002

# EMCY codes (manual §11)
EMCY_RESET = 0x0000
EMCY_STALL = 0x3230
EMCY_OVERTEMP = 0x4310
EMCY_ENN = 0x5441
EMCY_LIMIT = 0xFF01


class State(enum.Enum):
    NOT_READY = "not ready to switch on"
    SWITCH_ON_DISABLED = "switch on disabled"
    READY_TO_SWITCH_ON = "ready to switch on"
    SWITCHED_ON = "switched on"
    OPERATION_ENABLED = "operation enabled"
    QUICK_STOP_ACTIVE = "quick stop active"
    FAULT_REACTION_ACTIVE = "fault reaction active"
    FAULT = "fault"


STATE_BITS = {  # statusword low byte per manual table (bit 4 voltage enabled, bit 5 quick stop off)
    State.NOT_READY: 0x0000,
    State.SWITCH_ON_DISABLED: 0x0040,
    State.READY_TO_SWITCH_ON: 0x0031,
    State.SWITCHED_ON: 0x0033,
    State.OPERATION_ENABLED: 0x0037,
    State.QUICK_STOP_ACTIVE: 0x0017,
    State.FAULT_REACTION_ACTIVE: 0x001F,
    State.FAULT: 0x0008,
}
ENABLED_STATES = {State.OPERATION_ENABLED, State.QUICK_STOP_ACTIVE, State.FAULT_REACTION_ACTIVE}

MODE_PP, MODE_PV, MODE_HM, MODE_CSP, MODE_CSV = 1, 3, 6, 8, 9
SUPPORTED_MODES = {0, MODE_PP, MODE_PV, MODE_HM, MODE_CSP, MODE_CSV}

# Objects the firmware only accepts in SWITCH_ON_DISABLED (manual §4.2.6, §4.2.16, §4.2.8, §4.2.9)
ONLY_IN_SOD = {0x2005, 0x208C, 0x200B, 0x200C}


@dataclass
class Quirks:
    """Adversarial behaviour, all on by default. ASSUMPTIONS until checked on the real drive."""
    unwired_switches_trip: bool = True   # 2005h left at 0 with nothing wired -> limit switch EMCY on move
    stall_faults: bool = True            # stop-on-stall puts the drive into FAULT (not just stops)
    reject_targets_outside_limits: bool = True
    current_limit: int = 90              # 2003h above this while enabled -> overtemp pre-warning after a while
    overtemp_after_s: float = 10.0


class DriveCore:
    def __init__(
        self,
        store: ObjectStore,
        plant: S20,
        counts_per_mm: int,
        emcy: Callable[[int, int], None] = lambda code, byte1: None,
        quirks: Quirks | None = None,
    ):
        self.store = store
        self.plant = plant
        self.counts_per_mm = counts_per_mm
        self._emcy = emcy
        self.quirks = quirks or Quirks()
        self.power_on()

    # --- lifecycle --------------------------------------------------------------------------
    def power_on(self) -> None:
        self.store.reset()
        self.state = State.SWITCH_ON_DISABLED  # transitions 0 and 1 happen automatically
        self.position = 0.0                    # step counter (counts); unrelated to the rod at power-up
        self.velocity = 0.0                    # counts/s
        self.controlword = 0
        self.enn_ok = True
        self._pp_target: float | None = None
        self._pp_ack = False
        self._target_reached = True
        self._homing_attained = False
        self._homing_error = False
        self._csp_from = 0.0
        self._csp_to = 0.0
        self._csp_t = 0.0
        self._csp_period = 0.0
        self._stall_time = 0.0
        self._overcurrent_time = 0.0
        self._overtemp_warned = False
        self._limit_reported: set[int] = set()
        self.error_register = 0
        self._sync_store()

    # --- helpers ----------------------------------------------------------------------------
    def emcy(self, code: int, byte1: int = 0) -> None:
        if code != EMCY_RESET:
            self.error_register |= 0x01
        log.info("EMCY 0x%04X/%d in state %s", code, byte1, self.state.value)
        self._emcy(code, byte1)

    def get(self, index: int, sub: int = 0):
        return self.store.get(index, sub)

    @property
    def mode(self) -> int:
        return self.store.get(0x6060)

    @property
    def interpolation_period_s(self) -> float:
        return self.get(0x60C2, 1) * 10.0 ** self.get(0x60C2, 2)

    def fault(self, code: int, byte1: int = 0) -> None:
        """Fault reaction: slow down on the quick-stop ramp (605Eh = 2), then FAULT."""
        if self.state in (State.FAULT, State.FAULT_REACTION_ACTIVE):
            return
        self.emcy(code, byte1)
        self.state = State.FAULT_REACTION_ACTIVE if self.velocity else State.FAULT
        self._pp_target = None
        self._sync_store()

    @property
    def statusword(self) -> int:
        sw = STATE_BITS[self.state] | 0x0200  # bit 9 remote
        if self.error_register & 0x80 or self._overtemp_warned:
            sw |= 0x0080  # warning
        m = self.mode
        if self.state is State.OPERATION_ENABLED or self.state is State.QUICK_STOP_ACTIVE:
            if m == MODE_PP:
                sw |= (0x0400 if self._target_reached else 0) | (0x1000 if self._pp_ack else 0)
            elif m == MODE_HM:
                sw |= (0x0400 if self._target_reached else 0) | (0x1000 if self._homing_attained else 0)
                sw |= 0x2000 if self._homing_error else 0
            elif m == MODE_CSP:
                sw |= 0x1000  # target position used
            elif m == MODE_PV:
                sw |= 0x0400 if self.velocity == self._pv_target() else 0
        if abs(self.velocity) > 0:
            sw |= 0x4000
        if self.velocity < 0:
            sw |= 0x8000
        return sw

    def _sync_store(self) -> None:
        """Mirror internal state into the read-only objects."""
        s, pos = self.store, int(round(self.position))
        s.set(0x6041, 0, self.statusword)
        s.set(0x6061, 0, self.mode)
        for idx in (0x6062, 0x6063, 0x6064):
            s.set(idx, 0, pos)
        s.set(0x606C, 0, int(round(self.velocity)))
        s.set(0x1001, 0, self.error_register)
        if s.has(0x2107):
            s.set(0x2107, 0, s.get(0x2000))

    # --- writes from SDO or RPDO ------------------------------------------------------------
    def write(self, index: int, sub: int, value, source: str = "sdo") -> None:
        """Validate and apply a write. Raises SdoAbortedError like the firmware would."""
        if index in ONLY_IN_SOD and self.state is not State.SWITCH_ON_DISABLED:
            raise SdoAbortedError(ABORT_STATE)
        if index == 0x6060 and value not in SUPPORTED_MODES:
            raise SdoAbortedError(ABORT_RANGE)
        if index == 0x2000 and not 0 <= value <= 8:
            raise SdoAbortedError(ABORT_RANGE)
        if index == 0x209E and not -63 <= value <= 63:
            raise SdoAbortedError(ABORT_RANGE)
        if index == 0x605A and value not in (1, 2, 5, 6):
            raise SdoAbortedError(ABORT_RANGE)
        if 0x1600 <= index <= 0x1603 or 0x1A00 <= index <= 0x1A03:
            self._check_pdo_mapping(index, sub, value)
        mode_changed = index == 0x6060 and value != self.store.get(0x6060)
        self.store.set(index, sub, value)

        if index == 0x6040:
            self._on_controlword(value)
        elif index == 0x607A and self.mode == MODE_CSP and self.state is State.OPERATION_ENABLED:
            self._on_csp_target(value)
        elif mode_changed:  # RPDO2 repeats the mode in every frame: only a real change resets
            self._target_reached = True
            self._homing_attained = False
        self._sync_store()

    def _check_pdo_mapping(self, index: int, sub: int, value: int) -> None:
        comm = index - 0x200  # 1600h -> 1400h, 1A00h -> 1800h
        if not self.store.get(comm, 1) & 0x80000000:
            raise SdoAbortedError(ABORT_STATE)  # PDO must be disabled while remapping (§3.1.19)
        if sub == 0:
            if value > 3:
                raise SdoAbortedError(ABORT_PDO_LENGTH)
            total = sum(self.store.get(index, k) & 0xFF for k in range(1, value + 1))
            if total > 64:
                raise SdoAbortedError(ABORT_PDO_LENGTH)
            return
        if self.store.get(index, 0) != 0:
            raise SdoAbortedError(ABORT_STATE)  # count must be 0 while entries change
        if value == 0:
            return
        obj, osub, bits = value >> 16, (value >> 8) & 0xFF, value & 0xFF
        if not self.store.has(obj, osub) or not self.store.var(obj, osub).pdo_mappable:
            raise SdoAbortedError(ABORT_NOT_MAPPABLE)
        if self.store.bits(obj, osub) != bits:
            raise SdoAbortedError(ABORT_PDO_LENGTH)

    # --- CiA 402 state machine --------------------------------------------------------------
    def _on_controlword(self, cw: int) -> None:
        prev, self.controlword = self.controlword, cw
        st = self.state
        rising = lambda bit: (cw & bit) and not (prev & bit)

        if st is State.FAULT:
            if rising(0x0080):
                self.state = State.SWITCH_ON_DISABLED
                self.error_register = 0
                self._limit_reported.clear()
                self.emcy(EMCY_RESET)
            return
        if st in (State.FAULT_REACTION_ACTIVE, State.NOT_READY):
            return

        enable_voltage, quick_stop_off = bool(cw & 0x02), bool(cw & 0x04)
        low = cw & 0x0F
        if not enable_voltage:  # disable voltage (7, 9, 10, 12)
            self.state = State.SWITCH_ON_DISABLED
            self.velocity = 0.0
        elif not quick_stop_off:  # quick stop (7, 10, 11)
            if st is State.OPERATION_ENABLED:
                self.state = State.QUICK_STOP_ACTIVE
            elif st in (State.READY_TO_SWITCH_ON, State.SWITCHED_ON):
                self.state = State.SWITCH_ON_DISABLED
        elif low & 0x07 == 0x06:  # shutdown (2, 6, 8)
            if st in (State.SWITCH_ON_DISABLED, State.SWITCHED_ON, State.OPERATION_ENABLED):
                self.state = State.READY_TO_SWITCH_ON
                self.velocity = 0.0
        elif low == 0x07:  # switch on (3) / disable operation (5)
            if st is State.READY_TO_SWITCH_ON or st is State.OPERATION_ENABLED:
                self.state = State.SWITCHED_ON
        elif low == 0x0F:  # switch on + enable operation (3, 4) / enable operation (4, 16)
            if st in (State.READY_TO_SWITCH_ON, State.SWITCHED_ON, State.QUICK_STOP_ACTIVE):
                self._enter_operation_enabled()

        if self.state is State.OPERATION_ENABLED:
            self._on_mode_bits(cw, prev)

    def _enter_operation_enabled(self) -> None:
        if not self.enn_ok:
            self.fault(EMCY_ENN, 0)
            return
        self.state = State.OPERATION_ENABLED
        self._csp_from = self._csp_to = self.position
        self._csp_t = self._csp_period = 0.0
        self._overcurrent_time = 0.0
        if self.get(0x2003) > self.quirks.current_limit:
            log.warning("run current 2003h=%d exceeds the S20 limit (%d)", self.get(0x2003), self.quirks.current_limit)

    def _on_mode_bits(self, cw: int, prev: int) -> None:
        new_set_point = (cw & 0x10) and not (prev & 0x10)
        if self.mode == MODE_PP:
            if new_set_point:
                target = self.get(0x607A) + (self.position if cw & 0x40 else 0)
                if self._outside_limits(target):
                    return
                self._pp_target = float(target)
                self._target_reached = False
            self._pp_ack = bool(cw & 0x10)
        elif self.mode == MODE_HM and new_set_point:
            method = self.get(0x6098)
            if method == 35:
                self.position = float(self.get(0x607C))
                self._homing_attained, self._homing_error, self._target_reached = True, False, True
            else:
                # switch/index methods need inputs that are disabled (2005h) or not wired
                self._homing_attained, self._homing_error = False, True

    def on_nmt_stopped(self) -> None:
        """Manual §2.2: NMT stopped puts an axis in OPERATION_ENABLED into FAULT. (The EMCY that
        caused the NMT stop, e.g. 8130h heartbeat loss, is sent by the CAN adapter.)"""
        if self.state is State.OPERATION_ENABLED:
            self.state = State.FAULT
            self.velocity = 0.0
            self._pp_target = None
            self._sync_store()

    def set_enn(self, ok: bool) -> None:
        """ENN hardware enable (e-stop chain). Missing -> EMCY 5441h, driver off."""
        self.enn_ok = ok
        if not ok and self.state in ENABLED_STATES:
            self.velocity = 0.0
            self.fault(EMCY_ENN, 0)

    # --- csp --------------------------------------------------------------------------------
    def _on_csp_target(self, target: int) -> None:
        if self._outside_limits(target):
            return
        self._csp_from = self.position
        self._csp_to = float(target)
        self._csp_t = 0.0
        self._csp_period = self.interpolation_period_s

    # --- limits -----------------------------------------------------------------------------
    def _outside_limits(self, target: float) -> bool:
        if not self.quirks.reject_targets_outside_limits:
            return False
        offset = self.get(0x607C)
        lo, hi = self.get(0x607D, 1) - offset, self.get(0x607D, 2) - offset
        if target < lo or target > hi:
            byte1 = 2 if target < lo else 1
            if byte1 not in self._limit_reported:
                self._limit_reported.add(byte1)
                self.emcy(EMCY_LIMIT, byte1)
            return True
        return False

    def _switch_trips(self, direction: float) -> bool:
        """ASSUMPTION: with 2005h = 0 and nothing wired, the input pull-ups read as 'switch active'."""
        if not self.quirks.unwired_switches_trip or direction == 0:
            return False
        sw = self.get(0x2005)
        left_active, right_active = not sw & 0x01, not sw & 0x02
        if direction < 0 and left_active:
            byte1 = 4  # negative limit switch
        elif direction > 0 and right_active:
            byte1 = 3  # positive limit switch
        else:
            return False
        self.fault(EMCY_LIMIT, byte1)
        return True

    # --- motion -----------------------------------------------------------------------------
    def _pv_target(self) -> float:
        return float(self.get(0x60FF))

    @staticmethod
    def _ramp(v: float, v_target: float, accel: float, dt: float) -> float:
        if accel <= 0:
            return v  # acceleration 0: velocity never changes (real drive: no motion)
        step = accel * dt
        if v < v_target:
            return min(v + step, v_target)
        return max(v - step, v_target)

    def _decelerate(self, decel: float, dt: float) -> None:
        self.velocity = self._ramp(self.velocity, 0.0, decel, dt)

    def tick(self, dt: float) -> None:
        st = self.state
        if st is State.FAULT_REACTION_ACTIVE:
            self._decelerate(self.get(0x6085), dt)
            if self.velocity == 0:
                self.state = State.FAULT
        elif st is State.QUICK_STOP_ACTIVE:
            option = self.get(0x605A)
            self._decelerate(self.get(0x6085) if option in (2, 6) else self.get(0x6084), dt)
            if self.velocity == 0 and option in (1, 2):
                self.state = State.SWITCH_ON_DISABLED
        elif st is State.OPERATION_ENABLED:
            self._tick_mode(dt)
        else:
            self.velocity = 0.0

        if self.velocity and self._switch_trips(self.velocity):
            self.velocity = 0.0
        self._advance(dt)
        self._overcurrent(dt)
        self._sync_store()

    def _tick_mode(self, dt: float) -> None:
        m = self.mode
        if m == MODE_PP:
            self._tick_pp(dt)
        elif m == MODE_PV or m == MODE_CSV:
            self.velocity = self._ramp(self.velocity, self._pv_target(), self.get(0x6083), dt)
        elif m == MODE_CSP:
            self._tick_csp(dt)
        else:
            self._decelerate(self.get(0x6084), dt)

    def _tick_pp(self, dt: float) -> None:
        if self._pp_target is None:
            self._decelerate(self.get(0x6084), dt)
            return
        v_max, acc, dec = float(self.get(0x6081)), float(self.get(0x6083)), float(self.get(0x6084))
        remaining = self._pp_target - self.position
        if v_max <= 0 or acc <= 0 or dec <= 0:
            return  # profile values left at their default 0: the move never starts
        direction = 1.0 if remaining > 0 else -1.0
        stop_distance = self.velocity**2 / (2 * dec)
        if abs(remaining) <= max(stop_distance, abs(self.velocity) * dt) and self.velocity * direction >= 0:
            v_target = 0.0
        else:
            v_target = direction * v_max
        self.velocity = self._ramp(self.velocity, v_target, acc if abs(v_target) > abs(self.velocity) else dec, dt)
        step = self.velocity * dt
        if abs(step) >= abs(remaining) or (self.velocity == 0 and abs(remaining) < 1.0):
            self.velocity = 0.0
            self.position = self._pp_target
            self._pp_target = None
            self._target_reached = True

    def _tick_csp(self, dt: float) -> None:
        if self._csp_period <= 0:
            self.velocity = 0.0
            return
        self._csp_t = min(self._csp_t + dt, self._csp_period)
        frac = self._csp_t / self._csp_period
        desired = self._csp_from + (self._csp_to - self._csp_from) * frac
        self.velocity = (desired - self.position) / dt if dt > 0 else 0.0
        if self._csp_t >= self._csp_period:
            self._csp_period = 0.0  # no new target yet: hold

    def _advance(self, dt: float) -> None:
        """Move the step counter and, through it, the rod. Steps past an end stop are lost."""
        delta_counts = self.velocity * dt
        if delta_counts == 0:
            self.plant.stalled = False
            self._stall_time = 0.0
            return
        self.position += delta_counts
        self.plant.move(delta_counts / self.counts_per_mm)
        self._stallguard(dt)

    def _stallguard(self, dt: float) -> None:
        threshold_speed = self.get(0x20A4)
        load = 0 if self.plant.stalled else 600
        if self.store.has(0x2101):
            self.store.set(0x2101, 0, load)
        if not self.plant.stalled:
            self._stall_time = 0.0
            return
        self._stall_time += dt
        if threshold_speed > 0 and abs(self.velocity) > threshold_speed and self._stall_time >= 0.02:
            log.info("stallGuard: stall at the end stop, stopping")
            self.velocity = 0.0
            if self.quirks.stall_faults:
                self.state = State.FAULT
                self.emcy(EMCY_STALL)
            else:
                self.emcy(EMCY_STALL)
                self._pp_target = None
                if self.mode in (MODE_PV, MODE_CSV):
                    self.store.set(0x60FF, 0, 0)

    def _overcurrent(self, dt: float) -> None:
        if self.state is not State.OPERATION_ENABLED or self.get(0x2003) <= self.quirks.current_limit:
            return
        self._overcurrent_time += dt
        if not self._overtemp_warned and self._overcurrent_time >= self.quirks.overtemp_after_s:
            self._overtemp_warned = True
            self.emcy(EMCY_OVERTEMP, 1)
