"""CAN side of the TMCM-1270 emulator: NMT slave, heartbeat producer/consumer, SDO server,
PDOs (dynamic mapping as written by the master), SYNC and EMCY. Behaviour lives in core.DriveCore.

python-canopen's LocalNode provides the SDO server; NMT, heartbeat and PDOs are implemented here
because the firmware's details matter (boot-up on reset, NMT stopped -> FAULT, max 3 mapped
objects, bit 30 in PDO COB-IDs, TPDOs only in OPERATIONAL).
"""
from __future__ import annotations

import logging
import threading
import time

import canopen

import contract as C
from sim.drive.core import DriveCore, Quirks
from sim.drive.s20 import S20
from sim.od_store import ObjectStore

log = logging.getLogger(__name__)

NMT_INIT, NMT_STOPPED, NMT_OPERATIONAL, NMT_PREOP = 0x00, 0x04, 0x05, 0x7F
EMCY_HEARTBEAT = 0x8130
EMCY_PDO_SHORT, EMCY_PDO_LONG = 0x8210, 0x8220
TICK_S = 0.001


class CanDrive:
    def __init__(
        self,
        network: canopen.Network,
        node_id: int,
        contract: C.Contract | None = None,
        plant: S20 | None = None,
        quirks: Quirks | None = None,
    ):
        self.contract = contract or C.load()
        self.network = network
        self.node_id = node_id
        od = canopen.import_od(str(self.contract.path.parent / self.contract["device"]["eds"]), node_id=node_id)
        self.store = ObjectStore(od, self.contract.defaults_overlay)
        self.plant = plant or S20(stroke_mm=self.contract["actuator"]["stroke_mm"])
        self.lock = threading.RLock()
        self.core = DriveCore(self.store, self.plant, self.contract.counts_per_mm, emcy=self._send_emcy, quirks=quirks)

        self.node = canopen.LocalNode(node_id, od)
        network.add_node(self.node)
        # Take NMT/heartbeat away from LocalNode; keep only its SDO server.
        network.unsubscribe(0, self.node.nmt.on_command)
        self.node._write_callbacks.remove(self.node.nmt.on_write)
        self.node.add_read_callback(self._sdo_read)
        self.node.add_write_callback(self._sdo_write)

        self.nmt_state = NMT_INIT
        self._rpdo_cobs: dict[int, int] = {}  # COB-ID -> RPDO number
        self._sync_count: dict[int, int] = {}
        self._last_tpdo: dict[int, tuple[float, bytes]] = {}
        self._hb_last_rx: float | None = None
        self._hb_lost = False
        self._hb_next_tx = 0.0
        self.mute_heartbeat_until = 0.0  # fault injection: heartbeat frames lost (cable glitch)
        self._running = False
        self._thread: threading.Thread | None = None
        self.powered = False

    # --- lifecycle --------------------------------------------------------------------------
    def start(self) -> None:
        self.network.subscribe(0x000, self._on_nmt)
        self.network.subscribe(0x080, self._on_sync)
        self._running = True
        self._thread = threading.Thread(target=self._loop, name=f"drive{self.node_id}", daemon=True)
        self._thread.start()
        self.power_cycle()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)
        self.network.unsubscribe(0x000, self._on_nmt)
        self.network.unsubscribe(0x080, self._on_sync)
        self._unsubscribe_rpdos()
        self._unsubscribe_heartbeat()

    def power_cycle(self) -> None:
        """Power-on: all objects back to defaults, counter 0, boot-up. The rod stays where it is."""
        with self.lock:
            self.powered = True
            self.core.power_on()
            self._reset_communication()

    def power_off(self) -> None:
        with self.lock:
            self.powered = False
            self.core.velocity = 0.0
            self._unsubscribe_rpdos()
            self._unsubscribe_heartbeat()
            self.nmt_state = NMT_INIT

    # --- NMT --------------------------------------------------------------------------------
    def _reset_communication(self) -> None:
        self.store.reset(comm_only=True)
        self._unsubscribe_rpdos()
        self._unsubscribe_heartbeat()
        self._refresh_rpdos()
        self._refresh_heartbeat_consumer()
        self._sync_count.clear()
        self._last_tpdo.clear()
        self.nmt_state = NMT_INIT
        self.network.send_message(0x700 + self.node_id, [0x00])  # boot-up
        self.nmt_state = NMT_PREOP
        self._hb_next_tx = time.monotonic()

    def _on_nmt(self, can_id: int, data: bytearray, timestamp: float) -> None:
        if len(data) < 2 or data[1] not in (0, self.node_id) or not self.powered:
            return
        cmd = data[0]
        with self.lock:
            if cmd == 0x01:
                self.nmt_state = NMT_OPERATIONAL
            elif cmd == 0x02:
                self._enter_stopped()
            elif cmd == 0x80:
                self.nmt_state = NMT_PREOP
            elif cmd == 0x81:  # reset node = power-on behaviour of the application
                self.core.power_on()
                self._reset_communication()
            elif cmd == 0x82:
                self._reset_communication()
            log.info("node %d NMT command 0x%02X -> state 0x%02X", self.node_id, cmd, self.nmt_state)

    def _enter_stopped(self) -> None:
        self.nmt_state = NMT_STOPPED
        self.core.on_nmt_stopped()

    # --- heartbeat --------------------------------------------------------------------------
    def _refresh_heartbeat_consumer(self) -> None:
        self._unsubscribe_heartbeat()
        entry = self.store.get(0x1016, 1)
        producer, ms = (entry >> 16) & 0x7F, entry & 0xFFFF
        self._hb_producer = producer if ms else None
        self._hb_timeout = ms / 1000.0
        self._hb_last_rx = None
        self._hb_lost = False
        if self._hb_producer:
            self.network.subscribe(0x700 + producer, self._on_heartbeat)

    def _unsubscribe_heartbeat(self) -> None:
        producer = getattr(self, "_hb_producer", None)
        if producer:
            self.network.unsubscribe(0x700 + producer, self._on_heartbeat)
        self._hb_producer = None

    def _on_heartbeat(self, can_id: int, data: bytearray, timestamp: float) -> None:
        self._hb_last_rx = time.monotonic()

    def _check_heartbeat(self, now: float) -> None:
        if self._hb_producer is None or self._hb_last_rx is None or self._hb_lost:
            return  # monitoring starts with the first received heartbeat (manual §3.1.13)
        if now - self._hb_last_rx > self._hb_timeout:
            self._hb_lost = True
            log.warning("node %d lost heartbeat of node %d", self.node_id, self._hb_producer)
            self._send_emcy(EMCY_HEARTBEAT, 0)
            if self.store.get(0x1029, 1) == 0 and self.nmt_state == NMT_OPERATIONAL:
                self._enter_stopped()

    def _produce_heartbeat(self, now: float) -> None:
        period = self.store.get(0x1017) / 1000.0
        if period <= 0 or now < self._hb_next_tx:
            return
        self._hb_next_tx = now + period
        if now < self.mute_heartbeat_until:
            return
        self.network.send_message(0x700 + self.node_id, [self.nmt_state])

    # --- EMCY -------------------------------------------------------------------------------
    def _send_emcy(self, code: int, byte1: int) -> None:
        if not self.powered or self.nmt_state == NMT_STOPPED:
            return
        register = self.store.get(0x1001) | (0x01 if code else 0)
        if code == 0:
            register = 0
        self.node.emcy.send(code, register, bytes([byte1, 0xFF if code >= 0x5000 and byte1 == 0 else 0, 0, 0, 0]))

    # --- SDO --------------------------------------------------------------------------------
    def _sdo_read(self, index: int, subindex: int, od) -> object:
        with self.lock:
            return self.store.get(index, subindex)

    def _sdo_write(self, index: int, subindex: int, od, data: bytes) -> None:
        value = od.decode_raw(data)
        with self.lock:
            self.core.write(index, subindex, value)
            if 0x1400 <= index <= 0x1403 and subindex == 1:
                self._refresh_rpdos()
            elif index == 0x1016:
                self._refresh_heartbeat_consumer()
            elif index == 0x1017:
                self._hb_next_tx = time.monotonic()

    # --- PDOs -------------------------------------------------------------------------------
    def _mapping(self, map_index: int) -> list[tuple[int, int, int]]:
        count = self.store.get(map_index, 0)
        out = []
        for k in range(1, count + 1):
            v = self.store.get(map_index, k)
            out.append((v >> 16, (v >> 8) & 0xFF, v & 0xFF))
        return out

    def _refresh_rpdos(self) -> None:
        self._unsubscribe_rpdos()
        for n in range(1, 5):
            cob = self.store.get(0x1400 + n - 1, 1)
            if not cob & 0x80000000:
                self._rpdo_cobs[cob & 0x7FF] = n
                self.network.subscribe(cob & 0x7FF, self._on_rpdo)

    def _unsubscribe_rpdos(self) -> None:
        for cob in self._rpdo_cobs:
            self.network.unsubscribe(cob, self._on_rpdo)
        self._rpdo_cobs.clear()

    def _on_rpdo(self, can_id: int, data: bytearray, timestamp: float) -> None:
        with self.lock:
            n = self._rpdo_cobs.get(can_id)
            if n is None or self.nmt_state != NMT_OPERATIONAL:
                return
            mapping = self._mapping(0x1600 + n - 1)
            length = sum(bits for _, _, bits in mapping) // 8
            if len(data) < length:
                self._send_emcy(EMCY_PDO_SHORT, 0)
                return
            if len(data) > length:
                self._send_emcy(EMCY_PDO_LONG, 0)
                return
            offset = 0
            for index, sub, bits in mapping:
                raw = bytes(data[offset : offset + bits // 8])
                offset += bits // 8
                try:
                    self.core.write(index, sub, self.store.decode(index, sub, raw), source="rpdo")
                except canopen.SdoAbortedError:
                    self._send_emcy(0x6320, 0)  # parameter error

    def _tpdo_payload(self, n: int) -> bytes:
        return b"".join(self.store.encode(i, s) for i, s, _ in self._mapping(0x1A00 + n - 1))

    def _on_sync(self, can_id: int, data: bytearray, timestamp: float) -> None:
        with self.lock:
            if self.nmt_state != NMT_OPERATIONAL:
                return
            for n in range(1, 5):
                comm = 0x1800 + n - 1
                cob, ttype = self.store.get(comm, 1), self.store.get(comm, 2)
                if cob & 0x80000000 or not 1 <= ttype <= 240:
                    continue
                self._sync_count[n] = self._sync_count.get(n, 0) + 1
                if self._sync_count[n] >= ttype:
                    self._sync_count[n] = 0
                    self.network.send_message(cob & 0x7FF, self._tpdo_payload(n))

    def _async_tpdos(self, now: float) -> None:
        if self.nmt_state != NMT_OPERATIONAL:
            return
        for n in range(1, 5):
            comm = 0x1800 + n - 1
            cob, ttype = self.store.get(comm, 1), self.store.get(comm, 2)
            if cob & 0x80000000 or ttype not in (0xFE, 0xFF):
                continue
            inhibit = self.store.get(comm, 3) / 10000.0
            event = self.store.get(comm, 5) / 1000.0
            payload = self._tpdo_payload(n)
            last_t, last_payload = self._last_tpdo.get(n, (-1e9, b""))
            changed = payload != last_payload
            due = event > 0 and now - last_t >= event
            if (changed or due) and now - last_t >= inhibit:
                self._last_tpdo[n] = (now, payload)
                self.network.send_message(cob & 0x7FF, payload)

    # --- main loop --------------------------------------------------------------------------
    def _loop(self) -> None:
        last = time.monotonic()
        while self._running:
            time.sleep(TICK_S)
            now = time.monotonic()
            dt, last = now - last, now
            if not self.powered:
                continue
            with self.lock:
                self.core.tick(dt)
                self._check_heartbeat(now)
                self._produce_heartbeat(now)
                self._async_tpdos(now)

    # --- for the web view / tests -----------------------------------------------------------
    def snapshot(self) -> dict:
        with self.lock:
            return {
                "node": self.node_id,
                "nmt": self.nmt_state,
                "state": self.core.state.value,
                "statusword": self.store.get(0x6041),
                "counter": self.store.get(0x6064),
                "rod_mm": round(self.plant.position_mm, 4),
                "stalled": self.plant.stalled,
            }

