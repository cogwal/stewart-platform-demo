# Drive contract v0.1 (single TMCM-1270)

What the CANopen master and the drive emulator both build against. The machine-readable
version is [`drive-contract.yaml`](drive-contract.yaml); the loader in [`__init__.py`](__init__.py)
turns it into concrete SDO writes using the vendor EDS for sizes. Facts and sources are in
[docs/hardware-notes.md](../docs/hardware-notes.md).

```bash
python -m contract 1     # print the startup SDO sequence for node 1
pytest tests/contract    # check the contract against the EDS
```

Values marked *provisional* are placeholders until the bench measurements (S5).

## Files

| File | What |
|---|---|
| `drive-contract.yaml` | bus, heartbeat, units, current, profile, homing, PDOs, CiA 402 codes, EMCY handling |
| `defaults-overlay.yaml` | power-on defaults from the manual for objects whose EDS entry has none (for the emulator) |
| `eds/TMCM-1270_V3.27.eds` | vendor EDS, unmodified (fw 3.27, LSS supported) |

## Bus

| Item | Value |
|---|---|
| Bit rate | 1 Mbit/s (drive default) |
| Node IDs | master 127, drives 1 (v0), later 1–6. Drives ship as node 1, so IDs are set at commissioning |
| SYNC | produced by the master, COB-ID 0x80, every **10 ms** |
| Heartbeat | master and drives produce every 100 ms. Each side declares the other lost after 300 ms |

Bus load per drive per cycle: RPDO1 (6 bytes) + TPDO1 (6 bytes), plus one SYNC shared by all.
With 6 drives that's about 14 % at 1 Mbit/s.

## Units

Positions are in microsteps: **25,600 counts/mm** (1/256 microstepping, 0.01 mm per full step).
Velocities are in PPS (counts/s), accelerations in PPS/s. Zero is the retracted end stop after homing.
Software limits are 0.5 to 99.5 mm (12,800 to 2,547,200 counts).

## Master flow per drive

1. **Wait for boot-up** (0x700+id, data 00) up to 5 s. A boot-up while operational means the drive
   rebooted (e.g. e-stop): mark it unhomed, stop all legs, and restart this flow.
2. **Check identity** 1018h: vendor 0x286, product 1270, revision ≥ 0x0003001B.
3. **Make sure the drive is in SWITCH_ON_DISABLED** (6041h). From FAULT: fault reset (6040h bit 7 rising edge).
   From any enabled state: 6040h = 0x0000.
4. **Write the startup SDO list** (`python -m contract <id>`, 65 writes): error behaviour, heartbeat,
   switches off (2005h = 0x13), microsteps, units, chopper, current, soft limits, profile, csp period,
   homing objects, then the PDO remapping. 2005h and 208Ch are only writable in SWITCH_ON_DISABLED.
5. **NMT start** the drive → OPERATIONAL. PDOs are now active.
6. **Home** (below). Then switch to csp and follow setpoints.

Every value is written at every boot. Nothing relies on parameters stored in the drive except node ID
and bit rate (commissioning).

## PDOs

COB-ID = base + node ID. Bit 30 (no RTR) is always set, as the manual requires. Max 3 objects per PDO.

| PDO | COB-ID | Type | Content | Use |
|---|---|---|---|---|
| RPDO1 | 0x200+id | async (FFh) | 6040h controlword, 607Ah target position | csp stream every cycle, pp targets |
| RPDO2 | 0x300+id | async | 6040h, 6060h mode | mode switch |
| RPDO3 | 0x400+id | async | 6040h, 60FFh target velocity | pv travel during homing |
| RPDO4 | — | disabled | | |
| TPDO1 | 0x180+id | every SYNC | 6041h statusword, 6064h actual position | position feedback (also feeds the web view) |
| TPDO2 | 0x280+id | on change, ≥ 5 ms apart | 6041h, 6061h mode display | state and mode changes |
| TPDO3/4 | — | disabled | | the defaults were SYNC-driven and would double the bus load |

RPDOs are asynchronous (act on arrival) because the manual contradicts itself on SYNC-driven RPDOs.
The master sends RPDO1 right after each SYNC. Bench test in S5.

## CiA 402

Enable: 6040h = 0x06 (→ ready to switch on), 0x07 (→ switched on), 0x0F (→ operation enabled). Wait for each
state in 6041h, 500 ms max per step. Set the mode (6060h, via RPDO2) before enabling.

| Mode | 6060h | Used for |
|---|---|---|
| pp | 1 | single moves (park, safe pose). Bit 4 rising edge starts the move. No blending or queueing |
| pv | 3 | homing travel |
| hm | 6 | method 35 only (current position = 0) |
| csp | 8 | normal motion: the master streams 607Ah every 10 ms. The drive interpolates over 60C2h = 10 ms |

## Homing (all legs together)

1. Run current 2003h = 48 (*provisional*). Chopper is spreadCycle, stealthChop off (needed for stallGuard).
2. pv mode, enable, 60FFh = −4 mm/s (−102,400 PPS, *provisional*) via RPDO3.
3. The end stop is reached when **any** of these happens: EMCY 3230h (stallGuard stop, active above 2 mm/s, set via 20A4h),
   or 2101h load ≤ threshold (polled by SDO every 50 ms, not PDO-mappable), or the **30 s timeout** passes
   (blind overtravel: 105 mm at 4 mm/s is guaranteed to be on the stop).
4. 60FFh = 0, fault reset if the drive faulted, hm mode, start method 35 → position = 0.
5. 2003h = 88 (run current), csp mode, target = current position, enable. The leg is homed.

Needed after power-up, after any drive reboot, and after a stall in normal operation.

## csp streaming

Every 10 ms: SYNC, then one RPDO1 per drive with controlword 0x000F and the next target. TPDO1 comes
back on that SYNC with statusword and actual position. The master rate-limits targets to
10 mm/s and 50 mm/s² (*provisional*) and keeps them inside the soft limits. Because the drive runs open loop,
"actual position" is the commanded step count.

## EMCY handling

| Class | Codes | Master reaction |
|---|---|---|
| info | 0000h | log |
| warn | 4310h/1, 8100h, 8110h, 8120h | log, show on the status display |
| homing_stall | 3230h | expected during homing travel. Otherwise: stop all, re-home |
| stop_all | 1000h, 231xh, 8130h, 8611h, unknown codes | quick-stop all legs, wait for the operator |
| stop_all_rehome | 4310h/2, 5441h, 8140h, FF00h | quick-stop all, mark unhomed, re-home after the operator acknowledges |
| config_error | 6320h, 8210h, 8220h, FF01h | stop all, no automatic retry: the contract or master has a bug |

## Open items for the bench (S5)

- Run, homing and standby current (2003h/2004h) vs. measured phase current and temperature.
- stallGuard threshold (209Eh), homing speed and the load-value threshold for end-stop detection.
- Profile and csp limits vs. the S20's real force and speed.
- Do RPDOs act on SYNC (transmission type 1) or only on arrival?
- csp tracking at 10 ms and tolerance to jitter in the master's timing.
