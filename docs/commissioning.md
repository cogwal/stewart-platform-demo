# Commissioning plan: from the first TMCM-1270 on the bench to the platform

Goal: bring the real hardware up step by step, measure the facts the simulation assumes, and feed them back into
the contract and the emulator. The simulation stays the reference for the master: every hardware finding becomes
either a contract change (and the master follows) or an emulator change (so the simulation behaves like the real drive).

Scope: Phase 2 of [plan.md](plan.md) (S5 bench, then S6 with six real drives, then the platform). Hardware facts and
sources: [hardware-notes.md](hardware-notes.md). For an agent doing the emulator side afterwards:
[agents/sim-tuning.md](agents/sim-tuning.md).

Every step lists **what to do**, **what to expect** (what the simulation does today), and **what to record**. Write
findings into `docs/measurements/` (one file per session, dated), not only into chat or notes.

## 0. Before you start

Hardware:
- Ubuntu notebook (22.04/24.04) with Docker, this repo, and a **PEAK PCAN-USB**.
- Lab supply **24 V, current limit about 1 A** for one drive (the TMCM-1270 takes +6 to +28 V; never above 28 V).
- **Two 120 Ω terminators**: the TMCM-1270 has **no termination on board**. One at the PCAN end, one at the far end.
- One TMCM-1270 (CANopen firmware) and one S20-100-38-B, **unloaded and clamped**, with the rod free to travel the
  full 100 mm. Keep hands and cables away from the rod end.
- **ENN** (GPI3) must be pulled to GND for the driver to run. Wire it through a switch or the e-stop chain so that
  opening it disables the driver. Without it the drive reports EMCY 5441h and won't enable.
- A clamp meter or a current probe on one motor phase, and a thermometer (or the drive's own temperature readout) for the
  current check.

Software (on the notebook):
```bash
sudo modprobe peak_usb
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0            # state ERROR-ACTIVE, bitrate 1000000
make image
```
`can0` is used by the containers directly (host network). Everything below runs as `docker compose run --rm dev …`
or through `make` with `CAN_IFACE=can0`.

Rules for the whole session:
- Only one master on the bus at a time (the probe, `spmaster` or the Python test master). Each produces SYNC and a
  heartbeat as node 127.
- Keep a passive log running for every step: `docker compose run --rm dev candump -L can0 > build/bench-$(date +%F-%H%M).log`
  (in a second terminal). The logs are what the emulator gets tuned against.

## 1. First contact (no motion)

1. Start the log, then power the drive.
   - Expect: a boot-up frame `0x701 [1] 00` (factory node ID 1). Then **possibly nothing**: the power-on default of
     1017h (heartbeat) may be 0. The emulator assumes 0 as well: `first_heartbeat_after_bootup_ms = None` in the
     baseline. Record which is true.
2. Passive probe:
   ```bash
   docker compose run --rm dev python tools/drive_probe.py --iface can0 --node 1 --out build/probe-bench-passive.json
   docker compose run --rm dev python tools/drive_probe.py --compare docs/measurements/emulator-probe.json build/probe-bench-passive.json
   ```
   The probe checks identity, reads the power-on values of about 45 objects, writes the full contract startup sequence
   (65 SDO writes) and records every abort, checks that 2005h is refused outside SWITCH ON DISABLED, times the CiA 402
   transitions, heartbeat and TPDO1, and stops its own heartbeat to time the drive's reaction (EMCY 8130h, NMT stopped).
3. Firmware: 1018h.3 must be `0x0003001B` (fw 3.27) or newer. If it's older, flash `vendor/trinamic/.../*.hex` with TMCL-IDE
   (Windows) before continuing; the contract and the EDS are for 3.27.

Record: the compare output, the report JSON (`docs/measurements/YYYY-MM-DD-probe-passive.json`), any SDO aborts
(each one is either a contract error or a manual inaccuracy).

## 2. Current and temperature (no motion, then slow motion)

The contract sets run current 2003h = 88 (≈ 0.41 A RMS, assuming the S20's 0.6 A is a peak value). This is an
assumption to verify, not a measured value.

1. Probe with motion, small travel: `… drive_probe.py --iface can0 --node 1 --allow-motion --travel-mm 10 --out …`.
2. During the pv part, measure the phase current. At standstill after 1 s it should drop to the standby current (2004h = 32).
3. Run 10 minutes of slow back-and-forth (web slider, see step 4) and check the motor temperature. The S20 datasheet
   gives no thermal data; stay below what's comfortable to touch, or the datasheet's limit if one turns up.

Record: measured RMS current at 2003h = 88 and at standby; temperature after 10 min. Then decide 2003h/2004h and
update `contract/drive-contract.yaml` (`current:`). Also check: does a too-high current produce EMCY 4310h/1 (overtemp
pre-warning)? The emulator assumes it does after 10 s above 90 (`Quirks.current_limit`, `overtemp_after_s`).

## 3. Direction, pp, pv, csp, quick stop

From the motion probe (step 2) and by eye:
- **Direction:** a positive target must **extend** the rod. If it retracts, the contract needs a direction
  inversion (a drive parameter or a sign in the master). Nothing in the code assumes this yet; it's open.
- **pp:** `pp_move_s` vs `pp_move_theory_s`. A large difference means the profile objects are interpreted differently
  (units, 208Ch).
- **pv:** `pv_velocity_mm_s` vs commanded. Checks 25,600 counts/mm and PPS units together.
- **csp:** `csp_tracking_rms_mm` for a 1 Hz ±2 mm sine streamed at 10 ms. The emulator interpolates linearly over
  60C2h and gives 0.15 mm. On the real drive this shows whether RPDO1 targets act on arrival or on the next SYNC
  (manual is unclear; contract uses async RPDOs).
- **quick stop:** distance vs theory from 6085h.

Record all of these. Each difference beyond ~15 % goes into the sim-tuning list.

## 4. One leg with the real master

Run the real C master against the real drive, single leg, with the web view:
```bash
SIM_NO_DRIVES=1 SIM_LEGS=1 make sim CAN_IFACE=can0
```
Open `http://<notebook>:8080`. The master boots, configures and **homes on its own**: it retracts at 4 mm/s until
stallGuard (EMCY 3230h or 2101h load ≤ 10) or 30 s, sets home (method 35), parks at 5 mm, then follows the slider
(Legs input) at max 10 mm/s.

Before the first run, set expectations: with the provisional stallGuard values the stall may **not** be detected; the
30 s timeout then guarantees the end stop is reached (blind overtravel at reduced homing current, 2003h = 48). If the
motor screams against the stop for long, stop it (ENN) and go to step 5 first.

Also run `docker compose run --rm dev python tools/bus_stats.py --iface can0 --seconds 10` while it follows the slider:
SYNC period and jitter, TPDO1 rate, bus load (one drive ≈ 2.5 %).

## 5. stallGuard homing tuning

The values in `homing:` and `chopper:` are guesses. Procedure:
1. Probe with `--allow-motion --endstop`. It reads 2101h (stallGuard load) while moving freely at 4 mm/s and at standstill,
   then homes the way the master does.
2. Watch 2101h while the rod runs into the stop (e.g. a loop of SDO reads in a Python shell, or extend the probe). Free
   running should read clearly above the threshold; at the stop it should drop toward 0.
3. Tune in this order: homing speed (`velocity_mm_s`; stallGuard needs enough speed), homing current (`current.homing`),
   209Eh threshold (`stall_threshold`, −63..63, lower = more sensitive), 20A4h stop-on-stall speed, then the master's
   `load_stall_threshold` (2101h value treated as "at the stop").
4. Repeatability: home 10 times, after each one move to 50.00 mm and measure the rod position mechanically (calliper
   against a fixed reference). Spread should be well below 0.1 mm.
5. Check whether a stall **faults** the drive (statusword 0x0208 after EMCY 3230h) or only stops it. The emulator
   assumes it faults (`Quirks.stall_faults = True`); the master handles both.

Record: final values, the 2101h numbers, repeatability. Update `contract/drive-contract.yaml` and run `make gen` (the C
master's config is generated from it).

## 6. Fault behaviour of the real drive

Each of these is already handled by the master and emulated; verify that the real drive does the same:

| Action | Emulator / master expectation | Record |
|---|---|---|
| Open ENN while following | EMCY 5441h → master stops all, homing lost ("press Home") | EMCY code, drive state |
| Stop the master (Ctrl-C `spmaster`) | after ~300 ms: EMCY 8130h, NMT stopped, drive FAULT | times, final statusword (emulator: 0x0240 read after the fact) |
| Power-cycle the drive while following | boot-up while operational → master: "node rebooted: press Home" | boot-up time |
| Unplug CAN for 1 s | drive: bus-off or heartbeat loss; master: heartbeat lost → stopped, homing kept → Resume works | EMCY codes after reconnect (8140h?) |
| SDO in NMT STOPPED | emulator answers; CiA 301 says it shouldn't | whether the real drive answers |
| Drive in FAULT, fault reset via RPDO2 | 0x00 then 0x80 → SWITCH ON DISABLED | works / doesn't |

## 7. Node IDs for six drives

Every drive ships as node 1 at 1 Mbit/s. Connect **one drive at a time** and give it its ID (leg i = node i, see
`config/platform.yaml`):
```bash
docker compose run --rm dev python tools/set_node_id.py --iface can0 --from 1 --to 3
```
(2705h = new ID, 2706h.1 = "save", NMT reset, then it checks the boot-up from the new ID. Untested against the real
drive: watch the log. LSS is the alternative in fw 3.27.) Label each drive with its ID.

Then all six on the bus, legs **not** mounted in the platform yet:
1. Passive probe each node (`--node 1 … 6`).
2. `SIM_NO_DRIVES=1 make sim CAN_IFACE=can0` (six legs, Pose input). All legs home together. With free actuators the
   "pose" is meaningless, but leg motion, timing and bus load are real.
3. `bus_stats.py`: expect ≈ 14.5 % load (simulation: 14.47 %).

## 8. The platform

Only after steps 1 to 7 pass and the CAD geometry is in `config/platform.yaml` ([agents/cad-geometry.md](agents/cad-geometry.md)).

1. Check the geometry against the build: measure the six base and six platform joint centres (or take them from the CAD),
   the pin-to-pin length at the retracted stop, and compare with `python -c "from stewart import geometry as G; …"`
   strokes at neutral. Off by more than ~1 mm: fix `platform.yaml` first.
2. First homing with the platform mounted: all legs retract together, so the platform lowers onto its bottom position.
   Make sure that position is mechanically safe (no joint at its limit, nothing trapped). Consider a reduced homing speed.
3. First moves with small limits: reduce `motion.limits` in `config/platform.yaml` (e.g. ±5 mm, ±3°), `make gen`, then
   widen step by step. Watch for joints hitting their angle limits; the kinematics don't model joint limits yet.
4. Payload: with the fair payload on top, repeat the stallGuard check (homing must not trigger early under load) and the
   speed limits (missed steps show up only as a pose error you can see or measure).

## 9. Feed the results back

| Finding | Where it goes |
|---|---|
| currents, profile, homing, stallGuard values | `contract/drive-contract.yaml` → `make gen` → `make test` |
| power-on object values that differ from the EDS | `contract/defaults-overlay.yaml` (and the emulator follows) |
| behaviour differences (timing, faults, SDO rules) | emulator: [agents/sim-tuning.md](agents/sim-tuning.md) |
| geometry | `config/platform.yaml` → `make gen` |
| everything | a dated note in `docs/measurements/` + the probe JSON files |

When the probe compare between bench and simulation shows no differences that matter, the simulation is good enough
to develop the ECU port and the fair features against it (the original goal of the project).
