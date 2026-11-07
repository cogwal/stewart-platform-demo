# Plan: slices (draft)

Status (2026-11-07, end of day): S0 to S4 done, S6 and S7 done in simulation (S7 with provisional geometry until the CAD
arrives). Everything that needs hardware or files we don't have yet has a brief in [agents/](agents/README.md); the bench
plan is [commissioning.md](commissioning.md). Remaining open points at the end.
Hardware facts and their sources: [hardware-notes.md](hardware-notes.md).

## Decisions

| Topic | Decision | Why / note |
|---|---|---|
| ECU | **TTControl TTC2000 A2G series, Infineon AURIX TC3xx**, plain C/C++, **PXROS** RTOS, everything in **one task**, TTC's own CAN interface API (provided at porting time) | CANopenNode runs as-is. We write a `CO_driver` on the TTC CAN API |
| Actuator | **Actuonix S20, 100 mm stroke** (`S20-100-38-B`) | Closed 174.8 mm, extended 274.8 mm hole to hole. 0 to 2,560,000 µsteps at 1/256 |
| Platform CAD | exists and comes later | Simulation runs with a **provisional geometry** (`config/platform.yaml`) until then |
| ROS 2 | **Not used** | The ECU doesn't run it, the CAN bus is the only interface, and the web view doesn't need it |
| [tmcl_ros2](https://github.com/analogdevicesinc/tmcl_ros2) | **Not used** | It speaks TMCL (TMCL firmware), not CANopen, and only supports ROS 2 Humble |
| Master stack | **CANopenNode, starting from [CANopenLinux](https://github.com/CANopenNode/CANopenLinux)** on the PC | The same stack runs on the ECU. Only the CAN driver and the main loop differ |
| Master language | **C** for the portable core. C++ is allowed on the ECU side if useful | |
| Drive emulator | **Python** ([python-canopen](https://github.com/christiansandberg/canopen) local node from the EDS plus a defaults overlay) | Fast to iterate and easy to script faults. A separate implementation from the master |
| Dev environment | **Docker** (`.devcontainer/` + `docker compose`), Ubuntu image, `vcan` | Phase 1 on the **Mac** (simulation only). Phase 2 on an **Ubuntu notebook** with USB-CAN. **Docker Desktop on the Mac has no CAN support** (checked 2026-11-07: kernel `7.0.14-linuxkit`, `CONFIG_CAN is not set`), so Phase 1 uses **Colima** (Ubuntu 24.04 VM, kernel 6.8, `vcan` from `linux-modules-extra`): verified 2026-11-07, containers on host network exchange frames on `vcan0`. Setup: `scripts/mac/colima-vcan.sh`. See [PC setups](#pc-setups) |
| USB-CAN | **PEAK PCAN-USB** | On Ubuntu the `peak_usb` driver turns it into a `can0` network interface. The container uses `--network host`, so no USB passthrough is needed |
| Drive firmware | **fw 3.27, `TMCM-1270_V3.27.eds`** | Matches the included hex. Adds LSS |
| Bus | **1 Mbit/s, csp cycle 10 ms**, master and 6 drives only (a logger may listen passively) | ~14 % bus load. 1 Mbit/s is the drive default. Set 60C2h = 10 ms |
| Motion modes | **csp** for coordinated motion, **pp** for single moves, **pv** for homing travel | PP can't blend or queue setpoints |
| Run current | **assume the S20's 0.6 A is peak** → about 0.42 A RMS → 2003h ≈ 90 | Raise only after measuring on the bench |
| Homing | **retract against the end stop, stallGuard stall detection with a timeout fallback, then method 35**. All legs home **together** | See the homing sequence below |
| Fair input | **analog joystick on the ECU** and an **IMU whose orientation drives the platform** | On the PC, a browser slider stands in for both |
| Safety | **existing hard-disconnect e-stop**, plus a software stop and the drives' heartbeat watchdog | The master must recover from drives that reboot (see below) |
| 3D view at the fair | **decide later** | The view reads leg positions from the bus (TPDO 6064h), not from simulator internals, so it can mirror the real platform too |

## Architecture: what is shared and what is per platform

```
            ┌───────────────────────── master (C) ─────────────────────────┐
            │  app core (portable, shared PC + ECU)                        │
            │    supervisor / state machine   motion (traj, later IK)      │
            │    CiA 402 master per node      boot + SDO config sequencer  │
            │    operator setpoint input (rate/limit-checked)              │
            │  ───────────────── step(now_us) / hooks ──────────────────── │
            │  CANopenNode (shared)  +  master OD (generated, shared)      │
            │  ──────────────── CO_driver / input sources ──────────────── │
            │  platform/linux: CANopenLinux, SocketCAN, web slider input   │
            │  platform/ttc:   TTC CAN API, PXROS task, joystick ADC, IMU  │
            └──────────────────────────────┬───────────────────────────────┘
                                           │ CAN (vcan0 / can0 PCAN-USB / real bus)
            ┌──────────────────────────────┴───────────────────────────────┐
            │ sim (Python): N × TMCM-1270 emulator → S20 model (→ platform)│
            │               + fault injection + logging                    │
            └──────────────────────────────┬───────────────────────────────┘
                     web view: listens to the bus (6041h/6064h TPDOs) → browser
```

## PC setups

A container has no kernel of its own: it uses the kernel of the machine it runs on. `vcan` (and
SocketCAN in general) is a **kernel** feature, so whether a container can have `vcan0` depends on
that kernel, not on the container image.

**Phase 1: Mac (simulation only).** macOS has no Linux kernel, so every Docker runtime on the Mac runs a hidden
Linux VM and the containers use *its* kernel. Docker Desktop's VM kernel is built without CAN.
**Colima** runs an Ubuntu VM whose kernel can load `vcan` (verified). Run `scripts/mac/colima-vcan.sh`
after each `colima start`: it installs `linux-modules-extra` if needed, loads `vcan` and brings up `vcan0`.

```
┌──────────────────────────── Mac (macOS) ──────────────────────────────┐
│  Browser  http://localhost:8080  (slider + leg view)                  │
│     ▲ port mapping                                                    │
│ ┌───┴──────────── Linux VM (Docker runtime) ────────────────────────┐ │
│ │  Linux kernel  ── needs vcan module ──  vcan0 (virtual CAN bus)   │ │
│ │                                           ▲       ▲       ▲       │ │
│ │ ┌─────────────── container(s), host network ─────┼───────┼─────┐ │ │
│ │ │  master (C, CANopenLinux)  ─────────────────────┘       │     │ │ │
│ │ │    or Python test master                                │     │ │ │
│ │ │  drive emulator (Python, node 1) ───────────────────────┘     │ │ │
│ │ │  web server: slider → master,  view ← 6064h from vcan0 ───────┘ │ │
│ │ └───────────────────────────────────────────────────────────────┘ │ │
│ └───────────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────────┘
```

**Phase 2: Ubuntu notebook (real CAN).** The containers share the notebook's own kernel. The PCAN-USB
driver (`peak_usb`) turns the adapter into the network interface `can0`, and a container with
`--network host` sees it directly. No USB passthrough needed.

```
┌──────────────────────── Ubuntu notebook ─────────────────────────┐
│  Linux kernel: peak_usb → can0      vcan module → vcan0          │
│  containers (host network): master / emulator / web view         │
│      use vcan0 (pure sim)  or  can0 (real bus)                   │
└──────────────────────────────┬───────────────────────────────────┘
                     PCAN-USB  │ CAN 1 Mbit/s
          ┌────────────────────┴─────────────────────┐
     TMCM-1270 drives (bench)          or          TTC2000 ECU (HIL,
                                                    notebook runs emulator)
```

The same containers and code run in both phases. Only the CAN interface name changes (`vcan0` / `can0`).

Rules for the app core:
- Time is passed in (`now_us`), never read from a clock. That allows tests faster than real time,
  reproducible runs and replay.
- No blocking calls. SDO transfers and boot steps are state machines advanced on every step. This fits
  a single PXROS task.
- Configuration (node IDs, number of drives, limits, unit scale, current) comes from one generated header
  or OD parameters.
- Operator input arrives as a **setpoint** (a leg position now, a pose later) through a small interface.
  The core applies limits and rate limiting, whatever the source.
- CANopenLinux already provides application hooks (`CO_application.h`: program start, communication
  reset, async and RT callbacks). The core plugs in there, and the ECU port provides the same hooks.
  **To verify in S0.**

## Homing sequence (contract draft)

Per drive, all six in parallel:
1. Configure: chopper in spreadCycle (stallGuard2 needs it, **verify on the bench**), reduced run current,
   stallGuard threshold 209Eh, stop-on-stall speed 20A4h below the homing speed.
2. pv mode (3), enable, retract at homing speed.
3. Stall detected (EMCY 3230h, or load 2101h → 0), **or** timeout = time for a full stroke plus margin
   (blind overtravel, guaranteed to be on the stop).
4. Stop, fault reset if the drive went to fault, method 35 (current position = 0), set 607Dh software limits
   (e.g. 0.5 to 99.5 mm), restore the normal run current, switch to csp.

Homing is needed after: every power-up, **every drive reboot** (e-stop, brown-out) and any stall during normal operation.

## Recovering from a drive reboot

The hard e-stop probably cuts drive power. When power returns, a drive sends a boot-up message while the master is operational.
The master must: mark that leg as unhomed, stop the motion on all legs, reconfigure the drive (startup SDO list),
re-home all legs, then wait for the operator. The emulator must be able to simulate this (power-cycle a node).

## The contract (defined early so both sides can be built in parallel)

`contract/` holds what the master and the emulator both build against:

- **Bus:** 1 Mbit/s. Drives node 1–6 (factory default is node 1 for all, so IDs are assigned at commissioning),
  master node 127 (proposal).
- **Heartbeat:** drives produce at 100 ms (1017h). Drives consume the master's heartbeat (1016h, e.g. 300 ms).
  The master monitors each drive at e.g. 300 ms.
- **Startup SDO list per drive** (PRE-OPERATIONAL, SWITCH_ON_DISABLED):
  2003h ≈ 90 · 2004h · 2005h = 0x13 (no switches wired: L/R/home disabled) · 2000h = 8 · 208Ch = 181 ·
  607Dh · 6081/6083/6084h · 6085h · 60C2h = 10 ms · stallGuard/homing objects · 1016h/1017h · PDO remap.
- **PDO map (proposal):** RPDO1 = 6040 + 607A, RPDO2 = 6040 + 6060 (mode switch),
  RPDO3 = 6040 + 60FF (pv homing travel), TPDO1 = 6041 + 6064 on SYNC (10 ms), TPDO2 = 6041 + 6061 on change.
- **Sequences:** enable (6 → 7 → 15), pp handshake (bit 4 / statusword bit 12), homing (above), csp streaming.
- **EMCY handling:** classified as stop all, fault reset and retry, re-home, or log only.
- **Units:** 25,600 µsteps/mm (1/256, 0.01 mm per full step). Velocities in PPS.
- `eds/TMCM-1270_V3.27.eds` (vendor, unmodified) plus `defaults-overlay.yaml` (manual defaults missing from the EDS).
- Contract tests: the emulator accepts exactly this startup sequence, and the master produces it. Later, the same
  SDO dump runs against the real drive.

## Slices

Each slice ends with something that runs and is demonstrated. **⇉** marks slices that can run in parallel.

### Phase 1: Mac, simulation only, single actuator

**S0: Docker dev environment and CAN check** (done on the Mac, 2026-11-07)
- Ubuntu 24.04 image (`docker/Dockerfile`, `compose.yaml`, `.devcontainer/`): can-utils, build tools, Python venv
  (canopen 2.4.1, python-can 4.6.1, pytest). CANopenLinux pinned as a submodule (`third_party/CANopenLinux` @ `f1348d4`,
  CANopenNode `ef9ac3a`). The application hooks in `CO_application.h` exist as expected.
- Mac: **Colima** as the Docker runtime (Docker Desktop has no CAN). `make vcan` runs `scripts/mac/colima-vcan.sh`.
- `make s0-check`: builds `canopend` out of tree, runs it on `vcan0` as node 4 and checks it with python-canopen:
  NMT reset → boot-up, SDO upload (1000h, 1018h), SDO download (1017h = 100 ms), NMT start → 10 heartbeats/s in OPERATIONAL. **Passes.**
- Lesson: on a first run the CANopenNode storage files don't exist yet. That raises EMCY 5000h (non-volatile memory) and blocks
  NMT start until parameters are saved once (1010h). Our master will need a defined first-boot behavior for this, and so
  will the ECU port.
- CI (`.github/workflows/ci.yml`): builds the image and runs the same check. **`vcan` works on GitHub's ubuntu-24.04 runners**
  (first run passed 2026-11-07), so CAN tests run in CI on every push. If a runner ever lacks `vcan`, the check is skipped, not failed.

**S1: Contract v0, single drive** (done, 2026-11-07)
- [`contract/`](../contract/README.md): `drive-contract.yaml` (bus, heartbeat, units, current, profile, homing, PDO map,
  CiA 402 codes, EMCY classes), `defaults-overlay.yaml` (manual defaults missing from the EDS), the vendor EDS (unmodified),
  and a loader that generates the startup sequence per node (`python -m contract 1`: 65 SDO writes, sizes from the EDS).
- `make test` (also in CI): 28 checks against the EDS, the manual's ranges, the S20 current limit, PDO limits (max 3 entries,
  mappable, max 8 bytes), COB-ID collisions on a 6-drive bus, bus load (< 30 % for 6 drives), heartbeat margins, csp period and
  homing timeout. Mutation-checked: deliberate contract errors are caught.
- Decided here: master node 127, async RPDOs (RPDO1 = controlword + target), TPDO1 = statusword + position on SYNC,
  TPDO3/4 disabled, run current 2003h = 88, everything written at every boot (only node ID and bit rate stored in the drive).
  Found: 2101h (stallGuard load) isn't PDO-mappable, so the master polls it by SDO during homing.
- Provisional until S5: homing/standby current, profile and csp limits, homing speed, stallGuard threshold.

**S2a ⇉ Drive emulator + slider (first visible milestone)** (done, 2026-11-07)
- `sim/drive/`: TMCM-1270 emulator. `core.py` (no CAN, testable with fake time): CiA 402 state machine; pp, pv, hm (method 35), csp;
  open-loop step counter coupled to the S20 model (`s20.py`: steps past an end stop are lost); stallGuard with stop-on-stall;
  firmware quirks (SOD-only objects, PDO remap rules, profile defaults 0, NMT stopped → FAULT). Strict assumptions where the manual
  is silent, switchable in `Quirks`: unwired switches with 2005h = 0 trip a limit EMCY, a stall faults the drive, targets outside the soft
  limits are rejected, too-high current gives an overtemp warning. `can_node.py`: NMT with boot-up, heartbeat producer/consumer
  (1029h), SDO via python-canopen, our own PDO and SYNC handling, EMCY.
- `sim/master/`: Python test master following the contract: boot, identity check, the 65-write startup sequence, homing (pv into
  the end stop, stallGuard EMCY/load or timeout, method 35, pp to park), csp at 10 ms with rate limits. UDP operator interface
  (:5005 commands, :5006 status) that the C master will take over.
- `sim/web/`: browser view on :8080. The leg position comes from TPDO1 on the bus, plus the simulated rod position from the
  emulator, so lost steps are visible. Slider, Home, Resume, Quick stop.
- `make sim` runs all three. Colima forwards :8080 to the Mac automatically. `make test`: 47 tests (contract, 16 core, 3 end-to-end on
  `vcan0`: home/park/follow, heartbeat loss → drive FAULT, drive reboot detected).
- Verified in the browser on the Mac: auto-homing (stallGuard), park at 5 mm, slider moves at 10 mm/s, quick stop → switch on
  disabled, resume, re-home from 52 mm. During homing after a reset the bus shows −11.8 mm while the rod is at 40 mm (open loop).
- Observed once: both heartbeats lost within 0.7 s (VM pause on the Mac, most likely). A 30 s measurement shows normal jitter is small
  (heartbeat max 102 ms, SYNC max 12.6 ms). Recovery via Home works. → S4: decide on automatic recovery from a transient loss.

**S2b ⇉ Master v0 (C)** (done, 2026-11-07)
- [`master/`](../master/README.md): portable C99 core (`sp_core`: bring-up, homing all legs together, csp, EMCY classes,
  reboot / heartbeat loss / NMT stopped; `sp_kin`: IK + workspace; `sp_motion`: pose / joystick / IMU / demo / legs inputs,
  pose smoothing, leg speed cap; `sp_app`: JSON commands and status). No OS calls, no malloc, time passed in.
- The master's CANopenNode object dictionary is **generated from the contract** (`contract/gen_c.py`, `make gen`): SYNC
  producer, HB consumer for 6 drives, one SDO client, 12 RPDOs and 18 TPDOs mirroring the drives' PDOs, mapped to
  arrays 2110h..2123h. Generated files are committed (the ECU build needs no Python); a test fails if they're stale.
- Unit tests with fake drives and fake time (`master/tests/test_core.c`, 16 tests, ASan/UBSan): boot, homing by EMCY and
  by load polling, pose following within the leg speed limit, stop/resume, reboot, heartbeat loss, EMCY classes,
  NMT stopped recovery, missing drive retry, SDO hang guard, status JSON, IMU/demo inputs.

**S3: C master replaces the test master** (done, 2026-11-07)
- `spmaster` = CANopenLinux main loop (`CO_main_basic.c`, single thread, 1 ms) + the core through
  `master/canopennode/sp_co_adapter.c` (shared with the ECU) + `master/linux/sp_linux_app.c` (UDP JSON interface,
  same protocol as the Python test master).
- `make sim` now runs 6 emulated drives + `spmaster` + the web view; `make sim-leg` keeps the single-leg setup with the
  Python test master. End-to-end tests against 6 emulated drives on vcan0 run in CI (`tests/master/test_spmaster_e2e.py`).
- Not done: a shared-timestamp recording of CAN log and simulator state (use `candump -L` + the truth port for now).

**S4: Robustness** (done in simulation, 2026-11-07)
- Fault paths covered by unit and end-to-end tests: missing boot-up (retry), heartbeat loss both ways (drive: 8130h +
  NMT stopped → master stops, homing kept, Resume restarts NMT and fault-resets), drive reboot while operational (rehome),
  stall in operation (jam injection → EMCY 3230h → rehome), ENN (5441h → rehome), SDO client hang.
- Decision (S2a open point): a transient heartbeat loss does **not** auto-recover; the master stops and keeps homing,
  the operator presses Resume. Unattended auto-recovery belongs to S9 ([agents/fair-features.md](agents/fair-features.md)).
- Emulator fault injection port (:5008) and web buttons: power cycle, power off, ENN, jam, heartbeat mute, slip.
- Tools: `tools/bus_stats.py` (timing, bus load: 14.5 % with 6 legs, SYNC 10.16 ms mean on CANopenLinux),
  `tools/drive_probe.py` (drive characterisation + compare; emulator baseline in `docs/measurements/`).
- Known limit: open loop. Lost steps without a stall EMCY (slip) are invisible to the master; only the simulator truth
  shows them. Mitigation if the bench shows it matters: periodic re-homing or stallGuard monitoring in operation.

### Phase 2: Ubuntu notebook, real CAN

**S5: Real drive on the bench** → step-by-step plan: [commissioning.md](commissioning.md)
- Commissioning tools: `tools/drive_probe.py` (fw check, behaviour), `tools/set_node_id.py` (2705h + 2706h; or LSS).
- PC master → PCAN-USB (`can0`) → one TMCM-1270 + S20. Measure the current and temperature, and tune stallGuard homing
  (threshold, speed, current, timeout).
- Compare the real object dictionary with the emulator. Measure csp tracking at 10 ms and jitter tolerance, and whether
  RPDOs act on SYNC or on arrival. Tune the emulator.

**S6: Six drives and coordinated motion** (done in simulation, 2026-11-07; real drives after S5)
- Six emulated nodes, homing all together, coordinated csp from the pose, bus load 14.5 %.

**S7: Stewart platform** (done with **provisional geometry**, 2026-11-07; CAD: [agents/cad-geometry.md](agents/cad-geometry.md))
- `config/platform.yaml` (6-6 layout sized for the S20-100, neutral at 50 mm stroke), `stewart/geometry.py` (Python IK/FK),
  `master/core/sp_kin.c` (C IK, workspace limiting; cross-checked with Python on 200+ poses).
- Web view: 3D canvas (no external libraries, works offline), platform from bus FK and from simulator truth (lost steps
  visible), commanded pose; inputs: pose sliders, virtual joystick, IMU sliders or the device's orientation sensor, demo,
  per-leg sliders; legs table; fault injection.
- To do with the CAD: real joints and closed length, joint angle limits in the workspace check, meshes in the view.

### Phase 3: ECU

**S8: ECU port** → brief: [agents/ecu-port.md](agents/ecu-port.md)
- `CO_driver` on the TTC CAN API, a PXROS single task calling the core's step, the same OD and adapter.
- Setpoint sources: analog joystick (ADC) and the IMU on the ECU. HIL: ECU → bus → simulator on the Ubuntu notebook.

**S9: Real platform and fair hardening** → brief: [agents/fair-features.md](agents/fair-features.md)
- Power-up and homing flow, recovery after an e-stop, the operator input modes (joystick / IMU), status display,
  unattended behavior.

```
S0 ─► S1 ─┬─► S2a (emulator + slider) ─┐
          └─► S2b (C master) ──────────┴─► S3 ─► S4 ─► S6 ─► S7 ─► S8 ─► S9
hardware + Ubuntu notebook ─► S5 (bench) ──► feeds S4/S6
```

## Still open (not blocking Phase 1)

| # | Question | Needed by |
|---|---|---|
| 1 | What exactly does the hard e-stop cut: drive 24 V, ECU power, or both? | S4 (reboot recovery), S9 |
| 2 | How is the IMU connected to the ECU (its own CAN channel, SPI, …), and at what rate? | S7/S8 |
| 3 | Joystick mapping: which axes control which pose components? (default now: x → roll, y → pitch) | S9 |
| 4 | Idle behavior at the fair: hold the last pose, return to park, or a slow idle motion? (demo motion exists, off by default) | S9 |
| 5 | S20 max force and speed (curves only in the PDF image), and the platform payload | S5, S7 |
| 7 | Positive target = extend the rod? (nothing assumes otherwise; check on the bench) | S5 |
| 6 | ~~Does Docker on the Mac provide `vcan`?~~ Docker Desktop: no. **Colima: yes** (verified, `scripts/mac/colima-vcan.sh`) | done |
