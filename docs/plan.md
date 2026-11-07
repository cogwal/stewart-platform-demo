# Plan: slices (draft)

Status: first grill-me session done (2026-11-07). Answers recorded below. Remaining open points at the end.
Hardware facts and their sources: [hardware-notes.md](hardware-notes.md).

## Decisions

| Topic | Decision | Why / note |
|---|---|---|
| ECU | **TTControl TTC2000 A2G series, Infineon AURIX TC3xx**, plain C/C++, **PXROS** RTOS, everything in **one task**, TTC's own CAN interface API (provided at porting time) | CANopenNode runs as-is. We write a `CO_driver` on the TTC CAN API |
| Actuator | **Actuonix S20, 100 mm stroke** (`S20-100-38-B`) | Closed 174.8 mm, extended 274.8 mm hole to hole. 0 to 2,560,000 µsteps at 1/256 |
| Platform CAD | exists and comes later | Start with **a single actuator in simulation**. Add the Stewart platform from the CAD afterwards |
| ROS 2 | **Not used** | The ECU doesn't run it, the CAN bus is the only interface, and the web view doesn't need it |
| [tmcl_ros2](https://github.com/analogdevicesinc/tmcl_ros2) | **Not used** | It speaks TMCL (TMCL firmware), not CANopen, and only supports ROS 2 Humble |
| Master stack | **CANopenNode, starting from [CANopenLinux](https://github.com/CANopenNode/CANopenLinux)** on the PC | The same stack runs on the ECU. Only the CAN driver and the main loop differ |
| Master language | **C** for the portable core. C++ is allowed on the ECU side if useful | |
| Drive emulator | **Python** ([python-canopen](https://github.com/christiansandberg/canopen) local node from the EDS plus a defaults overlay) | Fast to iterate and easy to script faults. A separate implementation from the master |
| Dev environment | **Docker** (`.devcontainer/` + `docker compose`), Ubuntu image, `vcan` | Phase 1 on the **Mac** (simulation only). Phase 2 on an **Ubuntu notebook** with USB-CAN. **Docker Desktop on the Mac has no CAN support** (checked 2026-11-07: kernel `7.0.14-linuxkit`, `CONFIG_CAN is not set`), so Phase 1 needs a Docker runtime with an Ubuntu VM kernel (Colima/Lima, **to verify**). See [PC setups](#pc-setups) |
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
A runtime whose VM runs a normal Ubuntu kernel (Colima/Lima) can load the `vcan` module.

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

**S0: Docker dev environment and CAN check**
- Ubuntu image: can-utils, build tools, CANopenLinux as a pinned submodule, Python env. A `vcan0` setup script.
- Docker Desktop on the Mac: **no CAN in its kernel** (checked). Set up Colima (Lima VM, Ubuntu kernel) as the Docker
  runtime and check that `modprobe vcan` works in the VM. If not, use a plain Ubuntu VM (Lima/UTM) with Docker inside.
  Also check a cloud session and GitHub Actions.
- Done when: the stock CANopenLinux demo heartbeats on `vcan0`, and `candump` plus a python-can script see it, on the Mac and in CI.

**S1: Contract v0, single drive**
- `contract/drive-contract.yaml` + `.md` from the draft above, with node IDs as parameters. EDS and defaults overlay committed.

**S2a ⇉ Drive emulator + slider (first visible milestone)**
- Emulator: one TMCM-1270 node with NMT, heartbeat, SDO, dynamic PDOs, the CiA 402 state machine, pp and csp, pv plus
  stall at the end stop (EMCY 3230h), method 35. An S20 model with 0–100 mm travel and an end stop. Drive quirks
  (2005h only writable in SWITCH_ON_DISABLED, profile values 0 by default, NMT stopped faults the drive).
- A small **Python test master** (python-canopen) that configures, homes and follows a setpoint.
- **Browser page:** a slider sends the setpoint to the test master. The leg view reads 6064h from the bus.
- Done when: on the Mac, moving the slider moves the simulated S20 in the browser, through CANopen on `vcan0`.

**S2b ⇉ Master v0 (C)**
- App core on CANopenLinux: boot-up handling, startup SDO list (non-blocking), heartbeat monitoring, CiA 402
  enable, homing sequence, csp setpoint following with rate limits, and a setpoint input interface.
- Unit tests of the core with a fake CAN and fake time.

**S3: C master replaces the test master**
- The slider now feeds the C master. Master and emulator run on `vcan0` and are tested end to end in CI. CAN log and
  simulator state are recorded with a shared timestamp.

**S4: Robustness, single drive**
- Fault paths: missing or late boot-up, heartbeat loss in both directions (8130h), a drive reboot while operational,
  a stall in operation, limits (FF01h), a wrong or missing current setting, fault reset. Bus-off is emulated as silence.
- Scenario runner for scripted adversarial runs.

### Phase 2: Ubuntu notebook, real CAN

**S5: Real drive on the bench**
- Commissioning tool: fw 3.27 check or flash, node ID (2705h + 2706h, or LSS).
- PC master → PCAN-USB (`can0`) → one TMCM-1270 + S20. Measure the current and temperature, and tune stallGuard homing
  (threshold, speed, current, timeout).
- Compare the real object dictionary with the emulator. Measure csp tracking at 10 ms and jitter tolerance, and whether
  RPDOs act on SYNC or on arrival. Tune the emulator.

**S6: Six drives and coordinated motion**
- Six emulated nodes, synchronized homing, coordinated csp trajectories, bus-load check.

**S7: Stewart platform from CAD**
- Geometry from the CAD. IK in C in the master core (pose setpoint → 6 leg setpoints), FK in Python for the view.
  three.js scene with CAD meshes. Workspace limits.
- Setpoint sources on the PC: slider or virtual joystick for the pose, simulated IMU orientation.

### Phase 3: ECU

**S8: ECU port**
- `CO_driver` on the TTC CAN API, a PXROS single task calling the core's step, the same OD.
- Setpoint sources: analog joystick (ADC) and the IMU on the ECU. HIL: ECU → bus → simulator on the Ubuntu notebook.

**S9: Real platform and fair hardening**
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
| 3 | Joystick mapping: which axes control which pose components? | S7 |
| 4 | Idle behavior at the fair: hold the last pose, return to park, or a slow idle motion? | S9 |
| 5 | S20 max force and speed (curves only in the PDF image), and the platform payload | S5, S7 |
| 6 | ~~Does Docker Desktop on the Mac provide `vcan`?~~ **No** (CONFIG_CAN not set). Does Colima's VM? | S0 |
