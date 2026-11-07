# Plan: slices (draft)

Status: draft for the grill-me session. Nothing here is decided until the open questions are closed.
Hardware facts and their sources: [hardware-notes.md](hardware-notes.md).

## Decisions so far

| Decision | Choice | Why |
|---|---|---|
| ROS 2 | **Not used** | The ECU doesn't run it, the CAN bus is the only interface, and the web view doesn't need it. |
| [tmcl_ros2](https://github.com/analogdevicesinc/tmcl_ros2) | **Not used** | It speaks Trinamic's TMCL protocol (TMCL firmware), not CANopen, and only supports ROS 2 Humble. Our master is a CANopen master because the ECU will be one. |
| Master stack | **CANopenNode, starting from [CANopenLinux](https://github.com/CANopenNode/CANopenLinux)** | The same stack runs on the ECU. Only the CAN driver and the main loop differ. |
| Master language | **C** (portable core, no OS or driver calls) | Required by the ECU. |
| Drive emulator | **Python** ([python-canopen](https://github.com/christiansandberg/canopen) local nodes from the vendor EDS) | Fast to iterate and easy to script faults. Being a separate implementation means it won't share the master's misunderstandings. |
| Dev environment | **Docker container** (a `.devcontainer/` plus `docker compose`), Ubuntu, with `vcan` | Same setup on the Mac, in cloud sessions and in CI. VS Code can open it as a devcontainer, or it runs with plain `docker compose`. |
| Visualization | **Browser (three.js)** served from the simulator container on a mapped port | No GUI forwarding out of Docker needed. Suits a big screen at the fair. |
| Drive firmware | **fw 3.27, EDS `TMCM-1270_V3.27.eds`** as the contract basis | The EDS matches the included hex and adds LSS. Drives get flashed to 3.27 if they ship older. |
| Motion mode | **csp** for coordinated motion, **pp** for single moves (park, safe pose), **homing** mode for referencing | PP can't blend or queue setpoints (bits 5/9 unsupported), so 6 legs can only stay coordinated if the master streams the trajectory. |

## Architecture: what is shared and what is per platform

```
            ┌───────────────────────── master (C) ─────────────────────────┐
            │  app core (portable, shared PC + ECU)                        │
            │    supervisor / state machine   motion (traj, later IK)      │
            │    CiA 402 master per node      boot + SDO config sequencer  │
            │  ───────────────── step(now_us) / hooks ──────────────────── │
            │  CANopenNode (shared)  +  master OD (generated, shared)      │
            │  ───────────────────────── CO_driver ─────────────────────── │
            │  platform/linux: CANopenLinux, SocketCAN   platform/ttc: ECU │
            └──────────────────────────────┬───────────────────────────────┘
                                           │ CAN (vcan0 / USB-CAN / real bus)
            ┌──────────────────────────────┴───────────────────────────────┐
            │ sim (Python): N × TMCM-1270 emulator → S20 model → platform  │
            │               → web viz (WebSocket) + logging + fault inject │
            └──────────────────────────────────────────────────────────────┘
```

Rules for the app core:
- Time is passed in (`now_us`), never read from a clock. That allows tests faster than real time,
  reproducible runs and replay.
- No blocking calls. SDO transfers and boot steps are state machines advanced on every step.
- Configuration (node IDs, number of drives, limits, unit scale, current) comes from one generated header
  or OD parameters. The ECU has no file system.
- CANopenLinux already provides application hooks (`CO_application.h`: program start, communication
  reset, async and RT callbacks). The core plugs in there, and the ECU port provides the same hooks.
  **To verify in S0.**

## The contract (defined early so both sides can be built in parallel)

`contract/` holds what the master and the emulator both build against. A first draft, from the manual:

- **Bus:** bit rate (default 1000 kbit/s, to be decided), node IDs (e.g. drives 1–6, master 127). Every drive
  ships as node 1, so the IDs are assigned during commissioning.
- **Heartbeat:** drives produce at e.g. 100 ms (1017h, default off). Drives consume the master's heartbeat (1016h).
  The master monitors all drives.
- **Startup SDO list per drive** (in PRE-OPERATIONAL, statusword in SWITCH_ON_DISABLED):
  2003h current · 2004h standby current · 2005h switch config · 2000h microsteps · 208Ch units ·
  607Dh software limits · 6081/6083/6084h profile · 6085h quick-stop decel · 6098/6099/609Ah homing ·
  60C2h interpolation period = master cycle · 1016h/1017h heartbeat · PDO remap.
- **PDO map (proposal):** RPDO1 = 6040 + 607A (csp/pp setpoint), RPDO2 = 6040 + 6060 (mode switch),
  TPDO1 = 6041 + 6064 on SYNC, TPDO2 = 6041 + 6061 on change. Max 3 objects per PDO.
- **Modes and sequences:** enable sequence (6 → 7 → 15), pp move handshake (bit 4, statusword bit 12),
  homing start/finish, csp streaming and its period.
- **EMCY handling:** which codes are a stop, a fault reset, or just a log entry.
- **Units:** microsteps (2^2000h per full step, 100 full steps per mm on the S20) and PPS.
- `eds/TMCM-1270_V3.27.eds` (vendor, unmodified) plus `defaults-overlay.yaml`, the manual defaults that
  are missing from the EDS.
- Contract tests: the emulator accepts exactly this startup sequence, and the master produces it. Later,
  the same SDO dump is read from a real drive to check the emulator.

A second, smaller interface: sim → viz (pose plus per-leg state over WebSocket, JSON).

## Slices

Each slice ends with something that runs and is demonstrated. **⇉** marks slices that can run in parallel.

### S0: Docker dev environment and CAN check (blocks everything)
- Ubuntu image: can-utils, build tools, CANopenLinux as a pinned submodule, Python env.
  A `vcan0` setup script, run in a container with `NET_ADMIN`. `vcan` is a host kernel module, so it is checked per host.
- Check `vcan` works in: Docker Desktop on the Mac, a cloud session, and GitHub Actions. Where it
  doesn't, document the fallback (a Linux VM such as OrbStack/Lima, or a userspace bus for CI).
- Done when: the stock CANopenLinux demo runs as a node on `vcan0`, and `candump` plus a python-can
  script see its heartbeat. This works in CI.

### S1: Contract v0, single drive (now unblocked)
- Turn the draft above into `contract/drive-contract.yaml` plus a readable `.md`, with node IDs as parameters.
- Settle the open questions it depends on: bit rate, cycle time, current value, homing method.
- Done when: the contract is reviewed and the EDS and defaults overlay are committed.

### S2a ⇉ Drive emulator v0 (Python)
- One TMCM-1270 node: EDS plus defaults overlay, NMT boot-up, heartbeat producer and consumer, SDO server,
  dynamic PDO mapping (max 3 entries), CiA 402 state machine, pp with a trapezoidal ramp, homing method 35,
  an ideal S20 (travel limits).
- Behaves like the real drive in the details that bite: 2005h only writable in SWITCH_ON_DISABLED, switches
  enabled by default, profile values 0 by default (move rejected or never reached), NMT stopped faults the drive.
- Tested with pytest plus a python-canopen test master on `vcan0`.

### S2b ⇉ Master v0 (C)
- App core on CANopenLinux: wait for boot-up, run the startup SDO list (non-blocking), heartbeat
  monitoring, CiA 402 enable, pp moves through a list of positions.
- Unit tests of the core with a fake CAN and fake time. No emulator needed.

### S3: Tracer bullet, single actuator end to end
- Master and emulator on `vcan0`, started and checked by one pytest in CI.
- CAN log and simulator state recorded with a shared timestamp. A minimal web view shows one leg moving.
- Done when: `make demo` shows the PC master moving the simulated S20 back and forth in the browser.

### S4: csp and robustness, single drive
- csp: a trajectory generator in the master core, SYNC plus setpoint streaming at the contract's period. The emulator
  follows the drive's csp behavior (velocity = Δ/period).
- Homing per the contract. Fault paths: missing or late boot-up, heartbeat loss in both directions (EMCY 8130h),
  ENN off (5441h), limits (FF01h), wrong or missing current config, fault reset. Bus-off is emulated as silence.
- Scenario runner for scripted adversarial runs.

### S5 ⇉ Real drive on the bench (once hardware is here)
- Commissioning tool: flash fw 3.27 if needed, set the node ID and bit rate (2705h/2704h + 2706h, or LSS).
- PC master (Linux) → USB-CAN → one TMCM-1270 + S20. Set the current (measure phase current and temperature).
- Read the real object dictionary and compare it with the emulator. Measure pp ramps, csp tracking and jitter
  tolerance, and whether RPDOs act on SYNC or on arrival (the manual contradicts itself). Then tune the emulator.
- Can run alongside S4.

### S6: Six drives and coordinated motion
- Six emulated nodes. Bus-load check against the budget (about 14 % at 10 ms, 1 Mbit/s).
- Coordinated csp trajectories for all six legs, and synchronized enable, homing and fault handling across the drives.

### S7 ⇉ Kinematics and 3D view (independent of CAN, can start after S0)
- Geometry in one config (from CAD). IK in C in the master core. FK (Newton) in Python in the sim.
  IK and FK checked against each other in tests.
- three.js scene with CAD meshes driven by the sim's FK pose. Workspace and leg limits checked against the CAD.

### S8: ECU port
- TTControl `CO_driver`, timebase and main loop, with the same app core and the same OD.
- HIL: ECU → USB-CAN → sim on a Linux host. USB passthrough into Docker on a Mac is unreliable.

### S9: Real platform and fair hardening
- E-stop chain (ENN), safe pose, power-up and homing behavior, unattended demo loop, error display.

```
S0 ─► S1 ─┬─► S2a ─┐
          └─► S2b ─┴─► S3 ─► S4 ─► S6 ─► S8 ─► S9
S0 ──────────────► S7 (kinematics + 3D) ──────┘
hardware arrives ─► S5 (bench) ──► feeds S4/S6
```

## Open questions (for the grill-me session)

| # | Question | Blocks | Default if no answer |
|---|---|---|---|
| 1 | Exact TTControl ECU model, and is it programmed in C or CODESYS? | S8, whether CANopenNode fits | — |
| 2 | Which S20 stroke (15/30/50/100 mm)? Is there a CAD of the platform? | S1 limits, S7 | — |
| 3 | Homing: a switch per leg, or end stop + method 35? | S1, S4 | a switch on REFL, method 17 |
| 4 | Bit rate and csp cycle time | S1 | 1 Mbit/s, 10 ms |
| 5 | Does the ECU need extra frames on this bus (other devices, diagnostics)? | bus budget | no |
| 6 | Where does the PC side run for HIL and at the fair? | S8 | a Linux laptop/NUC |
| 7 | Run current: is the 0.6 A rating RMS or peak? | S1, S5 | assume peak (2003h ≈ 90) |
| 8 | Which USB-CAN adapter? | S5 | one with a mainline SocketCAN driver (PEAK, Kvaser, …) |
| 9 | What does the demo actually show? A scripted motion loop, live input (joystick), fault demos? | S6, S9 | a scripted loop |
| 10 | Safety at the fair: e-stop, enclosure, who can touch it? | S9 | e-stop → ENN on all drives |
