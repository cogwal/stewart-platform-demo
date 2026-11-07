# C master

The CANopen master for the six TMCM-1270 legs. One portable core, built on CANopenNode, runs on the PC
(CANopenLinux, `build/master/spmaster`) and later on the TTC2000 ECU ([../docs/agents/ecu-port.md](../docs/agents/ecu-port.md)).

```
core/            portable C99, no OS, no malloc, time passed in, never blocks
  sp_core.*        drives: bring-up (NMT reset, identity, SWITCH ON DISABLED, 65 startup SDOs, NMT start),
                   homing all legs together (pv into the end stop, stallGuard EMCY / 2101h load / timeout,
                   method 35, park in pp, csp), csp streaming with per-leg rate limits, EMCY classes,
                   reboot / heartbeat loss / NMT stopped handling, operator Home / Stop / Resume
  sp_kin.*         inverse kinematics, workspace check and limiting (float; same as stewart/geometry.py)
  sp_motion.*      inputs (pose, joystick, IMU, demo, legs) → limits → pose smoothing → leg speed cap → IK
  sp_app.*         glue: core + motion, JSON commands, JSON status
  sp_port.h        what the core needs from the bus (NMT, SDO, three RPDO kinds, log)
generated/       from contract/drive-contract.yaml + config/platform.yaml by `make gen` (committed)
  OD.c / OD.h      the master's CANopenNode object dictionary
  sp_config_gen.*  constants, startup SDO table, EMCY table, geometry
canopennode/     sp_co_adapter.*: sp_port_t on CANopenNode v4 (shared by PC and ECU)
linux/           sp_linux_app.c: CANopenLinux application hooks + UDP operator interface
tests/           test_core.c (fake drives, fake time), fake_bus.*, kin_dump.c (C vs Python IK)
```

## Build and run
From the repo root (in the dev container through make):
```bash
make ctest       # unit tests
make spmaster    # build/master/spmaster
make sim         # spmaster + 6 emulated drives + web view on :8080
```
Direct: `SP_LEGS=6 build/master/spmaster -i 127 -s build/master-storage/ vcan0`. Environment: `SP_LEGS`,
`SP_NODES` (e.g. `21,22,23,24,25,26`), `SP_AUTO_HOME`, `SP_CMD_PORT` (5005), `SP_STATUS_PORT` (5006).

## Operator interface (PC)
UDP JSON on 127.0.0.1, same as the Python test master: commands to :5005 (`home`, `stop`, `resume`, `source`, `pose`,
`joystick`, `imu`, `legs`, `setpoint`; see `core/sp_app.h`), status from :5006 every 100 ms (phase, reason, homed,
input, commanded pose, per leg: boot state, NMT, statusword, actual/target mm, EMCY log).

## Phases
`boot` → `idle` (configured, not homed) → `homing` → `run`. `stopped` = quick-stopped with homing kept (Resume);
`fault` = homing lost or failed (Home). What leads where:

| Event | Result |
|---|---|
| operator Stop, EMCY class stop_all (e.g. 8130h drive lost master heartbeat), drive heartbeat lost, drive left OPERATION ENABLED | all legs quick stop → `stopped` |
| drive rebooted (boot-up / pre-operational while configured), EMCY stall / stop_all_rehome / config_error (ENN 5441h, overtemp, stall in operation, …), any failure during homing | all legs quick stop → `fault`, unhomed |
| drive missing at boot | retried every 2 s, phase stays `boot` |

Unattended option: `supervisor.auto_recover_after_s` in `config/platform.yaml` (or `SP_AUTO_RECOVER_S`): after a stop or
fault the master waits that long, then resumes (homing kept) or homes, at most `auto_recover_max_tries` times per
incident. Off by default.

## Design notes
- One SDO client, shared by the legs in turn (65 writes × 6 drives ≈ 0.5 s on vcan).
- The master's PDOs mirror the drives' PDOs: TPDO send requests are flags in CANopenNode, so the core never sends two
  frames on the same drive RPDO in one step.
- The cyclic part runs on each SYNC the CANopenNode producer sends (the adapter detects it from the SYNC timer).
- Measured with `tools/bus_stats.py` on vcan: bus load 14.5 % with 6 legs, SYNC 10.16 ms mean.
