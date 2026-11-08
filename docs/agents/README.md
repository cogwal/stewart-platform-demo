# Context files for future sessions

Each file here is a self-contained brief for one piece of work that can't be done (or finished) without
hardware, files or decisions that weren't available yet. Start a new session with: *"Read
docs/agents/<file>.md and do it."* Each brief lists what is already in place, what to do, how to check it,
and what not to break. For the big picture first, see [../architecture.md](../architecture.md).

| File | Work | Blocked on |
|---|---|---|
| [ecu-port.md](ecu-port.md) | Run the C master on the TTC2000 A2G (PXROS, TTC CAN API) and swap it with the PC on the bus | TTC CAN API + toolchain from the user |
| [sim-tuning.md](sim-tuning.md) | Make the drive emulator behave like the measured TMCM-1270 | bench measurements ([../commissioning.md](../commissioning.md)) |
| [cad-geometry.md](cad-geometry.md) | Replace the provisional platform geometry with the CAD | the CAD model |
| [bench-session.md](bench-session.md) | Assist a commissioning session on the Ubuntu notebook with real drives | hardware on the bench |
| [fair-features.md](fair-features.md) | Joystick/IMU on the ECU, idle behaviour, status display, unattended operation | open questions in [../plan.md](../plan.md) |

Shared ground rules (all briefs):
- The contract (`contract/drive-contract.yaml`) and `config/platform.yaml` are the single sources of truth.
  The C master's config is generated from them (`make gen`); never hand-edit `master/generated/`.
- `make test` must stay green (contract checks, emulator, C unit tests, C vs Python kinematics, end-to-end on vcan0).
  CI runs the same on every push.
- The master core (`master/core/`) stays portable C99: no OS calls, no dynamic memory, time passed in,
  nothing blocking. Platform code lives in `master/canopennode/` (shared CANopenNode adapter) and per platform
  (`master/linux/`, later `master/ttc/`).
- Commit and push only when the user asks (or the session's brief says so).
