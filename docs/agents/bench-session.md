# Brief: assist a commissioning session on the bench

## Situation
The user is at the Ubuntu notebook with a PCAN-USB (`can0`) and one or more TMCM-1270 + S20 on the bench. The plan
is [../commissioning.md](../commissioning.md); follow it step by step and don't skip the safety notes (unloaded
actuator, ENN through a switch, current-limited supply, one master on the bus).

## Your job
- Run the commands of each step (the user confirms each motion step before it runs: real hardware moves).
- Keep a candump log running for every step and save it under `docs/measurements/` with the date and step.
- After each step, write down what was expected (from the simulation) and what happened, in a dated note
  `docs/measurements/YYYY-MM-DD-bench.md`. Numbers, not impressions.
- Turn findings into changes only when the user agrees: contract values (`make gen`, `make test`), the defaults
  overlay, or a list for the emulator ([sim-tuning.md](sim-tuning.md)).

## Tools
| Tool | Use |
|---|---|
| `tools/drive_probe.py` | characterise one drive; `--allow-motion`, `--endstop`; `--compare A B` |
| `tools/bus_stats.py` | SYNC/heartbeat/PDO timing, bus load |
| `tools/set_node_id.py` | set a drive's node ID (one drive on the bus); untested on hardware, keep a log running |
| `SIM_NO_DRIVES=1 [SIM_LEGS=1] make sim CAN_IFACE=can0` | the C master + web view against the real drives |
| `candump -L can0`, `cansend` | raw bus |

## Known unknowns to resolve on the bench (see commissioning.md for where each goes)
Power-on heartbeat (1017h), firmware revision, SDO aborts in the startup sequence, direction of positive targets,
real currents and temperature, stallGuard values and whether a stall faults the drive, csp on arrival vs on SYNC,
heartbeat-loss reaction and the statusword after it, SDO in NMT STOPPED, behaviour of unwired switches (2005h = 0x13).
