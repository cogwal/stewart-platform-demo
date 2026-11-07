# Brief: tune the drive emulator to the measured TMCM-1270

## Goal
After the bench session ([../commissioning.md](../commissioning.md)), the emulator (`sim/drive/`) must behave like the
real drive wherever the master or the operator could notice the difference. Done when
`tools/drive_probe.py --compare` between a fresh emulator report and the bench report shows no relevant differences,
and the bench's candump logs replayed through the master logic give the same decisions.

## Inputs (from the bench session)
- `docs/measurements/*-probe-*.json`: probe reports of the real drive (passive, motion, end stop).
- `build/bench-*.log` / `docs/measurements/*.log`: candump `-L` logs of each commissioning step.
- The dated notes in `docs/measurements/` (currents, stallGuard values, fault behaviour table from step 6).
- Baseline: `docs/measurements/emulator-probe.json` (emulator as of the C master milestone).

## Where the behaviour lives
| What | File | Notes |
|---|---|---|
| CiA 402 state machine, modes pp/pv/hm/csp, stallGuard, current/overtemp, soft limits, ENN | `sim/drive/core.py` (`DriveCore`) | no CAN, fake-time testable (`tests/sim/test_drive_core.py`) |
| Switchable assumptions where the manual is silent | `sim/drive/core.py` `Quirks` | `unwired_switches_trip`, `stall_faults`, `reject_targets_outside_limits`, `current_limit`, `overtemp_after_s` |
| NMT, heartbeat producer/consumer (1016h/1029h), SDO server, PDO mapping rules, SYNC/TPDO timing, EMCY frames | `sim/drive/can_node.py` (`CanDrive`) | 1 ms tick thread per drive |
| Power-on object values | EDS defaults + `contract/defaults-overlay.yaml` (via `sim/od_store.py`) | the overlay is the place for measured power-on values |
| The actuator: stroke, end stops, lost steps, jam | `sim/drive/s20.py` | no force/speed model yet |
| Fault injection | `sim/drive/__main__.py` (UDP :5008) | power_cycle, power_off, enn, jam, mute_hb, slip |

## Procedure
1. Regenerate the emulator report with the same options as the bench report, e.g.
   `python -m sim.drive --nodes 1 &` then `python tools/drive_probe.py --node 1 --allow-motion --endstop --out build/probe-sim.json`,
   and compare: `python tools/drive_probe.py --compare build/probe-sim.json docs/measurements/<bench>.json`.
2. For each difference decide: emulator wrong (fix it), contract wrong (fix the contract, `make gen`, check the master),
   or irrelevant (document why in the note). Typical items, in order of impact on the master:
   - **SDO aborts** during the startup sequence (an object the real drive refuses, or refuses in this state).
   - **Power-on values** that differ from the EDS (`objects` section) → `defaults-overlay.yaml`.
   - **Stall behaviour**: does EMCY 3230h fault the drive (`Quirks.stall_faults`)? 2101h values free running vs at the stop vs
     standstill (the emulator reports 600 / 0 / 600; the real stallGuard reading at standstill is likely meaningless or 0).
     Model 2101h as a function of speed and load if the real curve matters for homing.
   - **Heartbeat-loss reaction**: time to EMCY 8130h and to NMT stopped, the statusword after (emulator: 0x0240 read
     back), whether SDO still answers in NMT STOPPED (emulator does; CiA 301 says no).
   - **Timing**: CiA 402 transition latency (emulator 1.5 to 7 ms, mostly its TPDO2 inhibit time of 5 ms), TPDO1 after
     SYNC, boot-up time after NMT reset (emulator 0.4 ms; a real drive needs longer, which matters for the master's
     boot timeout), first heartbeat after boot-up.
   - **Motion**: pp duration vs theory, pv speed accuracy, csp tracking (emulator 0.15 mm RMS on a 1 Hz ±2 mm sine:
     linear interpolation over 60C2h), whether csp targets act on arrival or on SYNC, quick-stop distance.
   - **Unwired switches** with 2005h = 0x13: does anything trip (`Quirks.unwired_switches_trip`)?
   - **Overcurrent / overtemp** behaviour vs `Quirks.current_limit` and `overtemp_after_s`.
   - **S20 model**: speed/force limits and missed steps under load (`s20.py` has none). Only worth it if the bench shows
     missed steps within the planned speed range.
3. Make the emulator match, keep the old behaviour available as a `Quirks` flag if it was a deliberate assumption.
4. Update tests: `tests/sim/test_drive_core.py` (core behaviour), the end-to-end tests if the master's view changes.
   The C master's fake drive (`master/tests/fake_bus.c`) is intentionally simple; change it only if a master decision
   depends on the new behaviour.
5. Regenerate `docs/measurements/emulator-probe.json` and commit it with the emulator change, so the next compare starts
   from the new baseline.
6. Run the full simulation (`make sim`) and look at it: homing, pose following, fault injection.

## Don't
- Don't tune the emulator to the master: it must stay an independent implementation of the drive (that's what makes
  the end-to-end tests worth something).
- Don't change the contract without `make gen` + `make test` (the master's tables are generated from it).
