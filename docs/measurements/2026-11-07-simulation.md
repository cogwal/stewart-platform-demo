# 2026-11-07: simulation baseline (Mac, Colima, vcan0)

Setup: `spmaster` (C master on CANopenLinux, single thread, 1 ms loop) + 6 emulated TMCM-1270 (Python, one process,
one thread per drive) + web view, all in the dev container on the Colima VM.

## Bus (tools/bus_stats.py, 10 s, following a pose)
| Stream | Rate | Mean | Min | Max | σ |
|---|---|---|---|---|---|
| SYNC (master) | 98.4 Hz | 10.16 ms | 10.00 | 10.51 | 0.12 |
| RPDO1 per drive (csp target) | 98.3 Hz | 10.16 ms | 9.12 | 11.21 | 0.30 |
| TPDO1 per drive (statusword + position) | 98.3 Hz | 10.16 ms | 7.8 | 11.8 | 0.3–0.55 |
| Heartbeat drives / master | 10 Hz | 100.7 / 100.15 ms | | 102.7 / 100.6 | |

Bus load estimate 14.47 % at 1 Mbit/s (S1 estimate: ≈14 %). SYNC is 1.6 % slow because CANopenNode's producer
restarts its timer when it fires (granularity of the 1 ms loop); the ECU should do better with a hard timer.

## Timeline (6 legs, rods starting at 37–52 mm)
- boot (reset, identify, 65 SDO writes × 6, NMT start): ≈ 0.4 s
- homing travel at 4 mm/s until stallGuard: up to 13 s (longest leg), park at 5 mm: ≈ 1.3 s
- rise to neutral (50 mm) under the leg speed cap (10 mm/s): ≈ 5–7 s

## Drive probe (emulator): docs/measurements/emulator-probe.json
The baseline for the bench compare. Notable emulator assumptions visible in it: boot-up 0.4 ms after NMT reset,
no heartbeat before 1017h is written, CiA 402 transitions 1.5–7 ms (TPDO2 inhibit 5 ms), stallGuard load 600 when
free (also at standstill) and 0 at the stop, heartbeat-loss reaction 276 ms to EMCY 8130h and 303 ms to NMT stopped,
csp tracking 0.15 mm RMS on a 1 Hz ±2 mm sine, quick-stop distance 0.17 mm from 4 mm/s.

## Soak (tools/soak.py)
Random poses, demo, joystick, stop/resume, heartbeat glitches, drive reboots, ENN, jams and slips, with
operator-style recovery and a bus-vs-truth check at standstill (0.1 mm).
15 minutes, seed 3 (`2026-11-07-soak-15min.json`): 61 actions (17 poses, 7 demo, 4 joystick, 7 stop/resume,
5 heartbeat glitches, 6 drive reboots, 5 ENN drops, 2 jams, 6 slips, 2 homes), 26 recoveries, **0 failures**.
A 60 s soak runs in `make test` (`tests/master/test_soak.py`, `SOAK_SECONDS` to change).
