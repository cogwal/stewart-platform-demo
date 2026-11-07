# Measurements

Dated notes and raw data from simulation runs and bench sessions. The simulation baseline is what the bench
results get compared against ([../commissioning.md](../commissioning.md), [../agents/sim-tuning.md](../agents/sim-tuning.md)).

| File | What |
|---|---|
| `emulator-probe.json` | `tools/drive_probe.py --allow-motion --endstop` against the emulator: the baseline for `--compare` |
| `YYYY-MM-DD-*.md` | notes: setup, what was expected, what happened, numbers |
| `YYYY-MM-DD-*.json` | probe reports, soak summaries |
| `YYYY-MM-DD-*.jsonl` | session recordings (`SIM_RECORD=… make sim`): CAN frames, master status, truth, operator input, one JSON object per line with `t` (monotonic), `wall`, `kind`, `data` |
| `YYYY-MM-DD-*.log` | `candump -L` logs |

Keep raw files small enough for git (a few MB); trim long recordings to the interesting part.
