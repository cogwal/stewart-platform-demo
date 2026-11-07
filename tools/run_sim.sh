#!/usr/bin/env bash
# Run the single-leg simulation inside the dev container: drive emulator + Python test master +
# web view, all on one CAN interface. Open http://localhost:8080 on the host.
#
# Usage: tools/run_sim.sh [iface] [node] [rod-mm-at-power-up]
set -euo pipefail

IFACE="${1:-${CAN_IFACE:-vcan0}}"
NODE="${2:-1}"
ROD_MM="${3:-37}"
cd "$(dirname "$0")/.."

ip link show "$IFACE" >/dev/null 2>&1 || {
    echo "CAN interface $IFACE not found. On the Mac run 'make vcan' first." >&2
    exit 1
}

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

python3 -m sim.web --iface "$IFACE" --node "$NODE" & pids+=($!)
python3 -m sim.drive --iface "$IFACE" --nodes "$NODE" --rod-mm "$ROD_MM" & pids+=($!)
sleep 0.5
python3 -m sim.master --iface "$IFACE" --node "$NODE" & pids+=($!)

echo "simulation running on $IFACE (node $NODE): open http://localhost:8080   (Ctrl-C to stop)"
wait -n
