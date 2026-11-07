#!/usr/bin/env bash
# Run the simulation inside the dev container: drive emulator(s) + master + web view, all on one
# CAN interface. Open http://localhost:8080 on the host.
#
#   tools/run_sim.sh [iface]
#
# Environment:
#   SIM_LEGS=6        6 = Stewart platform (nodes 1..6), 1 = single leg (node 1)
#   SIM_MASTER=c      c = C master on CANopenLinux (spmaster), py = Python test master (single leg only),
#                     none = no master here (the ECU or another PC is the master: HIL)
#   SIM_ROD_MM=37     rod position at power-up (unknown to the drives); legs get +3 mm each
#   SIM_NO_DRIVES=1   don't start the emulator (real drives on the bus, e.g. can0 on the bench)
#   SIM_RECORD=file   record CAN frames, master status, truth and operator input (JSON lines)
set -euo pipefail

IFACE="${1:-${CAN_IFACE:-vcan0}}"
LEGS="${SIM_LEGS:-6}"
MASTER="${SIM_MASTER:-c}"
ROD_MM="${SIM_ROD_MM:-37}"
cd "$(dirname "$0")/.."

ip link show "$IFACE" >/dev/null 2>&1 || {
    echo "CAN interface $IFACE not found. On the Mac run 'make vcan' first." >&2
    exit 1
}
NODES=$(seq -s, 1 "$LEGS")

if [ "$MASTER" = c ]; then
    make -s -C master spmaster
fi

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

python3 -m sim.web --iface "$IFACE" --nodes "$NODES" ${SIM_RECORD:+--record "$SIM_RECORD"} & pids+=($!)
if [ -z "${SIM_NO_DRIVES:-}" ]; then
    python3 -m sim.drive --iface "$IFACE" --nodes "$NODES" --rod-mm "$ROD_MM" --rod-spread-mm 3 & pids+=($!)
    sleep 0.8
fi
if [ "$MASTER" = c ]; then
    mkdir -p build/master-storage
    SP_LEGS="$LEGS" build/master/spmaster -i 127 -s build/master-storage/ "$IFACE" & pids+=($!)
elif [ "$MASTER" = py ]; then
    python3 -m sim.master --iface "$IFACE" --node 1 & pids+=($!)
fi

echo "simulation running on $IFACE (nodes $NODES, master $MASTER): open http://localhost:8080   (Ctrl-C to stop)"
wait -n
