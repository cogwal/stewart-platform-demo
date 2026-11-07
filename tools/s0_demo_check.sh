#!/usr/bin/env bash
# S0 check, run inside the dev container: build the stock CANopenLinux demo (canopend),
# start it as a node on the CAN interface and talk to it with python-canopen.
#
# Usage: tools/s0_demo_check.sh [iface] [node-id]     (defaults: $CAN_IFACE or vcan0, 4)
set -euo pipefail

IFACE="${1:-${CAN_IFACE:-vcan0}}"
NODE="${2:-4}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build/CANopenLinux"

ip link show "$IFACE" >/dev/null 2>&1 || {
    echo "CAN interface $IFACE not found. On the Mac run scripts/mac/colima-vcan.sh first." >&2
    exit 1
}

# Build out of tree so the submodule stays clean.
mkdir -p "$BUILD"
cp -a "$ROOT/third_party/CANopenLinux/." "$BUILD/"
make -s -C "$BUILD" canopend >/dev/null
echo "built $BUILD/canopend"

STORAGE="$ROOT/build/storage"
mkdir -p "$STORAGE"
PID=""
trap '[ -n "$PID" ] && kill $PID 2>/dev/null || true' EXIT

start_node() {
    "$BUILD/canopend" "$IFACE" -i "$NODE" -s "$STORAGE/" >"$ROOT/build/canopend.log" 2>&1 &
    PID=$!
    sleep 0.5
    kill -0 $PID 2>/dev/null || { cat "$ROOT/build/canopend.log"; echo "canopend did not start" >&2; exit 1; }
    echo "canopend running on $IFACE as node $NODE (pid $PID)"
}

stop_node() { kill $PID 2>/dev/null || true; wait $PID 2>/dev/null || true; PID=""; }

# First run: the storage files don't exist yet. CANopenNode then raises a non-volatile memory
# error (EMCY 5000h), which blocks NMT start. Save once via 1010h to create them, then restart.
if [ ! -f "$STORAGE/od_comm.persist" ]; then
    start_node
    python3 "$ROOT/tools/check_canopen_node.py" --iface "$IFACE" --node "$NODE" --save-params
    stop_node
fi

start_node
python3 "$ROOT/tools/check_canopen_node.py" --iface "$IFACE" --node "$NODE"
