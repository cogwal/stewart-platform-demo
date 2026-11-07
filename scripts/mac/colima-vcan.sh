#!/usr/bin/env bash
# Make sure the Colima VM (Ubuntu kernel) has a virtual CAN bus for the containers.
# Docker Desktop's kernel has no CAN support, so on the Mac we use Colima instead.
# Idempotent: safe to run after every `colima start`.
#
# Usage: scripts/mac/colima-vcan.sh [interface]   (default: vcan0)
set -euo pipefail

IFACE="${1:-vcan0}"

command -v colima >/dev/null || { echo "colima not found: brew install colima" >&2; exit 1; }
colima status >/dev/null 2>&1 || colima start

colima ssh -- sudo bash -s "$IFACE" <<'EOF'
set -euo pipefail
iface="$1"
if ! modprobe vcan 2>/dev/null; then
  # Ubuntu cloud kernels ship vcan in linux-modules-extra
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq
  apt-get install -y -qq "linux-modules-extra-$(uname -r)" can-utils
  modprobe vcan
fi
ip link show "$iface" >/dev/null 2>&1 || ip link add dev "$iface" type vcan
ip link set up "$iface"
ip -brief link show "$iface"
EOF

echo "OK: $IFACE is up in the Colima VM. Run containers with --network host to use it."
