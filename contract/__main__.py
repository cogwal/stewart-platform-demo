"""Print the startup SDO sequence for a drive: python -m contract [node_id]"""
import sys

from contract import load

node = int(sys.argv[1]) if len(sys.argv) > 1 else load().node_ids[0]
seq = load().startup_sequence(node)
for w in seq:
    print(w)
print(f"# {len(seq)} SDO writes for node {node}")
