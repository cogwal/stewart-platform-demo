# stewart-platform-demo

Fair demo: a small Stewart platform (6 × Actuonix S20 linear steppers on Trinamic TMCM-1270
CANopen drives) controlled by a TTControl ECU. The same CANopen master logic also runs on a PC
against a simulated platform, so the PC and the ECU can be swapped.

- [docs/plan.md](docs/plan.md): decisions, architecture, slices, open questions (draft)
- [docs/hardware-notes.md](docs/hardware-notes.md): facts from the vendor documents and what they mean for the design
- [vendor/](vendor/): vendor manuals, EDS files and firmware

## Quick start (Mac)

Needs Docker CLI + [Colima](https://github.com/abiosoft/colima) (`brew install colima docker docker-compose`).
Docker Desktop doesn't work: its kernel has no CAN support.

```bash
git clone --recurse-submodules https://github.com/cogwal/stewart-platform-demo
cd stewart-platform-demo
make vcan        # start Colima if needed, bring up vcan0 in its VM
make image       # build the dev image
make test        # contract checks, drive emulator tests, end-to-end tests on vcan0
make sim         # single-leg simulation: open http://localhost:8080 (slider, Home, Quick stop)
make s0-check    # run the CANopenLinux demo node on vcan0 and check it with python-canopen
make shell       # shell in the dev container
```

On the Ubuntu notebook, skip `make vcan` and create the interface on the host instead
(`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`),
or use the PCAN adapter with `make s0-check CAN_IFACE=can0`.
