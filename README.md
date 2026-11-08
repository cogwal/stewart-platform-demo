# stewart-platform-demo

Fair demo: a small Stewart platform (6 × Actuonix S20 linear steppers on Trinamic TMCM-1270
CANopen drives) controlled by a TTControl TTC2000 ECU. The same CANopen master (C on CANopenNode) also
runs on a PC against six emulated drives, so the PC and the ECU can be swapped on the bus.

- [docs/architecture.md](docs/architecture.md): diagrams of the system and the C master layers
- [docs/plan.md](docs/plan.md): decisions, architecture, slices and their status, open questions
- [docs/commissioning.md](docs/commissioning.md): bringing up the real drives, measuring, feeding results back
- [docs/agents/](docs/agents/README.md): briefs for the work that waits on hardware or files (ECU port, CAD, sim tuning, bench, fair)
- [docs/hardware-notes.md](docs/hardware-notes.md): facts from the vendor documents and what they mean for the design
- [master/](master/README.md): the C master · [contract/](contract/README.md): the CANopen contract · `sim/`: drive emulator and web view
- [vendor/](vendor/): vendor manuals, EDS files and firmware

## Quick start (Mac)

Needs Docker CLI + [Colima](https://github.com/abiosoft/colima) (`brew install colima docker docker-compose`).
Docker Desktop doesn't work: its kernel has no CAN support.

```bash
git clone --recurse-submodules https://github.com/cogwal/stewart-platform-demo
cd stewart-platform-demo
make vcan        # start Colima if needed, bring up vcan0 in its VM
make image       # build the dev image
make test        # everything: contract, emulator, C master unit tests, C vs Python kinematics, end-to-end on vcan0
make sim         # Stewart platform: 6 emulated drives + C master + 3D view on http://localhost:8080
make sim-leg     # single leg with the Python test master
make shell       # shell in the dev container
```

In the web view: the master boots and homes all six legs by itself (about 15 s), then the platform rises to its
neutral pose. Pick an input (pose sliders, virtual joystick, IMU, demo, per-leg), try Quick stop / Resume / Home,
and inject faults (power-cycle a drive, ENN off, jam a leg, mute a heartbeat, make a leg slip). The solid platform
is what the master knows from the bus; the orange outline is where the simulated platform really is.

Other targets: `make gen` (regenerate the C master's config from the YAML), `make ctest`, `make spmaster`,
`make s0-check`, `make help`.

## Ubuntu notebook

Skip `make vcan`; create the interface on the host (`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan &&
sudo ip link set up vcan0`), or use the PCAN adapter (`sudo ip link set can0 up type can bitrate 1000000`) with
`CAN_IFACE=can0`. With real drives on the bus: `SIM_NO_DRIVES=1 make sim CAN_IFACE=can0`. With the ECU as the master
(HIL): `SIM_MASTER=none make sim CAN_IFACE=can0`.
