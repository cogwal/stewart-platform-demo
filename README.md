# stewart-platform-demo

Fair demo: a small Stewart platform (6 × Actuonix S20 linear steppers on Trinamic TMCM-1270
CANopen drives) controlled by a TTControl ECU. The same CANopen master logic also runs on a PC
against a simulated platform, so the PC and the ECU can be swapped.

- [docs/plan.md](docs/plan.md): decisions, architecture, slices, open questions (draft)
- [docs/hardware-notes.md](docs/hardware-notes.md): facts from the vendor documents and what they mean for the design
- [vendor/](vendor/): vendor manuals, EDS files and firmware
