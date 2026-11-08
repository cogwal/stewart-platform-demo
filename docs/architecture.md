# Architecture

Two views of the system: how the processes connect, and how the C master is layered. Dashed parts come later.
The decisions behind this are in [plan.md](plan.md#architecture-what-is-shared-and-what-is-per-platform).

## System: `make sim` on the Mac

```mermaid
flowchart TB
    browser["Browser<br/>3D view, inputs"]
    subgraph container["Dev container (Colima)"]
        web["Web server<br/>sim.web, recorder"]
        master["spmaster<br/>C master (node 127)"]
        emu["Drive emulator<br/>6 × TMCM-1270 + S20 (nodes 1–6)"]
        bus[["vcan0 CANopen bus"]]
    end
    ecu["TTC2000 ECU<br/>replaces spmaster"]:::later
    drives["Real TMCM-1270 drives<br/>replace the emulator"]:::later

    browser <-->|"WebSocket :8080"| web
    web -->|"commands, UDP :5005"| master
    master -->|"status, UDP :5006"| web
    emu -.->|"truth, UDP :5007 (sim only)"| web
    web -.->|"faults, UDP :5008 (sim only)"| emu
    master <--> bus
    emu <--> bus
    web ---|"listens only"| bus
    bus -.- ecu
    bus -.- drives

    classDef later stroke-dasharray: 5 5
```

- Everything on the CAN bus behaves like a real CANopen node. The web server only listens to it.
- The master's command and status channel is UDP on the PC. On the ECU it will be local I/O (joystick, IMU) and
  whatever debug link the ECU offers; the JSON commands are listed in [sp_app.h](../master/core/sp_app.h).
- The truth and fault channels exist only in simulation. Truth is the real rod position, which no drive can
  report, so the 3D view can show lost steps.
- Swapping in hardware:
  - Real drives: `SIM_NO_DRIVES=1` stops the emulator.
  - ECU as the master: `SIM_MASTER=none` stops `spmaster`.
  - Both use `CAN_IFACE=can0` for the PCAN adapter.

## C master layers

```mermaid
flowchart TB
    subgraph core["Portable C99 core: master/core (same code on PC and ECU)"]
        app["sp_app<br/>operator commands, status JSON, FK at 10 Hz"]
        motion["sp_motion<br/>inputs, smoothing, leg speed limit"]
        sp_core["sp_core<br/>boot, homing, CiA 402, faults"]
        kin["sp_kin<br/>IK, FK, workspace limit"]
        app --> motion
        app --> sp_core
        motion --> kin
    end
    gen["sp_config_gen<br/>generated from contract/ and config/"]
    port["sp_port.h<br/>nmt, sdo, rpdo callbacks"]
    adapter["sp_co_adapter + generated OD<br/>CANopenNode: SDO client, PDOs, heartbeat, EMCY"]
    linux["sp_linux_app<br/>CANopenLinux, UDP, vcan0 / can0"]
    ecuport["ECU port<br/>PXROS task, TTC CAN API"]:::later

    gen -.-> core
    sp_core --> port
    port --> adapter
    adapter --> linux
    adapter --> ecuport

    classDef later stroke-dasharray: 5 5
```

- `sp_core` reaches CANopen only through the callbacks in `sp_port.h`, so the unit tests can replace the bus with
  a fake one ([fake_bus.c](../master/tests/fake_bus.c)).
- `make gen` writes the config header and the master's object dictionary from the YAML. The ECU build needs no
  Python.
- The core must pass `make -C master portable-check`: strict C99, no implicit `double` (the TC3xx FPU is single
  precision).
- Only the bottom row depends on the platform. What the ECU port needs is in [agents/ecu-port.md](agents/ecu-port.md).
