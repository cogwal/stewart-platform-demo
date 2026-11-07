# Brief: port the C master to the TTC2000 A2G ECU

## Goal
The same master that runs on the PC (`build/master/spmaster`) runs on the TTControl TTC2000 A2G (Infineon AURIX
TC3xx, PXROS, everything in one task). Acceptance: swap PC and ECU on the same bus (emulated drives on the Ubuntu
notebook first, real drives later) and get the same behaviour: boot, home all legs, follow poses, the fault reactions.

## What already exists
- `master/core/` – portable C99, the whole master logic: `sp_core.c` (drive bring-up, homing, csp streaming, EMCY
  classes, reboot/heartbeat handling), `sp_kin.c` (IK, workspace), `sp_motion.c` (inputs: pose, joystick, IMU, demo,
  legs → smoothed pose → leg targets), `sp_app.c` (glue, JSON commands/status for the PC; optional on the ECU).
  Single-precision float only (TC3xx FPU). `__attribute__` use is guarded by `__GNUC__`.
- `master/generated/` – `OD.c/OD.h` (the master's CANopenNode object dictionary: SYNC producer 10 ms, HB consumer for
  6 drives, 1 SDO client, 12 RPDOs + 18 TPDOs mirroring the drives' PDOs, mapped to arrays 2110h..2123h) and
  `sp_config_gen.c/h` (contract constants, startup SDO table, EMCY table, geometry). Committed, so no Python is needed
  on the ECU build machine.
- `master/canopennode/sp_co_adapter.c` – implements the core's `sp_port_t` on CANopenNode v4 (NMT master, SDO client,
  HB consumer, EMCY consumer, TPDO send requests, SYNC detection). **Platform independent: reuse it unchanged.**
- `master/linux/sp_linux_app.c` + CANopenLinux's `CO_main_basic.c` – the PC main loop. Use it as the reference for
  the order of calls.
- `third_party/CANopenLinux/CANopenNode/example/main_blank.c` and `CO_driver_blank.c` – CANopenNode's templates for a
  new target. Start from these.

## What to build (`master/ttc/`)
1. `CO_driver_target.h` + `CO_driver_ttc.c`: CANopenNode's driver on the TTC CAN API (to be provided by the user).
   - `CO_CANmodule_init`, `CO_CANrxBufferInit` (filters), `CO_CANtxBufferInit`, `CO_CANsend`, `CO_CANclearPendingSyncPDOs`,
     `CO_CANmodule_process` (error counters / bus-off), and the receive path calling each rx buffer's callback.
   - Frames the master must receive: 0x080+n (EMCY), 0x180+n / 0x280+n (drive TPDOs), 0x580+n (SDO responses),
     0x700+n (heartbeats), 0x000 not needed. n = drive node IDs. Check the TTC API has enough mailboxes/filters, or use
     one catch-all receive and dispatch in software (CANopenNode's Linux driver does that).
   - Locking macros (`CO_LOCK_CAN_SEND` etc.): empty is fine with everything in one task and no CAN interrupts calling
     into CANopenNode; if the TTC driver receives in an ISR, buffer there and dispatch from the task.
2. `CO_config` flags (in `CO_driver_target.h`), matching the PC build (`master/Makefile`, `CO_DEFS`):
   - `CO_CONFIG_HB_CONS = ENABLE | CALLBACK_MULTI | QUERY_FUNCT` (the adapter needs the remote-reset callback and the
     state query functions),
   - NMT master, SDO client, EMCY consumer (`CO_CONFIG_EM_CONSUMER`), SYNC producer, PDOs; gateway, LSS, TIME producer,
     storage, LEDs: off.
   - NMT control: `CO_NMT_STARTUP_TO_OPERATIONAL` only (the master must be OPERATIONAL for its PDOs, and must not drop
     to pre-operational when a drive's heartbeat is lost; the core handles that).
3. `ttc_main.c` – one PXROS task, a 1 ms cycle:
   ```
   init:   sp_co_configure_od(nodes, 6, 127); CO_new; CO_CANinit(500/1000 kbit); CO_CANopenInit; CO_CANopenInitPDO;
           sp_co_port(&ad, &port, log); sp_app_init(&app, &port, nodes, 6, 127, auto_home=1); sp_co_init(&ad, &app, nodes, 6);
           sp_co_on_comm_reset(&ad, CO)     (also after every communication reset)
   every 1 ms (dt_us measured from a hardware timer):
           CO_process(CO, false, dt_us, NULL); CO_process_SYNC(...); CO_process_RPDO(...); CO_process_TPDO(...);
           sp_co_process(&ad, dt_us)        (polls inputs, runs the core, calls sp_app_on_sync() when SYNC was sent)
   ```
   Order matters: TPDO send requests made in `sp_co_process` go out in the next cycle's `CO_process_TPDO`; the core
   never sends two frames on the same drive RPDO in one step for this reason.
4. Operator inputs on the ECU: call `sp_motion_set_source()`, `sp_motion_set_joystick(x, y)` (−1..1 from the ADC),
   `sp_motion_set_imu(roll, pitch, yaw)` (degrees) and `sp_core_cmd_home/stop/resume()` directly. `sp_app_command()`
   (JSON) is only needed if a debug channel carries the PC's protocol. See [fair-features.md](fair-features.md).
5. Status: `sp_app_status_json()` is optional (it uses `snprintf` with doubles: fine for a debug UART, skip otherwise).
   `app.core.phase`, `app.core.reason`, `app.core.legs[i]` hold everything a display needs.

## Things to watch
- **SYNC timing:** CANopenNode's SYNC producer restarts its timer at 0 when it fires, so the period is 10 ms plus the
  loop granularity (10.16 ms mean on CANopenLinux, measured with `tools/bus_stats.py`). With an exact 1 ms tick it
  should be 10.0 ms. The drives interpolate csp over 60C2h = 10 ms; a slower SYNC means small velocity ripple. If it's
  off, produce SYNC from a hardware timer or adjust 1006h.
- **The heartbeat consumer** starts monitoring a drive only after its first heartbeat; a drive that never appears is
  caught by the core's boot-up timeout (5 s, retried every 2 s).
- **Storage:** none needed. Everything the master needs is in the generated OD defaults and the core re-configures
  every drive at every boot. (On Linux, CANopenNode's storage files caused EMCY 5000h on first run; irrelevant without
  storage.)
- **Floating point:** keep `float`; avoid `double` literals in new code (TC3xx has no double FPU).
- **Watchdog:** if the task overruns, the drives see no master heartbeat for 300 ms and fault (EMCY 8130h, NMT
  stopped). The core then stops all legs with homing kept; Resume recovers. Don't make the loop wait for anything.

## How to test
1. Build `master/core` + generated + adapter with the TTC toolchain first, without CAN, plus `master/tests/test_core.c`
   and `fake_bus.c` if the target can run them (they need only libc + libm). Same results as `make ctest`.
2. HIL on the Ubuntu notebook: PCAN-USB on the same bus as the ECU, emulated drives on the notebook:
   `SIM_MASTER=none make sim CAN_IFACE=can0` (emulator + web view, no PC master). The web view shows the ECU's
   behaviour from the bus. Expect: boot (65 SDO writes per drive), all legs home together, park, rise to neutral.
   Fault injection from the web view (power cycle, ENN, heartbeat mute, jam) must give the same reactions as with
   `spmaster` (the end-to-end tests in `tests/master/test_spmaster_e2e.py` list them).
3. Swap: stop the ECU, start `spmaster` on the notebook (`make sim CAN_IFACE=can0` with SIM_NO_DRIVES as needed):
   same behaviour.

## Needed from the user
The TTC CAN API (headers + docs), the toolchain / project template for the TTC2000 A2G, which CAN channel the bus is on,
how the joystick ADC and the IMU are read (open question 2 in plan.md), and a TTC2000 on the bench.
