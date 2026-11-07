# Brief: fair features (S9)

## Goal
The platform runs unattended at a fair: the ECU is the master, a visitor uses the analog joystick or the IMU, the
platform recovers from e-stops and drive resets without an engineer.

## What exists (in the portable core, so it runs on the PC today and on the ECU after the port)
- Input sources in `master/core/sp_motion.c`: pose, joystick (deadband + expo, x → roll, y → pitch), IMU (mirror roll /
  pitch, optional yaw, gain, clamp), demo (tilt circle + z wave), legs. Mapping parameters in `config/platform.yaml`
  (`inputs:`), generated into the C header.
- Idle demo: `inputs.demo.idle_after_s` > 0 starts the demo after that many seconds without joystick/IMU input
  (currently 0 = off).
- Pose smoothing with per-axis velocity/acceleration limits, then the step is shortened so no leg exceeds the csp
  speed limit; workspace limiting scales the pose toward neutral.
- Fault handling (`master/core/sp_core.c`): reboot → reconfigure + "press Home"; heartbeat loss → stop, homing kept,
  Resume; ENN/stall/overtemp → stop, Home needed; auto-home at start-up only.
- PC: all of it in the browser (`make sim`), including a virtual joystick, IMU sliders and the phone's orientation
  sensor as IMU (`deviceorientation`; on iOS this needs https).

## Open questions (plan.md "Still open"; ask the user first)
1. What the hard e-stop cuts (drive power, ECU power, both). Decides the recovery flow: drive power cut = all drives
   reboot = automatic re-home after release (needs a decision: automatic, or operator button?).
2. How the IMU reaches the ECU and at what rate.
3. Joystick mapping (tilt is the default; alternatives: x/y translation, or a mode switch).
4. Idle behaviour: hold, park, or demo motion after a timeout.

## Likely work
- Auto-recovery policy: the option exists (`supervisor:` in `config/platform.yaml`, off by default; unit test
  `auto_recover`). Decide with the user whether to turn it on for the fair and with which delay; add an e2e test with
  `SP_AUTO_RECOVER_S` and decide what happens when ENN stays open (today: homing fails, retries up to the limit).
- Return to a safe pose before homing (lower slowly instead of a fast retract) if the platform carries a payload.
- Input selection on the ECU (a switch or a button) → `sp_motion_set_source()`.
- Status display (whatever the ECU has: LEDs, a small display, or a CAN message for a display node): phase, reason,
  homed, input source. All of it is in `sp_core_t` / `sp_motion_t`.
- Logging of faults with timestamps for after-fair analysis (EMCY log exists: `core.emcy`, last 8).
- Long-run test in the simulation: hours of demo motion with random fault injection (a scenario runner that drives the
  emulator's control port :5008 and the master's command port), checking that it always comes back to `run`.
