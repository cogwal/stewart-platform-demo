# Hardware notes

Facts from the vendor documents, kept apart from the assumptions in the plan.
Anything marked **unverified** still needs a source or a bench test.

Vendor files live in [`vendor/trinamic/tmcm-1270_canopen_hw1.1_fw3.27/`](../vendor/trinamic/tmcm-1270_canopen_hw1.1_fw3.27/):
CANopen firmware manual fw 3.26 rev 1.06, `TMCM-1270_V3.27.eds` (matches the included fw 3.27 hex),
`TMCM-1270.eds` (older, fw 3.26 era), and the fw 3.27 hex.

## Motor controller: Trinamic (ADI) TMCM-1270, CANopen firmware

Sources: Hardware Manual HW V1.00 rev 1.20 (HW), CANopen Firmware Manual fw 3.26 rev 1.06 (FW), EDS V3.27 (EDS).

### Electrical and I/O (HW)

| Item | Value |
|---|---|
| Supply | +24 V nominal, +6 to +28 V DC (do not exceed 28 V) |
| Motor current | 0 to 1.2 A RMS (1.7 A peak). The FW manual cover says "up to 1 A RMS" (**discrepancy**) |
| Microstepping | full step down to 1/256 |
| CAN | 20 to 1000 kbit/s, TJA1051, no termination on board |
| Inputs | HOME (GPI0, also analog), REFL / REFR (GPI1/2, also encoder A/B), ENN (GPI3) |
| ENN | hardware driver enable, active low, pull-up: must be tied to GND to move. If missing, the drive sends EMCY 5441h |
| Order codes | `TMCM-1270-CANopen` vs `TMCM-1270-TMCL`. The firmware can be swapped via TMCL-IDE (FW §1.3) |

### CANopen communication (FW §1.1, §3; EDS)

| Item | Value |
|---|---|
| Standard | CiA 301 V4.1, NMT slave, 11-bit IDs |
| Node ID | 1 to 127, **default 1**. Set with 2705h, then store signature `0x65766173` to 2706h sub1, then reset |
| Bit rate | **default 1000 kbit/s**. Set with 2704h, then store with 2706h, then reset |
| LSS | supported in fw 3.27 (EDS V3.27 `LSS_Supported=1`), not in the older EDS |
| SDO | 1 server, expedited and segmented, **no block transfer** |
| PDOs | 4 RPDO + 4 TPDO, dynamic mapping, **max 3 mapped objects per PDO** |
| RPDO transmission | feature list: "asynchronous" only. §3.1.18 also allows 1 to 240 (SYNC). **Contradiction, test on the bench** |
| TPDO transmission | asynchronous, asynchronous with event timer, synchronous (1 to 240). Inhibit time in 0.1 ms |
| SYNC | consumer |
| Heartbeat | producer 1017h **default 0 (off)**, consumer 1016h (1 entry, default 0). Node guarding is gone in the V3.27 EDS |
| Error behaviour 1029h | comm error → **NMT stopped** (default). Entering NMT stopped puts any drive in OPERATION_ENABLED into **FAULT** (FW §2.2) |
| Store/restore | 1010h / 1011h. 2706h is only for bit rate and node ID |

Default PDO mapping:

| PDO | COB-ID | Type | Mapping |
|---|---|---|---|
| RPDO1 | 200h+id | FFh | 6040 |
| RPDO2 | 300h+id | FFh | 6040, 6060 |
| RPDO3 | 400h+id | FFh | 6040, 607A |
| RPDO4 | 500h+id | FEh | 6040, 60FF |
| TPDO1 | 180h+id | FFh | 6041 |
| TPDO2 | 280h+id | FFh | 6041, 6061 |
| TPDO3 | 380h+id | 1 (SYNC) | 6041, 6064 |
| TPDO4 | 480h+id | 1 (SYNC) | 6041, 606C |

### CiA 402 (FW §5–§10)

| Item | Value |
|---|---|
| Modes (6060h) | 1 pp, 3 pv, 6 homing, 8 csp, 9 csv. EDS 6502h = `0xA5` lists pp, pv, hm, csp but not csv (**minor discrepancy**) |
| PP limits | "change set immediately" (bit 5) and "change set point" (bit 9) are **not supported**: no blending or queued setpoints |
| CSP | no trajectory generator in the drive. The drive picks the velocity to reach each new 607A target within the interpolation period 60C2h (**default 1 ms**: value 1, index −3). Statusword bit 12 = target used, bit 13 = following error |
| Position units | **microsteps** (6062/6063/6064/607A/607C/60B0) |
| Velocity units | **default PPS** (208Ch = 181), acceleration PPS/s. 0 selects internal units. Only changeable in SWITCH_ON_DISABLED |
| Actual position | 6064h = 6063h = the step count. With no encoder it is the commanded position, not a measured one |
| Following error 6065h | only meaningful with an encoder. Default 0 = off. EMCY 8611h |
| Position window 6067h | default FFFFFFFFh = off. Target reached when the demand position reaches the target |
| Software limits 607Dh | default ±2³¹ (effectively none). **Must be set per leg** |
| Profile 6081/6083/6084 | all **default 0**. Must be written before any pp move |
| Quick stop | 605Ah default 2 (quick-stop ramp, then switch on disabled). 6085h default 51200 |
| Fault reaction | fixed: slow down on quick-stop ramp (605Eh = 2) |
| Homing (6098h) | 0, 1, 2, 3, 5 (switch + encoder index), **17/18 (left/right limit switch)**, **19/21 (home switch edge)**, 33/34 (index), **35 (current position = home, no motion)**. **No stall-based homing method** |
| Homing params | 6099h sub1/2 fast/slow speed (default 0), 609Ah acceleration (default 0), 607Ch offset |

### Manufacturer objects that matter (FW §4)

| Object | Meaning | Default | Note |
|---|---|---|---|
| 2000h | microstep resolution, 2^n | 8 (=256) | |
| 2003h | run current, 0 to 255 = 0 to 100 % of drive max, 32 effective steps | **128** | most important setting, see the current limit below |
| 2004h | standby current | 8 | applied after the standby delay 2089h (10 ms units) |
| 2005h | limit/home switch config: bits 0/1 disable L/R, 2/3 invert L/R, 4 disable home, 5 invert home | **0 = switches enabled** | the manual says to write 3 if no switches are wired. Only writable in SWITCH_ON_DISABLED |
| 209Eh / 20A4h | stallGuard2 threshold / stop-on-stall above this speed | 0 | EMCY 3230h on stall |
| 2101h | actual stallGuard load value 0 to 1023 | | |
| 2102h | driver error flags (OT, OTPW, UV, OCHS, OLB, OLA, OCB, OCA) | | |
| 2704h / 2705h / 2706h | bit rate / node ID / store | 1000 / 1 | |

### EMCY codes (FW §11)

1000h open load A/B · 2310h–2312h overcurrent · 3230h stallGuard · 4310h overtemp
(warning / shutdown) · 5441h ENN shutdown switch · 6320h PDO parameter error · 8100h comm ·
8110h CAN overflow (controller / Tx / Rx) · 8120h error passive · **8130h heartbeat/lifeguard loss** ·
8140h recovered from bus-off · 8210h/8220h PDO length · 8611h following error · FF00h undervoltage ·
**FF01h software limit (sub 1/2) and limit switch (sub 3/4)**.

### EDS caveat for the emulator

Many manufacturer objects in the EDS have **no DefaultValue** (e.g. 2003h, 2005h). An emulator built only
from the EDS would start them at 0. The emulator needs an overlay of defaults taken from the manual
(2003h = 128, 208Ch = 181, 6085h = 51200, …).

## Actuator: Actuonix S20 (`S20-SS-38-B`)

Source: Actuonix S20 datasheet, rev A (2020).

| Item | Value |
|---|---|
| Type | linear stepper actuator, motor wires only (option B), bipolar 4-wire |
| Stroke options | 15 / 30 / 50 / 100 mm |
| Closed length (hole to hole) | 89.8 / 104.8 / 124.8 / 174.8 mm |
| Mass | 62 / 64 / 70 / 84 g |
| Step size | 0.01 mm per full step |
| Mechanical backlash | 0.1 mm (= 10 full steps) |
| Max current | 0.60 A per phase (RMS or peak not stated) |
| Coil | 7 Ω, 2.1 mH per phase (rated input 0 to 4.2 V DC) |
| Backdrive force | 40 N unpowered, 60 N powered: holds position without power below that |
| Max side load | about 0.10 × max force |
| Duty cycle | 100 % |
| Feedback | none: no encoder, no limit or home switches |
| Environment | -10 to +40 °C, IP54 |

Vendor guidance: ramp the step frequency or the actuator won't move at high speed. Keep the load at 50 to 75 %
of max force for the given speed. Missed steps become more likely near max force.

**Unverified:** max force and max speed. The force/speed curves are images in the PDF.

## Consequences for the design

- **Current limit.** The drive's default 2003h = 128 is about 50 % of drive max: roughly 0.6 A RMS if full
  scale is 1.2 A RMS, but 0.6 A *peak* is only about 0.42 A RMS (2003h ≈ 90). Start conservative (2003h ≈ 88–95),
  measure the phase current and temperature on the bench, then write the value at every boot **and** store it.
  The emulator flags a missing or too-high current as a fault.
- **Limit-switch config is a boot trap.** With 2005h = 0 the drive expects switches. Unless switches are wired,
  the master must write 2005h before enabling, and only in SWITCH_ON_DISABLED.
- **Open loop.** The actual position is just the step count, and lost steps can't be seen (stallGuard and
  2101h are the only hints). The position after a power cycle is unknown, so **homing is mandatory**.
- **Homing needs a sensor or a workaround.** The drive only homes on switches (17/18/19/21) or by declaring
  the current position home (35). There's no stall-homing method. Options: (a) one switch per leg on
  REFL or HOME, with methods 17 or 19; (b) drive into the end stop at low current in pv mode, detect it
  (stallGuard EMCY 3230h or time-out), then method 35. (a) is more reliable.
- **Coordinated 6-leg motion means CSP.** PP can't blend or queue setpoints, so the master's trajectory has to be
  streamed in csp mode. The master owns the timing. 60C2h must match the master's cycle, so the 1 ms default
  must be changed.
- **Bus load sets the CSP period.** Per cycle: 1 SYNC + 6 RPDO (6040+607A, 6 bytes) + 6 TPDO (6041+6064)
  ≈ 13 frames × ~110 bit ≈ 1.4 kbit. At 1 Mbit/s that is ~14 % at 10 ms, ~28 % at 5 ms, and >100 % at 1 ms.
  **5–10 ms is realistic.**
- **Commissioning.** Every drive ships as node 1 at 1 Mbit/s. Each drive needs its own node ID
  set one at a time (2705h + 2706h) or by LSS (fw 3.27). That needs a small commissioning tool.
- **Safety defaults help us.** If the drive consumes the master's heartbeat (1016h) and keeps 1029h = 0,
  losing the master stops and faults the drive. ENN gives a hardware e-stop path.
- **Units.** counts per mm = 100 × 2^(2000h). At 1/256 that is 25,600 µsteps/mm. 100 mm = 2,560,000 (fits in int32).
  10 mm/s = 256,000 PPS.
- **Accuracy.** 0.1 mm backlash per leg limits pose accuracy. The plant model can include it.
- **Mechanics.** The side-load limit (about 10 % of max force) means the legs need proper joints at both ends.
