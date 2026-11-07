# Brief: bring the CAD geometry into the simulation and the master

## Goal
Replace the provisional 6-6 layout in `config/platform.yaml` with the real platform from the CAD, so the master's
inverse kinematics, the workspace limits and the 3D view match the hardware.

## What exists
- `config/platform.yaml`: joint circles (radius, half angle) for base and platform, the leg's pin-to-pin closed length
  (174.8 mm = S20-100 hole to hole, **without** joint hardware), the neutral stroke (50 mm), axis limits, smoothing
  limits, input mapping. It already accepts explicit joint lists: `base_joints_mm` and `platform_joints_mm`
  (6 × [x, y, z] each), which override the circles.
- `stewart/geometry.py`: Python reference (IK, FK by Newton, workspace check, limit by scaling toward neutral).
- `master/core/sp_kin.c`: C implementation (float), cross-checked against Python on 200+ random poses
  (`tests/master/test_c_master.py`).
- `contract/gen_c.py` writes the geometry into `master/generated/sp_config_gen.*` (`make gen`).
- `sim/web/static/index.html`: 3D canvas view (no meshes; plates and legs drawn from the joints).

## Conventions (keep them, or change them everywhere)
- Base frame: origin at the base joint circle centre, z up. Platform frame: origin at the platform joint circle centre.
- Leg i joins base joint B[i] to platform joint P[i] and is driven by `bus.drive_node_ids[i]` → node i+1. **The leg
  order must match the wiring** (which drive is on which leg). Write the mapping down in platform.yaml.
- Pose = (x, y, z, roll, pitch, yaw), mm and degrees, relative to the neutral pose, R = Rz(yaw)·Ry(pitch)·Rx(roll).
- Stroke = pin-to-pin length − closed length; 0 = retracted end stop = home.

## Steps
1. From the CAD, in one consistent frame: the six base joint centres, the six platform joint centres (in the platform
   frame, platform level), the pin-to-pin distance of a leg at the retracted stop **including** rod ends / clevises.
   If the joints are U-joints or ball joints with an offset, use the rotation centres.
2. Put them in `config/platform.yaml` (`base_joints_mm`, `platform_joints_mm`, `leg.closed_length_mm`), bump
   `geometry_version` (drop `-provisional`). Choose `neutral_stroke_mm` (mid-stroke unless the CAD says otherwise).
3. Joint angle limits: add the max joint angles from the CAD (base and platform side) to platform.yaml and to the
   workspace check in **both** `stewart/geometry.py` (`reachable`) and `master/core/sp_kin.c` (`sp_kin_reachable`), with a
   test in each. Today only the leg stroke is checked.
4. Revisit `motion.limits` and smoothing limits for the real workspace and payload; `make gen`; `make test`.
5. Optional: show the CAD in the web view. The view is a plain canvas renderer (no three.js, works offline at the
   fair). Simplest: export low-poly STL of base, platform and leg parts, convert to a compact JSON of triangles, draw them
   with the same projection (painter's sort). Keep the joint-based drawing as the fallback.
6. Check in the simulation (`make sim`): neutral pose level, legs at the neutral stroke, the limits in the Pose sliders
   reachable, demo motion inside the workspace.

## Checks
- `make test` (includes C vs Python IK on random poses; the generated header must be up to date).
- A pose sweep: every corner of the axis limits gives a reachable or correctly scaled pose (no leg outside
  soft limits − margin, no joint over its limit).
