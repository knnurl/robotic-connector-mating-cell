# Wrist camera mount — Intel RealSense D405 on the Franka FR3

The 3D-printed bracket that holds the D405 on the FR3 wrist. This is the
**physical realisation of the hand-eye transform**, so it is part of the
cell definition, not an accessory: change this part and `handeye_xyz` /
`handeye_quat` in `tools/fr3/fr3_mating.launch.py` are invalidated.

## Attribution (CC BY 4.0 — attribution is required)

| | |
|---|---|
| Model | Realsense D405 Camera Mount for Franka Robot Arms |
| Author | **Lele Burger** |
| Source | https://makerworld.com/en/models/1925363-realsense-d405-camera-mount-for-franka-robot-arms |
| Licence | [Creative Commons Attribution 4.0](https://creativecommons.org/licenses/by/4.0/) |
| Released | 2025-10-26 |
| Modifications | None — `franka_D405_camera_mount.step` is the unmodified original |

Designed for the Franka Emika Panda arm; used here on the FR3, which shares
the flange interface.

## Fasteners

- 2 × M3 × 10 mm socket head
- 1 × M6 × 25 mm socket head

## Print settings (designer profile)

0.2 mm layer, 6 walls, 25% infill, ~1.5 h, 1 plate.

Walls matter more than infill here: the bracket is a cantilever carrying the
camera, and any flex between the flange and the camera shows up directly as
hand-eye error that no amount of calibration removes — it is not a constant
offset if it deflects with arm pose.

## Geometry (parsed from the STEP)

Units are **metres**. Bounding box of the mount + camera envelope:

```
X  -0.08 .. 0.05   span 130 mm
Y  -0.02 .. 0.03   span  50 mm
Z  -0.02 .. 0.06   span  80 mm
```

## Relationship to hand-eye calibration

`tools/fr3/align_gui.py` recovers the hand-eye **rotation** empirically from
three probe moves, saved to `tools/fr3/handeye_rotation.json`. Measured on
this cell, the camera optical axis sits **2.23° off TCP Z** — i.e. the mount
points the camera very nearly straight down the tool axis. That is a useful
cross-check on both the print and the calibration.

The **translation** is still the dry-run guess `0.06 0.0 -0.04` and must come
from `handeye_calib`. Use this STEP as a sanity check on the result, not as a
substitute:

- the guess magnitudes do sit inside the envelope above, so it is plausible
  but unverified;
- deriving the translation from CAD alone also needs the D405's optical
  centre relative to its mounting holes (Intel datasheet) — the optical
  origin is inside the camera body and is not a feature of this part;
- a printed part has real tolerance, so CAD is a prior, not ground truth.
