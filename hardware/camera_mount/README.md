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

**Calibrated 2026-09-15** with `roscam.handeye_calib` (21 poses, Tsai; all
four solvers agreed to 0.05 mm / 0.01 deg; residual 3.17 mm / 1.57 deg). The
result is the default in `tools/fr3/fr3_mating.launch.py`:

```
handeye_xyz  : 0.061126 -0.011144 -0.046550        (|t| = 78 mm)
handeye_quat : 0.000855 0.003126 0.706706 0.707500 (~90 deg about Z)
```

Two properties of this bracket that the numbers confirm:

- **The camera is mounted rotated ~90°** about the optical axis. The launch
  file previously defaulted to identity, which was wrong by 89.94 deg and
  silently swapped camera X/Y for anything trusting the transform.
- **The optical axis comes out 0.37° off TCP Z** — the mount points the
  camera essentially straight down the tool axis, as its geometry suggests.

An earlier estimate from `align_gui`'s three-probe method (saved in
`tools/fr3/handeye_rotation.json`) put that axis at 2.23° and differs from
the full calibration by 5.20 deg overall. Prefer `handeye_calib`: 21 diverse
poses beats 3 probe moves for rotation. The probe method remains useful for
what it was built for — it needs no calibration procedure and recovers just
enough rotation to run a camera-frame position servo.

Using the STEP as a check, not a source:

- the CAD origin is the designer's, NOT the robot flange, so the raw
  bounding box above cannot be compared against `handeye_xyz` directly —
  only the magnitude is meaningful, and 78 mm suits this envelope;
- deriving the translation from CAD alone would also need the D405's optical
  centre relative to its mounting holes (Intel datasheet) — the optical
  origin is inside the camera body and is not a feature of this part;
- a printed part has real tolerance, so CAD is a prior, not ground truth.
