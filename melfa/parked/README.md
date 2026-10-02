# Parked: the autonomous mating path and its MELFA-default tooling

Parked 2026-09-23: the FR3 cell runs from `tools/fr3/cell_panel.py` and
`tracking_node`, every motion behind a button, so the autonomous phase machine
(`mating_node`) is off that path. Its impedance INSERT and seated/jam/snag
judging are to be extracted later into an FR3 stroke service.

- `mating_controller/` - its `launch/` and `config/rv5as_params.yaml` (MELFA defaults)
- `gui/` - was `tools/gui/` (Foxglove layout, `mating_panel.py`)
- `dryrun_fr3/` - was `tools/dryrun_fr3/` (mock-hardware phase-machine run)

`mating_node.cpp` and its gtests stay in `mating_controller`. To revive:
`git mv` these back, restore `install(DIRECTORY launch config ...)` in its
CMakeLists.txt, build with `-DBUILD_MATING_NODE=ON`, and delete
`melfa/COLCON_IGNORE` (it also hides `melfa_cell`). All of it, plus the
archived MoveIt Servo backend, is in tag `pre-cleanup-2026-09-23`.
