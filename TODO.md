# TODO — Connector Mating Cell

Ordered by priority. Details for each item: [HANDOFF.md](HANDOFF.md) §8 and
[SETUP_AND_CALIBRATION.md](SETUP_AND_CALIBRATION.md).

## Blocking real-world use (do on the robot PC / cell)

- [ ] Copy repo to the robot PC and `colcon build` there (`plc_`/`hmi_`
      build automatically once `melfa_msgs` is found)
- [ ] Fake-hardware dry run (`use_fake_hardware:=true`,
      `enable_insertion: false`): verify phase transitions and motion
      directions in RViz — **first-ever live run of the controller**
  - [ ] Confirm the marker-frame flip convention matches the RV-5AS TCP
        (tool must align tool-Z INTO the surface; fix in
        `mating_geometry::standoff_goal` + tests if not)
  - [ ] Confirm Pilz LIN accepts the small frequent goals (if fussy: check
        `pilz_cartesian_limits.yaml`, else consider `computeCartesianPath`)
- [ ] Hand-eye calibration on the real cell (`ros2 run roscam handeye_calib`),
      replace the guessed static TF in the bringup
- [ ] Teach target geometry in `rv5as_params.yaml`:
      `connector_offset_x/y/z` (currently 0,0,0 = marker centre!),
      `tool_yaw_offset_deg`, `standoff_height_m`, `insertion_depth_m`
- [ ] Validation ladder: fake HW → real HW insertion-disabled (incl.
      occlusion hold test) → full mate at reduced `insert_speed`
- [ ] Tune convergence on real kinematics (step clamps, tolerances,
      `filter_alpha`) if alignment oscillates or crawls

## Improvements (not blocking)

- [ ] Finish the FR3 dry-run harness (`tools/dryrun_fr3/`) — pipeline config
      already fixed, never re-run; success = MATED in the log (HANDOFF §6)
- [ ] `computeCartesianPath` fallback so the INSERT stroke is path-guaranteed
      on robots without Pilz
- [ ] Migrate ALIGN phases to `moveit_servo` for continuous (non-stepped)
      tracking; keep Pilz LIN for the committed insertion stroke
- [ ] Decide and implement a failure policy beyond FAULT-hold
      (e.g. retract-to-standoff after N insertion failures)
- [x] Kalman filter in vision (replaces EMA; outlier gate + ≤0.3 s
      dropout prediction) — done 2026-07-06
- [x] Connector-level 6-DOF pose via depth ICP against CAD STL
      (`connector_pose` node; marker = prior, gated fallback) — done
      2026-07-06; needs real-data tuning + STL export of the real connector
- [ ] Multi-marker / ArUco-board support in `cam_pub` for occlusion
      robustness and better pose accuracy (bigger effective marker)
- [x] Machine-readable cell state (/mating/phase, /mating/error_*,
      /diagnostics) + GUI options: Foxglove layout, configurable tkinter
      operator panel, rqt recipe (tools/gui/) — done 2026-07-06
- [ ] Consolidated single launch file for the whole cell (needs the MELFA
      packages present to test)
- [ ] Force-guarded insertion (F/T sensor or MELFA force option) for
      tight-tolerance connectors — hardware decision first

## Housekeeping

- [ ] Rename the workspace directory to remove spaces + trailing space
      (breaks colcon's `install/setup.sh`; workarounds in HANDOFF §2)
- [ ] Modernize or delete `pick_n_place_` (legacy demo, still old patterns)
- [ ] Push the repo to a remote (currently local-only git)
- [ ] Set a global git identity on this machine (commits currently use
      per-command `-c user.name/email`)
