# Target B's two slots and a slot state machine: plan

> **Status: plan, 2026-09-29. Nothing is built yet.** The decisions below come from the operator's answers on 2026-09-29, after the first PLACE AT B runs on the arm. The open questions must be settled before any code is written. It touches motion code (grip_node's PLACE target), so it gets a multi-agent review before commit. Evidence: `runs/2026-09-29/analysis/README.md` and `b_slots_candidates.png`.

## Why

- **PLACE AT B puts the cube on the marker itself.**
  - Today's PLACE AT B centres the cube on target B's marker: `grip_target_offset_m` is [0, 0] (`grip_node.py:98-99`, `grip_logic.place_tcp`).
  - On 2026-09-29 both attempts (17:14:45 and 17:16:01) planned exactly that. The operator stopped both during "lower".
- **The cube belongs in a slot.** B lies flat on a black box about 70 mm wide that runs along B's x axis, and has two slots, one either side of the marker.
- **The yaw is not fixed against B.**
  - `grasp_tcp` picks, of the four face-aligned yaws, the one needing the least wrist turn (`grip_logic.py:17-33`). Which way the fingers close at B therefore depends on how the hand arrives.
  - On 09-29 it happened to pick TCP x = −B x, TCP y (the finger-closing axis) = +B y.

## Decided (operator, 2026-09-29)

1. **Slot positions:** slot 1 at B + (−60 mm, 0), slot 2 at B + (+60 mm, 0), in B's own frame (along ±B x). These are candidates 1 and 2 in `b_slots_candidates.png`, the only two on the box top. 60 mm is the operator's figure: measure it.
2. **Finger direction at a slot:** the fingers close **across B y**, as in today's plan. The place yaw is restricted to the two orientations with TCP y ∥ ±B y, and the least wrist turn chooses between those two.
   - Note: the operator first described the goal as "90° more anticlockwise about TCP z, then 6 cm −x". The picture reconciled this: the slot offset is along B x, and the finger direction stays.
3. **Several cubes, each with its own ArUco id.** Today vision tracks a single cube id (`marker_id`, 0 on this cell) plus target B (`target_marker_id`, 1) (`cam_pub.py:108,143`; `fr3_cell.launch.py:65-67`), so vision has to grow.
4. **The operator presses every motion.** The state machine only chooses which slot to place into and where to pick from. It shows the choice, and the operator can override it by hand. GRIP and PLACE AT B still start from their buttons (the human-in-the-loop rule).
5. **Vision wins, refuse if unsure.**
   - Slot occupancy comes from seeing a cube's marker near a slot centre.
   - A slot seen occupied is never a place target.
   - If a slot's state has not been seen recently, PLACE AT B into it refuses and says why.
   - GRIP and PLACE results update the state between sightings, but a live sighting overrides them.

## Decided (operator, 2026-09-30)

6. **Two cubes, ids 0 and 2.** Both are 55 mm with the sticker centred, like today's cube 0. Id 1 stays target B.
7. **Flow is table ↔ slots.**
   - GRIP picks a cube from anywhere on the table, and PLACE AT B puts it into a free slot.
   - GRIP also picks from a slot, and a plain PLACE sets it down on the table.
8. **Slot 1 first** when both are free. The manual dropdown overrides.
9. **The slots are checked at PLACE AT B's "over B" step**, with the camera about 130 mm above B and a field about 214 mm wide.
   - If the chosen slot is seen occupied there, or neither slot can be judged, it refuses before "lower".
   - There is no separate inspect pose.

## Open questions (ask before building)

Answered 2026-09-30 (above): cubes, sources, the place choice, how the slots are seen. Still open: the slot pitch and geometry, the occupancy tolerance and freshness, the pick rule for GRIP with two cubes, and where the override goes.

- **Cubes:** how many, and which ids? Is each in a parts file like `tools/fr3/parts/cube55.yaml`, with the same size and sticker offset?
- **Sources:** where do cubes come from and go? Only the two slots (swap, fill, empty), or also a pick area on the table ("A")?
- **What the machine chooses:**
  - Place slot: first free, a preferred slot, or nearest?
  - Pick: which cube id, or which occupied slot?
- **Slot geometry:** pockets or walls, the clearance around a 55 mm cube, and the exact pitch (calipers). This decides the descent speed and the contact handling in "lower".
- **Seeing the slots:**
  - Is the approach pose over B (`grip_approach_m` 100 mm above the set-down, the camera roughly 130+ mm above B) enough to see both slots? At 130 mm the D405 colour field is about 214 mm wide, and the slots reach ±87.5 mm.
  - Or is a dedicated "inspect B" pose needed?
- **Freshness:** the occupancy tolerance (distance from the slot centre, e.g. 15 mm) and how old a sighting may be.
- **Manual override:** where in the panel? A "B slot" and a "pick" dropdown in the drawer next to GRIP were proposed.

## Proposed phases

Each phase ends with its own tests passing, then the arm check.

1. **Slot geometry in grip_node** (small; motion code, so it gets a review).
   - Parameters:
     - `grip_target_slots_m`: the slot centres in B's frame, [[−0.06, 0], [0.06, 0]].
     - `grip_target_finger_axis`: `y` means the fingers close across B y.
     - The slot index for PLACE AT B, which the panel sends before every PLACE AT B, as it already does with the box.
   - `place_tcp` gets a yaw restricted to the finger axis.
   - The panel gets a manual slot dropdown.
   - Tests: `test_grip_logic.py` (slot position in B's frame, the yaw restriction whatever the arrival yaw, the refusals) and the mock smoke test.
2. **Several cubes in vision.**
   - A list of cube ids.
   - Per-id poses on the `/object/*` contract, or one array topic with ids.
   - GRIP by id.
   - The bag recorder and `frames.jsonl` record every id.
3. **The slot state machine**, a pure module (e.g. `tools/fr3/cell/slots.py`, no ROS).
   - Per-slot state: empty, occupied(id) or unknown, each with a timestamp and its source (vision or action).
   - Updated from sightings (a cube marker within tolerance of a slot centre, with B seen in the same frame) and from GRIP/PLACE results.
   - It chooses the place slot and the pick target, and returns a refusal reason when unsure.
   - Unit tests over transition tables, including a sighting that disagrees with the record.
4. **Panel:** it shows the slot state and the choice (e.g. "PLACE AT B → slot 2"), with the manual override and the reason for any refusal.
5. **Arm checks:** PLACE AT B into each slot; a refusal when a slot is seen occupied; GRIP from a slot; a swap sequence with several cubes.

## Related findings from 2026-09-29

These don't block the plan, but matter for it.

- **Lateral drift with distance:** the marker's y drifts about −0.95 mm per 100 mm of camera distance (hand-eye rotation or principal point). A slot read from far away can be off by 1-2 mm.
- **The grasp sits about 2.5 mm high:** the descent stops inside the 3 mm tolerance, from above. This affects the set-down height too.
- **No finger width in the recordings:** `/joint_states` fr3_finger_joint1/2 read 0.0. Recording the gripper width would show whether a place released cleanly.
