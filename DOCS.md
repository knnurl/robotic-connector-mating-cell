# Documentation index

Every document in this repo, what it is for, and whether to trust it. Written
2026-09-22 during the workspace restructure, because the docs had grown to
twelve files with real overlap between them. Updated 2026-09-25.

**Nothing listed here has been deleted.** The redundancy section below says
what duplicates what, so the next person knows which copy to believe.

## Read these first

| Document | What it is | Trust |
|---|---|---|
| [GUIDE.md](GUIDE.md) | **Start here to operate the cell.** What changed, bring-up, the ladder, live tuning, shutdown, troubleshooting. Task-oriented; points at the authorities rather than repeating them. | **Current** - the operating authority |
| [PROJECT_STATE.md](PROJECT_STATE.md) | Where the project stands, what is next, open risks. The short current answer. | **Current** |
| [TODO.md](TODO.md) | Prioritised task list, plus lessons 1-20 that must not be re-learned. | **Current** |
| [ARM_CHECKLIST.md](ARM_CHECKLIST.md) | What the next arm session must check: everything built offline since 2026-09-25, in order, with pass criteria. | **Current** - tick it off at the arm |
| [TRACKING_SPEC.md](TRACKING_SPEC.md) | Implementation spec for continuous marker tracking, written against measured numbers. | **Current**, and amended in place where implementation proved it wrong — the **[AMENDED]** blocks are the parts to read |
| [PERCEPTION_PLAN.md](PERCEPTION_PLAN.md) | The marker-free perception plan: the `/object/*` contract and phases 0-7. | **Current** plan. Phases 0-2 are partly implemented; what is done is in TODO.md |

## Reference manuals

| Document | What it is | Trust |
|---|---|---|
| [SETUP_AND_CALIBRATION.md](SETUP_AND_CALIBRATION.md) | Calibration procedure, parameter reference, porting checklist, troubleshooting. | Current, but its launch sequence (section 4) is the MELFA one - for the FR3 use `tools/fr3/README.md` |
| [tools/fr3/README.md](tools/fr3/README.md) | FR3 cell: bandwidth/RT hardening, launch order, FR3-specific calibration. | **Current** - the FR3 bring-up authority |
| [fr3_mating_controllers/README.md](fr3_mating_controllers/README.md) | The Cartesian-impedance controller and its commissioning ladder. | **Current** - the ladder authority |
| [tools/fr3/cell/README.md](tools/fr3/cell/README.md) | The operator panel, and the controller-side TODOs C1-C10. | **Current** |
| [PERCEPTION_RESEARCH.md](PERCEPTION_RESEARCH.md) | The research behind PERCEPTION_PLAN.md: methods, sensors, benchmarks. | Current, as background |
| [melfa/parked/gui/README.md](melfa/parked/gui/README.md) | The old operator front-ends: Foxglove layout, tkinter panel, rqt recipe. | **Parked** 2026-09-23 with `mating_node` |
| [hardware/camera_mount/README.md](hardware/camera_mount/README.md) | The D405 bracket, its attribution, and the hand-eye numbers it produced. | Current |
| [CLAUDE.md](CLAUDE.md) | Instructions for AI assistants working in this repo. | Current |

## Historical - kept for the record, not for daily use

| Document | What it is | Trust |
|---|---|---|
| [HANDOFF.md](HANDOFF.md) | The 2026-07-05 audit: what the original code got wrong and why the rework looks as it does. | **Historical.** Section 4 (the original bugs) is still worth reading; its status claims are two months stale |
| [STATUS.md](STATUS.md) | The 2026-07-12 comprehensive snapshot: capability inventory, architecture diagram, verification matrix. | **Partly superseded** - see below |
| [melfa/BRINGUP.txt](melfa/BRINGUP.txt) | Six-terminal MELFA RV-5AS bring-up. | Parked with the rest of the MELFA side; the FR3 is the only active target |

## Redundant content (NOT removed - decide before deleting)

These overlap. Where two copies disagree, the one marked authority wins.

1. **Project state is told three times.** `PROJECT_STATE.md` (current),
   `STATUS.md` section 1 (2026-07-12), `HANDOFF.md` section 1 (2026-07-05).
   *Authority: PROJECT_STATE.md.* The other two are dated snapshots and
   should be read as history, not status.

2. **The verification matrix exists twice.** `STATUS.md` section 3 and
   `HANDOFF.md` section 7. Both predate the impedance commissioning, so both
   understate what has run. *Authority: STATUS.md section 3*, now updated.

3. **The launch order appears four times.** `GUIDE.md` section 2,
   `tools/fr3/README.md` ("Launch order"), `fr3_mating_controllers/README.md`
   ("Bring-up"), and `SETUP_AND_CALIBRATION.md` section 4 (the MELFA
   variant).
   *Authority: GUIDE.md section 2* - it is the only copy that states the
   `fr3_env.sh`-last rule, which is what actually broke a bring-up on
   2026-09-22. `tools/fr3/README.md` remains the authority for the *why*
   (bandwidth, RT hardening). This is the highest-risk duplication in the
   repo.

4. **The commissioning ladder appears three times.**
   `fr3_mating_controllers/README.md` (full), the `cell_panel.py` (IMPEDANCE & TRACK tab)
   module docstring (operator-facing), and `TODO.md` (checklist form).
   *Authority: fr3_mating_controllers/README.md.* The panel docstring is
   deliberately a short duplicate - it is what an operator reads at the
   terminal.

5. **The parameter reference exists twice.** `SETUP_AND_CALIBRATION.md`
   section 5 and the comments in `tools/fr3/fr3_params.yaml` /
   `fr3_mating_controllers/config/cartesian_impedance_stroke.yaml`.
   *Authority: the yaml files* - they cannot drift from the code without
   the range checks failing.

6. **`HANDOFF.md` section 2 and `tools/fr3/README.md` both describe the
   workspace-path-with-spaces problem.** *Authority: tools/fr3/fr3_env.sh*,
   which actually implements the workaround.

### If you want to cut

The safe deletions, in order of least regret: `STATUS.md` sections 1 and 5
(duplicated by PROJECT_STATE.md and by this repo's actual structure), then
`HANDOFF.md` sections 1, 3, 5-8 (superseded), keeping section 4 - the list
of original defects - somewhere permanent, because it is the only record of
what the patterns in this code exist to prevent.
