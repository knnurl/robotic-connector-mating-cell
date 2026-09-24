"""The settings drawer, kept across restarts.

The operator's drawer values (target and safety, the workspace box, motion
steps, TRACK entry, marker loss, pose-jump alarm, camera view) are written
to ~/.config/fr3_cell/ on every change and restored at the next start.
One file per cell: settings.yaml for the real one, settings_mock.yaml for the
mock, so a mock experiment never leaks into a real session.

Also kept: the last gains APPLIED (not unapplied edits). A relaunch reloads
the controller's yaml gains, so the panel writes these back at the next
FLOAT/HOLD activation, where the equilibrium seeds at the arm.

Deliberately NOT kept (they start at their safe defaults on every launch):
the two speed sliders (panel spec: never restore the previous session's speed)
and the robot-state gate (a safety interlock, ON at every launch as always).
"""

import os
import pathlib

import yaml

KEYS = ('standoff_mm', 'pos_tol_mm', 'inplane', 'floor_mm', 'box_x_mm', 'box_y_mm',
        'box_z_max_mm', 'step_mm', 'rot_deg', 'track_entry_mm', 'track_entry_deg',
        'marker_loss', 'marker_loss_ms', 'pose_jump_mm', 'camera', 'gains')
HEADER = ('# FR3 Cell Control - the settings drawer as last left, restored at start.\n'
          '# Written by the GUI on every change. Delete this file to go back to the\n'
          '# defaults in tools/fr3/cell/config/settings.yaml.\n')


def path(mock=False):
    root = os.environ.get('XDG_CONFIG_HOME') or str(pathlib.Path.home() / '.config')
    return pathlib.Path(root) / 'fr3_cell' / ('settings_mock.yaml' if mock
                                                  else 'settings.yaml')


def load(p):
    """(values, problem). Missing file = nothing to restore; a broken file is
    reported and ignored, never fatal. Unknown keys are dropped."""
    try:
        doc = yaml.safe_load(pathlib.Path(p).read_text())
    except FileNotFoundError:
        return {}, None
    except Exception as e:                                  # noqa: BLE001
        return {}, f'could not read {p}: {e}'
    if doc is None:
        return {}, None
    if not isinstance(doc, dict):
        return {}, f'{p} is not a mapping - ignored'
    return {k: v for k, v in doc.items() if k in KEYS}, None


def save(p, values):
    """Atomic: a crash mid-write leaves the previous file, not half of one."""
    p = pathlib.Path(p)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_name(p.name + '.tmp')
    tmp.write_text(HEADER + yaml.safe_dump({k: values[k] for k in KEYS if k in values},
                                           sort_keys=False))
    os.replace(tmp, p)
