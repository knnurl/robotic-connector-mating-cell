#!/usr/bin/env python3
"""Shadow mode (object_shadow): the depth estimate is seeded from the marker
and reports its agreement with it; a held part or a frame without the marker
is never estimated; the frame loop keeps inside its period; the outline goes
only on the image it is given."""

import functools

import numpy as np
import pytest

from roscam.object_pose import rpy_matrix
from roscam.object_shadow import DepthShadow, pose_matrix
from test_object_pose import K, PART, colour_scene, perturbed, pose

T_CUBE = pose([0.004, -0.003, 0.150], yaw=10.0, tilt=2.0)


@functools.lru_cache(maxsize=None)
def scene():
    return colour_scene(T_CUBE)


def part(xyz_mm=(0.0, 0.0, 0.0), yaw_deg=0.0):
    T_mo = np.eye(4)
    T_mo[:3, :3] = rpy_matrix([0.0, 0.0, yaw_deg])
    T_mo[:3, 3] = np.asarray(xyz_mm) / 1000.0
    return dict(PART, T_marker_object=T_mo)


def cand(out):
    v = [float(x) for x in out['cand_pose'].split(',')]
    return pose_matrix(v[:3], v[3:])


def test_the_estimate_is_seeded_from_the_marker_and_reports_its_agreement():
    depth, bgr = scene()
    marker = perturbed(T_CUBE, 0.6, 0.4)               # the marker, a little off
    out = DepthShadow(part()).step(depth, bgr, K, None, marker)
    assert out['seeded_from'] == 'marker' and out['depth_valid'] == 'true', out['depth_reason']
    assert out['holding'] == 'false' and out['edge_source'] == 'colour'
    D = np.linalg.inv(T_CUBE) @ cand(out)
    assert np.linalg.norm(D[:3, 3]) < 0.0003
    assert 0.4 < float(out['agree_mm']) < 0.8          # the marker's own offset, measured
    assert 0.2 < float(out['agree_deg']) < 0.6
    assert float(out['depth_ms']) > 0.0 and int(out['n_pts']) >= 200


def test_T_marker_object_takes_the_marker_to_the_part():
    """The sticker 3 mm off-centre and turned 5 deg: the seed is the part,
    not the marker."""
    depth, bgr = scene()
    p = part(xyz_mm=(3.0, -2.0, 0.0), yaw_deg=5.0)
    marker = T_CUBE @ np.linalg.inv(p['T_marker_object'])
    out = DepthShadow(p).step(depth, bgr, K, None, marker)
    assert out['depth_valid'] == 'true', out['depth_reason']
    assert float(out['agree_mm']) < 0.3 and float(out['agree_deg']) < 0.3


def test_nothing_is_estimated_while_held_or_without_a_marker():
    depth, bgr = scene()
    sh = DepthShadow(part())
    sh.est.process = lambda *a, **k: pytest.fail('estimated')
    sh.holding = True
    assert sh.step(depth, bgr, K, None, T_CUBE) == {
        'holding': 'true', 'seeded_from': 'none', 'depth_valid': 'false',
        'depth_reason': 'HELD'}
    sh.holding = False
    assert sh.step(depth, bgr, K, None, None)['depth_reason'] == 'no marker prior'
    assert sh.step(None, bgr, K, None, T_CUBE)['depth_reason'] == 'no depth'
    assert sh.last is None


def test_the_budget_skips_one_frame_never_two():
    sh = DepthShadow(part())
    q = {'reason': '', 'compute_ms': 40.0, 'n_pts': 0, 'weak_dof': [], 'sym_index': 0,
         'edge_source': None, 'rms_mm': None, 'inlier_frac': None, 'agree_mm': None,
         'agree_deg': None, 'cand_pose': None}
    sh.est.process = lambda *a, **k: (None, False, q)
    depth = np.zeros((4, 4))
    seeds = lambda n, left: [sh.step(depth, None, K, None, T_CUBE,         # noqa: E731
                                     budget_ms=left)['seeded_from'] for _ in range(n)]
    assert seeds(5, 30.0) == ['marker', 'none', 'marker', 'none', 'marker']
    assert seeds(3, 50.0) == ['marker'] * 3             # it fits: every frame
    assert seeds(2, None) == ['marker'] * 2             # no budget given: no limit


def test_the_outline_goes_on_the_given_image_only():
    depth, bgr = scene()
    clean = bgr.copy()
    sh = DepthShadow(part())
    sh.step(depth, bgr, K, None, T_CUBE)
    assert np.array_equal(bgr, clean)                  # never drawn on its input
    debug = bgr.copy()
    sh.draw_outline(debug, K, None)
    changed = np.argwhere(np.any(debug != bgr, axis=2))
    # the top face's corners in the image: the drawing stays on its outline
    c = np.array([[x, y, 0.0] for x in (-0.0275, 0.0275) for y in (-0.0275, 0.0275)])
    P = c @ T_CUBE[:3, :3].T + T_CUBE[:3, 3]
    uv = P[:, :2] / P[:, 2:] * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    assert len(changed) > 200
    assert changed[:, 1].min() >= uv[:, 0].min() - 3 and changed[:, 1].max() <= uv[:, 0].max() + 3
    assert changed[:, 0].min() >= uv[:, 1].min() - 3 and changed[:, 0].max() <= uv[:, 1].max() + 3
    sh.holding = True
    sh.step(depth, bgr, K, None, T_CUBE)
    again = bgr.copy()
    sh.draw_outline(again, K, None)                    # held: no estimate, nothing drawn
    assert np.array_equal(again, bgr)
