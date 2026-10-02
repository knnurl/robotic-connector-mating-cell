#!/usr/bin/env python3
"""depth_quality against synthetic recordings: a cube top (the marker at its
centre) standing on a table, ray cast with known intrinsics, written with
FrameRecorder as the camera loop writes them. The expected values come from
the ray-cast geometry, not from depth_quality's own masks.

    python3 -m pytest -q tools/fr3/vision/test_depth_quality.py
"""

import cv2
import numpy as np

import depth_quality as dq
from roscam.frame_recorder import FrameRecorder
from roscam.rs_capture import Frame

W, H = 640, 480
FX = FY = 430.0
CX, CY = 322.0, 238.0
SCALE = 1e-4                     # D405 depth unit, m
MARKER, CUBE = 0.021, 0.055
NOISE = 0.00025                  # injected depth noise, m
QUANT_RMS = SCALE / np.sqrt(12)  # the uint16 rounding adds this


def rot(axis, deg):
    return cv2.Rodrigues(np.asarray(axis, dtype=float) * np.radians(deg))[0]


def quat(axis, deg):
    """(x, y, z, w) about a unit axis."""
    a = np.radians(deg) / 2
    return np.array([*(np.asarray(axis, dtype=float) * np.sin(a)), np.cos(a)])


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def pose(z_mm, tilt=6.0, inplane=20.0):
    """Marker facing the camera (its z toward the lens), tilted about the
    camera x and turned in plane; R and the q the recorder stores."""
    R = rot([1, 0, 0], tilt) @ rot([1, 0, 0], 180) @ rot([0, 0, 1], inplane)
    q = qmul(qmul(quat([1, 0, 0], tilt), quat([1, 0, 0], 180)), quat([0, 0, 1], inplane))
    return R, np.array([0.004, -0.003, z_mm / 1000.0]), q


def render(R, t, table=True):
    """Ray cast: depth z per pixel, the rays' depth on the top plane and on
    the table, and where each ray meets the top plane in marker coordinates.
    The camera is over the top face, so no side face is visible."""
    v, u = np.mgrid[0:H, 0:W].astype(float)
    ray = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u)], axis=-1)
    n = R[:, 2]
    z_top = (n @ t) / (ray @ n)
    z_tab = (n @ (t - CUBE * n)) / (ray @ n)
    local = (ray * z_top[..., None] - t) @ R
    on_top = np.max(np.abs(local[..., :2]), axis=-1) <= CUBE / 2
    z = np.where(on_top, z_top, z_tab if table else 0.0)
    return z, z_top, z_tab, local, ray @ n


def outline(local, half_width):
    """Pixels whose top-plane point is within half_width of the cube outline."""
    return np.abs(np.max(np.abs(local[..., :2]), axis=-1) - CUBE / 2) <= half_width


def noisy(z, seed):
    return np.where(z > 0, z + np.random.default_rng(seed).normal(0, NOISE, z.shape), 0.0)


def record(path, frames, gain=1.0):
    """frames: (depth z or None, t, q or None for no pose, stamp). gain
    scales the depth written: a depth-scale error of gain - 1."""
    rec = FrameRecorder(max_queue=1000)
    rec.start(path, {'capture': {'width': W, 'height': H, 'fps': 15, 'depth': True,
                                 'preset': 'High Accuracy', 'spatial_filter': False,
                                 'exposure_us': 'auto', 'depth_scale': SCALE},
                     'intrinsics': {'fx': FX, 'fy': FY, 'cx': CX, 'cy': CY,
                                    'coeffs': [0.0] * 5, 'width': W, 'height': H},
                     'marker_id': 0, 'marker_size_m': MARKER})
    for i, (z, t, q, stamp) in enumerate(frames):
        raw = None if z is None else np.round(z * gain / SCALE).astype(np.uint16)
        rec.record(Frame(np.zeros((H, W, 3), np.uint8), None if raw is None else raw * SCALE,
                         100.0 + i / 15, 1e6 + i * 66.7, 'hardware_clock', raw, SCALE),
                   stamp, {} if q is None else {dq.RAW_TOPIC: (stamp, t, q)})
    assert rec.stop()['dropped'] == 0
    return path


def one_row(path):
    rows, _ = dq.session_rows(path, CUBE)
    assert len(rows) == 1
    return rows[0]


def static(path, z_mm, depth, n=2, gain=1.0):
    """n frames of the same pose; depth(i) gives frame i's depth."""
    _, t, q = pose(z_mm)
    return one_row(record(path, [(depth(i), t, q, 10.0 + i / 15) for i in range(n)], gain))


def test_q2R_takes_x_y_z_w():
    for axis, deg in (([1, 0, 0], 186), ([0.3, -0.5, 0.8], 40), ([0, 1, 0.2], -75)):
        axis = np.asarray(axis) / np.linalg.norm(axis)
        assert np.allclose(dq.q2R(quat(axis, deg)), rot(axis, deg))


def test_a_clean_frame_reads_full_fill_the_injected_noise_and_no_bias(tmp_path):
    z = render(*pose(150)[:2])[0]
    row = static(tmp_path / 's', 150, lambda i: noisy(z, i), n=3)
    assert row['bucket_mm'] == 150 and row['frames'] == '3/3'
    assert abs(row['dist_mm'] - 150.0) < 1e-6
    assert row['fill_pct'] > 99.9 and row['fill_p10_pct'] > 99.9
    expected_mm = np.hypot(NOISE, QUANT_RMS) * 1e3
    assert abs(row['rms_mm'] - expected_mm) < 0.05 * expected_mm
    assert abs(row['bias_pct']) < 0.1
    assert row['flying_pct'] < 0.05 and row['table_pct'] == 100.0


def test_holes_lower_the_fill_by_what_they_remove_from_the_ring(tmp_path):
    R, t, _ = pose(150)
    z, _, _, local, _ = render(R, t)
    # the marker itself is 1 / 1.6^2 of the ring cam_pub fits
    marker = np.max(np.abs(local[..., :2]), axis=-1) <= MARKER / 2
    row = static(tmp_path / 'm', 150, lambda i: np.where(marker, 0.0, noisy(z, i)))
    assert abs(row['fill_pct'] - 100 * (1 - 1 / 1.6 ** 2)) < 2.5
    # random dropout: fill is what is left
    drop = lambda i: np.random.default_rng(10 + i).random(z.shape) < 0.25  # noqa: E731
    row = static(tmp_path / 'r', 150, lambda i: np.where(drop(i), 0.0, noisy(z, i)))
    assert abs(row['fill_pct'] - 75.0) < 2.0 and abs(row['fill_p10_pct'] - 75.0) < 2.5
    assert abs(row['rms_mm'] - np.hypot(NOISE, QUANT_RMS) * 1e3) < 0.02


def test_a_depth_scale_error_reads_as_that_bias(tmp_path):
    # the 5x5 median moves in whole depth units: 0.1 mm = 0.067% at 150 mm
    for z_mm, gain in ((150, 1.014), (200, 0.986)):
        z = render(*pose(z_mm)[:2])[0]
        row = static(tmp_path / f's{z_mm}', z_mm, lambda i: noisy(z, i), n=5, gain=gain)
        assert row['bucket_mm'] == z_mm
        assert abs(row['bias_pct'] - (gain - 1) * 100) < 0.1
        assert row['fill_pct'] > 99.9 and row['flying_pct'] < 0.05


def _silhouette(table):
    """A frame with depth injected along the outline (within 1 mm of it, so
    inside the 2 mm band), in sets 1 mm either side of each threshold.
    Returns (depth, pixels that must count as flying, pixels with depth in
    the +/- 2 mm band)."""
    z, z_top, z_tab, local, ndotray = render(*pose(150)[:2], table=table)
    z = noisy(z, 0)
    idx = np.flatnonzero(outline(local, 0.001))
    below_top = lambda mm: z_top + mm / 1000.0 / np.abs(ndotray)    # noqa: E731
    above_tab = lambda mm: z_tab - mm / 1000.0 / np.abs(ndotray)    # noqa: E731
    if table:
        sets = [((z_top + z_tab) / 2, True), (below_top(4), True), (below_top(-4), True),
                (above_tab(4), True), (above_tab(2), False), (below_top(2), False)]
    else:       # the stand-in table: 3-20 mm below the top (or > 3 mm above) counts
        sets = [(below_top(4), True), (below_top(18), True), (below_top(-4), True),
                (below_top(2), False), (below_top(22), False)]
    flying = np.zeros(z.shape, bool)
    for k, (depth, counts) in enumerate(sets):
        sel = idx[k::len(sets)]
        z.flat[sel] = depth.flat[sel]
        flying.flat[sel] = counts
    band = outline(local, 0.002) & (z > 0)
    return z, flying, band


def test_flying_pixels_at_the_silhouette_are_counted(tmp_path):
    z, flying, band = _silhouette(table=True)
    row = static(tmp_path / 's', 150, lambda i: z)
    expected = 100.0 * flying.sum() / band.sum()
    assert expected > 5.0
    assert abs(row['flying_pct'] - expected) < 0.1 * expected
    assert row['table_pct'] == 100.0


def test_without_a_table_points_3_to_20_mm_below_the_top_count(tmp_path):
    z, flying, band = _silhouette(table=False)
    row = static(tmp_path / 's', 150, lambda i: z)
    expected = 100.0 * flying.sum() / band.sum()
    assert abs(row['flying_pct'] - expected) < 0.1 * expected
    assert row['table_pct'] == 0.0


def test_rows_split_by_distance_with_their_own_rate_and_sigma(tmp_path, capsys):
    """Frames 0-3 at 100 mm (frame 3 without depth), 4-7 at 200 mm (frame 6
    without a pose): per bucket the rate spans only its own poses, sigma
    only its own scatter, and 'used' counts frames with depth and pose."""
    jitter = np.array([[0.3, 0, 0], [-0.3, 0, 0], [0.3, 0.1, 0], [-0.3, -0.1, 0.2]]) / 1000
    frames = []
    for i in range(8):
        z_mm = 100 if i < 4 else 200
        R, t, q = pose(z_mm)
        z = None if i == 3 else noisy(render(R, t)[0], i)
        frames.append((z, t + jitter[i % 4], None if i == 6 else q, 10.0 + i / 15))
    path = record(tmp_path / 's', frames)
    rows, _ = dq.session_rows(path, CUBE)
    by = {r['bucket_mm']: r for r in rows}
    assert list(by) == [100, 200]
    assert by[100]['frames'] == '3/8' and by[200]['frames'] == '3/8'
    assert abs(by[100]['raw_hz'] - 15.0) < 1e-6        # 4 poses, 1/15 s apart
    assert abs(by[200]['raw_hz'] - 10.0) < 1e-6        # frame 6 missed: 3 poses over 3/15 s
    for b, keep in ((100, [0, 1, 2, 3]), (200, [0, 1, 3])):
        sigma = np.sqrt((jitter[keep] * 1e3).var(axis=0, ddof=1).sum())
        assert abs(by[b]['sigma_mm'] - sigma) < 1e-6
        assert abs(by[b]['dist_mm'] - b) < 0.5

    assert dq.main([str(tmp_path), '--csv', str(tmp_path / 'dq.csv')]) == 0
    table = capsys.readouterr().out.strip().splitlines()
    assert table[0].startswith('| session |') and len(table) == 4
    assert '| 100 | 3/8 |' in table[2] and '| 200 | 3/8 |' in table[3]
    csv_lines = (tmp_path / 'dq.csv').read_text().splitlines()
    assert csv_lines[0].split(',')[:2] == ['session', 'res'] and len(csv_lines) == 3
