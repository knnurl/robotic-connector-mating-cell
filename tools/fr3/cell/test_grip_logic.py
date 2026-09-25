"""grip_logic: where the Hand goes for a marked cube. No ROS, no robot."""

import math

import numpy as np
import pytest

import grip_logic as G


def rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


FLAT = np.eye(3)                                        # marker face up, z = base z
DOWN = np.diag([1.0, -1.0, -1.0])                       # a TCP pointing straight down


def test_the_grasp_sits_below_the_top_face_approaching_straight_down():
    p, R = G.grasp_tcp(np.array([0.5, 0.0, 0.2]), FLAT, DOWN, 0.025)
    assert p == pytest.approx([0.5, 0.0, 0.175])
    assert R[:, 2] == pytest.approx([0, 0, -1])         # approach into the face
    assert R.T @ R == pytest.approx(np.eye(3))
    assert np.linalg.det(R) == pytest.approx(1.0)


@pytest.mark.parametrize('cube_yaw', [10, 30, 60, 100, 135, 170, -80])
def test_the_fingers_close_across_the_face_pair_needing_the_least_turn(cube_yaw):
    """Against brute force over all four face-aligned yaws (a two-candidate
    shortcut picks the long way round for some cube yaws)."""
    marker = rz(cube_yaw)
    _, R = G.grasp_tcp(np.zeros(3), marker, DOWN, 0.02)
    y = R[:, 1]
    assert max(abs(float(y @ marker[:, 0])), abs(float(y @ marker[:, 1]))) == pytest.approx(1.0)
    best = min(G.rotation_angle(np.column_stack([np.cross(c, [0, 0, -1.0]), c, [0, 0, -1.0]])
                                @ DOWN.T)
               for c in (marker[:, 0], -marker[:, 0], marker[:, 1], -marker[:, 1]))
    assert G.rotation_angle(R @ DOWN.T) == pytest.approx(best)
    assert G.rotation_angle(R @ DOWN.T) <= math.radians(45) + 1e-9


def test_a_tilted_marker_gives_a_proper_rotation_into_its_face():
    tilt = G.from_rotvec(np.array([0.2, -0.1, 0.0])) @ rz(25)
    p, R = G.grasp_tcp(np.array([0.5, 0.1, 0.2]), tilt, DOWN, 0.025)
    assert R[:, 2] == pytest.approx(-tilt[:, 2])
    assert R.T @ R == pytest.approx(np.eye(3))
    assert np.linalg.det(R) == pytest.approx(1.0)
    assert p == pytest.approx(np.array([0.5, 0.1, 0.2]) - tilt[:, 2] * 0.025)


def test_approach_and_lift_move_along_the_marker_normal():
    grasp = (np.array([0.5, 0.0, 0.175]), DOWN)
    p, R = G.above(grasp, FLAT, 0.10)
    assert p == pytest.approx([0.5, 0.0, 0.275]) and R is DOWN


def test_glide_ends_where_it_should_and_never_overshoots():
    a = (np.array([0.5, 0.0, 0.3]), DOWN)
    b = (np.array([0.5, 0.1, 0.2]), rz(40) @ DOWN)
    assert G.glide(a, b, 0.0)[0] == pytest.approx(a[0])
    assert G.glide(a, b, 1.0)[0] == pytest.approx(b[0])
    assert G.glide(a, b, 1.0)[1] == pytest.approx(b[1])
    assert G.glide(a, b, 1.7)[0] == pytest.approx(b[0])
    half = G.glide(a, b, 0.5)
    assert G.rotation_angle(half[1] @ a[1].T) == pytest.approx(math.radians(20))
    t = G.glide_time(a, b, 0.05, 1.0)
    assert t == pytest.approx(np.linalg.norm(b[0] - a[0]) / 0.05)
    turn_only = (a[0], rz(90) @ DOWN)                   # no travel: the wrist sets the time
    assert G.glide_time(a, turn_only, 0.05, 0.4) == pytest.approx(math.radians(90) / 0.4)


def test_compose_and_inverse_round_trip():
    a = (np.array([0.1, 0.2, 0.3]), rz(25) @ DOWN)
    p, R = G.compose(a, G.inverse(a))
    assert p == pytest.approx(np.zeros(3), abs=1e-12) and R == pytest.approx(np.eye(3))


def test_rotvec_round_trips_including_half_turns():
    axes = [np.array(a) / np.linalg.norm(a) for a in ([1, 0, 0], [1, -1, 0], [-1, 2, -3],
                                                      [0.3, -0.4, 0.87])]
    for axis in axes:
        for angle in (0.3, math.pi - 1e-6, math.pi - 1e-9, math.pi):
            R = G.from_rotvec(axis * angle)
            assert G.from_rotvec(G.rotvec(R)) == pytest.approx(R, abs=1e-6), (axis, angle)


BOX = (0.20, 0.80, -0.45, 0.45, 0.80)


def test_targets_outside_the_floor_or_box_are_refused():
    assert G.target_problem((0.5, 0.0, 0.3), 0.10, BOX) is None
    assert 'floor' in G.target_problem((0.5, 0.0, 0.05), 0.10, BOX)
    assert 'box' in G.target_problem((0.9, 0.0, 0.3), 0.10, BOX)
    assert 'box' in G.target_problem((0.1, 0.0, 0.3), 0.10, BOX)
    assert 'box' in G.target_problem((0.5, 0.5, 0.3), 0.10, BOX)
    assert 'box' in G.target_problem((0.5, -0.5, 0.3), 0.10, BOX)
    assert 'top' in G.target_problem((0.5, 0.0, 0.9), 0.10, BOX)


def test_the_led_equilibrium_is_clamped_inside_the_floor_and_box():
    assert G.clamp_to_limits((0.5, 0.0, 0.05), 0.10, BOX) == pytest.approx([0.5, 0.0, 0.10])
    assert G.clamp_to_limits((0.9, -0.6, 0.9), 0.10, BOX) == pytest.approx([0.80, -0.45, 0.80])
    assert G.clamp_to_limits((0.5, 0.1, 0.3), 0.10, BOX) == pytest.approx([0.5, 0.1, 0.3])


def test_the_hand_opens_wide_enough_but_never_past_its_stroke():
    assert G.open_width(0.055, 0.020) == pytest.approx(0.075)
    assert G.open_width(0.070, 0.020) == pytest.approx(0.080)
    assert G.cube_problem(0.055, 0.020) is None
    assert G.cube_problem(G.max_cube_m(0.020), 0.020) is None
    assert 'does not fit' in G.cube_problem(0.085, 0.020)
    assert 'per side' in G.cube_problem(G.max_cube_m(0.020) + 0.001, 0.020)
    assert 'per side' in G.cube_problem(0.030, 0.006)       # margin too small, per side


def test_the_grasp_depth_never_passes_half_the_cube():
    assert G.grasp_depth(0.055, 0.025) == pytest.approx(0.025)
    assert G.grasp_depth(0.030, 0.025) == pytest.approx(0.015)


def test_a_joint_near_its_stop_halts_the_grip():
    lower = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
    upper = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]
    ok = [0.0, -0.5, 0.0, -2.0, 0.0, 1.8, 0.8]
    assert G.joint_problem(ok, lower, upper, 0.05) is None
    near = list(ok)
    near[1] = -1.76
    assert 'J2' in G.joint_problem(near, lower, upper, 0.05)
    high = list(ok)
    high[5] = 4.49                                      # J6 near its UPPER stop
    assert 'J6' in G.joint_problem(high, lower, upper, 0.05)


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@pytest.mark.parametrize('yaw', [-135, -90, -45, -10, 0, 30, 45, 90, 180])
def test_quaternions_of_straight_down_poses_are_right(yaw):
    """Every grasp points the TCP straight down: a 180 deg rotation, where a
    sign-from-off-diagonals conversion picks the wrong yaw."""
    R = rz(yaw) @ DOWN
    assert q2R(*G.R2q(R)) == pytest.approx(R, abs=1e-12)


def test_quaternions_round_trip_in_general():
    rng = np.random.default_rng(1)
    for _ in range(300):
        v = rng.normal(size=3)
        R = G.from_rotvec(v / np.linalg.norm(v) * rng.uniform(0, math.pi))
        q = G.R2q(R)
        assert np.linalg.norm(q) == pytest.approx(1.0)
        assert q2R(*q) == pytest.approx(R, abs=1e-9)

