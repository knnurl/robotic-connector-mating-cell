"""The grip node's geometry and checks: pure functions of numpy poses, no ROS.

A pose is (p, R): position (3,) in metres and rotation matrix (3, 3), both
in the robot base frame. The marker sits centred on the cube's top face,
its z axis out of the face (ArUco convention); the Franka Hand closes its
fingers along the TCP's y axis and approaches along the TCP's z axis.
"""

import math

import numpy as np

HAND_MAX_OPEN_M = 0.080         # Franka Hand stroke
MIN_SIDE_CLEARANCE_M = 0.004    # per side, fully open, before the descent


def grasp_tcp(marker_p, marker_R, tcp_R_now, depth_m):
    """The TCP pose that grips the cube: depth_m below the marked top face,
    approaching against the marker normal, fingers closing across one pair
    of faces. Of the four face-aligned yaws, the one nearest the hand's
    current yaw, so the wrist turns as little as possible."""
    n = marker_R[:, 2]
    z = -n                                              # approach: into the top face
    best = None
    for y in (marker_R[:, 0], -marker_R[:, 0], marker_R[:, 1], -marker_R[:, 1]):
        y = y - z * float(y @ z)
        y = y / np.linalg.norm(y)
        R = np.column_stack([np.cross(y, z), y, z])
        turn = rotation_angle(R @ tcp_R_now.T)
        if best is None or turn < best[0]:
            best = (turn, R)
    return marker_p - n * depth_m, best[1]


def above(pose, marker_R, height_m):
    """pose moved height_m along the marker normal (up, off the top face)."""
    p, R = pose
    return p + marker_R[:, 2] * height_m, R


def R2q(R):
    """(x, y, z, w) of rotation matrix R. Shepperd's method: built from the
    largest of w, x, y, z, so it stays right at 180 deg - which every
    straight-down TCP pose is (a sign-from-off-diagonals shortcut is not)."""
    t = np.trace(R)
    d = [t, R[0, 0], R[1, 1], R[2, 2]]
    i = int(np.argmax(d))
    if i == 0:
        w = math.sqrt(1.0 + t) / 2.0
        x, y, z = (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)
    elif i == 1:
        x = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) / 2.0
        w, y, z = (R[2, 1] - R[1, 2]) / (4 * x), (R[0, 1] + R[1, 0]) / (4 * x), (R[0, 2] + R[2, 0]) / (4 * x)
    elif i == 2:
        y = math.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) / 2.0
        w, x, z = (R[0, 2] - R[2, 0]) / (4 * y), (R[0, 1] + R[1, 0]) / (4 * y), (R[1, 2] + R[2, 1]) / (4 * y)
    else:
        z = math.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) / 2.0
        w, x, y = (R[1, 0] - R[0, 1]) / (4 * z), (R[0, 2] + R[2, 0]) / (4 * z), (R[1, 2] + R[2, 1]) / (4 * z)
    return float(x), float(y), float(z), float(w)


def _axis_sin(R):
    """sin(angle) * axis, from R's antisymmetric part."""
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2.0


def rotation_angle(R):
    """The rotation angle of R in [0, pi], well conditioned everywhere
    (atan2 of the antisymmetric and symmetric parts, not acos of the trace)."""
    return math.atan2(float(np.linalg.norm(_axis_sin(R))), (float(np.trace(R)) - 1.0) / 2.0)


def rotvec(R):
    """Axis * angle of rotation matrix R (the short way). Near 180 deg the
    antisymmetric part vanishes, so the axis comes from the symmetric part
    there and only its sign from the antisymmetric one."""
    a = rotation_angle(R)
    if a < 1e-12:
        return np.zeros(3)
    w = _axis_sin(R)
    if a < math.pi - 1e-3:
        return w / np.linalg.norm(w) * a
    B = np.eye(3) + (R + R.T - 2.0 * np.eye(3)) / (2.0 * (1.0 - math.cos(a)))   # = k k^T
    i = int(np.argmax(np.diag(B)))
    k = B[:, i] / math.sqrt(max(B[i, i], 1e-300))
    if float(k @ w) < 0.0:
        k = -k
    return k / np.linalg.norm(k) * a


def from_rotvec(v):
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def glide_time(a, b, speed_mps, turn_radps):
    """How long a straight glide from pose a to b takes at these speeds."""
    dist = float(np.linalg.norm(b[0] - a[0]))
    turn = rotation_angle(b[1] @ a[1].T)
    return max(dist / speed_mps, turn / turn_radps, 1e-3)


def glide(a, b, s):
    """The pose a fraction s in [0, 1] of the way from a to b."""
    s = min(1.0, max(0.0, s))
    return (a[0] + (b[0] - a[0]) * s, from_rotvec(rotvec(b[1] @ a[1].T) * s) @ a[1])


def compose(a, b):
    """a * b for (p, R) poses."""
    return a[0] + a[1] @ b[0], a[1] @ b[1]


def inverse(a):
    return -a[1].T @ a[0], a[1].T


def target_problem(p, floor_m, box):
    """None if an equilibrium position may be commanded, else why not.
    box: (x_min, x_max, y_min, y_max, z_max) in metres."""
    x, y, z = p
    if z < floor_m:
        return (f'z {z*1000:.0f} mm is below the floor {floor_m*1000:.0f} mm - the camera '
                'bracket hangs below the flange')
    if not box[0] <= x <= box[1] or not box[2] <= y <= box[3]:
        return (f'({x*1000:.0f}, {y*1000:.0f}) mm is outside the workspace box '
                f'x [{box[0]*1000:.0f}, {box[1]*1000:.0f}] y [{box[2]*1000:.0f}, '
                f'{box[3]*1000:.0f}]')
    if z > box[4]:
        return f'z {z*1000:.0f} mm is above the box top {box[4]*1000:.0f} mm'
    return None


def open_width(cube_m, margin_m):
    """How wide to open before the descent, never past the Hand's stroke."""
    return min(cube_m + margin_m, HAND_MAX_OPEN_M)


def grasp_depth(cube_m, depth_m):
    """How far below the top face the TCP grips: grip_depth_m, but never
    past half the cube, so a small cube keeps the fingertips off the table."""
    return min(depth_m, cube_m / 2.0)


def cube_problem(cube_m, margin_m):
    if not 0.005 < cube_m < HAND_MAX_OPEN_M:
        return (f'a {cube_m*1000:.0f} mm cube does not fit the Hand '
                f'({HAND_MAX_OPEN_M*1000:.0f} mm stroke)')
    side = (open_width(cube_m, margin_m) - cube_m) / 2.0
    if side < MIN_SIDE_CLEARANCE_M - 1e-9:
        return (f'{side*1000:.1f} mm of clearance per side at full opening, under '
                f'{MIN_SIDE_CLEARANCE_M*1000:.0f} mm - the fingers would hit the cube')
    return None


def max_cube_m(margin_m):
    """The largest cube cube_problem accepts, for the panel's drawer range:
    0 when the margin itself leaves under MIN_SIDE_CLEARANCE_M per side."""
    if margin_m / 2.0 < MIN_SIDE_CLEARANCE_M - 1e-9:
        return 0.0
    return HAND_MAX_OPEN_M - 2 * MIN_SIDE_CLEARANCE_M


def clamp_to_limits(p, floor_m, box):
    """p moved just inside the floor and box - for the equilibrium the
    friction lead pushes, whose unleaded target was already checked."""
    return np.array([min(max(p[0], box[0]), box[1]), min(max(p[1], box[2]), box[3]),
                     min(max(p[2], floor_m), box[4])])


def joint_problem(q, lower, upper, margin_rad):
    """Why a joint is too close to its end stop to keep moving, or None."""
    for j, (qj, lo, hi) in enumerate(zip(q, lower, upper)):
        room = min(qj - lo, hi - qj)
        if room < margin_rad:
            return (f'J{j+1} is {math.degrees(room):.1f} deg from its end stop - '
                    'the grip stops here')
    return None


def target_tilt_deg(target_R):
    """How far a place target's normal leans from the base's vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(target_R[2, 2])))))


def place_tcp(target_p, target_R, offset_xy, cube_m, depth_m, tcp_R_now):
    """The TCP pose that sets a held cube down on a flat target marker: the
    cube centred on target + offset (in the target's own x/y), resting on
    the target's plane, its faces turned to the target's axes by the yaw
    needing the least wrist turn. The Hand holds the cube depth_m below its
    top face, so the TCP ends up (cube - depth) above the plane."""
    centre = target_p + target_R[:, 0] * offset_xy[0] + target_R[:, 1] * offset_xy[1]
    top = centre + target_R[:, 2] * cube_m
    return grasp_tcp(top, target_R, tcp_R_now, depth_m)


def carry_path(tcp_now, approach, clearance_m):
    """Waypoints from the lifted cube to above the target: straight up to
    a carry height above both ends, level across, then down to approach.
    Never a diagonal with the cube low."""
    z = max(tcp_now[0][2], approach[0][2]) + clearance_m
    up = (np.array([tcp_now[0][0], tcp_now[0][1], z]), tcp_now[1])
    across = (np.array([approach[0][0], approach[0][1], z]), approach[1])
    return [up, across, approach]
