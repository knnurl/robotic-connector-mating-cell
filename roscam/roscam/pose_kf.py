"""6-DOF pose Kalman filter for marker tracking. Pure numpy, no ROS.

Translation: linear Kalman filter with a constant-velocity model
(state = [position, velocity]). Orientation: multiplicative small-angle
error filter around the current quaternion estimate (constant-orientation
model with process noise) — effectively a covariance-weighted slerp with a
principled innovation gate.

Both share a Mahalanobis gate: measurements inconsistent with the predicted
state are rejected (replaces ad-hoc jump thresholds), and the filter can
predict forward briefly when detections drop out.
"""

import math

import numpy as np


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_from_rotvec(v):
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = np.asarray(v) / angle
    s = math.sin(angle / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0)])


def small_angle_error(q_meas, q_est):
    """Rotation vector taking q_est to q_meas (shortest way)."""
    dq = quat_multiply(q_meas, np.array([-q_est[0], -q_est[1], -q_est[2], q_est[3]]))
    if dq[3] < 0.0:
        dq = -dq
    # For small errors, rotvec ~= 2 * vector part / scalar part
    return 2.0 * dq[:3] / max(dq[3], 1e-6)


class PoseKF:
    def __init__(self, sigma_accel=0.08, sigma_rot_rate_deg=15.0,
                 meas_std_pos=0.002, meas_std_rot_deg=1.0, gate_sigma=3.0):
        self.sigma_accel = float(sigma_accel)                       # m/s^2
        self.sigma_rot_rate = math.radians(float(sigma_rot_rate_deg))  # rad/s
        self.r_pos = float(meas_std_pos) ** 2
        self.r_rot = math.radians(float(meas_std_rot_deg)) ** 2
        self.gate2 = float(gate_sigma) ** 2

        self.x = None       # [px py pz vx vy vz]
        self.P = None       # 6x6 translation covariance
        self.q = None       # quaternion estimate (x, y, z, w)
        self.P_rot = None   # 3x3 attitude-error covariance

    @property
    def initialized(self):
        return self.x is not None

    def reset(self):
        self.x = self.P = self.q = self.P_rot = None

    def _init(self, pos, quat):
        self.x = np.zeros(6)
        self.x[:3] = pos
        self.P = np.diag([self.r_pos] * 3 + [0.25] * 3)  # generous velocity prior
        self.q = np.asarray(quat, dtype=float)
        self.q /= np.linalg.norm(self.q)
        self.P_rot = np.eye(3) * self.r_rot

    def predict(self, dt):
        if not self.initialized or dt <= 0.0:
            return
        dt = min(dt, 0.5)  # a huge gap must not explode the covariance
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        q_a = self.sigma_accel ** 2
        Q = np.zeros((6, 6))
        Q[:3, :3] = np.eye(3) * (q_a * dt ** 4 / 4.0)
        Q[:3, 3:] = Q[3:, :3] = np.eye(3) * (q_a * dt ** 3 / 2.0)
        Q[3:, 3:] = np.eye(3) * (q_a * dt ** 2)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.P_rot = self.P_rot + np.eye(3) * (self.sigma_rot_rate ** 2 * dt ** 2)

    def update(self, pos, quat):
        """Fuse a measurement (call predict() first). Returns True if the
        measurement passed the innovation gate and was applied."""
        pos = np.asarray(pos, dtype=float)
        quat = np.asarray(quat, dtype=float)
        if not self.initialized:
            self._init(pos, quat)
            return True

        # Translation gate + update
        H = np.zeros((3, 6))
        H[:, :3] = np.eye(3)
        innov = pos - self.x[:3]
        S = self.P[:3, :3] + np.eye(3) * self.r_pos
        maha_pos = float(innov @ np.linalg.solve(S, innov))

        # Rotation gate
        err = small_angle_error(quat, self.q)
        S_rot = self.P_rot + np.eye(3) * self.r_rot
        maha_rot = float(err @ np.linalg.solve(S_rot, err))

        if maha_pos > self.gate2 * 3.0 or maha_rot > self.gate2 * 3.0:
            return False  # (gate2*3 ~ chi-square 3-dof at the same confidence)

        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.P = (np.eye(6) - K @ H) @ self.P

        K_rot = self.P_rot @ np.linalg.inv(S_rot)
        delta = K_rot @ err
        self.q = quat_multiply(quat_from_rotvec(delta), self.q)
        self.q /= np.linalg.norm(self.q)
        self.P_rot = (np.eye(3) - K_rot) @ self.P_rot
        return True

    @property
    def position(self):
        return self.x[:3].copy()

    @property
    def quaternion(self):
        return self.q.copy()
