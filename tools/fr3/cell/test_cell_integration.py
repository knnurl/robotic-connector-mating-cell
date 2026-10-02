"""The two panel runs that need their own process: the screenshot diff against
the baselines (visual.py) and the ROS side against the mock cell
(mock_smoke.py, isolated DDS domain 88). Each is skipped, not failed, where
its dependencies are missing."""

import importlib.util
import os
import pathlib
import subprocess
import sys

import pytest

HERE = pathlib.Path(__file__).resolve().parent


def _have(*mods):
    return all(importlib.util.find_spec(m) is not None for m in mods)


@pytest.mark.skipif(not _have('PySide6', 'PIL'), reason='PySide6 / Pillow not installed')
def test_visual_baselines_unchanged():
    env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
    r = subprocess.run([sys.executable, str(HERE / 'visual.py')], env=env,
                       capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, r.stdout + r.stderr


@pytest.mark.skipif(not _have('rclpy', 'franka_msgs', 'topic_tools'),
                    reason='ROS 2 / franka_ros2 not sourced')
def test_ros_side_against_the_mock_cell():
    r = subprocess.run([sys.executable, str(HERE / 'mock_smoke.py')],
                       capture_output=True, text=True, timeout=400)
    tail = '\n'.join(r.stdout.splitlines()[-45:])
    assert r.returncode == 0, tail + r.stderr[-2000:]
