"""Relay a high-rate ROS topic down to a GUI-friendly rate, in C++.

Measured on this cell 2026-09-15: decoding `FrankaRobotState` at its native
1 kHz in Python costs **86% of a CPU core**, and taking the messages raw and
decoding only a few still costs 31%, because rclpy dispatches every callback
either way. A `topic_tools throttle` child relaying at 50 Hz costs 5%, and
20 ms is ample for anything a human looks at.

The child gets PR_SET_PDEATHSIG, so it dies with its parent even on kill -9
and relays cannot pile up across GUI restarts.

Subscribe to the relayed topic BEST EFFORT, depth 1: a slow reader must
never be able to back-pressure a publisher on the robot's side.
"""

import ctypes
import os
import pathlib
import signal
import subprocess

THROTTLE_HZ = 50


def throttle_executable():
    """Path to topic_tools' throttle, or raise if it is not installed."""
    from ament_index_python.packages import get_package_prefix
    exe = (pathlib.Path(get_package_prefix('topic_tools'))
           / 'lib' / 'topic_tools' / 'throttle')
    if not exe.exists():
        raise FileNotFoundError(f'{exe} not found (ros-humble-topic-tools)')
    return exe


def start_throttle(topic, out_topic, hz=THROTTLE_HZ, node_name='state_relay'):
    """Spawn the relay and return its Popen. Caller terminates it on exit."""
    exe = throttle_executable()
    libc = ctypes.CDLL('libc.so.6', use_errno=True)
    return subprocess.Popen(
        [str(exe), 'messages', topic, str(hz), out_topic, '--ros-args',
         '-r', f'__node:={node_name}_{os.getpid()}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=lambda: libc.prctl(1, signal.SIGTERM))   # PR_SET_PDEATHSIG


def stop_throttle(proc, timeout_s=2.0):
    """Terminate a relay started by start_throttle. Safe on None."""
    if proc is None or proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        proc.kill()
