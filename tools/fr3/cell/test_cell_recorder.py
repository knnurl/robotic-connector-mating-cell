"""REC: the bag keeps recording after the action thread that started it has
gone (PR_SET_PDEATHSIG is per thread), and stops cleanly into a bag."""

import shutil
import tempfile
import threading
import time

import pytest

import actions

pytestmark = pytest.mark.skipif(shutil.which('ros2') is None, reason='needs ros2 on PATH')


def test_the_bag_outlives_the_thread_that_started_it(monkeypatch):
    # Not tmp_path: rosbag2 writes nothing under a path with a backslash, and
    # pytest's has one here (/tmp/pytest-of-ISDADS\ses634). The space is the
    # real runs folder's ('Robotic Connector Handling').
    logs = tempfile.TemporaryDirectory(prefix='fr3 rec ')
    monkeypatch.setenv('ROS_DOMAIN_ID', '88')           # never the cell's domain
    monkeypatch.setenv('FR3_LOG_DIR', logs.name)
    rec = actions.Recorder()
    t = threading.Thread(target=rec.start)              # as cell.py's _thread does
    t.start()
    t.join()
    deadline = time.monotonic() + 10.0
    while not (rec.path / 'metadata.yaml').exists() and not list(rec.path.glob('*.db3')):
        assert rec.running(), f'ros2 bag exited with {rec.proc.poll()}'
        assert time.monotonic() < deadline, 'the bag never opened its storage'
        time.sleep(0.1)
    assert rec.running()
    ok, msg = rec.stop()
    assert ok, msg
    assert (rec.path / 'metadata.yaml').exists()
    logs.cleanup()
