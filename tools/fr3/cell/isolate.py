"""Put this process on the mock's own DDS domain before rclpy loads.

The mock cell and a --mock GUI both call isolate() first thing. It re-execs
the interpreter with ROS_DOMAIN_ID 88 and a loopback-only CycloneDDS config
(the tracking_smoke.py pattern, on a different domain), so a mock session can
neither see nor command the real cell on domain 0 - whatever the shell had
sourced.
"""

import os
import sys
import tempfile

DOMAIN = '88'
_FLAG = '_FR3_CELL_MOCK'
LO_ONLY = """<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any">
  <General><Interfaces><NetworkInterface name="lo"/></Interfaces>
    <AllowMulticast>false</AllowMulticast></General>
  <Discovery><ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
    <Peers><Peer address="localhost"/></Peers></Discovery>
</Domain></CycloneDDS>
"""


def isolated():
    return (os.environ.get('ROS_DOMAIN_ID') == DOMAIN
            and os.environ.get(_FLAG) == '1'
            and 'fr3_cell_mock_dds' in os.environ.get('CYCLONEDDS_URI', ''))


def isolate():
    """Re-exec onto the isolated domain; returns only once there."""
    if isolated():
        return
    xml = os.path.join(tempfile.gettempdir(), f'fr3_cell_mock_dds_{os.getuid()}.xml')
    with open(xml, 'w') as f:
        f.write(LO_ONLY)
    os.environ.update(ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='0', **{_FLAG: '1'},
                      RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file://' + xml)
    sys.stdout.flush()
    os.execv(sys.executable, [sys.executable] + sys.argv)
