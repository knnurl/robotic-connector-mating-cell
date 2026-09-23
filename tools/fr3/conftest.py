"""Shared fixture for the cell_panel tests: the GUI module with ROS stubbed.

No ROS, robot or display needed - only cell_panel's pure logic is exercised.
When cell_panel gains a ROS import, add its stub here, once.
"""

import importlib.util
import pathlib
import sys
import types

import pytest

HERE = pathlib.Path(__file__).resolve().parent

_STUBS = {
    'rclpy': {}, 'rclpy.node': {'Node': object},
    'rclpy.executors': {'SingleThreadedExecutor': object},
    'rclpy.action': {'ActionClient': object},
    'rclpy.signals': {'SignalHandlerOptions': object},
    'rclpy.qos': {'DurabilityPolicy': object, 'QoSDurabilityPolicy': object,
                  'QoSProfile': object, 'ReliabilityPolicy': object},
    'rcl_interfaces': {},
    'rcl_interfaces.msg': {'Parameter': object, 'ParameterType': object,
                           'ParameterValue': object},
    'rcl_interfaces.srv': {'GetParameters': object, 'SetParameters': object,
                           'SetParametersAtomically': object},
    'franka_msgs': {}, 'franka_msgs.msg': {'FrankaRobotState': object},
    'franka_msgs.srv': {'SetLoad': object,
                        'SetForceTorqueCollisionBehavior': object},
    'builtin_interfaces': {}, 'builtin_interfaces.msg': {'Duration': object},
    'controller_manager_msgs': {},
    'controller_manager_msgs.srv': {'SwitchController': object,
                                    'ListControllers': object},
    'diagnostic_msgs': {}, 'diagnostic_msgs.msg': {'DiagnosticStatus': object},
    'geometry_msgs': {},
    'geometry_msgs.msg': {'Pose': object, 'PoseStamped': object,
                          'TwistStamped': object},
    'moveit_msgs': {}, 'moveit_msgs.action': {'ExecuteTrajectory': object},
    'moveit_msgs.srv': {'GetCartesianPath': object},
    'sensor_msgs': {}, 'sensor_msgs.msg': {'JointState': object,
                                           'Image': object},
    'std_msgs': {}, 'std_msgs.msg': {'String': object, 'Bool': object,
                                     'Float64': object},
    'std_srvs': {}, 'std_srvs.srv': {'Trigger': object},
    'tf2_ros': {'Buffer': object, 'TransformListener': object},
}


def _load(filename, request):
    """Import one of the tools/fr3 GUIs with every ROS import stubbed."""
    with pytest.MonkeyPatch.context() as mp:
        for name, attrs in _STUBS.items():
            mod = types.ModuleType(name)
            for k, v in attrs.items():
                setattr(mod, k, v)
            mp.setitem(sys.modules, name, mod)
        stem = pathlib.Path(filename).stem
        spec = importlib.util.spec_from_file_location(
            f'{stem}_under_test_{request.module.__name__}', HERE / filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module


@pytest.fixture(scope='module')
def ag(request):
    """cell_panel: ALIGN tab of the merged control panel."""
    yield from _load('cell_panel.py', request)


@pytest.fixture(scope='module')
def ip(request):
    """cell_panel: IMPEDANCE & TRACK tab of the merged control panel."""
    yield from _load('cell_panel.py', request)
