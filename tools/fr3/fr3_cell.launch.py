"""FR3 connector-mating cell, terminal 2: everything except the driver.

    fr3_cell [arg:=value ...]      # fr3_env.sh; = ros2 launch <this file>

T1 runs the FCI driver + MoveIt; this spawns the impedance controller
INACTIVE into T1's controller manager, then starts the hand-eye static TF
(tools/fr3/calib/handeye.yaml), cam_pub (in-process D405 capture, filtered in
fr3_link0), tracking_node (idle until TRACK) and the cell panel
(tools/fr3/cell/cell.py), all with tools/fr3/fr3_params.yaml. Nothing here
moves the arm by itself: every motion is a panel button. The bring-up order
is GUIDE.md section 2, the only maintained copy.

mock:=true starts none of that: tools/fr3/cell/mock_cell.py fakes the whole
cell and the panel runs against it, both re-execing onto the isolated DDS
domain 88 (tools/fr3/cell/isolate.py), so no robot is needed or reachable.

vision_source:=topic instead expects a separate realsense2_camera driver
(tools/fr3/realsense_low_bw.yaml) and gives cam_pub colour only - no depth,
so the depth tilt and mirror disambiguation fall back to IPPE.

Tracking traces go to $FR3_LOG_DIR/YYYY-MM-DD/ when fr3_env.sh has set it.
"""

import datetime
import os

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
HANDEYE_FILE = os.path.join(THIS_DIR, 'calib', 'handeye.yaml')
# The SOURCE copy, like fr3_params.yaml: no rebuild between an edit and the
# next spawn.
IMPEDANCE_PARAMS = os.path.normpath(os.path.join(
    THIS_DIR, '..', '..', 'fr3_mating_controllers', 'config',
    'cartesian_impedance_stroke.yaml'))


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    mock = LaunchConfiguration('mock')

    # Read on every launch: the file is the one copy of the calibration and
    # its record (method, residual, validation) - see its header.
    with open(HANDEYE_FILE) as f:
        handeye = yaml.safe_load(f)

    args = [
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(THIS_DIR, 'fr3_params.yaml')),
        DeclareLaunchArgument('filter_frame', default_value='fr3_link0',
                              description="KF frame for cam_pub ('' = optical/legacy)"),
        # The default is the marker now on the cell, DICT_6X6_250 id 0.
        # Override it when the connector carries a different one - cam_pub
        # decodes every marker it sees and then DISCARDS any whose id does not
        # match, so a mismatch looks exactly like "no marker": debug_image
        # flows, /aruco/pose is silent.
        DeclareLaunchArgument('marker_id', default_value='0',
                              description='ArUco id to track (DICT_6X6_250)'),
        DeclareLaunchArgument('target_marker_id', default_value='1',
                              description='ArUco id of the static place target for PLACE '
                                          'AT B (-1 = off)'),
        DeclareLaunchArgument('vision_source', default_value='realsense',
                              description='realsense = in-process capture with depth, no '
                                          'image topics (default); standalone = the same, '
                                          'run by vision_standalone (frame recording, capture '
                                          'settings - PERCEPTION_PLAN Phase 0); topic = a '
                                          'separate camera driver publishes colour only'),
        DeclareLaunchArgument('capture_width', default_value='640'),
        DeclareLaunchArgument('capture_height', default_value='480'),
        # High Accuracy + spatial filter: half the top-face depth noise of the
        # device default up to 200 mm, same fill (runs/2026-09-25/analysis).
        DeclareLaunchArgument('capture_preset', default_value='High Accuracy',
                              description="D405 visual preset name ('' = device default)"),
        DeclareLaunchArgument('capture_spatial_filter', default_value='true',
                              description='SDK spatial filter on the depth'),
        DeclareLaunchArgument('capture_exposure_us', default_value='-1',
                              description='locked exposure in us (-1 = auto)'),
        DeclareLaunchArgument('range_source', default_value='depth',
                              description='marker distance: depth = the ArUco ray onto the '
                                          'depth plane around the marker; aruco = ArUco alone '
                                          '(the rollback)'),
        DeclareLaunchArgument('capture_fps', default_value='15',
                              description='D405 capture rate in realsense mode; '
                                          '/aruco/pose is published at this rate'),
        DeclareLaunchArgument('start_panel', default_value='true',
                              description='run the cell panel (tools/fr3/cell/cell.py) '
                                          'as part of this launch'),
        DeclareLaunchArgument('mock', default_value='false',
                              description='true = no robot: the mock cell and the panel '
                                          'on the isolated DDS domain 88'),
        # Hand-eye TCP->optical. Defaults come from calib/handeye.yaml; pass
        # these only to try a candidate calibration without editing it.
        DeclareLaunchArgument('handeye_xyz',
                              default_value=' '.join(str(v) for v in handeye['xyz']),
                              description='TCP->optical x y z [m]; default read '
                                          'from calib/handeye.yaml'),
        DeclareLaunchArgument('handeye_quat',
                              default_value=' '.join(str(v) for v in handeye['quat_xyzw']),
                              description='TCP->optical qx qy qz qw; default read '
                                          'from calib/handeye.yaml'),
    ]

    real = UnlessCondition(mock)
    return LaunchDescription(args + [_impedance_spawner(real),
                                     _static_tf(handeye, real),
                                     OpaqueFunction(function=_vision),
                                     TimerAction(period=3.0, condition=real, actions=[
                                         _tracking(params_file),
                                         _grip(params_file),
                                         _panel([])]),
                                     _mock_cell(IfCondition(mock)),
                                     TimerAction(period=2.0, condition=IfCondition(mock),
                                                 actions=[_panel(['--mock'])])])


def _impedance_spawner(condition):
    # Loads and configures the controller INACTIVE, then exits; while T1 is
    # not up yet it waits, retrying every 10 s. The operator may relaunch T2
    # while T1 keeps running, so what the Humble spawner (controller_manager
    # 2.53) does with a controller that is ALREADY loaded matters:
    #   * it skips the load, so the param file is NOT re-applied - an edit to
    #     it needs T1, then T2, restarted (or a live set from the panel);
    #   * it still asks for configure. The controller manager cleans up and
    #     re-configures an INACTIVE controller from the parameters it already
    #     holds (it stays inactive), and REFUSES an ACTIVE one ("can not be
    #     configured from 'active' state"), leaving it running untouched; the
    #     spawner then logs "Failed to configure controller" and exits 1.
    #   * --inactive means it never calls switch_controller, and without
    #     --unload-on-kill Ctrl-C never deactivates or unloads anything.
    # Its exit, 0 or 1, ends only the spawner: no action here has
    # on_exit=Shutdown, so it cannot take the launch down.
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_impedance_stroke_controller', '--inactive',
                   '--param-file', IMPEDANCE_PARAMS],
        output='screen',
        condition=condition,
    )


def _static_tf(handeye, condition):
    # static_transform_publisher needs individual tokens; split the two
    # space-separated launch args into positional arguments at launch time.
    def make(context):
        xyz = LaunchConfiguration('handeye_xyz').perform(context).split()
        quat = LaunchConfiguration('handeye_quat').perform(context).split()
        if ([float(v) for v in xyz] == handeye['xyz']
                and [float(v) for v in quat] == handeye['quat_xyzw']):
            source = (f"calib/handeye.yaml ({handeye['method']}, "
                      f"{handeye['calibrated']}, residual "
                      f"{handeye['residual_mm']} mm / {handeye['residual_deg']} deg)")
            aruco_range = (LaunchConfiguration('range_source').perform(context) == 'aruco'
                           # topic mode gets colour only: cam_pub keeps the ArUco distance
                           or LaunchConfiguration('vision_source').perform(context) == 'topic')
            if aruco_range and 'xyz_aruco_range' in handeye:
                # xyz is solved for depth-ranged marker poses; the raw ArUco
                # distance needs the solve from the same raw samples.
                xyz = [str(v) for v in handeye['xyz_aruco_range']]
                source = (f"calib/handeye.yaml xyz_aruco_range for the ArUco distance "
                          f"({handeye['method']}, {handeye['calibrated']})")
        else:
            source = 'the handeye_xyz/handeye_quat OVERRIDE, not calib/handeye.yaml'
        return [LogInfo(msg=f'fr3_cell: hand-eye TF from {source}'),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='handeye_tcp_to_optical',
                    arguments=['--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                               '--qx', quat[0], '--qy', quat[1], '--qz', quat[2],
                               '--qw', quat[3],
                               '--frame-id', handeye['parent_frame'],
                               '--child-frame-id', handeye['child_frame']],
                )]

    return OpaqueFunction(function=make, condition=condition)


def _vision(context):
    """cam_pub (realsense / topic) or vision_standalone (standalone): the
    same ArucoPosePublisher and the same parameters either way, so the
    marker pipeline is identical and only the camera owner changes."""
    arg = lambda name: LaunchConfiguration(name).perform(context)     # noqa: E731
    if arg('mock') == 'true':
        return []
    source = arg('vision_source')
    params = {
        # topic mode: RealSense D405 driver defaults (remap via -p).
        # realsense / standalone capture in-process - zero image topics
        # on the graph (see README defence 0).
        'image_topic': '/camera/camera/color/image_rect_raw',
        'camera_info_topic': '/camera/camera/color/camera_info',
        'filter_frame': arg('filter_frame'),
        'marker_id': int(arg('marker_id')),
        'target_marker_id': int(arg('target_marker_id')),
        'capture_fps': int(arg('capture_fps')),
        'capture_width': int(arg('capture_width')),
        'capture_height': int(arg('capture_height')),
        'capture_preset': arg('capture_preset'),
        'capture_spatial_filter': arg('capture_spatial_filter') == 'true',
        'capture_exposure_us': int(arg('capture_exposure_us')),
        # Measured 2026-09-25 (latency_fit, 640x480 @ 15 fps, High Accuracy +
        # spatial filter): 24 ms, 95% CI 23-25.
        'capture_latency_s': 0.024,
        'range_source': arg('range_source'),
        # Debug image is only encoded/published while something
        # subscribes - keep GUI subscriptions off the robot NIC.
        'publish_debug_image': True,
    }
    if source == 'standalone':
        executable = 'vision_standalone'
    else:
        params['source'] = source
        executable = 'cam_pub'
    # One BLAS thread: the per-frame plane fits are tiny SVDs, and OpenBLAS
    # spreads each over every core, next to the 1 kHz control loop
    # (measured on recorded frames: cam_pub kept 4.7 cores busy, 2.5 with
    # one BLAS thread, at the same speed).
    one_thread = {v: '1' for v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS',
                                   'MKL_NUM_THREADS')}
    return [Node(package='roscam', executable=executable, output='screen',
                 parameters=[params], additional_env=one_thread)]


def _tracking(params_file):
    # fr3_params.yaml reaches a node only through params_file, so without
    # this the tracking_* values never arrive. Idle until START TRACKING.
    # FR3_LOG_DIR (fr3_env.sh) overrides tracking_log_dir so every TRACK run
    # is recorded; the node does not create directories, so make the day
    # folder here.
    overrides = {}
    log_root = os.environ.get('FR3_LOG_DIR')
    if log_root:
        day_dir = os.path.join(log_root, datetime.date.today().isoformat())
        os.makedirs(day_dir, exist_ok=True)
        overrides['tracking_log_dir'] = day_dir
    return Node(
        package='mating_controller',
        executable='tracking_node',
        output='screen',
        parameters=[params_file, overrides],
    )


def _grip(params_file):
    # Python, run from the source tree like the panel: no rebuild after an
    # edit. It holds the equilibrium only while GRIP or PLACE runs.
    return ExecuteProcess(
        cmd=['python3', '-u', os.path.join(THIS_DIR, 'cell', 'grip_node.py'),
             '--ros-args', '--params-file', params_file],
        name='grip_node', output='screen')


def _mock_cell(condition):
    return ExecuteProcess(cmd=['python3', '-u', os.path.join(THIS_DIR, 'cell', 'mock_cell.py')],
                          name='mock_cell', output='screen', condition=condition)


def _panel(extra):
    # A child of this launch, so one Ctrl-C ends T2. The terminal's SIGINT
    # reaches the panel directly (launch does not re-send it) and the panel
    # hands the arm back before it exits; SIGTERM makes it try again.
    # Launch's default escalation - SIGTERM at 5 s, SIGKILL 5 s later - is
    # shorter than that bounded handoff (a tracking stop, 20 s timeout, then
    # a two-step controller switch, 10 s per step: ~45 s worst case, see
    # release_if_active), and a SIGKILL mid-handoff leaves the impedance
    # controller active. Hence 30 s + 30 s here. -u: stdout is a pipe here,
    # and the exit handoff's "released" / "could NOT release" lines must
    # reach T2 as they happen, not in a buffer a kill would lose.
    return ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_panel')),
        name='cell_panel',
        cmd=['python3', '-u', os.path.join(THIS_DIR, 'cell', 'cell.py')] + extra,
        output='screen',
        sigterm_timeout='30',
        sigkill_timeout='30',
    )
