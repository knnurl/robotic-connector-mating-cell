# shellcheck shell=bash
# Source this in EVERY terminal of the FR3 cell, BEFORE launching anything
# (~/.bashrc may source it in every shell - that is fine):
#
#     source tools/fr3/fr3_env.sh
#
# Self-contained and safe to source twice: ROS 2 Humble if nothing has
# sourced it yet, ~/franka_ros2_ws, then this workspace on top of both - so
# nothing needs sourcing after it. It also defines the two bring-up commands
# (GUIDE.md section 2), which work from any directory:
#
#     fr3_preflight [robot_ip]     read-only checks; exit code = FAIL count
#     fr3_cell [arg:=value ...]    T2: tools/fr3/fr3_cell.launch.py (mock:=true = no robot)
#
# Why this file exists: the FCI loop exchanges UDP packets with the robot
# every 1 ms. If DDS is not pinned away from the robot NIC, ROS traffic
# shares that link and the kernel UDP buffers, the RT thread misses its
# deadline, and the arm drops the connection mid-motion:
#
#     libfranka: Connection reset by peer   (franka::NetworkException)
#
# Pointing CYCLONEDDS_URI at a config WITHOUT an <Interfaces> block (e.g.
# the fr3_act cyclonedds_franka.xml) has exactly that effect: it tunes
# buffers but isolates nothing. Verify with fr3_preflight.

# Resolve the repo path even though it contains spaces.
_FR3_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
_FR3_WS="$(cd "${_FR3_DIR}/../.." && pwd)"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_FR3_DIR}/cyclonedds_fr3.xml"

# This cell's robot. fr3_preflight defaults to it.
export FR3_ROBOT_IP=172.16.0.2

# Run logs (panel traces, tracking_node V1-V6 data) live OUTSIDE the repo,
# next to the demo videos: <project>/runs/YYYY-MM-DD/. Writers create the
# day folder. Set FR3_LOG_DIR before sourcing to put them elsewhere.
export FR3_LOG_DIR="${FR3_LOG_DIR:-$(readlink -f "${_FR3_DIR}/../../..")/runs}"

# ROS, then franka_ros2, then this workspace LAST so it overlays both. The
# first two are skipped when already done (~/.bashrc): re-sourcing franka's
# setup.bash changes no path, but costs ~0.4 s and repeats its chain's
# warnings in every new shell.
if [ -z "$ROS_DISTRO" ] && [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
case ":${AMENT_PREFIX_PATH}:" in
    *":${HOME}/franka_ros2_ws/install/"*) ;;
    *)
        if [ -f "${HOME}/franka_ros2_ws/install/setup.bash" ]; then
            # shellcheck disable=SC1091
            source "${HOME}/franka_ros2_ws/install/setup.bash"
        fi
        ;;
esac
if [ -f "${_FR3_WS}/install/local_setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${_FR3_WS}/install/local_setup.bash"
fi

# The controller manager (T1) must find fr3_mating_controllers in THIS
# workspace, or spawning the impedance controller fails with "Loader for
# controller ... not found". An index lookup only - no DDS.
_FR3_PREFIX="$(ros2 pkg prefix fr3_mating_controllers 2>/dev/null || true)"

echo "FR3 env: ${_FR3_WS}"
echo "  DDS ${CYCLONEDDS_URI#file://} | robot ${FR3_ROBOT_IP} | logs ${FR3_LOG_DIR}"
case "$_FR3_PREFIX" in
    "${_FR3_WS}/install/"*)
        echo "  commands: fr3_preflight, fr3_cell (GUIDE.md section 2)" ;;
    *)
        echo "  [FAIL] fr3_mating_controllers resolves to '${_FR3_PREFIX:-nothing}', not ${_FR3_WS}/install"
        echo "  [FAIL] the impedance controller cannot be spawned - colcon build --symlink-install in ${_FR3_WS}" ;;
esac

# The paths are baked in, so `type fr3_cell` shows which checkout it runs.
eval "fr3_cell() { ros2 launch $(printf '%q' "${_FR3_DIR}/fr3_cell.launch.py") \"\$@\"; }"
eval "fr3_preflight() { $(printf '%q' "${_FR3_DIR}/fr3_preflight.sh") \"\$@\"; }"

unset _FR3_DIR _FR3_WS _FR3_PREFIX
