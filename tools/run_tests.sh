#!/usr/bin/env bash
# Every FR3 test in one go: colcon builds and tests the three FR3 packages,
# pytest runs the Python tests from the repo root (pytest.ini), then the
# tracking smoke test runs the real tracking_node in a fake cell.
#
# Usage:  tools/run_tests.sh     (from any directory)
# Runs in a clean environment - ROS 2 Humble and ~/franka_ros2_ws only - so
# nothing this shell has sourced (another workspace, another checkout's
# install) can leak in. It never touches the robot or the camera. The smoke
# test is the only step that starts ROS nodes, and it does so on its own DDS
# domain (87, loopback only), invisible to a live cell on domain 0.
# Exit code: 0 only if the build, every colcon test, pytest and the smoke
# test all pass.

SELF="$(readlink -f "${BASH_SOURCE[0]}")"
ROOT="$(cd "$(dirname "$SELF")/.." && pwd)"

if [ -z "${_FR3_RUN_TESTS_CLEAN:-}" ]; then
    exec env -i HOME="$HOME" PATH=/usr/bin:/bin TERM=dumb \
        _FR3_RUN_TESTS_CLEAN=1 bash --noprofile --norc "$SELF" "$@"
fi

source /opt/ros/humble/setup.bash
if [ -f "$HOME/franka_ros2_ws/install/setup.bash" ]; then
    source "$HOME/franka_ros2_ws/install/setup.bash"
else
    echo "WARNING: ~/franka_ros2_ws is not built - franka_msgs will be missing"
fi
cd "$ROOT" || exit 1

PKGS=(fr3_mating_controllers mating_controller roscam object_pose_cpp)
FAILED=()
OUT="$(mktemp)"
trap 'rm -f "$OUT"' EXIT

echo "== colcon build: ${PKGS[*]} =="
if colcon build --symlink-install --packages-select "${PKGS[@]}"; then
    BUILD=ok
else
    BUILD=FAILED
    FAILED+=("colcon build")
fi

echo "== colcon test =="
colcon test --packages-select "${PKGS[@]}" || FAILED+=("colcon test")
colcon test-result --all --verbose | tee "$OUT"
[ "${PIPESTATUS[0]}" -eq 0 ] || FAILED+=("colcon test-result")
COLCON="$(grep '^Summary:' "$OUT" | tail -n 1)"

echo "== pytest (${ROOT}) =="
# the workspace install for object_pose_cpp (the C++ estimator); pytest.ini
# puts roscam's source first, so the tests still run the source
source "$ROOT/install/local_setup.bash"
python3 -m pytest -q 2>&1 | tee "$OUT"
[ "${PIPESTATUS[0]}" -eq 0 ] || FAILED+=("pytest")
PYTEST="$(tail -n 1 "$OUT")"

echo "== tracking smoke test (isolated DDS domain 87) =="
source "$ROOT/install/local_setup.bash"
python3 tools/fr3/sim/tracking_smoke.py 2>&1 | tee "$OUT"
[ "${PIPESTATUS[0]}" -eq 0 ] || FAILED+=("tracking smoke")
SMOKE="$(grep 'checks passed' "$OUT" | tail -n 1)"

echo "== totals =="
echo "  colcon build   ${BUILD}"
echo "  colcon test    ${COLCON:-no results}"
echo "  pytest         ${PYTEST}"
echo "  smoke          ${SMOKE:-did not run}"
if [ "${#FAILED[@]}" -ne 0 ]; then
    (IFS=';'; echo "FAILED: ${FAILED[*]}")
    exit 1
fi
echo "all passed"
