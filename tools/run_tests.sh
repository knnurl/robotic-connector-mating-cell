#!/usr/bin/env bash
# Every FR3 test in one go: colcon builds and tests the three FR3 packages,
# then pytest runs the Python tests from the repo root (pytest.ini).
#
# Usage:  tools/run_tests.sh     (from any directory)
# Runs in a clean environment - ROS 2 Humble and ~/franka_ros2_ws only - so
# nothing this shell has sourced (another workspace, another checkout's
# install) can leak in. Offline: it builds and tests, and never launches
# anything or touches the robot, the camera or the DDS network.
# Exit code: 0 only if the build, every colcon test and pytest all pass.

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

PKGS=(fr3_mating_controllers mating_controller roscam)
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
python3 -m pytest -q 2>&1 | tee "$OUT"
[ "${PIPESTATUS[0]}" -eq 0 ] || FAILED+=("pytest")
PYTEST="$(tail -n 1 "$OUT")"

echo "== totals =="
echo "  colcon build   ${BUILD}"
echo "  colcon test    ${COLCON:-no results}"
echo "  pytest         ${PYTEST}"
if [ "${#FAILED[@]}" -ne 0 ]; then
    (IFS=';'; echo "FAILED: ${FAILED[*]}")
    exit 1
fi
echo "all passed"
