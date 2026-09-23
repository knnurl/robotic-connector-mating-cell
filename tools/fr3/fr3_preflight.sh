#!/usr/bin/env bash
# Preflight checks before running the connector-mating stack on a real FR3.
#
# Catches the failure modes that stop the arm mid-motion, above all the
# bandwidth/RT traps behind `communication_constraints_violation` (the FCI
# loop misses its 1 ms deadline when DDS/image traffic contends with it).
#
# Usage:  fr3_preflight [robot_ip]   (fr3_env.sh; default $FR3_ROBOT_IP)
#         tools/fr3/fr3_preflight.sh [robot_ip]
# Exit code: number of FAILs (0 = ready; WARNs don't fail the check), so
# `fr3_preflight && ros2 launch ...` starts the driver only when ready.
# Read-only: it changes nothing on this PC or the robot.

ROBOT_IP="${1:-$FR3_ROBOT_IP}"
if [ -z "$ROBOT_IP" ]; then
    echo "usage: $0 [robot_ip] - no robot_ip given and FR3_ROBOT_IP unset (source tools/fr3/fr3_env.sh)"
    exit 1
fi
_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS="$(cd "${_DIR}/../.." && pwd)"
PASS=0; WARN=0; FAIL=0

ok()   { echo -e "  [ OK ] $1"; PASS=$((PASS+1)); }
warn() { echo -e "  [WARN] $1"; WARN=$((WARN+1)); }
fail() { echo -e "  [FAIL] $1"; FAIL=$((FAIL+1)); }

echo "== FR3 preflight (robot: $ROBOT_IP) =="

# --- 1. Real-time kernel ---------------------------------------------------
echo "-- Real-time scheduling"
if uname -v | grep -q PREEMPT_RT; then
    ok "PREEMPT_RT kernel ($(uname -r))"
else
    fail "kernel is not PREEMPT_RT - FCI will be unstable ($(uname -r))"
fi

RTPRIO=$(ulimit -r)
if [ "$RTPRIO" = "unlimited" ] || [ "${RTPRIO:-0}" -ge 90 ] 2>/dev/null; then
    ok "rtprio limit: $RTPRIO"
else
    fail "rtprio limit is '$RTPRIO' (<90). Add to /etc/security/limits.conf: '$USER - rtprio 99' and re-login"
fi

GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)
if [ "$GOV" = "performance" ]; then
    ok "CPU governor: performance"
else
    warn "CPU governor is '${GOV:-unknown}' - prefer: sudo cpupower frequency-set -g performance"
fi

# The governor still reads "performance" on battery, but the CPU cannot hold
# its clocks: two stack crashes on 2026-09-22 (GUIDE.md section 1).
AC_ONLINE=$(cat /sys/class/power_supply/AC*/online /sys/class/power_supply/ADP*/online 2>/dev/null)
if [ -z "$AC_ONLINE" ]; then
    ok "no AC adapter reported (not a laptop?)"
elif echo "$AC_ONLINE" | grep -q 1; then
    ok "on mains power"
else
    fail "on BATTERY - plug in the charger; the 1 ms FCI deadline slips on battery"
fi

RT_RUNTIME=$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null)
if [ "$RT_RUNTIME" = "-1" ]; then
    ok "RT throttling off (sched_rt_runtime_us = -1)"
else
    fail "RT throttling on (sched_rt_runtime_us = ${RT_RUNTIME:-unknown}) - sudo sysctl -w kernel.sched_rt_runtime_us=-1 (persistent: GUIDE.md section 1)"
fi

# --- 2. Robot link: dedicated, low-latency ---------------------------------
echo "-- Robot network link"
ROBOT_IF=$(ip route get "$ROBOT_IP" 2>/dev/null | grep -oP 'dev \K\S+')
if [ -z "$ROBOT_IF" ]; then
    fail "no route to $ROBOT_IP - is the robot NIC up and configured (e.g. 172.16.0.1/24)?"
else
    ok "robot reachable via '$ROBOT_IF'"
    case "$ROBOT_IF" in
        wl*|ww*) fail "robot route goes over WIRELESS ($ROBOT_IF) - FCI needs a dedicated wired NIC" ;;
    esac
    # Latency: FCI wants ~<1 ms round trip, consistently.
    if PING_OUT=$(ping -c 20 -i 0.05 -q "$ROBOT_IP" 2>/dev/null); then
        AVG=$(echo "$PING_OUT" | grep -oP 'rtt [^=]*= [\d.]+/\K[\d.]+')
        MAX=$(echo "$PING_OUT" | grep -oP 'rtt [^=]*= [\d.]+/[\d.]+/\K[\d.]+')
        if awk "BEGIN{exit !($MAX < 1.0)}"; then
            ok "ping avg ${AVG} ms / max ${MAX} ms (20 pkts)"
        elif awk "BEGIN{exit !($MAX < 2.0)}"; then
            warn "ping max ${MAX} ms - marginal for the 1 kHz loop; check cabling/switch"
        else
            fail "ping max ${MAX} ms - FCI will throw communication_constraints_violation"
        fi
    else
        fail "cannot ping $ROBOT_IP"
    fi
fi

# An open Desk tab keeps HTTPS connections to the robot on the FCI link.
DESK=$(ss -Htn state established dst "$ROBOT_IP" dport = :443 2>/dev/null | wc -l)
if [ "$DESK" -gt 0 ]; then
    warn "$DESK HTTPS connection(s) to $ROBOT_IP:443 - close the Desk browser tab once FCI is active"
else
    ok "no Desk (HTTPS) connections to the robot"
fi

# --- 3. DDS isolation from the FCI link ------------------------------------
echo "-- DDS configuration"
if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then
    ok "RMW: rmw_cyclonedds_cpp"
else
    warn "RMW is '${RMW_IMPLEMENTATION:-default (fastrtps)}' - this cell is tuned for CycloneDDS"
fi

if [ -z "$CYCLONEDDS_URI" ]; then
    fail "CYCLONEDDS_URI not set - export it to tools/fr3/cyclonedds_fr3.xml (interface isolation!)"
else
    CFG="${CYCLONEDDS_URI#file://}"
    if [ ! -f "$CFG" ]; then
        fail "CYCLONEDDS_URI points to missing file: $CFG"
    else
        ok "CYCLONEDDS_URI -> $CFG"
        if [ -n "$ROBOT_IF" ] && grep -q "NetworkInterface[^<]*name=\"$ROBOT_IF\"" "$CFG"; then
            fail "DDS config binds the ROBOT NIC '$ROBOT_IF' - remove it, image traffic there kills FCI"
        elif grep -q "<Interfaces>" "$CFG"; then
            ok "DDS pinned to explicit interfaces (robot NIC not among them)"
        else
            warn "DDS config has no <Interfaces> block - DDS may use ALL NICs incl. the robot link"
        fi
    fi
fi

# --- 4. Camera bandwidth traps ----------------------------------------------
echo "-- Camera / topic bandwidth (needs a sourced ROS 2 env; skipped if ros2 absent)"
if command -v ros2 >/dev/null 2>&1; then
    TOPICS=$(timeout 10 ros2 topic list 2>/dev/null)
    if [ -z "$TOPICS" ]; then
        warn "no ROS graph visible (nothing running yet?) - re-run preflight with the camera up"
    else
        if echo "$TOPICS" | grep -qE "points|pointcloud"; then
            fail "a POINTCLOUD topic is live: $(echo "$TOPICS" | grep -E 'points|pointcloud' | head -1) - disable it (realsense_low_bw.yaml)"
        else
            ok "no pointcloud topics live"
        fi
        if echo "$TOPICS" | grep -q "/aruco/debug_image"; then
            warn "debug image topic exists - fine locally; do NOT view it over WiFi at full rate while mating"
        fi
    fi
else
    warn "ros2 CLI not on PATH - source the workspace and re-run for topic checks"
fi

# --- 5. This workspace and the camera --------------------------------------
echo "-- This workspace ($WS)"
PREFIX=$(ros2 pkg prefix fr3_mating_controllers 2>/dev/null)
case "$PREFIX" in
    "$WS/install/"*) ok "fr3_mating_controllers from this workspace" ;;
    *) fail "fr3_mating_controllers resolves to '${PREFIX:-nothing}', not $WS/install - build it and source tools/fr3/fr3_env.sh, or T1 cannot load the impedance controller" ;;
esac

# A copied (non-symlink) install silently runs whatever roscam was built
# last, not the source tree: it once ran pre-fix vision.
RC_SITE=""
for d in "$WS"/install/roscam/lib/python3*/site-packages; do
    [ -d "$d" ] && RC_SITE="$d"
done
if [ -z "$RC_SITE" ]; then
    warn "install/roscam missing - colcon build --symlink-install"
elif [ -d "$RC_SITE/roscam" ] || [ ! -e "$RC_SITE/roscam.egg-link" ]; then
    warn "install/roscam is a COPY, not a symlink install - rm -rf build/roscam install/roscam, then colcon build --symlink-install"
else
    ok "install/roscam is a symlink install (runs the source tree)"
fi

if python3 -c 'import pyrealsense2' 2>/dev/null; then
    ok "pyrealsense2 importable (vision_source:=realsense)"
else
    warn "python3 cannot import pyrealsense2 - vision_source:=realsense (the default) will fail"
fi

if [ -f /etc/udev/rules.d/99-realsense-no-suspend.rules ]; then
    ok "RealSense no-autosuspend udev rule installed"
else
    warn "udev rule not installed - the D405 can autosuspend and never come back; see tools/fr3/setup/99-realsense-no-suspend.rules"
fi

# --- Summary -----------------------------------------------------------------
echo "== ${PASS} ok / ${WARN} warn / ${FAIL} fail =="
[ "$FAIL" -eq 0 ] && echo "READY (address warnings before long runs)" || echo "NOT READY - fix FAILs first"
exit "$FAIL"
