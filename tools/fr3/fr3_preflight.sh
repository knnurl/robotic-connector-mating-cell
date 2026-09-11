#!/usr/bin/env bash
# Preflight checks before running the connector-mating stack on a real FR3.
#
# Catches the failure modes that stop the arm mid-motion, above all the
# bandwidth/RT traps behind `communication_constraints_violation` (the FCI
# loop misses its 1 ms deadline when DDS/image traffic contends with it).
#
# Usage:  tools/fr3/fr3_preflight.sh [robot_ip]      (default 172.16.0.3)
# Exit code: number of FAILs (0 = ready; WARNs don't fail the check).

ROBOT_IP="${1:-172.16.0.3}"
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

# --- Summary -----------------------------------------------------------------
echo "== ${PASS} ok / ${WARN} warn / ${FAIL} fail =="
[ "$FAIL" -eq 0 ] && echo "READY (address warnings before long runs)" || echo "NOT READY - fix FAILs first"
exit "$FAIL"
