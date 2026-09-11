# Source this in EVERY terminal of the FR3 cell, BEFORE launching anything:
#
#     source tools/fr3/fr3_env.sh
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
# buffers but isolates nothing. Verify with tools/fr3/fr3_preflight.sh.

# Resolve the repo path even though it contains spaces.
_FR3_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_FR3_DIR}/cyclonedds_fr3.xml"

# This cell's robot. fr3_preflight.sh takes it as a positional arg, and its
# built-in default (172.16.0.3) is NOT this cell.
export FR3_ROBOT_IP=172.16.0.2

echo "FR3 env set:"
echo "  RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION"
echo "  CYCLONEDDS_URI     = $CYCLONEDDS_URI"
echo "  FR3_ROBOT_IP       = $FR3_ROBOT_IP"
echo "check with: tools/fr3/fr3_preflight.sh \$FR3_ROBOT_IP"

unset _FR3_DIR
