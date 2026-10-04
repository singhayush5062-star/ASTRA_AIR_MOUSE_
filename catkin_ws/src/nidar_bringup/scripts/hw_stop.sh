#!/bin/bash
# Stop the onboard stack started by hw_bringup.sh (real drone). Refuses while the drone is
# armed: stopping the stack in flight leaves PX4 without setpoints (it lands, COM_OBL_RC_ACT).
#   hw_stop.sh           stop if disarmed
#   hw_stop.sh --force   stop anyway
source /opt/ros/noetic/setup.bash
if [ "${1:-}" != "--force" ] && timeout 3 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q "armed: True"; then
    echo "[hw_stop] the drone is ARMED -- land first (or hw_stop.sh --force)"
    exit 1
fi
if pkill -INT -f "nidar_bringup/scripts/hw_bringup.sh"; then
    echo "[hw_stop] stopping hw_bringup.sh (it shuts the stack down and closes the rosbag)"
else
    echo "[hw_stop] hw_bringup.sh is not running"
fi
