#!/usr/bin/env bash
# Brings up the full Calixto AMR web HMI stack in one command: sources
# ROS2 + this workspace, then runs launch/hmi_bringup.launch.py (control
# stack + cmd_vel_mux + the RF-remote bridge + the web HMI server).
#
# Usage:
#   ./run_hmi.sh                        # normal bring-up
#   ./run_hmi.sh serial_port:=/dev/... web_port:=8081   # override any
#                                        # hmi_bringup.launch.py argument
# No `set -u`: ROS2's own setup.bash scripts reference unset variables
# internally (e.g. AMENT_TRACE_SETUP_FILES) and aren't nounset-safe.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
    echo "error: $SCRIPT_DIR/install/setup.bash not found - run 'colcon build' in $SCRIPT_DIR first." >&2
    exit 1
fi
source "$SCRIPT_DIR/install/setup.bash"

exec ros2 launch calixto-ros-bot hmi_bringup.launch.py "$@"
