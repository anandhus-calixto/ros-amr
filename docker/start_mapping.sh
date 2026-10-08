#!/usr/bin/env bash
# Starts online mapping (slam_toolbox, mapping mode) inside the ros2_humble container.
# Needs the ROS stack running (odom->base_link from the EKF, /scan from bluesea2).
# Writes the map to /map. Save it with stop_mapping.sh --save, which runs map_saver.
# The process group is recorded in $PIDFILE so stop_mapping.sh can stop it.
set -eo pipefail

PIDFILE=/tmp/mapping.pids
PARAMS=/ros2_ws/src/ros-amr/config/mapper_params_online_async.yaml

if [ -s "$PIDFILE" ]; then
    echo "Mapping already running - stop it first with stop_mapping.sh" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# The LiDAR's position comes from the robot model (lidar.xacro), so no extra transform here.

setsid ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file "$PARAMS" -p mode:=mapping -p scan_topic:=/scan_filtered \
    > /tmp/mapping_slam.log 2>&1 &
echo "slam $!" >> "$PIDFILE"

echo "Mapping started. Watch /tmp/mapping_slam.log in the container; map topic: /map"
