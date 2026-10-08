#!/usr/bin/env bash
# Stops mapping started by start_mapping.sh. With --save, first writes the current
# map to /ros2_ws/src/ros-amr/maps/<name>.{yaml,pgm} using map_saver.
#   stop_mapping.sh --save <name>    save the map, then stop
#   stop_mapping.sh                  stop without saving

PIDFILE=/tmp/mapping.pids
MAPS=/ros2_ws/src/ros-amr/maps

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

if [ "${1:-}" = "--save" ]; then
    name=${2:?usage: stop_mapping.sh --save <name>}
    ros2 run nav2_map_server map_saver_cli -f "$MAPS/$name" --ros-args -p save_map_timeout:=10.0
fi

if [ -s "$PIDFILE" ]; then
    while read -r _ pgid; do
        kill -INT -- "-$pgid" 2>/dev/null || true
    done < "$PIDFILE"
    sleep 3
    while read -r _ pgid; do
        kill -KILL -- "-$pgid" 2>/dev/null || true
    done < "$PIDFILE"
    rm -f "$PIDFILE"
fi
echo "Mapping stopped."
