#!/usr/bin/env bash
# Starts Nav2 (localization on a saved map + planning + control) inside the
# ros2_humble container. Needs the ROS stack running (odom->base_link from the
# EKF, /scan_filtered from the LiDAR + body filter).
#
# Usage: start_navigation.sh <map_name>   (name only, no .yaml - from maps/)
#
# Nav2's own output is remapped to cmd_vel_nav, not the default /cmd_vel -
# /cmd_vel is already the real final command topic on this robot (see
# config/twist_mux.yaml's comment). twist_mux's navigation entry reads
# cmd_vel_nav at the lowest priority, so teleop/remote/E-stop still override it.
#
# AMCL needs an initial pose (2D pose estimate) before it can localize - send
# one to /initialpose from the web UI once this is up.
#
# 2026-10-07: briefly used a Fast-DDS Discovery Server here, reverted the
# same day - see run_full_stack.sh's comment for why. Back to the default
# (SIMPLE/multicast), matching that script.
set -eo pipefail

PIDFILE=/tmp/navigation.pids
MAPS=/ros2_ws/src/ros-amr/maps
# 2026-10-08: optional 2nd argument picks the params file, so step1/step2
# tuning variants can be tried without overwriting nav2_params.yaml. Name only
# (no .yaml, no path) - it is resolved against config/.
# Default is nav2_params_step1 (2026-10-08), NOT the old baseline: the web
# UI's "Start navigation" button calls this script with the map name only, so
# whatever is defaulted here is what the button runs. step1 carries the day's
# tuning (0.40 m/s, rotate-in-place + rotate-to-goal, the forward_point_distance
# weaving fix, CPU-sized sampling). Pass nav2_params to get the old baseline.
# step3 (Hybrid planner) was tried and rejected the same day: forward-only
# curves need room for a U-turn, and 52 planner failures in one session.
PARAMS_NAME="${2:-nav2_params_step1}"
PARAMS="/ros2_ws/src/ros-amr/config/${PARAMS_NAME}.yaml"
MAP_NAME="${1:?usage: start_navigation.sh <map_name> [params_name]}"

if [ ! -f "$PARAMS" ]; then
    echo "No such params file: $PARAMS" >&2
    exit 1
fi
echo "params: $PARAMS"
MAP_YAML="$MAPS/$MAP_NAME.yaml"

if [ ! -f "$MAP_YAML" ]; then
    echo "No such map: $MAP_YAML" >&2
    exit 1
fi
if [ -s "$PIDFILE" ]; then
    echo "Navigation already running - stop it first with stop_navigation.sh" >&2
    exit 1
fi
: > "$PIDFILE"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

launch() {
    local name=$1; shift
    setsid "$@" > "/tmp/nav_${name}_stack.log" 2>&1 &
    echo "$name $!" >> "$PIDFILE"
}

echo "[1/8] map_server ($MAP_YAML) ..."
launch map_server ros2 run nav2_map_server map_server --ros-args --params-file "$PARAMS" -p yaml_filename:="$MAP_YAML"
sleep 2

echo "[2/8] amcl ..."
launch amcl ros2 run nav2_amcl amcl --ros-args --params-file "$PARAMS"
sleep 2

echo "[3/8] planner_server ..."
launch planner ros2 run nav2_planner planner_server --ros-args --params-file "$PARAMS"
sleep 1

echo "[4/8] controller_server (cmd_vel -> cmd_vel_nav) ..."
launch controller ros2 run nav2_controller controller_server --ros-args --params-file "$PARAMS" -r cmd_vel:=cmd_vel_nav
sleep 1

echo "[5/8] behavior_server (cmd_vel -> cmd_vel_nav) ..."
# 2026-10-07: was missing this remap - recovery behaviors (Spin, BackUp, ...)
# publish velocity directly, and without it they'd go straight to the real
# /cmd_vel, bypassing twist_mux's E-stop/remote/teleop priority arbitration
# entirely (same reasoning as controller_server above).
launch behavior ros2 run nav2_behaviors behavior_server --ros-args --params-file "$PARAMS" -r cmd_vel:=cmd_vel_nav
sleep 1

echo "[6/8] smoother_server ..."
launch smoother ros2 run nav2_smoother smoother_server --ros-args --params-file "$PARAMS"
sleep 1

echo "[7/8] bt_navigator ..."
launch bt_navigator ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file "$PARAMS"
sleep 2

echo "[8/8] lifecycle_manager (bringing the above up to ACTIVE) ..."
launch lifecycle ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -p use_sim_time:=false -p autostart:=true \
    -p 'node_names:=[map_server, amcl, controller_server, planner_server, behavior_server, smoother_server, bt_navigator]'
sleep 2

echo "Done. Send an initial pose (2D pose estimate) before setting a goal."
