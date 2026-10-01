#!/usr/bin/env bash
# Starts the complete real-hardware ROS2 stack inside the ros2_humble
# container on the i.MX8M Plus: BNO055 IMU, the ros2_control+EKF control
# stack (talking to the real i.MX RT1176 MCU over UART3), the real
# twist_mux (replacing the host laptop's cmd_vel_mux.py - see README),
# a small relay that re-stamps twist_mux's plain Twist output into the
# TwistStamped diffbot_base_controller actually expects (the two message
# types never connect on their own - confirmed on hardware, 2026-10-01),
# and the web HMI.
#
# This is a bring-up/manual-start script, NOT a persistent auto-start-on-
# boot mechanism (see PENDING_ISSUES.md's "bno055 + IMU monitor don't
# survive a board/container reboot" entry - that's a deliberately separate,
# not-yet-done piece of work). Run this by hand each time after a board/
# container reboot:
#
#   ssh <board>
#   docker exec -it ros2_humble bash /ros2_ws/src/ros-amr/docker/run_full_stack.sh
#
# (or `docker exec -d ...` instead of `-it` to return control immediately -
# all the actual ROS2 processes are launched detached either way).
#
# Each process's log goes to /tmp/<name>.log inside the container, same
# convention used throughout this project's own bring-up sessions - check
# those first if something doesn't come up.
set -eo pipefail

SERIAL_PORT="${1:-/dev/ttymxc2}"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "[1/5] Starting bno055 ..."
ros2 launch bno055 bno055.launch.py > /tmp/bno055_stack.log 2>&1 &
sleep 5

echo "[2/5] Starting control stack + EKF (serial_port=$SERIAL_PORT) ..."
ros2 launch calixto-ros-bot diffbot.launch.py serial_port:="$SERIAL_PORT" gui:=false > /tmp/diffbot_stack.log 2>&1 &
sleep 8

echo "[3/5] Starting twist_mux ..."
ros2 run twist_mux twist_mux --ros-args \
    --params-file /ros2_ws/src/ros-amr/config/twist_mux.yaml \
    -r cmd_vel_out:=/cmd_vel_unstamped > /tmp/twist_mux_stack.log 2>&1 &
sleep 2

echo "[4/5] Starting twist_stamper relay ..."
python3 /ros2_ws/src/ros-amr/scripts/twist_stamper.py > /tmp/twist_stamper_stack.log 2>&1 &
sleep 2

echo "[5/6] Starting web HMI (port 8080, odom remapped to /odometry/filtered) ..."
python3 /ros2_ws/src/ros-amr/web_hmi/status_web_server.py --port 8080 --ros-args \
    -r /diffbot_base_controller/odom:=/odometry/filtered > /tmp/status_hmi_stack.log 2>&1 &
sleep 3

echo "[6/6] Starting RF remote control (requires the ch341 kernel module - see README) ..."
python3 /ros2_ws/src/ros-amr/web_hmi/remote_ros_node.py > /tmp/remote_node_stack.log 2>&1 &
sleep 2

echo "Done. HMI: http://<board-ip>:8080  -  logs in /tmp/*_stack.log inside the container."
