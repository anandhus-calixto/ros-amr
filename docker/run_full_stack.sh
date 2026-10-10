#!/usr/bin/env bash
# Starts the complete real-hardware ROS2 stack inside the ros2_humble
# container on the i.MX8M Plus: BNO055 IMU, the ros2_control+EKF control
# stack (talking to the real i.MX RT1176 MCU over UART3), the real
# twist_mux (replacing the host laptop's cmd_vel_mux.py - see README),
# a small relay that re-stamps twist_mux's plain Twist output into the
# TwistStamped diffbot_base_controller actually expects (the two message
# types never connect on their own - confirmed on hardware, 2026-10-01),
# the web HMI, the RF remote reader, and the LDS-50C-E-R 2D LiDAR driver.
#
# Normally run through the host's amr-stack.service (start/stop with
# systemctl), see docker/amr-stack.service. Run by hand as:
#   docker exec -d ros2_humble bash /ros2_ws/src/ros-amr/docker/run_full_stack.sh
# and stop with docker/stop_full_stack.sh.
#
# Every process is started in its own session (setsid), so its whole process
# group can be stopped together later. Their process-group IDs are recorded in
# $PIDFILE; stop_full_stack.sh uses that file. Refuses to start if a stack from
# this file is already running, so processes can't end up duplicated.
#
# The container uses --network host, so this script also (re-)assigns the
# LiDAR's subnet IP and the office-network IP + default route to the board's
# wired `end1` interface before starting any node. These are NOT persistent (plain `ip addr add`, not nmcli -
# `end1` is NetworkManager-unmanaged) and is lost on every board reboot.
#
# Each process's log goes to /tmp/<name>_stack.log inside the container.
#
# 2026-10-07: briefly switched DDS discovery to a Fast-DDS Discovery Server
# (board's WiFi AP blocks the multicast default discovery needs - confirmed
# with `ros2 multicast send`/`receive`), to let a laptop on WiFi see topics
# for rviz2. REVERTED same day: SERVICE discovery between the robot's own
# processes was unreliable under it even with every node set to
# ROS_SUPER_CLIENT - joint_state_broadcaster's spawner would retry
# "Could not contact service /controller_manager/list_controllers" for
# minutes without ever completing, so /joint_states (and from there TF,
# /mcu_status) would sometimes just never come up after a restart. Back to
# the default (SIMPLE/multicast) here, which was reliable all session before
# this detour. If a laptop needs to see these topics again, use a wired
# connection instead of changing this stack's discovery mode - see
# docker/start_discovery_server.sh's own header for the history.
set -eo pipefail

PIDFILE=/tmp/amr_stack.pids
SERIAL_PORT="${1:-/dev/ttymxc2}"
LIDAR_IFACE="${2:-end1}"
LIDAR_HOST_IP="${3:-192.168.31.7/24}"
# Office-network address on the same wire. The onboard USR-G806w router bridges
# its LAN ports to the office WiFi (WLAN STA page: network = lan), so end1 sits
# directly on the office subnet - this replaces the removed USB WiFi dongle and
# keeps its old address, so `ssh imx8mp`, the HMI URL and rviz_remote's peer
# list are unchanged.
OFFICE_HOST_IP="${4:-192.168.1.62/24}"
OFFICE_GATEWAY="${5:-192.168.1.1}"

if [ -s "$PIDFILE" ]; then
    while read -r _ pgid; do
        if kill -0 -- "-$pgid" 2>/dev/null; then
            echo "A stack is already running (process group $pgid). Stop it first: stop_full_stack.sh" >&2
            exit 1
        fi
    done < "$PIDFILE"
fi
: > "$PIDFILE"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

launch() {
    local name=$1; shift
    setsid "$@" > "/tmp/${name}_stack.log" 2>&1 &
    echo "$name $!" >> "$PIDFILE"
}

# Addresses before any node starts: a DDS participant only announces the
# addresses present when it's created, so a node started before the office
# address exists stays invisible to a laptop on the office network.
sudo ip addr add "$LIDAR_HOST_IP" dev "$LIDAR_IFACE" 2>/dev/null || true
sudo ip addr add "$OFFICE_HOST_IP" dev "$LIDAR_IFACE" 2>/dev/null || true
sudo ip route replace default via "$OFFICE_GATEWAY" dev "$LIDAR_IFACE" 2>/dev/null || true

echo "[1/7] Starting bno055 ..."
launch bno055 ros2 launch bno055 bno055.launch.py
sleep 5

echo "[2/7] Starting control stack + EKF (serial_port=$SERIAL_PORT) ..."
launch diffbot ros2 launch calixto-ros-bot diffbot.launch.py serial_port:="$SERIAL_PORT" gui:=false
sleep 8

echo "[3/7] Starting twist_mux ..."
launch twist_mux ros2 run twist_mux twist_mux --ros-args \
    --params-file /ros2_ws/src/ros-amr/config/twist_mux.yaml \
    -r cmd_vel_out:=/cmd_vel_unstamped
sleep 2

echo "[4/7] Starting twist_stamper relay ..."
launch twist_stamper python3 /ros2_ws/src/ros-amr/scripts/twist_stamper.py
sleep 2

echo "[5/7] Starting web HMI (port 8080, odom remapped to /odometry/filtered) ..."
launch status_hmi python3 /ros2_ws/src/ros-amr/web_hmi/status_web_server.py --port 8080 --ros-args \
    -r /diffbot_base_controller/odom:=/odometry/filtered
sleep 3

echo "[6/7] Starting RF remote control (requires the ch341 kernel module - see README) ..."
launch remote_node python3 /ros2_ws/src/ros-amr/web_hmi/remote_ros_node.py
sleep 2

echo "[7/7] Starting LiDAR driver (bluesea2) ..."
launch lidar ros2 launch bluesea2 udp_lidar.launch
sleep 3

echo "[8/8] Starting body filter (laser_filters box, removes LiDAR returns on the robot body) ..."
launch scan_filter ros2 run laser_filters scan_to_scan_filter_chain --ros-args \
    --params-file /ros2_ws/src/ros-amr/config/scan_box_filter.yaml -r scan:=/scan -r scan_filtered:=/scan_filtered
sleep 2

echo "Done. HMI: http://<board-ip>:8080  -  logs in /tmp/*_stack.log inside the container."
