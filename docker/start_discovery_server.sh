#!/usr/bin/env bash
# Starts a Fast-DDS Discovery Server inside the ros2_humble container, bound
# to the board's real LAN IP on the default discovery port (11811).
#
# 2026-10-07: added because the board's WiFi AP silently drops UDP multicast
# between wireless and wired clients, which ROS2's default (SIMPLE) discovery
# depends on - confirmed with `ros2 multicast send`/`receive` across the two
# hosts. Discovery Server mode only needs ordinary unicast UDP, which the AP
# does pass. run_full_stack.sh and start_navigation.sh both export
# ROS_DISCOVERY_SERVER pointing at this server, so it must be started BEFORE
# either of them, and kept running - if it dies, nothing can (re)discover
# anything until it's restarted. A remote machine (e.g. a laptop running
# rviz2) needs to export the same ROS_DISCOVERY_SERVER value, using this
# board's real LAN IP, before running any ros2/rviz2 command.
#
# Usage: start_discovery_server.sh [listen-ip] [port]
# Stop with stop_discovery_server.sh.
set -eo pipefail

PIDFILE=/tmp/discovery_server.pid
LISTEN_IP="${1:-192.168.1.62}"
PORT="${2:-11811}"

if [ -s "$PIDFILE" ] && kill -0 -- "-$(cat "$PIDFILE")" 2>/dev/null; then
    echo "Discovery server already running (pgid $(cat "$PIDFILE")) - stop it first: stop_discovery_server.sh" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

setsid /opt/ros/humble/bin/fastdds discovery -i 0 -l "$LISTEN_IP" -p "$PORT" \
    > /tmp/discovery_server_stack.log 2>&1 &
echo "$!" > "$PIDFILE"
sleep 1
echo "Discovery server listening on $LISTEN_IP:$PORT (pgid $(cat "$PIDFILE"))."
