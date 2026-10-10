#!/usr/bin/env bash
# Host-side supervisor for the ROS2 stack inside the ros2_humble container.
# Run by amr-stack.service (see amr-stack.service). It:
#   - waits for the container, then starts the stack
#   - checks every CHECK_INTERVAL seconds that the core processes are still
#     alive (check_full_stack.sh) and restarts the whole stack if not
#   - on SIGTERM (systemctl stop), stops the stack and exits 0, which systemd
#     treats as a deliberate stop, so it does not restart it
set -u

CONTAINER=ros2_humble
DX="docker exec $CONTAINER"
RUN=/ros2_ws/src/ros-amr/docker/run_full_stack.sh
STOP=/ros2_ws/src/ros-amr/docker/stop_full_stack.sh
CHECK=/ros2_ws/src/ros-amr/docker/check_full_stack.sh
CHECK_INTERVAL=5

# Same values as run_full_stack.sh's defaults (which re-applies them,
# harmlessly, when the stack is started by hand) - keep the two in sync.
IFACE=end1
LIDAR_HOST_IP=192.168.31.7/24
OFFICE_HOST_IP=192.168.1.62/24
OFFICE_GATEWAY=192.168.1.1
TIME_SYNC_TIMEOUT=60
CLOCK_JUMP_LIMIT=30   # seconds

# 2026-10-10: the board has no battery-backed clock, so it boots with the
# time it last saved and only corrects it once it reaches an NTP server.
# Humble's ros2_control_node schedules its loop by wall-clock time, so if
# the clock jumps forward AFTER the stack starts, it tries to "catch up" by
# running back-to-back (seen: ~195 Hz instead of 20 Hz after a 14 h jump),
# flooding /joint_states and /mcu_status and pinning the CPU. So: bring up
# the network, then wait for the clock to sync, THEN start the stack.
setup_network() {
    ip addr add "$LIDAR_HOST_IP" dev "$IFACE" 2>/dev/null
    ip addr add "$OFFICE_HOST_IP" dev "$IFACE" 2>/dev/null
    ip route replace default via "$OFFICE_GATEWAY" dev "$IFACE" 2>/dev/null
    return 0
}

wait_time_sync() {
    # timesyncd backs off between failed attempts; restart it so it tries
    # now that the route exists, instead of whenever its next retry is.
    systemctl restart systemd-timesyncd 2>/dev/null
    local i
    for ((i = 0; i < TIME_SYNC_TIMEOUT; i++)); do
        if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = yes ]; then
            echo "clock synchronized after ${i}s: $(date)"
            return 0
        fi
        sleep 1
    done
    echo "clock not synchronized after ${TIME_SYNC_TIMEOUT}s (no internet?) - starting anyway"
}

# Offset between wall clock and the monotonic uptime counter; it only
# changes if the wall clock is stepped (an NTP slew moves it by far less
# than CLOCK_JUMP_LIMIT).
clock_offset() {
    local up
    read -r up _ < /proc/uptime
    echo $(( $(date +%s) - ${up%.*} ))
}

wait_container() {
    until [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = true ]; do
        sleep 2
    done
}

on_term() {
    echo "stop requested - stopping stack"
    $DX bash "$STOP"
    exit 0
}
trap on_term TERM INT

setup_network
wait_time_sync
wait_container
echo "starting stack"
$DX bash "$RUN" || { echo "stack failed to start"; exit 1; }
base_offset=$(clock_offset)

while true; do
    sleep "$CHECK_INTERVAL" &
    wait $!
    offset=$(clock_offset)
    drift=$(( offset - base_offset ))
    if [ "${drift#-}" -gt "$CLOCK_JUMP_LIMIT" ]; then
        # e.g. internet (and so NTP) only arrived after the stack started
        echo "wall clock jumped ${drift}s - restarting stack"
        $DX bash "$STOP"
        wait_container
        $DX bash "$RUN" || { echo "stack failed to restart"; exit 1; }
        base_offset=$(clock_offset)
    elif ! $DX bash "$CHECK" 2>/dev/null; then
        echo "core process lost - restarting stack"
        $DX bash "$STOP"
        wait_container
        $DX bash "$RUN" || { echo "stack failed to restart"; exit 1; }
        base_offset=$(clock_offset)
    fi
done
