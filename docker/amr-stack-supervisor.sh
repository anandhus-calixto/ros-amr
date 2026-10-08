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

wait_container
echo "starting stack"
$DX bash "$RUN" || { echo "stack failed to start"; exit 1; }

while true; do
    sleep "$CHECK_INTERVAL" &
    wait $!
    if ! $DX bash "$CHECK" 2>/dev/null; then
        echo "core process lost - restarting stack"
        $DX bash "$STOP"
        wait_container
        $DX bash "$RUN" || { echo "stack failed to restart"; exit 1; }
    fi
done
