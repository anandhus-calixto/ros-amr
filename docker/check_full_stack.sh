#!/usr/bin/env bash
# Exit 0 only if the stack from run_full_stack.sh is recorded and every core
# process is running (zombies don't count). Used by amr-stack-supervisor.sh.
# RF remote and LiDAR are deliberately not required: they depend on hardware
# that may be unplugged, and restarting the whole stack for that would loop.
PIDFILE=/tmp/amr_stack.pids
[ -s "$PIDFILE" ] || exit 1

running() {
    local pid state
    for pid in $(pgrep -f -- "$1"); do
        state=$(awk '{print $3}' "/proc/$pid/stat" 2>/dev/null)
        [ -n "$state" ] && [ "$state" != Z ] && return 0
    done
    return 1
}

for proc in "lib/bno055/bno055" "ros2_control_node" "ekf_node" \
            "lib/twist_mux/twist_mux" "twist_stamper.py" "status_web_server.py"; do
    running "$proc" || { echo "not running: $proc" >&2; exit 1; }
done
exit 0
