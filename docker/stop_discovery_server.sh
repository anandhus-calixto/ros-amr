#!/usr/bin/env bash
# Stops the Fast-DDS Discovery Server started by start_discovery_server.sh.
# Safe to run when nothing is running. Stopping it drops ALL ROS2 discovery
# between every process using ROS_DISCOVERY_SERVER (the whole amr_stack +
# navigation, and any remote rviz2) until it's started again - normally only
# needed when deliberately switching discovery modes, not routine cleanup.
PIDFILE=/tmp/discovery_server.pid

if [ ! -s "$PIDFILE" ]; then
    echo "No discovery server recorded in $PIDFILE - nothing to stop."
    exit 0
fi

pgid="$(cat "$PIDFILE")"
kill -INT -- "-$pgid" 2>/dev/null || true

for _ in $(seq 1 10); do
    kill -0 -- "-$pgid" 2>/dev/null || break
    sleep 1
done

if kill -0 -- "-$pgid" 2>/dev/null; then
    echo "Process group $pgid did not stop - killing."
    kill -KILL -- "-$pgid" 2>/dev/null || true
fi

rm -f "$PIDFILE"
echo "Discovery server stopped."
