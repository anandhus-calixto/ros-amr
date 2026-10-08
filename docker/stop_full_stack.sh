#!/usr/bin/env bash
# Stops the stack started by run_full_stack.sh. Sends SIGINT to each recorded
# process group (clean shutdown), waits up to 10s, then SIGKILLs anything left.
# Safe to run when nothing is running.
PIDFILE=/tmp/amr_stack.pids

if [ ! -s "$PIDFILE" ]; then
    echo "No stack recorded in $PIDFILE - nothing to stop."
    exit 0
fi

while read -r _ pgid; do
    kill -INT -- "-$pgid" 2>/dev/null || true
done < "$PIDFILE"

for _ in $(seq 1 10); do
    alive=0
    while read -r _ pgid; do
        kill -0 -- "-$pgid" 2>/dev/null && alive=1
    done < "$PIDFILE"
    [ "$alive" -eq 0 ] && break
    sleep 1
done

while read -r _ pgid; do
    if kill -0 -- "-$pgid" 2>/dev/null; then
        echo "Process group $pgid did not stop - killing."
        kill -KILL -- "-$pgid" 2>/dev/null || true
    fi
done < "$PIDFILE"

rm -f "$PIDFILE"
echo "Stack stopped."
