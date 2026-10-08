#!/usr/bin/env bash
# Records a navigation run to a rosbag ON THE ROBOT, for replaying into RViz on
# a laptop afterwards.
#
# Usage: record_bag.sh <name> [seconds]     (seconds: omit = until Ctrl-C)
#   record_bag.sh costmap_run1 60
#
# Why record instead of just opening RViz live
# --------------------------------------------
# 2026-10-08: RViz on the laptop freezes within minutes when streaming from the
# robot over WiFi, and has to be force-quit (the "rviz2 is not responding"
# dialog, which SIGKILLs it - every "crash" seen that day was this).
#
# The cause is NOT bandwidth - /tf is only ~4.8 KB/s. It is latency jitter
# (measured laptop->robot: avg 54 ms, max 260 ms, mdev 60; earlier avg 113 ms,
# max 384 ms) combined with RELIABLE QoS. RELIABLE means a late packet is
# retransmitted and delivered in order rather than dropped, so a latency spike
# builds a backlog instead of losing a frame. RViz was caught working on data
# 56 SECONDS stale. That backlog then floods tf2's buffer, whose mutex the GUI
# thread needs for every transform lookup on every redraw, so the GUI blocks.
# Confirmed from the thread states while frozen:
#   main thread     futex_wait_queue             (waiting on the tf2 mutex)
#   3x DDS threads  __skb_wait_for_more_packets  (waiting on the network)
#   QXcbEventQueue  do_poll                      (X11 thread perfectly healthy)
# Nothing to do with graphics - switching RViz to the laptop's NVIDIA GPU
# changed none of it.
#
# Recording here happens on the robot with no network in the path, so nothing
# is delayed or queued. Replaying from local disk on the laptop cannot stall
# for the same reason. It is also simply better for analysis: you can pause,
# scrub backwards and replay the same approach as many times as you like.
#
# Replay on the laptop:
#   scp -r imx8mp:/root/ros2_ws/src/ros-amr/bags/<name> ~/bags/
#   ros2 bag play ~/bags/<name> --clock --loop
#   ros2 launch calixto-ros-bot rviz_remote.launch.py unicast_discovery:=false
# unicast_discovery:=false matters: the bag is local, so RViz must use ordinary
# localhost discovery and NOT the robot's unicast peer profile.
set -eo pipefail

BAGS=/ros2_ws/src/ros-amr/bags
NAME="${1:?usage: record_bag.sh <name> [seconds]}"
SECS="${2:-}"
OUT="$BAGS/$NAME"

if [ -e "$OUT" ]; then
    echo "Already exists: $OUT - pick another name or delete it." >&2
    exit 1
fi
mkdir -p "$BAGS"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# /map, /robot_description and the costmaps are TRANSIENT_LOCAL (latched,
# published once). rosbag2 adopts each publisher's offered QoS, so it does pick
# them up on subscribe - but only if they are already being published when
# recording starts. Start navigation BEFORE recording, or the replay will have
# no map and RViz will show an empty world.
TOPICS=(
  /tf /tf_static
  /map
  /local_costmap/costmap /local_costmap/costmap_updates
  /global_costmap/costmap /global_costmap/costmap_updates
  /local_costmap/published_footprint /global_costmap/published_footprint
  /plan /local_plan /received_global_plan /transformed_global_plan
  /scan /scan_filtered
  /odometry/filtered /diffbot_base_controller/odom
  /joint_states /robot_description
  /cmd_vel /cmd_vel_nav
  /amcl_pose /particle_cloud
  /mcu_status /bno055/imu
)

missing=()
for t in "${TOPICS[@]}"; do
    ros2 topic info "$t" >/dev/null 2>&1 || missing+=("$t")
done
if [ ${#missing[@]} -gt 0 ]; then
    echo "WARNING: these topics do not exist yet and will be absent from the bag:"
    printf '  %s\n' "${missing[@]}"
    echo "(start navigation first if you wanted the map/costmaps/plans)"
fi

echo "recording ${#TOPICS[@]} topics -> $OUT"
if [ -n "$SECS" ]; then
    echo "duration: ${SECS}s"
    timeout --signal=INT "$SECS" \
        ros2 bag record -o "$OUT" "${TOPICS[@]}" || true
else
    echo "Ctrl-C to stop."
    ros2 bag record -o "$OUT" "${TOPICS[@]}" || true
fi

sync
echo
echo "size: $(du -sh "$OUT" 2>/dev/null | cut -f1)"
ros2 bag info "$OUT" 2>/dev/null | sed -n '1,8p' || true
echo
echo "copy to the laptop with:"
echo "  scp -r imx8mp:/root/ros2_ws/src/ros-amr/bags/$NAME ~/bags/"
