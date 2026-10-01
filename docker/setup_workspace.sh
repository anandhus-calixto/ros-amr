#!/usr/bin/env bash
# One-time (or re-run-anytime) workspace setup: clones ros-amr via HTTPS
# (read-only - this board only needs to run the code, not push commits
# from it) and builds it with colcon. Run this INSIDE the container:
#
#   docker exec -it ros2_humble bash /ros2_ws/docker/setup_workspace.sh
#
# (it's available at that path automatically once ros-amr itself has been
# cloned once - see the bootstrap clone command below for the very first
# run, before this script exists inside the workspace yet)
set -eo pipefail

cd /ros2_ws

if [ ! -d src/ros-amr ]; then
    echo "Cloning ros-amr..."
    git clone https://github.com/anandhus-calixto/ros-amr.git src/ros-amr
else
    echo "ros-amr already cloned - pulling latest..."
    git -C src/ros-amr pull
fi

echo "Resolving dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -y --rosdistro humble || true

echo "Building with colcon..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "Done. Source /ros2_ws/install/setup.bash (or start a new shell) to use it."
