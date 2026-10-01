#!/usr/bin/env bash
# Builds (if needed) and (re)starts the long-running ros2_humble container.
# Safe to re-run any time you change the Dockerfile - it rebuilds the image
# and replaces the container, but never touches the bind-mounted workspace
# (~/ros2_ws), so your cloned source/build artifacts survive.
#
# Usage (from the ros-amr repo root, or anywhere - paths below are absolute):
#   ./docker/run_container.sh
set -eo pipefail  # no -u: see run_hmi.sh's note on the other board for why

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="calixto-ros-amr:humble"
CONTAINER_NAME="ros2_humble"
WORKSPACE_DIR="$HOME/ros2_ws"

echo "Building $IMAGE_NAME from $SCRIPT_DIR/Dockerfile ..."
docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$SCRIPT_DIR/.."

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "Removing existing $CONTAINER_NAME container (workspace volume is untouched)..."
    docker rm -f "$CONTAINER_NAME"
fi

mkdir -p "$WORKSPACE_DIR/src"

echo "Starting $CONTAINER_NAME ..."
docker run -d \
    --name "$CONTAINER_NAME" \
    --privileged \
    --network host \
    --restart=always \
    -v "$WORKSPACE_DIR":/ros2_ws \
    -v /etc/localtime:/etc/localtime:ro \
    -w /ros2_ws \
    "$IMAGE_NAME" \
    sleep infinity

echo "Done. Shell in with: docker exec -it $CONTAINER_NAME bash"
