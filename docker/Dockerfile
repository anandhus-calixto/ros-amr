# Calixto AMR ROS2 Humble environment (2026-10-01)
#
# Built to run the ros-amr stack on real hardware without fighting distro
# packaging mismatches: ROS2 Humble's only officially supported binary
# target is Ubuntu 22.04 (Jammy), not whatever the host board happens to
# run (this was first built for an i.MX8M Plus running Debian 12
# "bookworm" - no ROS2 apt repo exists for that at all). This image is
# Ubuntu-based (FROM the official ros:humble image) and runs identically
# on any aarch64 host via Docker, regardless of the host's own distro -
# that's also what makes it portable to the i.MX95 later with zero
# changes needed here.
#
# This Dockerfile is the ENVIRONMENT only (system packages). The actual
# ros-amr source code is NOT baked in here - it's git-cloned once into the
# bind-mounted workspace volume (see docker/setup_workspace.sh) and built
# with colcon there instead. That split matters: rebuilding this image
# (e.g. to add a new system dependency) never touches your built
# workspace/git history, and pulling new ros-amr commits or rebuilding
# colcon packages never requires rebuilding this image.
#
# Build:
#   docker build -t calixto-ros-amr:humble -f docker/Dockerfile .
# Run (see docker/run_container.sh for the full command this mirrors):
#   docker run -d --name ros2_humble --privileged --network host \
#     --restart=always -v ~/ros2_ws:/ros2_ws calixto-ros-amr:humble
#
# Device/permission note (2026-10-01): using --privileged for now during
# development (simplest, full hardware access) rather than an explicit
# --device list. Revisit once cameras/LIDAR are actually connected and the
# final hardware list is known - see the project conversation this came
# from for the tradeoffs.

FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    # --- Build tooling ---
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    # --- ros2_control stack (DiffBotSystemHardware plugin + diff drive) ---
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    libserial-dev \
    # --- Navigation / SLAM / localization ---
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    # --- Teleop + cmd_vel arbitration ---
    # Real twist_mux - replaces web_hmi/cmd_vel_mux.py's hand-rolled
    # arbitration, which existed ONLY because this package needed
    # `sudo apt install` on the bare Debian host with no interactive sudo
    # available. That blocker doesn't exist in this Ubuntu-based image.
    ros-humble-teleop-twist-keyboard \
    ros-humble-twist-mux \
    # --- Visualization (useful with a display or X11 forwarding later) ---
    ros-humble-rviz2 \
    # --- CAN bus + general networking diagnostics ---
    can-utils \
    iproute2 \
    net-tools \
    iputils-ping \
    # --- Python deps for the bench/remote-bridge tools ---
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update || true

WORKDIR /ros2_ws

# Source the ROS2 environment (and the workspace overlay, once built) in
# every interactive shell automatically.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /root/.bashrc

# ros_entrypoint.sh (inherited from the base ros:humble image) sources
# /opt/ros/humble/setup.bash and then execs whatever CMD is - overridden
# here (and again at `docker run` time) to just stay alive, since this
# container is meant to be a long-running environment you `docker exec`
# into, not a one-shot command runner.
CMD ["sleep", "infinity"]
