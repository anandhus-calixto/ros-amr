"""Online mapping with slam_toolbox on the real robot (2026-10-07).

The launch-file equivalent of docker/start_mapping.sh, which is still the
supported way to run this inside the container in the background. Use this
when you want it in the foreground of a terminal, with Ctrl-C to stop.

Needs the rest of the stack already running (docker/run_full_stack.sh):
odom->base_link from the EKF and the LiDAR publishing /scan + /scan_filtered.

Two parameters are overridden on purpose, exactly as start_mapping.sh does,
because config/mapper_params_online_async.yaml is checked in with values for
the OTHER mode:
  - mode:=mapping        (the file says mode: localization)
  - scan_topic:=/scan_filtered   (the file says /scan)
/scan_filtered is the LiDAR with the robot's own body boxed out
(config/scan_box_filter.yaml); mapping against raw /scan paints the chassis
into the map.

Unlike launch/online_async_launch.py (the stock slam_toolbox file, which
defaults use_sim_time to true for Gazebo), this one defaults it to false.

Usage, inside the container:
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    ros2 launch calixto-ros-bot amr_mapping.launch.py

Drive the robot around with the RF remote or the web HMI, watch the map in
RViz (ros2 launch calixto-ros-bot rviz_remote.launch.py from a laptop), then
SAVE IT - the map only exists in slam_toolbox's memory until you do:

    ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/ros-amr/maps/<name>

which writes <name>.pgm + <name>.yaml. Then feed that name to
amr_navigation.launch.py. docker/stop_mapping.sh --save does the same thing.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("calixto-ros-bot"), "config",
                 "mapper_params_online_async.yaml"]
            ),
            description="slam_toolbox parameter file.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan_filtered",
            description="Scan to map against. /scan_filtered has the robot's own body "
                        "boxed out; raw /scan would paint the chassis into the map.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="False on the real robot. Only true under Gazebo.",
        ),
    ]

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                # Overrides the checked-in file, which is set to localization.
                "mode": "mapping",
                "scan_topic": LaunchConfiguration("scan_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    return LaunchDescription(declared_arguments + [slam_node])
