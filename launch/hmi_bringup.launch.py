"""Full bring-up for the web HMI (2026-10-01): everything that used to be
started by hand as separate `nohup ... &` commands throughout development,
in one launch file - control stack, cmd_vel arbitration, RF remote bridge,
and the web HMI server itself.

Serial device defaults use the stable /dev/serial/by-id/ paths (by USB
vendor/model/serial-number, not enumeration order) instead of /dev/ttyUSB0/1
- those renumber on every reconnect/reboot, which bit this project
repeatedly during development. These by-id paths are stable across reboots
for the same physical devices; if either device is ever swapped for a
different unit of the same model, re-check `ls /dev/serial/by-id/` and
update the defaults (or pass the new path as a launch argument).

Usage (after a fresh terminal / system restart):
    source /opt/ros/humble/setup.bash
    source ~/AMR/ros2_workspace/ros-amr/install/setup.bash
    ros2 launch calixto-ros-bot hmi_bringup.launch.py

Then open http://<this-machine's-LAN-IP>:8080 from any device on the same
network (or http://localhost:8080 on this machine itself).

Does NOT include RViz2 or any VNC/x11vnc/websockify pieces - those were a
2026-09-30/10-01 dead end (see web_hmi/status_web_server.py's own docstring)
superseded by the web HMI's own 2D canvas view.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
            description="Stable by-id path to the MCU's CP2102 serial adapter.",
        ),
        DeclareLaunchArgument(
            "remote_device",
            default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            description="Stable by-id path to the RF remote's CH340 USB receiver.",
        ),
        DeclareLaunchArgument(
            "web_port",
            default_value="8080",
            description="Port for the web HMI's HTTP server.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start a native RViz2 window too (independent of the web HMI's own canvas view).",
        ),
    ]

    serial_port = LaunchConfiguration("serial_port")
    remote_device = LaunchConfiguration("remote_device")
    web_port = LaunchConfiguration("web_port")
    gui = LaunchConfiguration("gui")

    web_hmi_dir = PathJoinSubstitution([FindPackageShare("calixto-ros-bot"), "web_hmi"])
    cmd_vel_mux_params = PathJoinSubstitution(
        [FindPackageShare("calixto-ros-bot"), "config", "cmd_vel_mux.yaml"]
    )

    control_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("calixto-ros-bot"), "launch", "diffbot_no_ekf.launch.py"])
        ),
        launch_arguments={"gui": gui, "serial_port": serial_port}.items(),
    )

    cmd_vel_mux_process = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([web_hmi_dir, "cmd_vel_mux.py"]),
             "--ros-args", "--params-file", cmd_vel_mux_params],
        output="screen",
    )

    remote_ros_process = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([web_hmi_dir, "remote_ros_node.py"]),
             "--remote-device", remote_device],
        output="screen",
    )

    web_hmi_process = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([web_hmi_dir, "status_web_server.py"]),
             "--port", web_port],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [
        control_stack,
        cmd_vel_mux_process,
        remote_ros_process,
        web_hmi_process,
    ])
