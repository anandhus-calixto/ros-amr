"""Nav2 on a saved map, for the real robot (2026-10-07).

The launch-file equivalent of docker/start_navigation.sh, which is still the
supported way to run this inside the container in the background. Use this
when you want it in the foreground of a terminal, with Ctrl-C to stop
everything at once.

Needs the rest of the stack already running (docker/run_full_stack.sh):
odom->base_link from the EKF and /scan_filtered from the LiDAR + body filter.

Why this exists next to the stock launch/navigation_launch.py: that file is
the unmodified Nav2 one. It brings up neither map_server nor amcl, and - the
part that matters on this robot - it does not remap cmd_vel.

cmd_vel remapping (do not drop this)
------------------------------------
controller_server AND behavior_server both publish velocity, and both are
remapped cmd_vel -> cmd_vel_nav here. On this robot /cmd_vel is the real
final command topic (see config/twist_mux.yaml), and twist_mux reads
cmd_vel_nav at the LOWEST priority so the E-stop, the RF remote and teleop
can all override Nav2. Publishing straight to /cmd_vel would bypass that
arbitration entirely. behavior_server was missing this remap until
2026-10-07, which let recovery behaviours (Spin, BackUp, ...) drive the
robot with no E-stop in the path.

AMCL needs an initial pose before it can localize: send a 2D Pose Estimate
from RViz, or use the web HMI, before setting a goal.

Usage, inside the container:
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    ros2 launch calixto-ros-bot amr_navigation.launch.py map:=lab-beta

`map` takes a name from the maps directory (no .yaml) or a full path to a
map yaml. Make one first with amr_mapping.launch.py + map_saver_cli.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Brought up by the lifecycle manager, in this order.
MANAGED_NODES = [
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "behavior_server",
    "smoother_server",
    "bt_navigator",
]


def _resolve_map(context, *args, **kwargs):
    """Accept either a full path to a map yaml or a bare name from maps/."""
    requested = LaunchConfiguration("map").perform(context)
    if not requested:
        raise RuntimeError("map:=<name|path> is required, e.g. map:=lab-beta")

    candidates = [requested]
    if not requested.endswith((".yaml", ".yml")):
        candidates.append(requested + ".yaml")

    maps_dirs = [
        os.path.join(get_package_share_directory("calixto-ros-bot"), "maps"),
        # The source checkout, where map_saver_cli usually writes.
        "/ros2_ws/src/ros-amr/maps",
    ]
    for cand in list(candidates):
        if not os.path.isabs(cand):
            candidates += [os.path.join(d, cand) for d in maps_dirs]

    for cand in candidates:
        if os.path.isfile(cand):
            print("[amr_navigation] map: {}".format(cand))
            return [Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[LaunchConfiguration("params_file"),
                            {"yaml_filename": cand,
                             "use_sim_time": LaunchConfiguration("use_sim_time")}],
            )]

    available = []
    for d in maps_dirs:
        if os.path.isdir(d):
            available += [f[:-5] for f in sorted(os.listdir(d)) if f.endswith(".yaml")]
    raise RuntimeError(
        "No such map: {!r}. Looked in {}. Available: {}. "
        "Make one with amr_mapping.launch.py, then map_saver_cli.".format(
            requested, ", ".join(maps_dirs), ", ".join(available) or "(none)")
    )


def _nav_node(package, executable, name, remap_cmd_vel=False):
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        parameters=[LaunchConfiguration("params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        # See this file's header: never publish straight to /cmd_vel.
        remappings=[("cmd_vel", "cmd_vel_nav")] if remap_cmd_vel else [],
    )


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "map",
            description="Map name from the maps directory (no .yaml), or a full path "
                        "to a map yaml.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("calixto-ros-bot"), "config", "nav2_params.yaml"]
            ),
            description="Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="False on the real robot. Only true under Gazebo.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Have the lifecycle manager drive the nodes to ACTIVE itself.",
        ),
    ]

    nodes = [
        # map_server is created by _resolve_map, which needs the resolved path.
        OpaqueFunction(function=_resolve_map),
        _nav_node("nav2_amcl", "amcl", "amcl"),
        _nav_node("nav2_planner", "planner_server", "planner_server"),
        _nav_node("nav2_controller", "controller_server", "controller_server",
                  remap_cmd_vel=True),
        _nav_node("nav2_behaviors", "behavior_server", "behavior_server",
                  remap_cmd_vel=True),
        _nav_node("nav2_smoother", "smoother_server", "smoother_server"),
        _nav_node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": LaunchConfiguration("autostart"),
                "node_names": MANAGED_NODES,
            }],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)
