"""RViz2 on a laptop, viewing the robot's live topics over the network
(2026-10-07): robot model + LiDAR scan by default, costmaps optionally.

Why this exists
---------------
The robot's WiFi AP drops multicast, so a laptop on WiFi sees none of the
robot's topics with the default SIMPLE discovery - `ros2 topic list` returns
only /parameter_events and /rosout. Confirmed with `ros2 multicast send` on
the robot and `ros2 multicast receive` on the laptop: the datagram never
arrives.

This launch file fixes that on the LAPTOP SIDE ONLY, by generating a Fast DDS
profile that lists the robot as a unicast initial peer and pointing
FASTRTPS_DEFAULT_PROFILES_FILE at it. The robot's stack is not touched and
keeps its default SIMPLE/multicast discovery - this is deliberately NOT the
Fast-DDS Discovery Server that was tried and reverted on 2026-10-07, which
broke SERVICE discovery between the robot's own nodes (see the header of
docker/run_full_stack.sh and docker/start_discovery_server.sh).

maxInitialPeersRange is raised well above the Fast DDS default of 4 because
the robot runs far more than 4 DDS participants (nav2, controllers, LiDAR,
EKF, ...), and the laptop can only discover participants whose unicast
discovery port it actually probes.

127.0.0.1 is in the peer list as well. Without it, this profile (which turns
off the builtin transports, including shared memory) leaves two nodes on THIS
laptop unable to discover each other - rviz2's own subscriptions were
invisible to `ros2 topic info` run in another terminal here.

What the default config shows
-----------------------------
lidar_robot_minimal.rviz: robot model, LiDAR scan, /map, and the local +
global costmaps.

Fixed frame MUST be map here, not base_link. /map is latched (transient
local) and published exactly once, so RViz sees it a second or two after
startup. RViz's Map display runs messages through a TF message filter, and
with a base_link fixed frame the map->base_link transform is not in RViz's
buffer yet at that moment, so the one and only /map message is dropped and
never comes again - the map silently stays blank forever. With fixed frame
map the transform is the identity and always available. Verified both ways:
base_link produced no "Trying to create a map of size" line at all, map
produced it within a second. The costmaps are immune because they republish
at 0.9-2.9 Hz, so they simply arrive again once TF is ready.

Stability
---------
With the full lidar_transient_local.rviz (map + local costmap + global
costmap + plans), RViz dies after 15-60 seconds. Observed exit codes were -9
(SIGKILL, i.e. killed from outside, not an internal crash) and a clean 0, at
random lifetimes. Ruled out: memory (RSS flat at ~165 MB, 5.6 GB free,
cgroup oom_kill 0, no systemd-oomd activity), the robot connection (bare
`rviz2` with NO config and no ROS data died the same way), the display
session generally (a plain Tk window outlived it), Claude Code hooks, cron,
autostart and screen blanking. Not yet identified: what actually sends the
signal; `dmesg` needs root here, and GNOME/mutter force-quitting an
unresponsive XWayland client (check-alive-timeout 5000 ms) remains the
leading suspect, which fits RViz blocking its GUI thread on DDS work over a
lossy WiFi link.

The GLSL "active samplers with a different type refer to the same texture
image unit" error is NOT the cause - it is a harmless Ogre/Mesa warning from
the Map display that also appears in runs that keep going. LIBGL_ALWAYS_SOFTWARE
did not help and made RViz slower, so that option was removed.

The deaths are INTERMITTENT, and no display has been shown to cause them.
A robot-model+scan-only config ran past 90 s, but so did a costmap config
that had died at 57 s in an identical run minutes earlier - and in a
side-by-side test, a costmap instance and a costmap-free instance both ran
113 s without dying. Beware of concluding "display X causes it" from a
single run; it needs repeated trials.

The "Message Filter dropping message: frame 'laser_frame' ... queue is full"
lines are the cost of the map fixed frame: each scan needs map->laser_frame
at its own timestamp, and WiFi latency sometimes beats TF to it. Measured at
3 drops in 43 s against a 16 Hz scan (~99.5% rendered), so it is cosmetic -
unlike a blank map, which is not.

Note that the unattended tests above ran against DISPLAY=:0 from a shell
whose session may no longer have been the live one (screenshots came back
black and the resolution changed mid-session), so they are not a reliable
verdict on stability in an interactive GNOME/Wayland session.

Usage
-----
    source /opt/ros/humble/setup.bash
    source ~/AMR/ros2_workspace/ros-amr/install/setup.bash
    ros2 launch calixto-ros-bot rviz_remote.launch.py

The older full config (plans + nav2/slam_toolbox panels):
    ros2 launch calixto-ros-bot rviz_remote.launch.py \
        rviz_config:=$(ros2 pkg prefix calixto-ros-bot)/share/calixto-ros-bot/rviz/lidar_transient_local.rviz
(that one adds the global/local plans and the nav2 + slam_toolbox panels)

If the board's IP changed:
    ros2 launch calixto-ros-bot rviz_remote.launch.py robot_ip:=192.168.1.99

On the robot itself, or over a wired link where multicast works, skip the
unicast profile entirely:
    ros2 launch calixto-ros-bot rviz_remote.launch.py unicast_discovery:=false

Note that `ros2 topic list` run by hand in another terminal will still look
empty unless that terminal exports FASTRTPS_DEFAULT_PROFILES_FILE too (the
path is printed at startup) AND the ROS 2 daemon is restarted with it -
`ros2 daemon stop` first, because a daemon already running without the
profile is what answers the query.
"""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PROFILE_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by launch/rviz_remote.launch.py - do not edit by hand. -->
<dds xmlns="http://www.eprosima.com">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_wide_range</transport_id>
        <type>UDPv4</type>
        <maxInitialPeersRange>{max_peers}</maxInitialPeersRange>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="unicast_to_robot" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>udp_wide_range</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
        <builtin>
          <initialPeersList>
            <locator>
              <udpv4>
                <address>{robot_ip}</address>
              </udpv4>
            </locator>
            <locator>
              <udpv4>
                <address>127.0.0.1</address>
              </udpv4>
            </locator>
          </initialPeersList>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
"""


def _write_dds_profile(context, *args, **kwargs):
    """Write the unicast-peer profile and export it, unless disabled."""
    if LaunchConfiguration("unicast_discovery").perform(context).lower() not in ("true", "1"):
        return []

    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    max_peers = LaunchConfiguration("max_peers_range").perform(context)

    # Stable, per-IP path so the same file can be reused by hand from any
    # terminal (export FASTRTPS_DEFAULT_PROFILES_FILE=<printed path>).
    profile_path = os.path.join(
        tempfile.gettempdir(), "fastdds_unicast_{}.xml".format(robot_ip.replace(".", "_"))
    )
    with open(profile_path, "w") as handle:
        handle.write(PROFILE_TEMPLATE.format(robot_ip=robot_ip, max_peers=max_peers))

    print("[rviz_remote] unicast peers {} + 127.0.0.1 via {}".format(robot_ip, profile_path))

    return [
        # The profile is a Fast DDS file, so pin the RMW that reads it.
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", profile_path),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.62",
            description="IP of the machine running the robot stack, probed by unicast "
                        "because the WiFi AP drops the multicast discovery needs.",
        ),
        DeclareLaunchArgument(
            "unicast_discovery",
            default_value="true",
            description="Generate and use the Fast DDS unicast-peer profile. Set false on "
                        "the robot itself or on a wired link where multicast works.",
        ),
        DeclareLaunchArgument(
            "max_peers_range",
            default_value="40",
            description="Fast DDS maxInitialPeersRange: how many participant discovery ports "
                        "to probe on robot_ip. The default of 4 is far too low for this stack.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("calixto-ros-bot"), "rviz", "lidar_robot_minimal.rviz"]
            ),
            description="RViz2 config. Default shows the robot model, LiDAR scan, /map and "
                        "both costmaps. See this file's header for the intermittent-exit issue.",
        ),
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=_write_dds_profile),
        rviz_node,
    ])
