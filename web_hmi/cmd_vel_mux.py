#!/usr/bin/env python3
"""cmd_vel arbitrator for the web HMI's mode selector (2026-09-30).

Merges cmd_vel_remote / cmd_vel_teleop / cmd_vel_nav (all TwistStamped) into
a single /cmd_vel that diffbot_base_controller reads (diffbot.launch.py's
control_node already remaps /diffbot_base_controller/cmd_vel -> /cmd_vel, so
no launch-file change was needed on the output side).

This is NOT the real `twist_mux` package - that (and rosbridge_suite, and
novnc/websockify for embedding RViz2 in a browser) all need
`sudo apt install`, and there is no interactive sudo password available
here. This node reimplements just the priority+timeout arbitration part of
twist_mux, reading the same {topics: {name: {topic, timeout, priority}}}
shape from config/cmd_vel_mux.yaml (a separate file from config/twist_mux.yaml,
which stays as-is for nav2_sim.launch.py in case the real twist_mux package
is installed for Gazebo sim later).

On top of that, it adds explicit mode selection (what the HMI's "an option
to select remote control, teleop, nav" requirement actually needs, which
plain priority racing doesn't guarantee): publishing a std_msgs/String on
mode_topic (default /hmi/cmd_vel_mode) with one of "auto" (default -
priority+timeout arbitration, same as real twist_mux), or one of the
configured source names (e.g. "remote") to force that source exclusively,
ignoring the others regardless of priority, until the mode changes again or
that source itself goes stale.

Whichever source is currently actually driving the robot (or "none" if
every source is stale/absent) is published on active_source_topic (default
/hmi/cmd_vel_active_source) so the HMI can show it.

Usage (after `source /opt/ros/humble/setup.bash` and the workspace's
install/setup.bash, alongside any diffbot*.launch.py):
    python3 web_hmi/cmd_vel_mux.py --ros-args --params-file config/cmd_vel_mux.yaml
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

_LATCHED_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class CmdVelMux(Node):
    def __init__(self):
        super().__init__(
            "cmd_vel_mux",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        output_topic = self._param_or("output_topic", "/cmd_vel")
        mode_topic = self._param_or("mode_topic", "/hmi/cmd_vel_mode")
        active_topic = self._param_or("active_source_topic", "/hmi/cmd_vel_active_source")
        rate = self._param_or("rate", 20.0)

        self._sources = {}  # name -> {"topic", "timeout", "priority", "msg", "stamp"}
        topic_params = self.get_parameters_by_prefix("topics")
        names = sorted({key.split(".", 1)[0] for key in topic_params.keys()})
        if not names:
            self.get_logger().fatal(
                "No 'topics.<name>.*' parameters found - pass --params-file config/cmd_vel_mux.yaml"
            )
            raise SystemExit(1)

        for name in names:
            topic = topic_params[f"{name}.topic"].value
            timeout = topic_params[f"{name}.timeout"].value
            priority = topic_params[f"{name}.priority"].value
            self._sources[name] = {
                "topic": topic, "timeout": timeout, "priority": priority,
                "msg": None, "stamp": None,
            }
            self.create_subscription(
                TwistStamped, topic,
                lambda msg, n=name: self._on_source(n, msg), 10,
            )
            self.get_logger().info(
                f"Source '{name}': {topic} (priority={priority}, timeout={timeout}s)"
            )

        self._mode = "auto"
        # TRANSIENT_LOCAL on both this subscription and the HMI's publisher of
        # mode_topic means a (re)started mux picks up the last-set mode
        # automatically, without needing a publisher of its own here.
        self.create_subscription(String, mode_topic, self._on_mode, _LATCHED_QOS)

        self._output_pub = self.create_publisher(TwistStamped, output_topic, 10)
        self._active_pub = self.create_publisher(String, active_topic, _LATCHED_QOS)
        self._last_active = None

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"cmd_vel_mux ready: output={output_topic}, mode_topic={mode_topic} "
            f"(default 'auto'), active_source_topic={active_topic}"
        )

    def _param_or(self, name, default):
        return self.get_parameter(name).value if self.has_parameter(name) else default

    def _on_source(self, name, msg):
        self._sources[name]["msg"] = msg
        self._sources[name]["stamp"] = time.monotonic()

    def _on_mode(self, msg):
        valid = set(self._sources.keys()) | {"auto"}
        if msg.data not in valid:
            self.get_logger().warn(f"Ignoring unknown mode '{msg.data}' (valid: {sorted(valid)})")
            return
        if msg.data != self._mode:
            self.get_logger().info(f"Mode changed: '{self._mode}' -> '{msg.data}'")
            self._mode = msg.data

    def _fresh(self, name):
        s = self._sources[name]
        return s["stamp"] is not None and (time.monotonic() - s["stamp"]) < s["timeout"]

    def _tick(self):
        if self._mode == "auto":
            candidates = [n for n in self._sources if self._fresh(n)]
            active = max(candidates, key=lambda n: self._sources[n]["priority"]) if candidates else None
        else:
            active = self._mode if self._fresh(self._mode) else None

        if active != self._last_active:
            self._active_pub.publish(String(data=active or "none"))
            self.get_logger().info(f"Active source: {active or 'none'}")
            self._last_active = active

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        if active is not None:
            out.header.frame_id = self._sources[active]["msg"].header.frame_id
            out.twist = self._sources[active]["msg"].twist
        # else: leave twist zeroed - explicit stop when nothing is active/fresh,
        # on top of (not instead of) diff_drive_controller's own cmd_vel_timeout.
        self._output_pub.publish(out)


def main():
    rclpy.init()
    try:
        node = CmdVelMux()
    except SystemExit as e:
        rclpy.shutdown()
        sys.exit(e.code)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
