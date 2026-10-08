#!/usr/bin/env python3
"""Removes LiDAR returns that hit the robot's own body (wires, chassis parts).

Reads /scan and publishes /scan_filtered with every return inside the body
rectangle set to out-of-range (inf). The rectangle is in the LiDAR frame, metres:
the chassis (780 x 480 mm, centred on base_link) with the LiDAR 310 mm forward of
centre, plus a margin. Adjust with the x_min/x_max/y_min/y_max parameters.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

SCAN_QOS = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)


class BodyFilter(Node):
    def __init__(self):
        super().__init__("scan_body_filter")
        # Chassis in the LiDAR frame: x from -0.39-0.31 to 0.39-0.31, y +-0.24, plus 0.03 margin.
        self.declare_parameter("x_min", -0.73)
        self.declare_parameter("x_max", 0.11)
        self.declare_parameter("y_min", -0.27)
        self.declare_parameter("y_max", 0.27)
        self.box = (
            self.get_parameter("x_min").value,
            self.get_parameter("x_max").value,
            self.get_parameter("y_min").value,
            self.get_parameter("y_max").value,
        )
        self.pub = self.create_publisher(LaserScan, "/scan_filtered", SCAN_QOS)
        self.create_subscription(LaserScan, "/scan", self._on_scan, SCAN_QOS)
        self.get_logger().info(f"removing returns inside x[{self.box[0]}, {self.box[1]}] y[{self.box[2]}, {self.box[3]}]")

    def _on_scan(self, msg: LaserScan):
        x_min, x_max, y_min, y_max = self.box
        angle = msg.angle_min
        ranges = list(msg.ranges)
        for i, r in enumerate(ranges):
            if math.isfinite(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if x_min <= x <= x_max and y_min <= y <= y_max:
                    ranges[i] = float("inf")
            angle += msg.angle_increment
        msg.ranges = ranges
        self.pub.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    node = BodyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
