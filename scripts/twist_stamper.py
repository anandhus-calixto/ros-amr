#!/usr/bin/env python3
"""Relays geometry_msgs/Twist (twist_mux's output) to geometry_msgs/TwistStamped
(what diffbot_base_controller actually subscribes to) - the two never connect
on their own since ROS2 requires exact type match, not just topic name match.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel_unstamped', self.cb, 10)

    def cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(TwistStamper())


if __name__ == '__main__':
    main()
