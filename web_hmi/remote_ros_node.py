#!/usr/bin/env python3
"""RF remote -> ROS2 cmd_vel_remote publisher (2026-09-30).

Replaces remote-control/remote_bridge.py's direct-serial-to-MCU approach for
normal operation: this publishes geometry_msgs/msg/TwistStamped on
/cmd_vel_remote instead of writing raw per-wheel Command Packets straight to
the MCU's serial port. cmd_vel_mux.py (priority 150 by default - see
config/cmd_vel_mux.yaml) merges that in with teleop/nav and
diffbot_base_controller does the robot-frame -> per-wheel conversion, so
ros2_control stays the sole owner of the MCU serial port (matches the same
"single port owner" reasoning as the /mcu_status wiring).

remote_bridge.py itself is left in place (not deleted) - it's still useful
as a standalone bench tool for testing the RF remote or the MCU link without
the full ROS2 stack running.

Button -> Twist mapping (P: value from the remote's #CM:REMOTE,P:<n>$
frames):
    Top    (P:1)  -> linear.x  = +linear_speed
    Down   (P:2)  -> linear.x  = -linear_speed
    Left   (P:4)  -> angular.z = +angular_speed  (CCW, REP-103)
    Right  (P:8)  -> angular.z = -angular_speed
    Centre (P:16) -> stop
    idle (CM:FREE), release (P:0), an unrecognised/combo P value, or the
    remote going silent for more than --watchdog-timeout seconds, all stop.

Usage (after sourcing /opt/ros/humble/setup.bash and this workspace's
install/setup.bash):
    python3 web_hmi/remote_ros_node.py
    python3 web_hmi/remote_ros_node.py --remote-device /dev/ttyUSB1 --linear-speed 0.5 --angular-speed 1.5
"""
import argparse
import glob
import sys
import threading
import time

import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

DEFAULT_REMOTE_DEVICE = None  # None = auto-detect the first /dev/ttyUSB*
DEFAULT_REMOTE_BAUD = 115200
DEFAULT_LINEAR_SPEED = 0.5   # m/s
DEFAULT_ANGULAR_SPEED = 1.5  # rad/s

BUTTON_NAMES = {1: "TOP", 2: "DOWN", 4: "LEFT", 8: "RIGHT", 16: "CENTRE"}


def find_remote_port():
    candidates = sorted(glob.glob("/dev/ttyUSB*"))
    if not candidates:
        sys.exit("No /dev/ttyUSB* device found. Is the RF receiver plugged in?")
    return candidates[0]


def cm_field(msg):
    if "CM:" not in msg:
        return None
    return msg.split("CM:", 1)[1].split(",", 1)[0]


def p_field(msg):
    if "P:" not in msg:
        return None
    try:
        return int(msg.split("P:", 1)[1].split(",", 1)[0].rstrip("$"))
    except ValueError:
        return None


class RemoteReader(threading.Thread):
    """Background thread: reads the RF remote, decodes the active command."""

    def __init__(self, port, baud, linear_speed, angular_speed):
        super().__init__(daemon=True)
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.ser.reset_input_buffer()

        self._lock = threading.Lock()
        self._label = "STARTING"
        self._linear = 0.0
        self._angular = 0.0
        self._last_frame_time = None
        self._stop = False

    def snapshot(self):
        with self._lock:
            return self._label, self._linear, self._angular, self._last_frame_time

    def stop(self):
        self._stop = True

    def run(self):
        buf = b""
        while not self._stop:
            chunk = self.ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"#" in buf and b"$" in buf:
                start = buf.index(b"#")
                if b"$" not in buf[start:]:
                    break
                end = buf.index(b"$", start) + 1
                raw = buf[start:end]
                buf = buf[end:]
                try:
                    msg = raw.decode("ascii")
                except UnicodeDecodeError:
                    continue
                self._handle_frame(msg)
        self.ser.close()

    def _handle_frame(self, msg):
        cm = cm_field(msg)
        now = time.time()
        lin_s, ang_s = self.linear_speed, self.angular_speed
        if cm == "FREE":
            label, linear, angular = "FREE (idle)", 0.0, 0.0
        elif cm == "REMOTE":
            p = p_field(msg)
            if p == 0:
                label, linear, angular = "RELEASE (P:0)", 0.0, 0.0
            elif p in BUTTON_NAMES:
                name = BUTTON_NAMES[p]
                if name == "TOP":
                    linear, angular = lin_s, 0.0
                elif name == "DOWN":
                    linear, angular = -lin_s, 0.0
                elif name == "LEFT":
                    linear, angular = 0.0, ang_s
                elif name == "RIGHT":
                    linear, angular = 0.0, -ang_s
                else:  # CENTRE
                    linear, angular = 0.0, 0.0
                label = name
            else:
                label, linear, angular = f"UNKNOWN/COMBO (P:{p})", 0.0, 0.0
        else:
            label, linear, angular = f"UNKNOWN FRAME ({msg!r})", 0.0, 0.0

        with self._lock:
            self._label = label
            self._linear = linear
            self._angular = angular
            self._last_frame_time = now


class RemoteCmdVelPublisher(Node):
    def __init__(self, reader, rate, watchdog_timeout, frame_id):
        super().__init__("remote_cmd_vel_publisher")
        self._reader = reader
        self._watchdog_timeout = watchdog_timeout
        self._frame_id = frame_id
        self._warned_stale = False
        self._pub = self.create_publisher(TwistStamped, "/cmd_vel_remote", 10)
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        label, linear, angular, last_frame_time = self._reader.snapshot()

        stale = last_frame_time is None or (time.time() - last_frame_time) > self._watchdog_timeout
        if stale:
            if last_frame_time is not None and not self._warned_stale:
                self.get_logger().warn("No remote frame received recently, stopping for safety.")
                self._warned_stale = True
            linear, angular = 0.0, 0.0
            label = "NO SIGNAL (stopped)" if last_frame_time is not None else label
        else:
            self._warned_stale = False

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self._pub.publish(msg)

        self.get_logger().info(f"remote={label} -> linear.x={linear:.3f} angular.z={angular:.3f}", throttle_duration_sec=1.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--remote-device", default=DEFAULT_REMOTE_DEVICE,
                     help="RF remote's serial port (default: auto-detect /dev/ttyUSB*)")
    ap.add_argument("--remote-baud", type=int, default=DEFAULT_REMOTE_BAUD)
    ap.add_argument("--rate", type=float, default=20.0, help="Publish rate in Hz (default 20)")
    ap.add_argument("--linear-speed", type=float, default=DEFAULT_LINEAR_SPEED,
                     help=f"linear.x magnitude (m/s) for Top/Down (default {DEFAULT_LINEAR_SPEED})")
    ap.add_argument("--angular-speed", type=float, default=DEFAULT_ANGULAR_SPEED,
                     help=f"angular.z magnitude (rad/s) for Left/Right (default {DEFAULT_ANGULAR_SPEED})")
    ap.add_argument("--watchdog-timeout", type=float, default=1.5,
                     help="Stop if no remote frame arrives for this many seconds (default 1.5; "
                          "must be > 1.0s since the remote's own idle heartbeat is 1 Hz, or every "
                          "idle gap falsely triggers this)")
    ap.add_argument("--frame-id", default="", help="header.frame_id for published TwistStamped messages")
    args, ros_args = ap.parse_known_args()

    remote_port = args.remote_device or find_remote_port()
    print(f"Reading remote on {remote_port} @ {args.remote_baud} 8N1.")

    reader = RemoteReader(remote_port, args.remote_baud, args.linear_speed, args.angular_speed)
    reader.start()

    rclpy.init(args=ros_args)
    node = RemoteCmdVelPublisher(reader, args.rate, args.watchdog_timeout, args.frame_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        reader.stop()


if __name__ == "__main__":
    main()
