#!/usr/bin/env python3
"""Web HMI - 2D canvas pose view + status/mode as indicators (2026-10-01).

Replaces the earlier VNC-streamed-RViz2 embed entirely. That approach kept
biting us: RViz2 and x11vnc silently died and left the page stuck
"connecting" forever; then the self-healing watchdog fought the user by
relaunching RViz2 the instant they tried to close its window. It was also
never genuinely "web" - it was a native Qt/Ogre process screen-scraped over
VNC. This version is honest, dependency-free web tech instead: no native
window, no VNC, nothing to crash or fight with.

Subscribes to:
  - /mcu_status (diagnostic_msgs/DiagnosticArray, from diffbot_system.cpp)
  - /hmi/cmd_vel_active_source (std_msgs/String, from cmd_vel_mux.py)
  - /diffbot_base_controller/odom (nav_msgs/Odometry, from diff_drive_controller)
      -> served as /pose.json, drawn as a live arrow on a world-fixed,
         pannable/zoomable HTML5 <canvas> (RViz-style "north-up" grid; the
         robot moves within it, the view doesn't recentre on its own).
Publishes:
  - /hmi/cmd_vel_mode (std_msgs/String) - "teleop" or "none", purely for this
    page's own keyboard/joystick on-off state (2026-10-01: simplified from a
    3-way "drive mode" selector - real twist_mux, unlike the old cmd_vel_mux.py
    this originally targeted, has no concept of a selected mode at all; it
    only ever does fixed-priority arbitration on whichever cmd_vel_* topics
    are actually publishing. "Remote" and "Navigation" buttons never gated
    anything even before this change, so they're gone - see the "RF remote"/
    "Navigation" panels in the HTML below, now always-visible reference info
    instead of fake toggles).
  - /goal_pose (geometry_msgs/PoseStamped) - click-and-drag on the canvas
    sets a goal position + heading (drag direction = heading), published in
    the odom frame since there's no map/localization yet. Nav2 isn't running
    on the real robot yet either, so this currently publishes into the void,
    but it's wired up and ready for when both exist. Always active, not
    gated by any mode selection.
  - /cmd_vel_teleop (geometry_msgs/Twist, plain - not TwistStamped, see
    publish_teleop()'s comment) - keyboard teleop straight from the browser
    (2026-10-01): the exact same u/i/o/j/k/l/m/,/. bindings as
    `ros2 run teleop_twist_keyboard`, captured by the page's own keydown/
    keyup handlers and POSTed to /teleop, instead of asking the user to run
    that node in a separate terminal. Only active while keyboard control is
    toggled on. See TELEOP_PANEL in the JS below for the exact keys.

There's no LIDAR or map yet, so the canvas only draws a grid + the live pose
+ a breadcrumb trail for now. Known gap: no /map or /scan subscription yet -
add nav_msgs/OccupancyGrid and sensor_msgs/LaserScan subscriptions to
HmiNode + matching canvas draw layers once a LIDAR is integrated.

Usage:
    source /opt/ros/humble/setup.bash
    source <workspace>/install/setup.bash   # for message bindings
    python3 status_web_server.py [--port 8080]

    Then open http://localhost:8080 - works alongside any ros2_control
    launch (diffbot.launch.py / diffbot_no_ekf.launch.py) for /mcu_status and
    pose, and alongside cmd_vel_mux.py for mode selection to have any effect.
"""
import argparse
import base64
import json
import math
import os
import subprocess
import sys
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped, Vector3
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

_LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
_VALID_MODES = {"none", "teleop"}  # the only two states the page itself can toggle - see setMode()'s comment
_LATCHED_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
_MAPS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps")


def _valid_map_name(name: str) -> bool:
    return bool(name) and len(name) <= 40 and name.replace("_", "").replace("-", "").isalnum()


def _list_maps():
    if not os.path.isdir(_MAPS_DIR):
        return []
    return sorted(f[:-5] for f in os.listdir(_MAPS_DIR) if f.endswith(".yaml"))


def _save_map(payload: dict) -> str:
    """Writes an edited map as a standard map_server pair (YAML + binary PGM).
    Refuses to overwrite an existing map name."""
    name = payload["name"]
    if not _valid_map_name(name):
        raise ValueError("map name: letters, digits, _ and - only")
    w, h = int(payload["width"]), int(payload["height"])
    data = base64.b64decode(payload["data"])
    if len(data) != w * h:
        raise ValueError("pixel data does not match width x height")
    if os.path.exists(os.path.join(_MAPS_DIR, name + ".yaml")):
        raise FileExistsError(f"map '{name}' already exists - choose another name")
    os.makedirs(_MAPS_DIR, exist_ok=True)
    resolution = float(payload["resolution"])
    ox, oy = (float(v) for v in payload["origin"][:2])
    with open(os.path.join(_MAPS_DIR, name + ".pgm"), "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii") + data)
    with open(os.path.join(_MAPS_DIR, name + ".yaml"), "w") as f:
        f.write(
            f"image: {name}.pgm\nmode: trinary\nresolution: {resolution}\n"
            f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
    return name


def _load_map(name: str) -> dict:
    """Reads a saved map (ROS map_server YAML + binary P5 PGM) for the web UI."""
    with open(os.path.join(_MAPS_DIR, name + ".yaml")) as f:
        meta = {}
        for line in f:
            if ":" in line and not line.lstrip().startswith("#"):
                key, value = line.split(":", 1)
                meta[key.strip()] = value.strip()
    resolution = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"].strip("[]").split(",")]

    with open(os.path.join(_MAPS_DIR, name + ".pgm"), "rb") as f:
        raw = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b"#":
            pos = raw.index(b"\n", pos) + 1
            continue
        start = pos
        while not raw[pos:pos + 1].isspace():
            pos += 1
        tokens.append(raw[start:pos])
    pos += 1  # single whitespace after maxval
    magic, width, height, _maxval = tokens
    if magic != b"P5":
        raise ValueError("not a binary PGM map")
    width, height = int(width), int(height)
    data = raw[pos:pos + width * height]
    if len(data) != width * height:
        raise ValueError("map file is shorter than its header says")
    return {
        "name": name,
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": origin,
        "data": base64.b64encode(data).decode("ascii"),
    }

_status_lock = threading.Lock()
_latest_status = {"received": False}

_active_lock = threading.Lock()
_latest_active = {"received": False}

_pose_lock = threading.Lock()
_latest_pose = {"received": False}

_scan_lock = threading.Lock()
_latest_scan = {"received": False}
# Sensor data convention (2026-10-02) - LaserScan publishers (bluesea2
# included) commonly use BEST_EFFORT reliability; a default RELIABLE
# subscription wouldn't connect at all (same class of silent QoS/type
# mismatch hit earlier with cmd_vel_teleop/cmd_vel_remote - topic name
# alone isn't enough, the whole profile has to be compatible).
_SCAN_QOS = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
_live_map_lock = threading.Lock()
_latest_live_map = None  # latest /map from SLAM, already in PGM pixel values, rows top-first


def _level_to_int(level) -> int:
    """DiagnosticStatus.level is a ROS 'byte' - rclpy hands it back as a
    length-1 bytes object in some message versions, a plain int in others."""
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else 0
    return int(level)


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class HmiNode(Node):
    def __init__(self):
        super().__init__("web_hmi_bridge")
        self.create_subscription(DiagnosticArray, "/mcu_status", self._on_status, 10)
        self.create_subscription(String, "/hmi/cmd_vel_active_source", self._on_active, _LATCHED_QOS)
        self.create_subscription(Odometry, "/diffbot_base_controller/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan_filtered", self._on_scan, _SCAN_QOS)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 1)
        self._mode_pub = self.create_publisher(String, "/hmi/cmd_vel_mode", _LATCHED_QOS)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        # Plain Twist (2026-10-01), not TwistStamped - twist_mux (the sole
        # subscriber on this topic) only supports plain geometry_msgs/Twist
        # inputs in the installed version (4.3.0), no stamped variant. A
        # type mismatch here means the two endpoints never connect at all
        # (ROS2 requires an exact type match, topic name alone isn't
        # enough) - confirmed on hardware: keyboard teleop silently
        # produced zero motion despite the mode/topic/subscriber count all
        # looking correct, because of exactly this.
        self._teleop_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self._teleop_lock = threading.Lock()
        self._teleop_session = None
        self._teleop_seq = -1
        # Software-only "EMERGENCY STOP" (2026-10-02) - see twist_mux.yaml's
        # software_stop entry for exactly what this does and doesn't
        # guarantee (not a real E-stop - no firmware involved, nothing
        # happens if ROS2/the network is down).
        self._estop_pub = self.create_publisher(Twist, "/cmd_vel_estop", 10)
        # Shared speed limits for keyboard/joystick (this page) and the RF
        # remote (remote_ros_node.py subscribes to this topic). x = max linear
        # m/s, y = max angular rad/s.
        self._speed_pub = self.create_publisher(Vector3, "/hmi/teleop_speed", _LATCHED_QOS)
        self.speed_limits = (0.5, 1.0)
        self.publish_speed(*self.speed_limits)
        self.get_logger().info("Subscribed to /mcu_status, /hmi/cmd_vel_active_source, /diffbot_base_controller/odom")

    def _on_status(self, msg: DiagnosticArray):
        if not msg.status:
            return
        status = msg.status[0]
        level = _level_to_int(status.level)
        data = {
            "received": True,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "level": level,
            "level_name": _LEVEL_NAMES.get(level, f"UNKNOWN({level})"),
            "message": status.message,
            "hardware_id": status.hardware_id,
            "values": {kv.key: kv.value for kv in status.values},
        }
        with _status_lock:
            _latest_status.clear()
            _latest_status.update(data)

    def _on_active(self, msg: String):
        with _active_lock:
            _latest_active.clear()
            _latest_active.update({"received": True, "source": msg.data})

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        data = {
            "received": True,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame_id": msg.header.frame_id or "odom",
            "x": p.x, "y": p.y, "theta": _quat_to_yaw(msg.pose.pose.orientation),
        }
        with _pose_lock:
            _latest_pose.clear()
            _latest_pose.update(data)

    def get_map_to_odom(self):
        """The correction AMCL produces once localized - None before that. Composed
        onto the odom-frame pose (see pollPose() in the page) so the robot, its
        outline, and the scan all draw aligned with the saved map's own frame."""
        try:
            t = self._tf_buffer.lookup_transform("map", "odom", Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        q = t.transform.rotation
        return {
            "x": t.transform.translation.x,
            "y": t.transform.translation.y,
            "theta": 2.0 * math.atan2(q.z, q.w),
        }

    def _on_scan(self, msg: LaserScan):
        # Ranges only, rounded - angle_min/increment let the browser derive
        # each point's angle itself rather than sending it per-point. inf/nan
        # (no return) become null so the client can skip them cheaply.
        ranges = [
            None if (math.isinf(r) or math.isnan(r)) else round(r, 3)
            for r in msg.ranges
        ]
        data = {
            "received": True,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame_id": msg.header.frame_id or "laser",
            "angle_min": msg.angle_min,
            "angle_increment": msg.angle_increment,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "ranges": ranges,
        }
        with _scan_lock:
            _latest_scan.clear()
            _latest_scan.update(data)

    def publish_mode(self, mode: str):
        self._mode_pub.publish(String(data=mode))

    def _on_map(self, msg: OccupancyGrid):
        global _latest_live_map
        w, h = msg.info.width, msg.info.height
        rows = []
        for r in reversed(range(h)):  # OccupancyGrid row 0 is the bottom; PGM row 0 is the top
            row = msg.data[r * w:(r + 1) * w]
            rows.append(bytes(205 if v < 0 else (0 if v >= 65 else 254) for v in row))
        with _live_map_lock:
            _latest_live_map = {
                "width": w, "height": h, "resolution": msg.info.resolution,
                "origin": [msg.info.origin.position.x, msg.info.origin.position.y, 0.0],
                "data": b"".join(rows),
            }

    def publish_speed(self, linear: float, angular: float):
        linear = min(max(linear, 0.1), 1.0)
        angular = min(max(angular, 0.2), 2.0)
        self.speed_limits = (linear, angular)
        self._speed_pub.publish(Vector3(x=linear, y=angular))
        return self.speed_limits

    def publish_initial_pose(self, x: float, y: float, theta: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        # Same defaults RViz's "2D Pose Estimate" tool uses.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        self._initial_pose_pub.publish(msg)
        self.get_logger().info(f"Published /initialpose: x={x:.2f} y={y:.2f} theta={theta:.2f}")

    def publish_goal(self, x: float, y: float, theta: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # the canvas's x,y are map-frame once AMCL is localized (see pollPose)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        self._goal_pub.publish(msg)
        self.get_logger().info(f"Published /goal_pose: x={x:.2f} y={y:.2f} theta={theta:.2f}")

    def publish_teleop(self, linear: float, angular: float, session=None, seq=None):
        # Commands from the browser carry a per-page session id and an
        # increasing sequence number. Over a laggy link, older motion commands
        # can arrive after a newer stop; dropping anything older than the newest
        # accepted command stops that from restarting motion.
        with self._teleop_lock:
            if session is not None and seq is not None:
                if session != self._teleop_session:
                    self._teleop_session = session
                    self._teleop_seq = -1
                if seq <= self._teleop_seq:
                    return False
                self._teleop_seq = seq
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self._teleop_pub.publish(msg)
            return True

    def publish_estop(self):
        # Always zero - this is a stop button, not a drive command. The
        # client resends this repeatedly while engaged (see /estop below);
        # it must stop resending to actually release, same as remote_ros_node
        # not publishing while idle - twist_mux keeps blocking lower-priority
        # sources for as long as fresh messages keep arriving here.
        self._estop_pub.publish(Twist())


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Calixto AMR - HMI</title>
<style>
  :root { color-scheme: light; }
  * { box-sizing: border-box; }
  html, body {
    height: 100%; margin: 0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: #f5f6f8; color: #1a1d24;
  }
  body { display: flex; flex-direction: column; }
  header.topbar {
    display: flex; align-items: center; justify-content: space-between;
    padding: 0.6rem 1rem; border-bottom: 1px solid #dfe2e7; flex: 0 0 auto; background: #ffffff;
  }
  header.topbar h1 { font-size: 1.05rem; margin: 0; font-weight: 600; }
  .badges { display: flex; gap: 0.5rem; flex-wrap: wrap; align-items: center; }
  .battery { position: relative; width: 96px; height: 24px; border: 2px solid #5a5f66; border-radius: 4px; background: #ffffff; overflow: hidden; }
  .battery::after { content: ''; position: absolute; right: -6px; top: 6px; width: 4px; height: 8px; background: #5a5f66; border-radius: 0 2px 2px 0; }
  .battery-fill { position: absolute; left: 0; top: 0; bottom: 0; width: 0; background: #2e9d4f; transition: width 0.6s, background 0.6s; }
  .battery.low .battery-fill { background: #e0a030; }
  .battery.critical .battery-fill { background: #d64541; }
  .battery-pct { position: absolute; inset: 0; display: flex; align-items: center; justify-content: center; font-size: 0.8rem; font-weight: 600; color: #1d2026; mix-blend-mode: difference; }
  .badge {
    padding: 0.35rem 0.7rem; border-radius: 999px; font-size: 0.8rem; font-weight: 600;
    display: inline-flex; align-items: center; gap: 0.35rem; white-space: nowrap;
  }
  .badge::before { content: "\\25CF"; font-size: 0.7rem; }
  .badge.ok      { background: #e6f4ea; color: #1e7e34; }
  .badge.warn    { background: #fff6da; color: #93690a; }
  .badge.error   { background: #fde8e8; color: #c0392b; }
  .badge.unknown { background: #eceef1; color: #6b7280; }

  .layout { flex: 1 1 auto; display: grid; grid-template-columns: 1fr 340px; gap: 0; min-height: 0; }
  @media (max-width: 900px) { .layout { grid-template-columns: 1fr; } }

  main.map-main { position: relative; background: #eef0f3; min-height: 360px; }
  #map-canvas { position: absolute; inset: 0; width: 100%; height: 100%; cursor: crosshair; touch-action: none; }
  .map-hint {
    position: absolute; left: 0.8rem; bottom: 0.6rem; font-size: 0.78rem; color: #555;
    background: rgba(255,255,255,0.85); padding: 0.3rem 0.6rem; border-radius: 6px; pointer-events: none;
    border: 1px solid #dfe2e7;
  }
  #recenter-btn {
    position: absolute; right: 0.8rem; bottom: 0.6rem; font-size: 0.78rem;
    padding: 0.35rem 0.7rem; border-radius: 6px; border: 1px solid #dfe2e7;
    background: rgba(255,255,255,0.9); color: #1a1d24; cursor: pointer;
  }
  #recenter-btn:hover { background: #ffffff; }
  .joystick-base {
    position: absolute; left: 1.2rem; bottom: 1.2rem; width: 130px; height: 130px;
    border-radius: 50%; background: rgba(255,255,255,0.6); border: 2px solid #c7cbd1;
    touch-action: none; user-select: none; display: none;
  }
  .joystick-knob {
    position: absolute; width: 56px; height: 56px; border-radius: 50%; background: #2f6fed;
    left: 50%; top: 50%; transform: translate(-50%, -50%); box-shadow: 0 2px 6px rgba(0,0,0,0.25);
  }

  aside.sidebar { overflow-y: auto; padding: 1rem; border-left: 1px solid #dfe2e7; background: #ffffff; }
  aside.sidebar h2 {
    font-size: 0.8rem; text-transform: uppercase; letter-spacing: 0.04em; color: #6b7280;
    margin: 1.4rem 0 0.5rem; font-weight: 600;
  }
  aside.sidebar h2:first-child { margin-top: 0; }
  .amr-status-box {
    padding: 1.1rem 1.1rem; border-radius: 10px; font-size: 0.95rem; font-weight: 600;
    min-height: 4.5rem; line-height: 1.6; white-space: pre-line;
  }
  .amr-status-box.ok      { background: #e6f4ea; color: #1e7e34; }
  .amr-status-box.warn    { background: #fff6da; color: #93690a; }
  .amr-status-box.error   { background: #fde8e8; color: #c0392b; }
  .amr-status-box.unknown { background: #eceef1; color: #6b7280; }
  .modes { display: flex; gap: 0.4rem; flex-wrap: wrap; }
  button {
    font: inherit; padding: 0.45rem 0.75rem; border-radius: 8px; border: 1px solid #d8dbe0;
    background: #f7f8fa; color: #1a1d24; cursor: pointer; font-size: 0.85rem;
  }
  button:hover { background: #eceef1; }
  button.active { border-color: #2f6fed; color: #2f6fed; background: #eaf1fe; }
  .estop-btn {
    width: 100%; padding: 0.9rem; font-size: 1.1rem; font-weight: 700; letter-spacing: 0.03em;
    border-radius: 10px; border: 2px solid #b91c1c; background: #dc2626; color: #fff;
  }
  .estop-btn:hover { background: #c81e1e; }
  .estop-btn.engaged { background: #7f1d1d; border-color: #fff; animation: estop-pulse 1s infinite; }
  @keyframes estop-pulse { 50% { opacity: 0.75; } }
  .hint { font-size: 0.78rem; color: #6b7280; margin-top: 0.4rem; line-height: 1.4; }
  .mode-panel {
    margin-top: 0.6rem; padding: 0.7rem 0.8rem; border-radius: 8px; font-size: 0.85rem;
    background: #f7f8fa; border: 1px solid #eceef1; line-height: 1.5;
  }
  .modal { position: fixed; inset: 0; background: rgba(20, 22, 26, 0.45); display: flex; align-items: center; justify-content: center; z-index: 50; }
  .modal[hidden] { display: none; }
  .modal-box { background: #ffffff; border-radius: 8px; padding: 1rem; min-width: 320px; max-width: 90vw; }
  .modal-head { display: flex; justify-content: space-between; align-items: center; margin-bottom: 0.5rem; }
  .topleft-panel { position: absolute; top: 10px; left: 10px; z-index: 5; width: 240px; background: rgba(255,255,255,0.95); padding: 8px; border-radius: 6px; box-shadow: 0 1px 4px rgba(0,0,0,0.15); }
  .tabs { display: flex; gap: 4px; margin-bottom: 0.75rem; }
  .tab { flex: 1; padding: 0.4rem 0.3rem; }
  .tab.active { background: #2f6fed; color: #ffffff; border-color: #2f6fed; }
  .tabpanel[hidden] { display: none; }
  .editor-box { width: min(900px, 92vw); }
  .editor-tools { display: flex; flex-wrap: wrap; gap: 6px; align-items: center; margin-bottom: 0.5rem; }
  .etool.active { background: #2f6fed; color: #ffffff; border-color: #2f6fed; }
  #nav-pose-mode-btn.active { background: #2f6fed; color: #ffffff; border-color: #2f6fed; }
  #editor-canvas { display: block; width: min(720px, 80vw); height: 480px; border: 1px solid #d8dbe0; cursor: crosshair; touch-action: none; }
  .editor-save { display: flex; gap: 8px; align-items: center; margin-top: 0.5rem; }
  .logo { height: 44px; margin-right: 12px; vertical-align: middle; }
  .speedctl label { display: block; margin: 0.4rem 0; }
  .speedctl input[type=range] { width: 60%; vertical-align: middle; }
  .keygrid {
    display: grid; grid-template-columns: repeat(3, 2.6rem); gap: 0.3rem;
    font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 1rem;
    text-align: center; margin: 0.5rem 0;
  }
  .keygrid span { background: #eceef1; border: 1px solid #d8dbe0; border-radius: 6px; padding: 0.4rem 0; }
  .keygrid span.active { background: #2f6fed; border-color: #2f6fed; color: #fff; }
  .remote-map td:first-child { font-weight: 600; color: #2f6fed; }
  code { font-size: 0.78rem; background: #eceef1; padding: 0.1rem 0.3rem; border-radius: 4px; }
</style>
</head>
<body>
  <header class="topbar">
    <img class="logo" src="/assets/calixto-systems-logo.png" alt="Calixto Systems">
    <h1>Autonomous Mobile Robot</h1>
    <div class="badges">
      <div id="battery" class="battery" title="Battery (placeholder values until the BMS is connected)">
        <div class="battery-fill"></div><span class="battery-pct">--%</span>
      </div>
      <span id="pose-badge" class="badge unknown">Pose: unknown</span>
    </div>
  </header>

  <div class="layout">
    <main class="map-main">
      <div class="topleft-panel">
        <button id="estop-btn" class="estop-btn">EMERGENCY STOP</button>
        <div class="hint">Software stop only - does not cut motor power.</div>
        <div id="amr-status" class="amr-status-box unknown">No data yet</div>
        <div id="conn" class="hint">connecting...</div>
      </div>
      <canvas id="map-canvas"></canvas>
      <div class="map-hint">Mouse: left-drag sets goal, right-drag pans, scroll zooms &middot;
      Touch: drag pans, pinch zooms, long-press-then-drag sets a goal</div>
      <button id="recenter-btn">Center on robot</button>
      <div id="joystick-base" class="joystick-base"><div id="joystick-knob" class="joystick-knob"></div></div>
    </main>

    <aside class="sidebar">
      <div class="tabs">
        <button class="tab active" data-tab="control">Speed &amp; control</button>
        <button class="tab" data-tab="mapping">Mapping</button>
        <button class="tab" data-tab="navigation">Navigation</button>
      </div>

      <section class="tabpanel" data-panel="control">
        <div class="speedctl">
          <label>Max linear speed <input type="range" id="teleop-lin" min="0.1" max="1.0" step="0.05"> <span id="teleop-lin-val"></span> m/s
            <span class="hint" id="teleop-lin-rpm"></span></label>
          <label>Max angular speed <input type="range" id="teleop-ang" min="0.2" max="2.0" step="0.1"> <span id="teleop-ang-val"></span> rad/s</label>
          <div class="hint">Applies to keyboard, joystick and RF remote.</div>
        </div>
        <h2>Keyboard input</h2>
        <div class="modes">
          <button data-mode="teleop">Keyboard control: OFF</button>
        </div>
        <div id="mode-panel" class="mode-panel"></div>
        <div class="hint">RF remote: always active, highest priority.</div>
      </section>

      <section class="tabpanel" data-panel="mapping" hidden>
        <div class="speedctl">
          <label>Map name <input type="text" id="map-name" placeholder="e.g. lab_1" style="width:60%"></label>
          <div class="modes">
            <button onclick="mapping('start')">Start mapping</button>
            <button onclick="mapping('stop')">Save &amp; stop</button>
            <button onclick="mapping('discard')">Stop without saving</button>
          </div>
          <label>Show saved map <select id="map-select"><option value="">none</option></select></label>
          <div class="modes">
            <button id="map-edit-btn">Edit map</button>
          </div>
          <div id="map-status" class="hint"></div>
        </div>
      </section>

      <section class="tabpanel" data-panel="navigation" hidden>
        <div class="speedctl">
          <label>Map for navigation <select id="nav-map-select"><option value="">none</option></select></label>
          <div class="modes">
            <button id="nav-start-btn">Start navigation</button>
            <button id="nav-stop-btn">Stop navigation</button>
          </div>
          <div class="modes">
            <button id="nav-pose-mode-btn">Set initial pose</button>
          </div>
          <div class="hint">Click and drag on the map to set a goal. Drag direction sets heading.
          While "Set initial pose" is on, the next drag sets the robot's starting pose instead.</div>
          <div id="nav-status" class="hint"></div>
        </div>
        <div class="hint">Waypoints: coming next.</div>
      </section>
    </aside>

    <div id="editor-modal" class="modal" hidden>
      <div class="modal-box editor-box">
        <div class="modal-head">
          <strong>Map editor</strong>
          <button id="editor-close">Close</button>
        </div>
        <div class="editor-tools">
          <button class="etool active" data-tool="occupied">Brush (wall)</button>
          <button class="etool" data-tool="free">Eraser (free)</button>
          <button class="etool" data-tool="unknown">Unknown</button>
          <label>Brush radius (cells) <input type="range" id="editor-size" min="0" max="5" step="0.1" value="1"> <span id="editor-size-val">1.0</span></label>
          <button id="editor-undo">Undo</button>
          <button id="zoom-out">&minus;</button>
          <button id="zoom-in">+</button>
          <button id="zoom-fit">Fit</button>
          <span id="zoom-val" class="hint"></span>
        </div>
        <canvas id="editor-canvas"></canvas>
        <div class="editor-save">
          <label>Save as new map <input type="text" id="editor-name" placeholder="e.g. lab_edited"></label>
          <button id="editor-save">Save map</button>
        </div>
        <div id="editor-status" class="hint"></div>
      </div>
    </div>
  </div>

<script>
const poseBadge = document.getElementById('pose-badge');
const connEl = document.getElementById('conn');
async function mapping(action) {
  const name = document.getElementById('map-name').value.trim();
  const status = document.getElementById('map-status');
  if (action === 'stop' && !name) { status.textContent = 'Enter a map name to save.'; return; }
  const body = { action: action === 'discard' ? 'stop' : action, name: action === 'stop' ? name : '' };
  status.textContent = 'working...';
  try {
    const res = await fetch('/mapping', { method: 'POST', body: JSON.stringify(body) });
    const data = await res.json();
    status.textContent = data.detail || data.status;
  } catch (e) {
    status.textContent = 'Failed to reach the server.';
  }
}

document.querySelectorAll('.tab').forEach(btn => btn.addEventListener('click', () => {
  document.querySelectorAll('.tab').forEach(b => b.classList.toggle('active', b === btn));
  document.querySelectorAll('.tabpanel').forEach(p => { p.hidden = p.dataset.panel !== btn.dataset.tab; });
}));
const editorModal = document.getElementById('editor-modal');
document.getElementById('map-edit-btn').addEventListener('click', () => openEditor());
document.getElementById('editor-close').addEventListener('click', () => { editorModal.hidden = true; });
editorModal.addEventListener('click', (e) => { if (e.target === editorModal) editorModal.hidden = true; });


// ---- Map editor ----
// Pixel values follow map_server's trinary convention: 0 = occupied, 254 = free, 205 = unknown.
const PIXEL_FOR_TOOL = { occupied: 0, free: 254, unknown: 205 };
const ZOOM_MAX = 40;  // screen px per map cell
let edit = null;  // {name, width, height, resolution, origin, pixels, undo[], tool, size, src, view}
let editHover = null;  // last hovered screen point, for the brush preview
const editCanvas = document.getElementById('editor-canvas');
const editCtx = editCanvas.getContext('2d');
const editStatus = document.getElementById('editor-status');
const zoomVal = document.getElementById('zoom-val');

function editSize() {
  return { w: editCanvas.clientWidth, h: editCanvas.clientHeight };
}

function fitView() {
  const { w, h } = editSize();
  const scale = Math.min(w / edit.width, h / edit.height);
  edit.view = { scale, ox: (w - edit.width * scale) / 2, oy: (h - edit.height * scale) / 2 };
}

// Zoom by factor, keeping the screen point (cx, cy) fixed.
function editZoomAt(cx, cy, factor) {
  const v = edit.view;
  const scale = Math.min(ZOOM_MAX, Math.max(v.minScale, v.scale * factor));
  const k = scale / v.scale;
  v.ox = cx - (cx - v.ox) * k;
  v.oy = cy - (cy - v.oy) * k;
  v.scale = scale;
  renderEditor();
}

async function openEditor() {
  const name = document.getElementById('map-select').value;
  if (!name) { document.getElementById('map-status').textContent = 'Choose a saved map first.'; return; }
  const res = await fetch('/map.json?name=' + encodeURIComponent(name));
  const m = await res.json();
  if (m.status === 'error') { document.getElementById('map-status').textContent = m.detail; return; }
  const pixels = Uint8Array.from(atob(m.data), c => c.charCodeAt(0));
  const keep = edit ? { tool: edit.tool, size: edit.size } : { tool: 'occupied', size: 1.0 };
  edit = { name, width: m.width, height: m.height, resolution: m.resolution, origin: m.origin,
           pixels, undo: [], tool: keep.tool, size: keep.size, src: null, view: null };
  document.getElementById('editor-name').value = name + '_edited';
  editorModal.hidden = false;
  editStatus.textContent = `Editing ${name} (${m.width} x ${m.height}). Drag to paint. Wheel or pinch to zoom, right-drag to pan.`;
  requestAnimationFrame(() => {
    fitView();
    edit.view.minScale = edit.view.scale * 0.5;
    renderEditor();
  });
}

function buildSource() {
  const src = document.createElement('canvas');
  src.width = edit.width;
  src.height = edit.height;
  const sctx = src.getContext('2d');
  const img = sctx.createImageData(edit.width, edit.height);
  for (let i = 0; i < edit.pixels.length; i++) {
    img.data[4 * i] = img.data[4 * i + 1] = img.data[4 * i + 2] = edit.pixels[i];
    img.data[4 * i + 3] = 255;
  }
  sctx.putImageData(img, 0, 0);
  return src;
}

function renderEditor() {
  if (!edit || !edit.view) return;
  const dpr = window.devicePixelRatio || 1;
  const { w, h } = editSize();
  if (editCanvas.width !== Math.round(w * dpr) || editCanvas.height !== Math.round(h * dpr)) {
    editCanvas.width = Math.round(w * dpr);
    editCanvas.height = Math.round(h * dpr);
  }
  edit.src = buildSource();
  editCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  editCtx.fillStyle = '#eef0f3';
  editCtx.fillRect(0, 0, w, h);
  editCtx.imageSmoothingEnabled = false;
  const v = edit.view;
  editCtx.drawImage(edit.src, v.ox, v.oy, edit.width * v.scale, edit.height * v.scale);
  if (editHover && editPointers.size === 0) drawBrushPreview();
  zoomVal.textContent = Math.round(100 * v.scale / (v.minScale * 2)) + '%';
}

// Outline of the brush at its real size, so you can judge it against the walls.
// The label gives the brush's diameter in cm, from the map resolution.
function drawBrushPreview() {
  const v = edit.view;
  const cx = Math.floor((editHover.x - v.ox) / v.scale);
  const cy = Math.floor((editHover.y - v.oy) / v.scale);
  const centreX = v.ox + (cx + 0.5) * v.scale;
  const centreY = v.oy + (cy + 0.5) * v.scale;
  const radius = Math.max(edit.size, 0.5) * v.scale;
  const colour = { occupied: '#000000', free: '#1f9d55', unknown: '#7a7f87' }[edit.tool];
  editCtx.save();
  editCtx.lineWidth = 2;
  editCtx.strokeStyle = colour;
  editCtx.beginPath();
  editCtx.arc(centreX, centreY, radius, 0, 2 * Math.PI);
  editCtx.stroke();
  editCtx.fillStyle = colour;
  const cm = (2 * edit.size * edit.resolution * 100).toFixed(0);
  editCtx.font = '12px sans-serif';
  editCtx.fillText(`brush ${(2 * edit.size).toFixed(1)} cells = ${cm} cm`, centreX + radius + 6, centreY + 4);
  editCtx.restore();
}

function paintAt(x, y) {
  const v = edit.view;
  const cx = Math.floor((x - v.ox) / v.scale);
  const cy = Math.floor((y - v.oy) / v.scale);
  const value = PIXEL_FOR_TOOL[edit.tool];
  const r = edit.size;
  const reach = Math.ceil(r);
  for (let dy = -reach; dy <= reach; dy++) {
    for (let dx = -reach; dx <= reach; dx++) {
      if (dx * dx + dy * dy > r * r) continue;
      const px = cx + dx, py = cy + dy;
      if (px < 0 || py < 0 || px >= edit.width || py >= edit.height) continue;
      edit.pixels[py * edit.width + px] = value;
    }
  }
  renderEditor();
}

// Pointer state: one pointer paints (left button or touch), right/middle drag pans,
// two touch pointers pinch-zoom and pan together.
const editPointers = new Map();  // pointerId -> {x, y, button}
let editMode = null;             // 'paint' | 'pan' | 'pinch'
let pinchStart = null;           // {dist, mx, my}

function localXY(e) {
  const rect = editCanvas.getBoundingClientRect();
  return { x: e.clientX - rect.left, y: e.clientY - rect.top };
}

editCanvas.addEventListener('contextmenu', (e) => e.preventDefault());

editCanvas.addEventListener('pointerdown', (e) => {
  if (!edit || !edit.view) return;
  const p = localXY(e);
  editPointers.set(e.pointerId, { x: p.x, y: p.y, button: e.button });
  editCanvas.setPointerCapture(e.pointerId);
  if (editPointers.size === 1) {
    if (e.button === 1 || e.button === 2) {
      editMode = 'pan';
    } else {
      editMode = 'paint';
      edit.undo.push(edit.pixels.slice());
      if (edit.undo.length > 30) edit.undo.shift();
      paintAt(p.x, p.y);
    }
  } else if (editPointers.size === 2) {
    if (editMode === 'paint') {
      edit.pixels = edit.undo.pop();  // the first finger's stroke was only a tap; drop it
    }
    editMode = 'pinch';
    const [a, b] = [...editPointers.values()];
    pinchStart = { dist: Math.hypot(a.x - b.x, a.y - b.y), mx: (a.x + b.x) / 2, my: (a.y + b.y) / 2 };
  }
});

editCanvas.addEventListener('pointermove', (e) => {
  const cur = editPointers.get(e.pointerId);
  if (!edit || !edit.view) return;
  const p = localXY(e);
  if (!cur) {
    editHover = p;
    renderEditor();
    return;
  }
  const dx = p.x - cur.x, dy = p.y - cur.y;
  cur.x = p.x;
  cur.y = p.y;
  if (editMode === 'paint') {
    paintAt(p.x, p.y);
  } else if (editMode === 'pan') {
    edit.view.ox += dx;
    edit.view.oy += dy;
    renderEditor();
  } else if (editMode === 'pinch' && editPointers.size === 2) {
    const [a, b] = [...editPointers.values()];
    const dist = Math.hypot(a.x - b.x, a.y - b.y);
    const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
    if (pinchStart.dist > 0) editZoomAt(mx, my, dist / pinchStart.dist);
    edit.view.ox += mx - pinchStart.mx;
    edit.view.oy += my - pinchStart.my;
    pinchStart = { dist, mx, my };
    renderEditor();
  }
});

function endPointer(e) {
  editPointers.delete(e.pointerId);
  if (editPointers.size === 0) editMode = null;
  if (editPointers.size < 2) pinchStart = null;
}
editCanvas.addEventListener('pointerup', endPointer);
editCanvas.addEventListener('pointerleave', () => { editHover = null; renderEditor(); });
editCanvas.addEventListener('pointercancel', endPointer);

editCanvas.addEventListener('wheel', (e) => {
  if (!edit || !edit.view) return;
  e.preventDefault();
  const p = localXY(e);
  editZoomAt(p.x, p.y, Math.exp(-e.deltaY * 0.0015));
}, { passive: false });

document.getElementById('zoom-in').addEventListener('click', () => {
  if (!edit || !edit.view) return;
  const { w, h } = editSize();
  editZoomAt(w / 2, h / 2, 1.25);
});
document.getElementById('zoom-out').addEventListener('click', () => {
  if (!edit || !edit.view) return;
  const { w, h } = editSize();
  editZoomAt(w / 2, h / 2, 0.8);
});
document.getElementById('zoom-fit').addEventListener('click', () => {
  if (!edit) return;
  fitView();
  edit.view.minScale = edit.view.scale * 0.5;
  renderEditor();
});

document.querySelectorAll('.etool').forEach(btn => btn.addEventListener('click', () => {
  if (!edit) return;
  edit.tool = btn.dataset.tool;
  document.querySelectorAll('.etool').forEach(b => b.classList.toggle('active', b === btn));
}));
document.getElementById('editor-size').addEventListener('input', (e) => {
  if (!edit) return;
  edit.size = Number(e.target.value);
  document.getElementById('editor-size-val').textContent = edit.size.toFixed(1);
  renderEditor();
});
document.getElementById('editor-undo').addEventListener('click', () => {
  if (!edit || edit.undo.length === 0) return;
  edit.pixels = edit.undo.pop();
  renderEditor();
});

document.getElementById('editor-save').addEventListener('click', async () => {
  if (!edit) return;
  const name = document.getElementById('editor-name').value.trim();
  if (!name) { editStatus.textContent = 'Enter a name for the new map.'; return; }
  let bin = '';
  for (let i = 0; i < edit.pixels.length; i++) bin += String.fromCharCode(edit.pixels[i]);
  editStatus.textContent = 'saving...';
  try {
    const res = await fetch('/map/save', { method: 'POST', body: JSON.stringify({
      name, width: edit.width, height: edit.height, resolution: edit.resolution,
      origin: edit.origin, data: btoa(bin) }) });
    const data = await res.json();
    editStatus.textContent = data.status === 'ok'
      ? `Saved as ${data.saved}. It appears in the map list after a page reload.`
      : data.detail;
  } catch (e) {
    editStatus.textContent = 'Failed to reach the server.';
  }
});

// Battery indicator. Placeholder only: random walk until the BMS is wired in.
// To connect the real BMS, replace batteryPlaceholderPercent() with a read of
// the BMS value (for example a /battery_state topic, polled like the other status).
const batteryEl = document.getElementById('battery');
let batteryPct = 72;
function batteryPlaceholderPercent() {
  batteryPct = Math.min(100, Math.max(5, batteryPct + (Math.random() * 6 - 3)));
  return Math.round(batteryPct);
}
function renderBattery(pct) {
  batteryEl.querySelector('.battery-fill').style.width = pct + '%';
  batteryEl.querySelector('.battery-pct').textContent = pct + '%';
  batteryEl.classList.toggle('critical', pct < 15);
  batteryEl.classList.toggle('low', pct >= 15 && pct < 30);
}
renderBattery(batteryPlaceholderPercent());
setInterval(() => renderBattery(batteryPlaceholderPercent()), 3000);

const amrStatusEl = document.getElementById('amr-status');
const modeButtons = document.querySelectorAll('button[data-mode]');
const modePanelEl = document.getElementById('mode-panel');

// Software-only "EMERGENCY STOP" (2026-10-02) - see this button's own hint
// text in the HTML and twist_mux.yaml's software_stop entry for exactly
// what this does and doesn't guarantee. While engaged, keeps resending a
// zero-velocity command faster than twist_mux's 0.5s timeout so it keeps
// blocking every lower-priority source (teleop/remote/nav); stopping the
// resend on release is what lets twist_mux fall through to them again -
// same mechanism as remote_ros_node.py not publishing while idle.
const estopBtn = document.getElementById('estop-btn');
let estopEngaged = false;
let estopInterval = null;

function sendEstop() {
  fetch('/estop', { method: 'POST' }).catch(() => {});
}

estopBtn.addEventListener('click', () => {
  estopEngaged = !estopEngaged;
  estopBtn.classList.toggle('engaged', estopEngaged);
  estopBtn.textContent = estopEngaged ? 'STOP ENGAGED - CLICK TO RELEASE' : 'EMERGENCY STOP';
  if (estopEngaged) {
    sendEstop();
    estopInterval = setInterval(sendEstop, 150);
  } else {
    clearInterval(estopInterval);
    estopInterval = null;
  }
});

function levelClass(levelName) {
  if (levelName === 'OK') return 'ok';
  if (levelName === 'WARN') return 'warn';
  if (levelName === 'ERROR') return 'error';
  return 'unknown';
}

// AMR Status is deliberately a single clear-or-error line, not a checklist -
// diffbot_system.cpp already computes one summary message covering E-stop/
// bumper-latch/CAN-fault/comms-timeout/drive-fault, in that priority order,
// so there's no need to duplicate its per-field breakdown here too.
async function pollStatus() {
  try {
    const res = await fetch('/status.json', { cache: 'no-store' });
    const data = await res.json();
    if (!data.received) {
      connEl.textContent = 'Waiting for /mcu_status ...';
      amrStatusEl.className = 'amr-status-box unknown';
      amrStatusEl.textContent = 'No data yet';
      return;
    }
    const age = (Date.now() / 1000) - data.stamp;
    const stale = age > 2.0;
    connEl.textContent = `${data.hardware_id} - updated ${age.toFixed(1)}s ago` + (stale ? ' (STALE)' : '');

    const cls = stale ? 'unknown' : levelClass(data.level_name);
    amrStatusEl.className = 'amr-status-box ' + cls;
    amrStatusEl.textContent = stale ? 'Stack offline - no recent status update' :
      (data.level_name === 'OK' ? 'Everything clear' : data.message);
  } catch (e) {
    connEl.textContent = 'Cannot reach status server';
    amrStatusEl.className = 'amr-status-box unknown';
    amrStatusEl.textContent = 'Cannot reach status server';
  }
}

// Exact key bindings from ros2 run teleop_twist_keyboard teleop_twist_keyboard
// (its own printed banner) - i/,/j/l drive, u/o/m/. are diagonals, k or any
// other key stops.
const TELEOP_PANEL = `<div class="keygrid">
      <span data-key="u">u</span><span data-key="i">i</span><span data-key="o">o</span>
      <span data-key="j">j</span><span data-key="k">k</span><span data-key="l">l</span>
      <span data-key="m">m</span><span data-key=",">,</span><span data-key=".">.</span>
    </div>
    <div class="hint">k or release = stop.</div>
    `;

// One on/off toggle now, not a 3-way mode selector (2026-10-01) - "Remote"
// and "Navigation" never gated anything here even before this simplification
// (real twist_mux has no concept of a selected mode; it just arbitrates by
// fixed priority on whichever cmd_vel_* topics are actually publishing).
// The only thing this page can actually turn on/off is its own keyboard/
// joystick capture, so that's the only toggle left.
let currentMode = 'none';

async function setMode(mode) {
  if (currentMode === 'teleop' && mode !== 'teleop') {
    stopTeleopKey();  // leaving teleop - don't leave a key "stuck" driving in the background
    endJoystick();     // same for the joystick, if it was mid-drag
  }
  currentMode = mode;
  const btn = modeButtons[0];
  btn.textContent = mode === 'teleop' ? 'Keyboard control: ON' : 'Keyboard control: OFF';
  btn.classList.toggle('active', mode === 'teleop');
  modePanelEl.innerHTML = mode === 'teleop' ? TELEOP_PANEL : '';
  syncTeleopSliders();
  setJoystickVisible(mode === 'teleop');
  try {
    await fetch('/mode', { method: 'POST', body: mode });
  } catch (e) {
    // Mode is UI-only state; a failed POST just leaves the server's last value.
  }
}
modeButtons.forEach(b => b.addEventListener('click', () => setMode(currentMode === 'teleop' ? 'none' : 'teleop')));

// ---- Keyboard teleop (only active while currentMode === 'teleop') ----
// Exact bindings from teleop_twist_keyboard's own moveBindings table
// (non-shift/non-holonomic subset): [linear.x sign, angular.z sign].
const TELEOP_BINDINGS = {
  i: [1, 0], o: [1, -1], j: [0, 1], l: [0, -1], u: [1, 1],
  ',': [-1, 0], '.': [-1, 1], m: [-1, -1],
};
// Defaults match teleop_twist_keyboard's own; the sliders in the teleop panel change them.
let teleopLinearMax = 0.5;   // m/s
let teleopAngularMax = 1.0;  // rad/s

// The slider elements are recreated every time the panel is rendered, so their
// values are re-applied from these variables after each render.
function syncTeleopSliders() {
  const lin = document.getElementById('teleop-lin');
  if (!lin) return;
  document.getElementById('teleop-ang').value = teleopAngularMax;
  lin.value = teleopLinearMax;
  document.getElementById('teleop-lin-val').textContent = teleopLinearMax.toFixed(2);
  document.getElementById('teleop-ang-val').textContent = teleopAngularMax.toFixed(1);
  // 17 cm wheel diameter, same as the ROS config's wheel_radius = 0.085 m.
  const rpm = teleopLinearMax / (Math.PI * 0.17) * 60;
  document.getElementById('teleop-lin-rpm').textContent = `(about ${rpm.toFixed(0)} wheel rpm)`;
}
document.addEventListener('input', (evt) => {
  if (evt.target.id === 'teleop-lin') teleopLinearMax = parseFloat(evt.target.value);
  else if (evt.target.id === 'teleop-ang') teleopAngularMax = parseFloat(evt.target.value);
  else return;
  syncTeleopSliders();
});
// Posted on release, not on every drag tick: the server republishes the value
// for the RF remote too, so this only needs to happen once per adjustment.
document.addEventListener('change', (evt) => {
  if (evt.target.id !== 'teleop-lin' && evt.target.id !== 'teleop-ang') return;
  fetch('/speed', { method: 'POST', body: JSON.stringify({ linear: teleopLinearMax, angular: teleopAngularMax }) })
    .then(r => r.json())
    .then(d => { teleopLinearMax = d.linear; teleopAngularMax = d.angular; syncTeleopSliders(); })
    .catch(() => {});
});
fetch('/speed.json').then(r => r.json()).then(d => {
  teleopLinearMax = d.linear;
  teleopAngularMax = d.angular;
  syncTeleopSliders();
}).catch(() => {});
let teleopKey = null;

// Every command is tagged with this page's session and an increasing sequence
// number, so the server can drop late, stale commands (see publish_teleop).
const teleopSession = Math.random().toString(36).slice(2);
let teleopSeq = 0;
function teleopPublish(linear, angular) {
  const seq = ++teleopSeq;
  fetch('/teleop', { method: 'POST', body: JSON.stringify({ linear, angular, session: teleopSession, seq }) })
    .catch(() => {});
}

function highlightTeleopKey() {
  document.querySelectorAll('.keygrid span').forEach(s => {
    s.classList.toggle('active', s.dataset.key === teleopKey);
  });
}

function stopTeleopKey() {
  if (teleopKey === null) return;
  teleopKey = null;
  highlightTeleopKey();
  teleopPublish(0, 0);
}

function isTypingTarget(el) {
  return el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA');
}

window.addEventListener('keydown', (evt) => {
  if (currentMode !== 'teleop' || isTypingTarget(document.activeElement) || evt.repeat) return;
  if (evt.key === 'k') { stopTeleopKey(); return; }
  const binding = TELEOP_BINDINGS[evt.key];
  if (!binding) return;
  evt.preventDefault();
  teleopKey = evt.key;
  highlightTeleopKey();
  teleopPublish(binding[0] * teleopLinearMax, binding[1] * teleopAngularMax);
});
window.addEventListener('keyup', (evt) => {
  if (evt.key === teleopKey) stopTeleopKey();
});
window.addEventListener('blur', stopTeleopKey);
// Resend the held key's command periodically so cmd_vel_mux's 0.5s teleop
// timeout never lapses just because the OS's own key-repeat rate is slower.
setInterval(() => {
  if (currentMode === 'teleop' && teleopKey && TELEOP_BINDINGS[teleopKey]) {
    const binding = TELEOP_BINDINGS[teleopKey];
    teleopPublish(binding[0] * teleopLinearMax, binding[1] * teleopAngularMax);
  }
}, 200);

// ---- Virtual joystick (touch or mouse, only shown in Keyboard mode) ----
// Same /teleop endpoint and speeds as keyboard teleop - just a second, more
// touch-friendly way to drive from a phone. Uses Pointer Events so the same
// code handles both mouse (desktop testing) and touch (phone) without
// separate handlers.
const joystickBase = document.getElementById('joystick-base');
const joystickKnob = document.getElementById('joystick-knob');
let joystickActive = false;
let joystickVec = [0, 0];  // normalized [x, y], each in [-1, 1]

function setJoystickVisible(visible) {
  joystickBase.style.display = visible ? 'block' : 'none';
}

function joystickVecFromEvent(evt) {
  const rect = joystickBase.getBoundingClientRect();
  const radius = rect.width / 2;
  let dx = evt.clientX - (rect.left + radius);
  let dy = evt.clientY - (rect.top + radius);
  const dist = Math.hypot(dx, dy);
  if (dist > radius) { dx = dx / dist * radius; dy = dy / dist * radius; }
  return [dx / radius, dy / radius];
}

function updateJoystickKnob(nx, ny) {
  joystickKnob.style.left = `${50 + nx * 50}%`;
  joystickKnob.style.top = `${50 + ny * 50}%`;
}

function joystickPublish() {
  const [nx, ny] = joystickVec;
  // Pushing the stick up (negative ny) drives forward; pushing right
  // (positive nx) turns right (angular.z negative, CW per REP-103).
  teleopPublish(-ny * teleopLinearMax, -nx * teleopAngularMax);
}

joystickBase.addEventListener('pointerdown', (evt) => {
  if (currentMode !== 'teleop') return;
  joystickBase.setPointerCapture(evt.pointerId);
  joystickActive = true;
  joystickVec = joystickVecFromEvent(evt);
  updateJoystickKnob(joystickVec[0], joystickVec[1]);
  joystickPublish();
});
joystickBase.addEventListener('pointermove', (evt) => {
  if (!joystickActive) return;
  joystickVec = joystickVecFromEvent(evt);
  updateJoystickKnob(joystickVec[0], joystickVec[1]);
  joystickPublish();
});
function endJoystick() {
  if (!joystickActive) return;
  joystickActive = false;
  joystickVec = [0, 0];
  updateJoystickKnob(0, 0);
  teleopPublish(0, 0);
}
joystickBase.addEventListener('pointerup', endJoystick);
joystickBase.addEventListener('pointercancel', endJoystick);
window.addEventListener('blur', endJoystick);
// Same resend reasoning as keyboard teleop - keep cmd_vel_mux's timeout fed
// while the stick is held still (pointermove doesn't fire without motion).
setInterval(() => {
  if (currentMode === 'teleop' && joystickActive) joystickPublish();
}, 200);

// ---- 2D canvas pose view ----
// World-fixed camera (map stays put, robot moves/rotates within it - like a
// normal map app), independently pannable/zoomable by the user. This is the
// opposite of an earlier version that recentred the view on the robot every
// frame; that made the robot look stationary with the world scrolling under
// it, which read as backwards.
const canvas = document.getElementById('map-canvas');
const ctx = canvas.getContext('2d');
const recenterBtn = document.getElementById('recenter-btn');
let latestPose = null;   // {x, y, theta} in odom frame
let latestScan = null;   // {angle_min, angle_increment, ranges[]} in the laser's own frame
let trail = [];          // recent [x, y] breadcrumb points
let drag = null;         // {startPx, startPy, curPx, curPy} while left-dragging a goal
let panLast = null;      // {px, py} last pointer position while right-dragging to pan
const view = { originX: 0, originY: 0, scale: 60, initialized: false };  // scale = px/meter

function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = rect.width * devicePixelRatio;
  canvas.height = rect.height * devicePixelRatio;
  canvas.style.width = rect.width + 'px';
  canvas.style.height = rect.height + 'px';
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// World (odom frame, ROS convention: +x forward, +y left) <-> canvas pixel.
// view.originX/Y is the world point currently at the canvas center (i.e. the
// camera's look-at point) - panning/zooming only ever change this and
// view.scale, never latestPose. North-up: world +x maps to screen "up".
function worldToPx(wx, wy) {
  const cx = canvas.width / 2, cy = canvas.height / 2;
  const scale = view.scale * devicePixelRatio;
  return [cx - (wy - view.originY) * scale, cy - (wx - view.originX) * scale];
}
function pxToWorld(px, py) {
  const cx = canvas.width / 2, cy = canvas.height / 2;
  const scale = view.scale * devicePixelRatio;
  return [view.originX + (cy - py) / scale, view.originY + (cx - px) / scale];
}
function centerOnRobot() {
  if (!latestPose) return;
  view.originX = latestPose.x;
  view.originY = latestPose.y;
}
recenterBtn.addEventListener('click', centerOnRobot);

let mapImage = null;  // saved map shown on the canvas: {canvas, width, height, res, ox, oy}

function buildMapImage(m) {
  const raw = Uint8Array.from(atob(m.data), c => c.charCodeAt(0));
  const off = document.createElement('canvas');
  off.width = m.width;
  off.height = m.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(m.width, m.height);
  for (let i = 0; i < raw.length; i++) {
    img.data[4 * i] = raw[i];
    img.data[4 * i + 1] = raw[i];
    img.data[4 * i + 2] = raw[i];
    img.data[4 * i + 3] = 255;
  }
  octx.putImageData(img, 0, 0);
  return { canvas: off, width: m.width, height: m.height, res: m.resolution, ox: m.origin[0], oy: m.origin[1] };
}

// While mapping runs, the SLAM map is drawn on the main canvas as it grows.
async function pollLiveMap() {
  try {
    const d = await (await fetch('/live_map.json', { cache: 'no-store' })).json();
    if (!d.active || !d.ready) return;
    mapImage = buildMapImage(d);
    document.getElementById('map-status').textContent = `Live map: ${d.width} x ${d.height} cells`;
  } catch (e) {}
}
setInterval(pollLiveMap, 2000);
pollLiveMap();

async function showMap(name) {
  if (!name) { mapImage = null; return; }
  const status = document.getElementById('map-status');
  try {
    const res = await fetch('/map.json?name=' + encodeURIComponent(name));
    const m = await res.json();
    if (m.status === 'error') { status.textContent = m.detail; mapImage = null; return; }
    mapImage = buildMapImage(m);
    status.textContent = `Showing ${name}: ${m.width} x ${m.height} cells at ${m.resolution} m`;
  } catch (e) {
    status.textContent = 'Failed to load map.';
    mapImage = null;
  }
}

// The image's rows run along world -y and its columns along world +x. The
// canvas is north-up with +x up, so the map is drawn through a transform
// that maps columns to screen-vertical and rows to screen-horizontal.
function drawMap() {
  if (!mapImage) return;
  const s = view.scale * devicePixelRatio;
  const [px0, py0] = worldToPx(mapImage.ox, mapImage.oy + mapImage.height * mapImage.res);
  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.setTransform(0, -mapImage.res * s, mapImage.res * s, 0, px0, py0);
  ctx.drawImage(mapImage.canvas, 0, 0);
  ctx.restore();
}

document.getElementById('map-select').addEventListener('change', (e) => showMap(e.target.value));
document.getElementById('nav-map-select').addEventListener('change', (e) => showMap(e.target.value));
fetch('/maps.json').then(r => r.json()).then(d => {
  const sel = document.getElementById('map-select');
  const navSel = document.getElementById('nav-map-select');
  for (const name of d.maps) {
    const opt = document.createElement('option');
    opt.value = name;
    opt.textContent = name;
    sel.appendChild(opt);
    navSel.appendChild(opt.cloneNode(true));
  }
}).catch(() => {});

let canvasClickMode = 'goal';  // 'goal' or 'initialpose' - toggled by nav-pose-mode-btn
document.getElementById('nav-pose-mode-btn').addEventListener('click', (e) => {
  canvasClickMode = canvasClickMode === 'goal' ? 'initialpose' : 'goal';
  e.target.classList.toggle('active', canvasClickMode === 'initialpose');
  e.target.textContent = canvasClickMode === 'initialpose' ? 'Click the map to set initial pose...' : 'Set initial pose';
});

async function sendNavigation(action) {
  const status = document.getElementById('nav-status');
  const map = document.getElementById('nav-map-select').value;
  status.textContent = 'working...';
  try {
    const res = await fetch('/navigation', { method: 'POST', body: JSON.stringify({ action, map }) });
    const data = await res.json();
    status.textContent = data.detail || data.status;
  } catch (e) {
    status.textContent = 'Failed to reach the server.';
  }
}
document.getElementById('nav-start-btn').addEventListener('click', () => {
  if (!document.getElementById('nav-map-select').value) {
    document.getElementById('nav-status').textContent = 'Choose a map first.';
    return;
  }
  sendNavigation('start');
});
document.getElementById('nav-stop-btn').addEventListener('click', () => sendNavigation('stop'));

// Robot footprint, from calixto-ros-bot.md: length 780 mm, width 480 mm.
// LiDAR: 80 mm behind the front edge, centred side to side -> 390 - 80 = 310 mm forward of centre.
// LIDAR_YAW is the LiDAR zero direction relative to robot forward (lidar.xacro: rpy yaw 3.14159).
const ROBOT_LENGTH = 0.78;
const ROBOT_WIDTH = 0.48;
const LIDAR_OFFSET = [0.31, 0.0];
const LIDAR_YAW = 0;

function drawRobotOutline() {
  if (!latestPose) return;
  const c = Math.cos(latestPose.theta), s = Math.sin(latestPose.theta);
  const toWorld = (x, y) => [latestPose.x + x * c - y * s, latestPose.y + x * s + y * c];
  const hl = ROBOT_LENGTH / 2, hw = ROBOT_WIDTH / 2;
  const corners = [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]];
  ctx.save();
  ctx.beginPath();
  corners.forEach(([x, y], i) => {
    const [px, py] = worldToPx(...toWorld(x, y));
    if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  });
  ctx.closePath();
  ctx.strokeStyle = '#e0a030';
  ctx.lineWidth = 2 * devicePixelRatio;
  ctx.stroke();
  const [lx, ly] = worldToPx(...toWorld(LIDAR_OFFSET[0], LIDAR_OFFSET[1]));
  ctx.fillStyle = '#1f9d55';
  ctx.beginPath();
  ctx.arc(lx, ly, 4 * devicePixelRatio, 0, 2 * Math.PI);
  ctx.fill();
  ctx.restore();
}

function draw() {
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#eef0f3';
  ctx.fillRect(0, 0, w, h);

  // Grid, 1m spacing, fixed in world space (pans/zooms with the view).
  const scale = view.scale * devicePixelRatio;
  const offX = ((w / 2) - ((view.originY * scale) % scale));
  const offY = ((h / 2) - ((view.originX * scale) % scale));
  ctx.strokeStyle = '#d5d9e0';
  ctx.lineWidth = 1;
  for (let x = offX % scale; x < w; x += scale) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
  }
  for (let y = offY % scale; y < h; y += scale) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }

  drawMap();
  drawRobotOutline();

  if (latestPose) {
    // Breadcrumb trail.
    ctx.fillStyle = '#9fb3d6';
    for (const [tx, ty] of trail) {
      const [px, py] = worldToPx(tx, ty);
      ctx.beginPath(); ctx.arc(px, py, 2 * devicePixelRatio, 0, 7); ctx.fill();
    }

    // LiDAR scan points, transformed into the world frame using the robot's
    // CURRENT pose (scan and pose arrive on separate polls, so this is a
    // "latest of each" overlay, not a timestamp-matched one - fine for a
    // bring-up sanity check, not a substitute for a real TF chain). Assumes
    // the laser's own 0deg points along the robot's forward +x - true until
    // an actual mount offset/static transform is set up.
    if (latestScan && latestScan.ranges) {
      ctx.fillStyle = 'rgba(224, 70, 50, 0.8)';
      const a0 = latestScan.angle_min, da = latestScan.angle_increment;
      const rmin = latestScan.range_min, rmax = latestScan.range_max;
      // Scan rays start at the LiDAR, not the robot centre.
      const lc = Math.cos(latestPose.theta), ls = Math.sin(latestPose.theta);
      const laserX = latestPose.x + LIDAR_OFFSET[0] * lc - LIDAR_OFFSET[1] * ls;
      const laserY = latestPose.y + LIDAR_OFFSET[0] * ls + LIDAR_OFFSET[1] * lc;
      for (let i = 0; i < latestScan.ranges.length; i++) {
        const r = latestScan.ranges[i];
        if (r === null || r < rmin || r > rmax) continue;
        const angle = latestPose.theta + LIDAR_YAW + a0 + i * da;
        const wx = laserX + r * Math.cos(angle);
        const wy = laserY + r * Math.sin(angle);
        const [px, py] = worldToPx(wx, wy);
        // 2026-10-08: radius 1.5 -> 3.0, the scan was hard to read on the map.
        ctx.beginPath(); ctx.arc(px, py, 3.0 * devicePixelRatio, 0, 7); ctx.fill();
      }
    }

    // Robot arrow at its actual world position; rotated by heading (canvas
    // rotate() is clockwise, world +x/forward maps to screen "up", so
    // rotation = -theta).
    const [px, py] = worldToPx(latestPose.x, latestPose.y);
    const size = 14 * devicePixelRatio;
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-latestPose.theta);
    ctx.fillStyle = '#2f6fed';
    ctx.beginPath();
    ctx.moveTo(0, -size);
    ctx.lineTo(size * 0.6, size * 0.7);
    ctx.lineTo(-size * 0.6, size * 0.7);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  } else {
    ctx.fillStyle = '#8b93a0';
    ctx.font = `${14 * devicePixelRatio}px sans-serif`;
    ctx.textAlign = 'center';
    ctx.fillText('Waiting for /diffbot_base_controller/odom ...', w / 2, h / 2);
  }

  // Goal drag preview.
  if (drag) {
    ctx.strokeStyle = '#e0a300';
    ctx.fillStyle = '#e0a300';
    ctx.lineWidth = 2 * devicePixelRatio;
    ctx.beginPath();
    ctx.arc(drag.startPx, drag.startPy, 5 * devicePixelRatio, 0, 7);
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(drag.startPx, drag.startPy);
    ctx.lineTo(drag.curPx, drag.curPy);
    ctx.stroke();
  }

  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

function canvasPoint(evt) {
  const rect = canvas.getBoundingClientRect();
  return [(evt.clientX - rect.left) * devicePixelRatio, (evt.clientY - rect.top) * devicePixelRatio];
}

canvas.addEventListener('contextmenu', (evt) => evt.preventDefault());

canvas.addEventListener('mousedown', (evt) => {
  const [px, py] = canvasPoint(evt);
  if (evt.button === 2) {
    panLast = { px, py };
  } else if (evt.button === 0) {
    drag = { startPx: px, startPy: py, curPx: px, curPy: py };
  }
});
canvas.addEventListener('mousemove', (evt) => {
  const [px, py] = canvasPoint(evt);
  if (panLast) {
    const scale = view.scale * devicePixelRatio;
    view.originX += (py - panLast.py) / scale;
    view.originY += (px - panLast.px) / scale;
    panLast = { px, py };
  } else if (drag) {
    drag.curPx = px; drag.curPy = py;
  }
});
// Shared by mouse-drag and touch long-press-drag goal setting.
async function publishGoalFromDrag(d) {
  const [gx, gy] = pxToWorld(d.startPx, d.startPy);
  const dx = d.curPx - d.startPx, dy = d.curPy - d.startPy;
  const dragDist = Math.hypot(dx, dy);
  // Drag vector is in canvas pixels (screen up = world +x, screen left =
  // world +y) - convert to a world heading; a negligible drag just faces
  // the goal's current heading (0).
  const theta = dragDist > 8 ? Math.atan2(-dx, -dy) : 0;
  const isPose = canvasClickMode === 'initialpose';
  const url = isPose ? '/initial_pose' : '/goal';
  const status = document.getElementById('nav-status');
  const body = JSON.stringify({ x: gx, y: gy, theta: theta });
  if (isPose) {
    // Resending is safe here: AMCL just takes the latest estimate, it does not
    // treat each message as a new request. This guards against a one-shot
    // message landing before AMCL's subscription has finished connecting.
    for (let i = 0; i < 5; i++) {
      try { await fetch(url, { method: 'POST', body }); } catch (e) { /* best-effort */ }
      if (i < 4) await new Promise(r => setTimeout(r, 250));
    }
  } else {
    // Exactly one send: bt_navigator treats every /goal_pose message as a new
    // navigation request and cancels whatever was running - resending here
    // would cancel and restart navigation repeatedly instead of helping it.
    try { await fetch(url, { method: 'POST', body }); } catch (e) { /* best-effort */ }
  }
  status.textContent = isPose
    ? `Initial pose sent: x=${gx.toFixed(2)} y=${gy.toFixed(2)}. Check "Pose:" above says "localized".`
    : `Goal sent: x=${gx.toFixed(2)} y=${gy.toFixed(2)}.`;
  if (isPose) {
    // One-shot: back to goal mode so a stray drag can't send another pose.
    canvasClickMode = 'goal';
    const btn = document.getElementById('nav-pose-mode-btn');
    btn.classList.remove('active');
    btn.textContent = 'Set initial pose';
  }
}

window.addEventListener('mouseup', (evt) => {
  if (panLast && evt.button === 2) { panLast = null; return; }
  if (!drag) return;
  const d = drag;
  drag = null;
  publishGoalFromDrag(d);
});

// Shared by the mouse wheel and touch pinch - zoom centered on a screen
// point (the world point under it stays under it).
function zoomAt(px, py, factor) {
  const [wx, wy] = pxToWorld(px, py);
  view.scale = Math.min(300, Math.max(10, view.scale * factor));
  const cx = canvas.width / 2, cy = canvas.height / 2;
  const newScale = view.scale * devicePixelRatio;
  view.originX = wx - (cy - py) / newScale;
  view.originY = wy - (cx - px) / newScale;
}

canvas.addEventListener('wheel', (evt) => {
  evt.preventDefault();
  const [px, py] = canvasPoint(evt);
  zoomAt(px, py, evt.deltaY < 0 ? 1.1 : (1 / 1.1));
}, { passive: false });

// ---- Touch gestures (phone/tablet) ----
// One finger: pans immediately once it moves past a small threshold: below
// that threshold, it's held as a "pending" tap in case a long-press follows.
// A long-press (350ms without moving) instead arms goal-placement, exactly
// like a mouse left-drag - drag before lifting sets the heading.
// Two fingers: pinch to zoom, centered on the midpoint between them.
const LONG_PRESS_MS = 350;
const PAN_THRESHOLD_PX = 12 * devicePixelRatio;
let touchState = null;
let longPressTimer = null;

function touchCanvasPoint(touch) {
  const rect = canvas.getBoundingClientRect();
  return [(touch.clientX - rect.left) * devicePixelRatio, (touch.clientY - rect.top) * devicePixelRatio];
}

canvas.addEventListener('touchstart', (evt) => {
  evt.preventDefault();
  clearTimeout(longPressTimer);
  if (evt.touches.length === 2) {
    const [t0, t1] = evt.touches;
    touchState = { mode: 'pinch', dist: Math.hypot(t1.clientX - t0.clientX, t1.clientY - t0.clientY) };
    drag = null;
    return;
  }
  if (evt.touches.length === 1) {
    const [px, py] = touchCanvasPoint(evt.touches[0]);
    touchState = { mode: 'pending', startPx: px, startPy: py };
    longPressTimer = setTimeout(() => {
      if (touchState && touchState.mode === 'pending') {
        touchState.mode = 'goal';
        drag = { startPx: px, startPy: py, curPx: px, curPy: py };
      }
    }, LONG_PRESS_MS);
  }
}, { passive: false });

canvas.addEventListener('touchmove', (evt) => {
  evt.preventDefault();
  if (!touchState) return;
  if (touchState.mode === 'pinch' && evt.touches.length === 2) {
    const [t0, t1] = evt.touches;
    const dist = Math.hypot(t1.clientX - t0.clientX, t1.clientY - t0.clientY);
    const rect = canvas.getBoundingClientRect();
    const midPx = ((t0.clientX + t1.clientX) / 2 - rect.left) * devicePixelRatio;
    const midPy = ((t0.clientY + t1.clientY) / 2 - rect.top) * devicePixelRatio;
    zoomAt(midPx, midPy, dist / touchState.dist);
    touchState.dist = dist;
    return;
  }
  if (evt.touches.length !== 1) return;
  const [px, py] = touchCanvasPoint(evt.touches[0]);
  if (touchState.mode === 'pending') {
    if (Math.hypot(px - touchState.startPx, py - touchState.startPy) > PAN_THRESHOLD_PX) {
      clearTimeout(longPressTimer);
      touchState.mode = 'pan';
      touchState.lastPx = px; touchState.lastPy = py;
    }
    return;
  }
  if (touchState.mode === 'pan') {
    const scale = view.scale * devicePixelRatio;
    view.originX += (py - touchState.lastPy) / scale;
    view.originY += (px - touchState.lastPx) / scale;
    touchState.lastPx = px; touchState.lastPy = py;
    return;
  }
  if (touchState.mode === 'goal' && drag) {
    drag.curPx = px; drag.curPy = py;
  }
}, { passive: false });

canvas.addEventListener('touchend', (evt) => {
  evt.preventDefault();
  clearTimeout(longPressTimer);
  if (touchState && touchState.mode === 'goal' && drag) {
    const d = drag;
    drag = null;
    publishGoalFromDrag(d);
  }
  if (evt.touches.length === 0) touchState = null;
}, { passive: false });

async function pollPose() {
  try {
    const res = await fetch('/pose.json', { cache: 'no-store' });
    const data = await res.json();
    if (!data.received) {
      poseBadge.className = 'badge unknown';
      poseBadge.textContent = 'Pose: unknown';
      latestPose = null;
      return;
    }
    // Pose/odom is in the odom frame. Once AMCL is localized, the server also
    // sends the map->odom correction; apply it so the robot, its outline, and
    // the scan draw aligned with the saved map (which is in the map frame).
    let x = data.x, y = data.y, theta = data.theta, frame = data.frame_id;
    if (data.map_to_odom) {
      const m = data.map_to_odom;
      const c = Math.cos(m.theta), s = Math.sin(m.theta);
      const mx = m.x + x * c - y * s, my = m.y + x * s + y * c;
      x = mx; y = my; theta = theta + m.theta; frame = 'map';
    }
    latestPose = { x, y, theta, stamp: data.stamp, frame_id: frame };
    if (!view.initialized) { centerOnRobot(); view.initialized = true; }
    trail.push([x, y]);
    if (trail.length > 400) trail.shift();
    const age = (Date.now() / 1000) - data.stamp;
    poseBadge.className = 'badge ' + (age > 1.0 ? 'unknown' : 'ok');
    poseBadge.textContent = `Pose: ${x.toFixed(2)}, ${y.toFixed(2)} (${frame}${data.map_to_odom ? ', localized' : ''})`;
  } catch (e) {
    poseBadge.className = 'badge unknown';
    poseBadge.textContent = 'Pose: unknown';
  }
}

async function pollScan() {
  try {
    const res = await fetch('/scan.json', { cache: 'no-store' });
    const data = await res.json();
    latestScan = data.received ? data : null;
  } catch (e) {
    latestScan = null;
  }
}

pollStatus();
pollPose();
pollScan();
setInterval(pollStatus, 500);
setInterval(pollPose, 150);
setInterval(pollScan, 200);
</script>
</body>
</html>
"""


class HmiHTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # keep the console quiet - this is a status page, not a web server demo

    def _send_json(self, obj, status=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/status.json":
            with _status_lock:
                data = dict(_latest_status)
            self._send_json(data)
        elif self.path == "/active.json":
            with _active_lock:
                data = dict(_latest_active)
            self._send_json(data)
        elif self.path == "/pose.json":
            with _pose_lock:
                data = dict(_latest_pose)
            if data.get("received"):
                data["map_to_odom"] = self.server.hmi_node.get_map_to_odom()
            self._send_json(data)
        elif self.path == "/scan.json":
            with _scan_lock:
                data = dict(_latest_scan)
            self._send_json(data)
        elif self.path == "/speed.json":
            lin, ang = self.server.hmi_node.speed_limits
            self._send_json({"linear": lin, "angular": ang})
        elif self.path == "/live_map.json":
            if not os.path.exists("/tmp/mapping.pids"):
                self._send_json({"active": False})
            else:
                with _live_map_lock:
                    m = _latest_live_map
                if m is None:
                    self._send_json({"active": True, "ready": False})
                else:
                    self._send_json({"active": True, "ready": True, "width": m["width"],
                                     "height": m["height"], "resolution": m["resolution"],
                                     "origin": m["origin"],
                                     "data": base64.b64encode(m["data"]).decode("ascii")})
        elif self.path == "/maps.json":
            self._send_json({"maps": _list_maps()})
        elif self.path.startswith("/map.json"):
            query = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            name = query.get("name", [""])[0]
            if not _valid_map_name(name) or not os.path.isfile(os.path.join(_MAPS_DIR, name + ".yaml")):
                self._send_json({"status": "error", "detail": "unknown map"}, status=404)
                return
            try:
                self._send_json(_load_map(name))
            except (OSError, ValueError, KeyError) as e:
                self._send_json({"status": "error", "detail": f"could not read map: {e}"}, status=500)
        elif self.path.startswith("/assets/"):
            fpath = os.path.join(_ASSETS_DIR, os.path.basename(self.path))
            if not os.path.isfile(fpath):
                self.send_response(404)
                self.end_headers()
                return
            with open(fpath, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path in ("/", "/index.html"):
            body = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b""

        if self.path == "/mode":
            body = raw.decode("utf-8").strip()
            if body not in _VALID_MODES:
                self._send_json({"status": "error", "detail": f"invalid mode '{body}'"}, status=400)
                return
            self.server.hmi_node.publish_mode(body)
            self._send_json({"status": "ok", "mode": body})
        elif self.path == "/goal":
            try:
                data = json.loads(raw.decode("utf-8"))
                x, y, theta = float(data["x"]), float(data["y"]), float(data["theta"])
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad goal payload: {e}"}, status=400)
                return
            self.server.hmi_node.publish_goal(x, y, theta)
            self._send_json({"status": "ok"})
        elif self.path == "/initial_pose":
            try:
                data = json.loads(raw.decode("utf-8"))
                x, y, theta = float(data["x"]), float(data["y"]), float(data["theta"])
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad initial pose payload: {e}"}, status=400)
                return
            self.server.hmi_node.publish_initial_pose(x, y, theta)
            self._send_json({"status": "ok"})
        elif self.path == "/navigation":
            try:
                data = json.loads(raw.decode("utf-8"))
                action = data["action"]
                map_name = data.get("map", "")
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad navigation payload: {e}"}, status=400)
                return
            docker_dir = "/ros2_ws/src/ros-amr/docker"
            if action == "start":
                if not _valid_map_name(map_name):
                    self._send_json({"status": "error", "detail": "choose a map first"}, status=400)
                    return
                cmd = ["bash", f"{docker_dir}/start_navigation.sh", map_name]
            elif action == "stop":
                cmd = ["bash", f"{docker_dir}/stop_navigation.sh"]
            else:
                self._send_json({"status": "error", "detail": f"unknown action '{action}'"}, status=400)
                return
            try:
                res = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
                detail = (res.stdout + res.stderr).strip().splitlines()[-1:] or ["done"]
                self._send_json({"status": "ok" if res.returncode == 0 else "error", "detail": detail[0]})
            except subprocess.TimeoutExpired:
                self._send_json({"status": "error", "detail": "navigation script timed out"}, status=500)
        elif self.path == "/teleop":
            try:
                data = json.loads(raw.decode("utf-8"))
                linear, angular = float(data["linear"]), float(data["angular"])
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad teleop payload: {e}"}, status=400)
                return
            session, seq = data.get("session"), data.get("seq")
            accepted = self.server.hmi_node.publish_teleop(linear, angular, session, seq)
            self._send_json({"status": "ok", "accepted": accepted})
        elif self.path == "/mapping":
            try:
                data = json.loads(raw.decode("utf-8"))
                action = data["action"]
                name = data.get("name", "")
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad mapping payload: {e}"}, status=400)
                return
            docker_dir = "/ros2_ws/src/ros-amr/docker"
            if action == "start":
                cmd = ["bash", f"{docker_dir}/start_mapping.sh"]
            elif action == "stop":
                if name and not (name.replace("_", "").replace("-", "").isalnum() and len(name) <= 40):
                    self._send_json({"status": "error", "detail": "map name: letters, digits, _ and - only"}, status=400)
                    return
                cmd = ["bash", f"{docker_dir}/stop_mapping.sh"] + (["--save", name] if name else [])
            else:
                self._send_json({"status": "error", "detail": f"unknown action '{action}'"}, status=400)
                return
            try:
                res = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
                detail = (res.stdout + res.stderr).strip().splitlines()[-1:] or ["done"]
                self._send_json({"status": "ok" if res.returncode == 0 else "error", "detail": detail[0]})
            except subprocess.TimeoutExpired:
                self._send_json({"status": "error", "detail": "mapping script timed out"}, status=500)
        elif self.path == "/map/save":
            try:
                data = json.loads(raw.decode("utf-8"))
                saved = _save_map(data)
            except FileExistsError as e:
                self._send_json({"status": "error", "detail": str(e)}, status=409)
                return
            except (ValueError, KeyError, TypeError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad map save: {e}"}, status=400)
                return
            self._send_json({"status": "ok", "saved": saved})
        elif self.path == "/speed":
            try:
                data = json.loads(raw.decode("utf-8"))
                lin, ang = float(data["linear"]), float(data["angular"])
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad speed payload: {e}"}, status=400)
                return
            new_lin, new_ang = self.server.hmi_node.publish_speed(lin, ang)
            self._send_json({"status": "ok", "linear": new_lin, "angular": new_ang})
        elif self.path == "/estop":
            self.server.hmi_node.publish_estop()
            self._send_json({"status": "ok"})
        else:
            self.send_response(404)
            self.end_headers()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    # parse_known_args(), not parse_args() - leaves standard ROS2 args
    # (--ros-args -r old:=new, etc.) for rclpy to consume below instead of
    # argparse rejecting them as unrecognized.
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = HmiNode()

    def _spin():
        try:
            rclpy.spin(node)
            print("[web_hmi] rclpy.spin() returned normally (unexpected)", file=sys.stderr, flush=True)
        except ExternalShutdownException:
            print("[web_hmi] ROS context externally shut down", file=sys.stderr, flush=True)
        except Exception as e:
            print(f"[web_hmi] ROS spin thread crashed: {e!r}", file=sys.stderr, flush=True)
        # rclpy.spin() only runs in this background thread - if the ROS
        # context gets shut down (e.g. SIGTERM), the main thread's
        # server.serve_forever() below has no way to know unless we force it
        # here too.
        os._exit(0)

    threading.Thread(target=_spin, daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), HmiHTTPHandler)
    server.hmi_node = node
    print(f"HMI web page: http://localhost:{args.port}  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
