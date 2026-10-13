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
import collections
import json
import math
import os
import subprocess
import sys
import threading
import time
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
import dock_manager
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from lifecycle_msgs.srv import GetState
try:
    from nav2_msgs.action import NavigateToPose
    _NAV_FEEDBACK = NavigateToPose.Impl.FeedbackMessage
except ImportError:  # nav2 not installed - status still works, no distance
    _NAV_FEEDBACK = None

_LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
_VALID_MODES = {"none", "teleop"}  # the only two states the page itself can toggle - see setMode()'s comment
_LATCHED_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
_MAPS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps")


def _valid_map_name(name: str) -> bool:
    return bool(name) and len(name) <= 40 and name.replace("_", "").replace("-", "").isalnum()


def _valid_waypoint_name(name: str) -> bool:
    # Looser than a map name (spaces, punctuation are fine - this is a label,
    # not a filename), just capped and restricted to plain printable ASCII so
    # it can't do anything odd landing in the JSON file or the page.
    return bool(name) and len(name) <= 60 and all(32 <= ord(c) < 127 for c in name)


def _waypoints_path(map_name: str) -> str:
    return os.path.join(_MAPS_DIR, map_name + ".waypoints.json")


def _load_waypoints(map_name: str) -> list:
    """Waypoints are saved per-map (2026-10-09) - name, x, y, theta - in
    maps/<map>.waypoints.json, alongside that map's own .yaml/.pgm, so they
    are always there whenever that map is loaded, on this robot or a copy of
    the repo, with no separate setup step."""
    path = _waypoints_path(map_name)
    if not os.path.isfile(path):
        return []
    try:
        with open(path, "r") as f:
            data = json.load(f)
        return data if isinstance(data, list) else []
    except (OSError, ValueError):
        return []


def _save_waypoints(map_name: str, waypoints: list):
    os.makedirs(_MAPS_DIR, exist_ok=True)
    path = _waypoints_path(map_name)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(waypoints, f, indent=2)
    os.replace(tmp, path)  # atomic - a reader never sees a half-written file


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

# Readiness (2026-10-12) - one place that answers "can I set the initial pose /
# drive yet?". Before this, the page had no such indicator: poses sent while
# navigation was still starting were silently dropped (AMCL takes ~15-20 s to
# become active after a restart and ignores /initialpose until then), and
# nothing said whether a pose had actually aligned with the LiDAR.
_readiness_lock = threading.Lock()
_readiness = {"state": "unknown", "title": "Checking...", "detail": "", "level": "unknown"}
# The five nav2 lifecycle nodes that have to be ACTIVE before goals work.
_NAV_NODES = ["map_server", "amcl", "planner_server", "controller_server", "bt_navigator"]
# Alignment score = fraction of LiDAR returns that land on (or next to) an
# occupied cell of the saved map when the scan is placed using the current
# map->odom correction. 1.0 = every return sits on a wall of the map. Furniture
# or people lower it even when localization is right, so the thresholds are
# deliberately loose - they were checked against live scans, see
# PENDING_ISSUES.md. READY needs the score at or above _ALIGN_OK; the bar
# only drops back out of READY below _ALIGN_OK - _ALIGN_HYST so it doesn't flicker.
_ALIGN_OK = 0.50
_ALIGN_LOW = 0.30
_ALIGN_HYST = 0.08
# AMCL's own confidence (std-dev of its particle cloud, from /amcl_pose).
_SIGMA_XY_OK = 0.35   # metres
_SIGMA_YAW_OK = 0.30  # radians (~17 deg)
# Sensor data convention (2026-10-02) - LaserScan publishers (bluesea2
# included) commonly use BEST_EFFORT reliability; a default RELIABLE
# subscription wouldn't connect at all (same class of silent QoS/type
# mismatch hit earlier with cmd_vel_teleop/cmd_vel_remote - topic name
# alone isn't enough, the whole profile has to be compatible).
_SCAN_QOS = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
# Navigation goal status (2026-10-08), from bt_navigator's own action
# status topic - so it covers goals sent from anywhere (this page, RViz,
# the CLI), not just the ones this page published.
_goal_lock = threading.Lock()
_latest_goal = {"received": False}
_GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}

# Waypoint loop (2026-10-09). Deliberately NOT nav2's own waypoint_follower
# node - that would mean adding and lifecycle-managing a ninth process on a
# board that is already tight on CPU for navigation (see PENDING_ISSUES.md).
# Instead this reuses what already exists: publish_goal() over /goal_pose,
# and the goal-status tracking above. One failed/aborted waypoint stops the
# whole loop rather than ploughing on to the next one - safer default for
# something meant to run unattended.
_loop_lock = threading.Lock()
_loop_stop = threading.Event()
_loop_state = {"running": False, "detail": "idle"}


def _loop_wait_for_goal(prev_id, stop_event, start_timeout=5.0, finish_timeout=180.0):
    """Blocks until the goal just published (not `prev_id`) settles. There is
    no goal handle to wait on directly - goals here go through bt_navigator's
    own internal action client via the /goal_pose topic, not an action client
    this process holds - so this detects the new goal by watching
    _latest_goal's id change, then polls its status."""
    t0 = time.monotonic()
    gid = None
    while time.monotonic() - t0 < start_timeout:
        if stop_event.is_set():
            return "stopped"
        with _goal_lock:
            cur = _latest_goal.get("id")
        if cur and cur != prev_id:
            gid = cur
            break
        time.sleep(0.2)
    if gid is None:
        return "not_accepted"
    t0 = time.monotonic()
    while time.monotonic() - t0 < finish_timeout:
        if stop_event.is_set():
            return "stopped"
        with _goal_lock:
            if _latest_goal.get("id") != gid:
                return "superseded"  # something else sent a goal mid-loop
            status_name = _latest_goal.get("status_name")
        if status_name in ("succeeded", "aborted", "canceled"):
            return status_name
        time.sleep(0.3)
    return "timeout"


def _run_loop(hmi_node, map_name, names, repeat):
    all_wps = {w["name"]: w for w in _load_waypoints(map_name)}
    seq = [all_wps[n] for n in names if n in all_wps]
    cycle = 0
    detail = "no matching waypoints"
    while seq and not _loop_stop.is_set() and (repeat <= 0 or cycle < repeat):
        failed = False
        for wp in seq:
            if _loop_stop.is_set():
                detail = "stopped by user"
                failed = True
                break
            with _loop_lock:
                _loop_state.update({"running": True, "cycle": cycle + 1,
                                     "total_cycles": repeat if repeat > 0 else None,
                                     "waypoint": wp["name"], "detail": "navigating"})
            with _goal_lock:
                prev_id = _latest_goal.get("id")
            hmi_node.publish_goal(wp["x"], wp["y"], wp["theta"])
            result = _loop_wait_for_goal(prev_id, _loop_stop)
            if result == "stopped":
                detail = "stopped by user"
                failed = True
                break
            if result != "succeeded":
                detail = f"'{wp['name']}' {result} - loop stopped"
                hmi_node.cancel_navigation()
                failed = True
                break
        if failed:
            break
        cycle += 1
        detail = (f"completed {cycle} cycle(s)" if repeat <= 0
                  else f"completed {cycle}/{repeat} cycle(s)")
    with _loop_lock:
        _loop_state.update({"running": False, "detail": detail, "waypoint": None})


def _start_loop(hmi_node, map_name, names, repeat):
    with _loop_lock:
        if _loop_state.get("running"):
            return False, "a loop is already running - stop it first"
    if not names:
        return False, "select at least one waypoint"
    _loop_stop.clear()
    threading.Thread(target=_run_loop, args=(hmi_node, map_name, names, repeat),
                      daemon=True).start()
    return True, "loop started"


def _stop_loop():
    with _loop_lock:
        was_running = _loop_state.get("running")
    _loop_stop.set()
    return True, "stopping..." if was_running else "no loop was running"
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
        self._last_scan_build = 0.0  # throttle gate - see _on_scan()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._map_to_odom = None
        self._m2o_stamp = None        # stamp of the last map->odom seen, to tell a live AMCL from a stale one
        self._m2o_changed = 0.0       # monotonic time that stamp last changed
        self.create_timer(0.2, self._refresh_map_to_odom)  # 5Hz - see get_map_to_odom()
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 1)
        # --- readiness (2026-10-12), see _poll_readiness() ---
        self._last_scan_msg = None
        self._last_scan_seen = 0.0    # monotonic
        self._nav_map = None          # saved map as occupied-cell flags, for the alignment score
        self._amcl_cov = None         # AMCL's own position/heading std-dev
        self._pending_pose = None     # initial pose requested while AMCL wasn't active yet
        self._lc_clients = {n: self.create_client(GetState, f"/{n}/get_state") for n in _NAV_NODES}
        self._lc_state = {n: None for n in _NAV_NODES}   # lifecycle label, None = node not there
        self._lc_pending = {}         # node -> (future, started monotonic)
        self._align_score = None
        self._ready_latched = False
        self._pose_sent_at = 0.0
        # map_server publishes /map once, TRANSIENT_LOCAL; amcl_pose is latched too.
        self.create_subscription(OccupancyGrid, "/map", self._on_nav_map,
                                 QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=QoSReliabilityPolicy.RELIABLE))
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose,
                                 QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=QoSReliabilityPolicy.RELIABLE))
        self.create_timer(1.0, self._poll_readiness)
        # Action status is published TRANSIENT_LOCAL by the action server, so a
        # latched subscription also gets the current state on connect.
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                 self._on_goal_status, _LATCHED_QOS)
        if _NAV_FEEDBACK is not None:
            self.create_subscription(_NAV_FEEDBACK, "/navigate_to_pose/_action/feedback",
                                     self._on_goal_feedback, 10)
        # Goal cancellation (2026-10-09). A plain service client, not an
        # action client - this process never holds the goal handle (goals go
        # out via /goal_pose, not an action request from here), so it uses
        # the raw CancelGoal service every ROS2 action server exposes.
        self._cancel_client = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
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
        # Raw /scan for dock detection (2026-10-13). /scan_filtered drops most of the dock V's points, so
        # the dock vision needs the raw scan. Subscribed only while someone asks (see get_raw_scan()):
        # a 2000-point message at 15 Hz is not free on this board.
        self._raw_sub = None
        self._raw_scan_msg = None
        self._raw_scan_t = 0.0
        self._raw_used = 0.0
        self.dock_busy = False        # a dock/undock job owns the wheels: the page's joystick is ignored meanwhile
        self._teleop_session = None
        self._teleop_seq = -1
        # Per-page clock offsets for the stale-command check, see _teleop_age_ms().
        self._teleop_offsets = collections.deque()
        self.teleop_stale_count = 0
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

    def _refresh_map_to_odom(self):
        """Timer callback (2026-10-09, 5Hz) - looks the transform up ONCE here
        regardless of how many browser tabs are polling /pose.json or how
        often, instead of doing a fresh tf2 lookup per HTTP request."""
        try:
            t = self._tf_buffer.lookup_transform("map", "odom", Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self._map_to_odom = None
            return
        q = t.transform.rotation
        self._map_to_odom = {
            "x": t.transform.translation.x,
            "y": t.transform.translation.y,
            "theta": 2.0 * math.atan2(q.z, q.w),
        }
        stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
        if stamp != self._m2o_stamp:
            self._m2o_stamp = stamp
            self._m2o_changed = time.monotonic()

    def get_map_to_odom(self):
        """The correction AMCL produces once localized - None before that. Composed
        onto the odom-frame pose (see pollPose() in the page) so the robot, its
        outline, and the scan all draw aligned with the saved map's own frame."""
        return self._map_to_odom

    def _on_scan(self, msg: LaserScan):
        # Kept by reference for the 1 Hz alignment score (readiness) - cheap,
        # unlike the per-point conversion below.
        self._last_scan_msg = msg
        self._last_scan_seen = time.monotonic()
        # 2026-10-09: throttled to ~5Hz (matches the page's pollScan
        # interval) instead of doing this ~1000-point Python loop at the
        # LiDAR's full ~15Hz publish rate regardless of whether anyone is
        # looking at it.
        now = time.monotonic()
        if now - self._last_scan_build < 0.18:
            return
        self._last_scan_build = now
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
        self._pose_sent_at = time.monotonic()
        self.get_logger().info(f"Published /initialpose: x={x:.2f} y={y:.2f} theta={theta:.2f}")

    # ------------------------------------------------------------------
    # Readiness (2026-10-12)
    # ------------------------------------------------------------------
    def request_initial_pose(self, x: float, y: float, theta: float) -> str:
        """Entry point for the page's "set initial pose". Returns "sent" or "queued".

        AMCL ignores /initialpose until it is ACTIVE, which takes ~15-20 s
        after a navigation (re)start - on 2026-10-12 two poses in a row were
        silently lost that way, and the page had no way to say so. So: if AMCL
        is not active yet the pose is kept and sent the moment it is. When it
        is active the pose goes out twice, 0.5 s apart, instead of the page
        resending it five times (each message makes AMCL rebuild its whole
        particle cloud, which is not free on this board)."""
        if self._lc_state.get("amcl") == "active":
            self._pending_pose = None
            self.publish_initial_pose(x, y, theta)
            threading.Timer(0.5, self.publish_initial_pose, args=(x, y, theta)).start()
            return "sent"
        self._pending_pose = (x, y, theta)
        self.get_logger().info(f"AMCL not active yet - queued initial pose x={x:.2f} y={y:.2f} theta={theta:.2f}")
        return "queued"

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        self._amcl_cov = {
            "sxy": math.sqrt(max(c[0], c[7], 0.0)),
            "syaw": math.sqrt(max(c[35], 0.0)),
        }

    def _on_nav_map(self, msg: OccupancyGrid):
        self._nav_map = {
            "w": msg.info.width, "h": msg.info.height, "res": msg.info.resolution,
            "ox": msg.info.origin.position.x, "oy": msg.info.origin.position.y,
            "occ": bytearray(1 if v >= 65 else 0 for v in msg.data),
        }

    def _compute_alignment(self):
        """Fraction (0..1) of the latest LiDAR returns that fall on, or next
        to, an occupied cell of the saved map when placed with the current
        map->odom correction. None if it can't be computed yet. ~400 rays at
        1 Hz in plain Python: a couple of ms."""
        m, msg = self._nav_map, self._last_scan_msg
        if m is None or msg is None:
            return None
        try:
            t = self._tf_buffer.lookup_transform("map", msg.header.frame_id or "laser_frame", Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        q = t.transform.rotation
        yaw = 2.0 * math.atan2(q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = t.transform.translation.x, t.transform.translation.y
        w, h, res, ox, oy, occ = m["w"], m["h"], m["res"], m["ox"], m["oy"], m["occ"]
        n_rays = len(msg.ranges)
        step = max(1, n_rays // 400)
        rmax = msg.range_max * 0.98
        hits = total = 0
        for i in range(0, n_rays, step):
            r = msg.ranges[i]
            if not (msg.range_min < r < rmax):  # also rejects nan / inf (no return)
                continue
            a = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            ci = int((tx + lx * c - ly * s - ox) / res)
            cj = int((ty + lx * s + ly * c - oy) / res)
            total += 1
            found = False
            for jj in (cj - 1, cj, cj + 1):
                if 0 <= jj < h:
                    row = jj * w
                    for ii in (ci - 1, ci, ci + 1):
                        if 0 <= ii < w and occ[row + ii]:
                            found = True
                            break
                if found:
                    break
            if found:
                hits += 1
        return hits / total if total >= 30 else None

    def _poll_readiness(self):
        """1 Hz. Works out one state for the page's status bar:
        base_down -> nav_off -> nav_starting -> need_pose -> aligning -> ready."""
        now_m, now = time.monotonic(), time.time()
        self._drop_idle_raw_scan()

        # Lifecycle state of each nav2 node: one async GetState call each, never
        # more than one outstanding, so a node that is stuck can't pile requests up.
        for name, cli in self._lc_clients.items():
            pend = self._lc_pending.get(name)
            if pend is not None:
                fut, t0 = pend
                if fut.done():
                    try:
                        self._lc_state[name] = fut.result().current_state.label
                    except Exception:
                        self._lc_state[name] = None
                    del self._lc_pending[name]
                elif now_m - t0 > 4.0:
                    fut.cancel()
                    try:
                        cli.remove_pending_request(fut)
                    except Exception:
                        pass
                    self._lc_state[name] = None
                    del self._lc_pending[name]
                continue
            if cli.service_is_ready():
                self._lc_pending[name] = (cli.call_async(GetState.Request()), now_m)
            else:
                self._lc_state[name] = None
        states = dict(self._lc_state)

        # Base stack: MCU telemetry, LiDAR and odometry all flowing.
        with _status_lock:
            mcu_t = _latest_status.get("stamp") if _latest_status.get("received") else None
        with _pose_lock:
            odom_t = _latest_pose.get("stamp") if _latest_pose.get("received") else None
        mcu_ok = mcu_t is not None and now - mcu_t < 3.0
        odom_ok = odom_t is not None and now - odom_t < 3.0
        scan_ok = (now_m - self._last_scan_seen) < 3.0 and self._last_scan_msg is not None
        base_ok = mcu_ok and odom_ok and scan_ok

        nav_seen = any(v is not None for v in states.values())
        nav_active = all(v == "active" for v in states.values())

        # A pose requested before AMCL was up goes out the moment it is.
        if self._pending_pose and states.get("amcl") == "active":
            px, py, pth = self._pending_pose
            self._pending_pose = None
            self.publish_initial_pose(px, py, pth)
            threading.Timer(0.5, self.publish_initial_pose, args=(px, py, pth)).start()

        tf_fresh = self._map_to_odom is not None and (now_m - self._m2o_changed) < 3.0
        align = self._compute_alignment() if (nav_active and tf_fresh) else None
        self._align_score = align
        cov = self._amcl_cov if (nav_active and tf_fresh) else None

        # Hysteresis: once READY, only drop out when clearly worse.
        need_align = _ALIGN_OK - (_ALIGN_HYST if self._ready_latched else 0.0)
        align_good = align is None or align >= need_align
        cov_good = cov is None or (cov["sxy"] <= _SIGMA_XY_OK and cov["syaw"] <= _SIGMA_YAW_OK)

        pct = f"{round(align * 100)}%" if align is not None else "n/a"
        if not base_ok:
            missing = [n for n, ok in (("motor controller", mcu_ok), ("LiDAR", scan_ok), ("odometry", odom_ok)) if not ok]
            state, level = "base_down", "error"
            title = "ROBOT SOFTWARE NOT READY"
            detail = "Waiting for: " + ", ".join(missing) + ". It is either still starting up or stopped - use Restart everything if this lasts more than a minute or two."
        elif not nav_seen:
            state, level = "nav_off", "warn"
            title = "NAVIGATION OFF"
            detail = "The robot software is up but navigation is not running. Choose a map and press Start navigation."
        elif not tf_fresh and states.get("amcl") == "active" and states.get("map_server") == "active":
            # nav2's planner_server cannot finish activating until the "map" frame
            # exists, i.e. until AMCL has a pose (documented in PENDING_ISSUES.md),
            # and bt_navigator waits behind it. While it waits it doesn't even answer
            # status requests. So with AMCL up and no pose yet, the planner/BT being
            # "not up" is NORMAL - the pose is what unblocks them.
            state, level = "need_pose", "warn"
            title = "READY FOR INITIAL POSE"
            detail = ("Pose received - sending it to AMCL..." if self._pending_pose else
                      "Click \"Set initial pose\", then drag on the map where the robot really is, pointing the way it faces. "
                      "(The planner and navigator finish starting only after this.)")
        elif not nav_active or not tf_fresh:
            state, level = "nav_starting", "warn"
            waiting = [f"{n} ({states[n] or 'not up'})" for n in _NAV_NODES if states[n] != "active"]
            title = "NAVIGATION STARTING..."
            detail = (("Pose received - finishing navigation start-up. " if tf_fresh else
                       "You can already set the initial pose - it is held and sent the moment AMCL is ready. ")
                      + "Waiting for: " + ", ".join(waiting))
        elif align_good and cov_good:
            state, level = "ready", "ok"
            title = "READY TO DRIVE"
            extra = f", position +/-{round(cov['sxy'] * 100)} cm" if cov else ""
            detail = f"Localized - LiDAR matches the map ({pct}){extra}."
        else:
            state, level = "aligning", "warn"
            if align is not None and align < _ALIGN_LOW:
                title = "NOT ALIGNED WITH THE MAP"
                detail = (f"The LiDAR scan does not match the map ({pct}). Set the initial pose again - closer to where the robot really "
                          "is, and pointing the way it really faces.")
            elif align is not None and not align_good:
                title = "ALIGNING..."
                detail = f"LiDAR match {pct} - still settling. Give it a few seconds; if it stays low, set the initial pose again."
            else:
                title = "ALIGNING..."
                detail = (f"LiDAR match {pct}, but AMCL is still unsure of the position "
                          f"(+/-{round(cov['sxy'] * 100)} cm). Wait a few seconds, or drive a short way.")
        self._ready_latched = (state == "ready")

        data = {
            "state": state, "level": level, "title": title, "detail": detail,
            "checks": {
                "base": base_ok, "mcu": mcu_ok, "scan": scan_ok, "odom": odom_ok,
                "nav": states, "nav_active": nav_active, "localized": tf_fresh,
                "align": align, "sigma_xy": cov["sxy"] if cov else None,
                "sigma_yaw": cov["syaw"] if cov else None,
                "pending_pose": self._pending_pose is not None,
            },
            "stamp": now,
        }
        with _readiness_lock:
            _readiness.clear()
            _readiness.update(data)

    def _on_goal_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        # The list keeps finished goals around for a while; the newest-accepted
        # one is the goal the robot is (or was last) working on.
        st = max(msg.status_list,
                 key=lambda s: s.goal_info.stamp.sec + s.goal_info.stamp.nanosec * 1e-9)
        gid = bytes(st.goal_info.goal_id.uuid).hex()
        now = self.get_clock().now().nanoseconds * 1e-9
        with _goal_lock:
            prev = _latest_goal.get("id")
            if prev != gid:
                # new goal - drop the previous goal's feedback numbers
                for k in ("distance_remaining", "recoveries", "nav_time"):
                    _latest_goal.pop(k, None)
                _latest_goal["accepted_at"] = (st.goal_info.stamp.sec
                                                + st.goal_info.stamp.nanosec * 1e-9)
            if _latest_goal.get("status") != st.status:
                _latest_goal["changed_at"] = now
            _latest_goal.update({
                "received": True,
                "id": gid,
                "status": int(st.status),
                "status_name": _GOAL_STATUS_NAMES.get(st.status, str(st.status)),
            })

    def _on_goal_feedback(self, msg):
        fb = msg.feedback
        with _goal_lock:
            if bytes(msg.goal_id.uuid).hex() != _latest_goal.get("id"):
                return
            _latest_goal["distance_remaining"] = round(float(fb.distance_remaining), 2)
            _latest_goal["recoveries"] = int(fb.number_of_recoveries)
            _latest_goal["nav_time"] = round(fb.navigation_time.sec
                                             + fb.navigation_time.nanosec * 1e-9, 1)

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

    def _on_raw_scan(self, msg):
        self._raw_scan_msg = msg
        self._raw_scan_t = time.monotonic()

    def get_raw_scan(self):
        """Latest raw /scan, or None if it has not arrived (the first call after idle only subscribes)."""
        self._raw_used = time.monotonic()
        if self._raw_sub is None:
            self._raw_sub = self.create_subscription(LaserScan, "/scan", self._on_raw_scan, _SCAN_QOS)
        return self._raw_scan_msg if time.monotonic() - self._raw_scan_t < 1.0 else None

    def _drop_idle_raw_scan(self):
        if self._raw_sub is not None and time.monotonic() - self._raw_used > 15.0:
            self.destroy_subscription(self._raw_sub)
            self._raw_sub = None
            self._raw_scan_msg = None

    def get_map_pose(self):
        """Robot pose in the map frame (x, y, theta) from tf, or None if not localized."""
        try:
            t = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y, 2.0 * math.atan2(q.z, q.w))

    def cancel_navigation(self) -> bool:
        """Cancels whatever nav2 goal is in flight right now, however it was
        sent - this page, RViz, the CLI. A CancelGoal.Request with its
        all-zero default goal_id and zero stamp is the ROS2 action protocol's
        "cancel every goal on this server" form."""
        if not self._cancel_client.service_is_ready():
            return False
        self._cancel_client.call_async(CancelGoal.Request())
        return True

    # A motion command that spent more than this longer in transit than the page's
    # fastest recent commands is obsolete: the robot would be obeying where the
    # thumb WAS. 250 ms at 0.3 m/s is already 7.5 cm of unwanted travel.
    TELEOP_MAX_EXTRA_AGE_MS = 250.0

    def _teleop_age_ms(self, t_ms: float) -> float:
        """How much LATER than usual this command arrived, in ms.

        The page stamps each command with its own clock (t_ms), which has no fixed
        relation to this machine's clock. So: offset = (arrival time here) - (page
        stamp) for every command, and the SMALLEST offset seen in the last 10 s is
        taken as the "no extra delay" baseline - the extra delay of this command is
        its offset above that. A constant slow link is not detected here (the
        page's Link badge and its slow-link stop handle that); queueing bursts and
        hiccups are. Caller holds _teleop_lock."""
        now = time.monotonic()
        off = now * 1000.0 - t_ms
        dq = self._teleop_offsets
        dq.append((now, off))
        while dq and now - dq[0][0] > 10.0:
            dq.popleft()
        return off - min(o for _, o in dq)

    def publish_teleop(self, linear: float, angular: float, session=None, seq=None, t_ms=None) -> str:
        """Returns "ok", "old" (arrived after a newer command) or "stale" (held up in transit)."""
        # Commands from the browser carry a per-page session id and an
        # increasing sequence number. Over a laggy link, older motion commands
        # can arrive after a newer stop; dropping anything older than the newest
        # accepted command stops that from restarting motion.
        if self.dock_busy:
            return "dock_busy"
        with self._teleop_lock:
            if session is not None and seq is not None:
                if session != self._teleop_session:
                    self._teleop_session = session
                    self._teleop_seq = -1
                    self._teleop_offsets.clear()   # a different page = a different clock
                if seq <= self._teleop_seq:
                    return "old"
                self._teleop_seq = seq
            if t_ms is not None:
                age = self._teleop_age_ms(float(t_ms))
                # Only MOTION is refused when late. A stop is always applied, however
                # late it is - delaying a stop is the dangerous direction.
                if (linear != 0.0 or angular != 0.0) and age > self.TELEOP_MAX_EXTRA_AGE_MS:
                    self.teleop_stale_count += 1
                    return "stale"
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self._teleop_pub.publish(msg)
            return "ok"

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
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Calixto AMR - HMI</title>
<script>
// Phone-friendly layout switch (2026-10-12). Runs before the page paints so
// there is no flash of the wrong layout. Order: ?ui=mobile / ?ui=desktop in the
// address (also remembered), then the saved choice from the "Mobile view"
// toggle, then a first-visit guess (touch screen AND narrow window = phone).
(function () {
  var mode = null, touch = false;
  try {
    var q = new URLSearchParams(location.search).get('ui');
    if (q === 'mobile' || q === 'desktop') { localStorage.setItem('amr_ui_mode', q); }
    mode = localStorage.getItem('amr_ui_mode');
  } catch (e) { /* private mode / storage blocked: fall through to the guess */ }
  touch = ('ontouchstart' in window) || (navigator.maxTouchPoints > 0);
  if (!mode) { mode = (touch && window.matchMedia('(max-width: 800px)').matches) ? 'mobile' : 'desktop'; }
  var h = document.documentElement;
  if (mode === 'mobile') { h.classList.add('mobile'); }
  if (touch) { h.classList.add('touch'); }
  window.AMR_MOBILE = (mode === 'mobile');   // read by the page script (joystick is always live in this layout)
})();
</script>
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
  button.danger { border-color: #c0392b; color: #c0392b; background: #fde8e8; }
  button.danger:hover { background: #fbd4d4; }
  fieldset.wp-box { border: 1px solid #d8dbe0; border-radius: 8px; padding: 0.6rem 0.7rem; margin-top: 0.6rem; }
  fieldset.wp-box legend { font-size: 0.8rem; color: #6b7280; padding: 0 0.3rem; }
  .wp-list { display: flex; flex-direction: column; gap: 0.25rem; margin: 0.4rem 0; }
  .wp-row { display: flex; align-items: center; gap: 0.4rem; font-size: 0.85rem; }
  .wp-row .wp-name { flex: 1; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  .wp-row button { padding: 0.2rem 0.5rem; font-size: 0.78rem; }
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
  /* Readiness bar (2026-10-12): one line that says whether the robot can be driven yet. */
  .ready-bar {
    flex: 0 0 auto;   /* a wrapping flex row can otherwise shrink below its real height and spill over the map */
    display: flex; flex-wrap: wrap; align-items: center; gap: 0.5rem 0.9rem;
    padding: 0.45rem 1rem; border-bottom: 1px solid #d8dbe0; background: #eceef1; color: #4b5563;
  }
  .ready-pill {
    font-weight: 700; font-size: 0.85rem; letter-spacing: 0.03em; padding: 0.25rem 0.8rem;
    border-radius: 999px; background: #6b7280; color: #fff; white-space: nowrap;
  }
  .ready-detail { font-size: 0.82rem; flex: 1 1 22rem; line-height: 1.35; }
  .ready-checks { font-size: 0.74rem; color: #6b7280; white-space: nowrap; }
  /* Dock bar (2026-10-13): Dock / Undock / Save home / Pose from dock, right under the readiness bar. */
  .dock-bar {
    flex: 0 0 auto; display: flex; flex-wrap: wrap; align-items: center; gap: 0.4rem 0.5rem;
    padding: 0.3rem 1rem; border-bottom: 1px solid #d8dbe0; background: #f5f6f8;
  }
  .dock-bar .dock-label { font-weight: 700; font-size: 0.8rem; color: #4b5563; margin-right: 0.2rem; }
  .dock-bar button { padding: 0.3rem 0.7rem; font-size: 0.82rem; }
  .dock-bar button[hidden] { display: none; }
  .dock-bar button:disabled { opacity: 0.45; cursor: not-allowed; }
  .dock-status { font-size: 0.8rem; color: #374151; flex: 1 1 14rem; line-height: 1.3; }
  .dock-status.fail { color: #8f2a1f; } .dock-status.run { color: #1d4ed8; font-weight: 600; }
  html.mobile .dock-bar { padding: 0.25rem 0.6rem; }
  html.mobile .dock-bar button { padding: 0.45rem 0.7rem; font-size: 0.85rem; }
  .ready-bar.ok    { background: #e6f4ea; } .ready-bar.ok .ready-pill    { background: #1e7e34; } .ready-bar.ok .ready-detail    { color: #1e5a2a; }
  .ready-bar.warn  { background: #fff6da; } .ready-bar.warn .ready-pill  { background: #b7791f; } .ready-bar.warn .ready-detail  { color: #6b4a08; }
  .ready-bar.error { background: #fde8e8; } .ready-bar.error .ready-pill { background: #c0392b; } .ready-bar.error .ready-detail { color: #8f2a1f; }
  button.needs-pose { animation: poseglow 1.4s ease-in-out infinite; }
  @keyframes poseglow { 0%, 100% { box-shadow: 0 0 0 0 rgba(183,121,31,0.0); } 50% { box-shadow: 0 0 0 5px rgba(183,121,31,0.45); } }
  /* Restart overlay: shown while the robot software is being torn down and rebuilt. */
  .restart-overlay {
    position: fixed; inset: 0; z-index: 9999; display: none; align-items: center; justify-content: center;
    background: rgba(20,24,33,0.72);
  }
  .restart-overlay.show { display: flex; }
  .restart-card {
    width: min(34rem, 92vw); background: #fff; border-radius: 12px; padding: 1.2rem 1.4rem;
    box-shadow: 0 12px 40px rgba(0,0,0,0.35);
  }
  .restart-card h2 { margin: 0 0 0.2rem; font-size: 1.1rem; }
  .restart-card .sub { font-size: 0.8rem; color: #6b7280; margin-bottom: 0.7rem; }
  .restart-steps { list-style: none; margin: 0 0 0.8rem; padding: 0; font-size: 0.88rem; }
  .restart-steps li { display: flex; gap: 0.6rem; padding: 0.22rem 0; color: #9ca3af; }
  .restart-steps li .mark { width: 1.2rem; text-align: center; }
  .restart-steps li.running { color: #1a1d24; font-weight: 600; }
  .restart-steps li.done { color: #1e7e34; }
  .restart-steps li.failed { color: #c0392b; font-weight: 600; }
  .restart-steps li .det { font-weight: 400; color: #6b7280; font-size: 0.78rem; margin-left: 0.4rem; }
  .restart-msg { font-size: 0.85rem; min-height: 1.2em; margin-bottom: 0.6rem; }
  .restart-msg.err { color: #c0392b; }
  .restart-msg.ok { color: #1e7e34; font-weight: 600; }

  /* ---- Touch tool + phone layout (2026-10-12) ---- */
  .uitoggle { display: inline-flex; align-items: center; gap: 0.35rem; font-size: 0.8rem; color: #4b5563; cursor: pointer; user-select: none; }
  .uitoggle input { width: 1.1rem; height: 1.1rem; }
  /* Touch tool: what ONE finger does on the map. Hidden on mouse-only machines. */
  #tool-btn { position: absolute; right: 0.8rem; top: 0.6rem; z-index: 6; display: none; font-weight: 600;
              background: #2f6fed; color: #fff; border-color: #2f6fed; }
  #tool-btn.pan { background: #fff; color: #1a1d24; border-color: #9aa3b0; }
  html.touch #tool-btn { display: inline-block; }
  .mbar { display: none; }
  .sheet-close { display: none; }
  button:disabled { opacity: 0.55; cursor: not-allowed; }
  /* Wrappers added for the phone layout. On desktop they must change nothing:
     the stage fills the map area exactly like the canvas used to, and the
     control zone generates no box at all (its children keep their old absolute
     positions relative to the map area). */
  .map-stage { position: absolute; inset: 0; }
  .ctl-zone { display: contents; }
  .ph-speed { display: none; }
  #map-expand-btn { position: absolute; left: 0.4rem; top: 0.4rem; z-index: 6; display: none; font-weight: 600; background: #fff; }
  .link-badge { position: absolute; right: 0.4rem; top: 0.4rem; z-index: 6; display: none; padding: 0.3rem 0.65rem;
                border-radius: 999px; font-size: 0.78rem; font-weight: 700; background: #eceef1; color: #6b7280; }
  .link-badge.ok { background: #e6f4ea; color: #1e7e34; }
  .link-badge.warn { background: #fff6da; color: #93690a; }
  .link-badge.bad { background: #fde8e8; color: #c0392b; }
  html.mobile .link-badge { display: inline-block; }

  html.mobile header.topbar { padding: 0.35rem 0.6rem; gap: 0.4rem; flex-wrap: wrap; }
  html.mobile header.topbar h1 { display: none; }
  html.mobile .logo { height: 30px; margin-right: 6px; }
  html.mobile .badges { gap: 0.35rem; }
  html.mobile .badge { padding: 0.25rem 0.55rem; font-size: 0.75rem; }
  html.mobile .ready-bar { padding: 0.3rem 0.6rem; gap: 0.25rem 0.6rem; }
  html.mobile .ready-pill { font-size: 0.78rem; padding: 0.2rem 0.65rem; }
  html.mobile .ready-detail { font-size: 0.74rem; flex-basis: 100%; order: 3; max-height: 3.9em; overflow: hidden; }
  html.mobile .ready-checks { display: none; }
  /* The map fills the screen; the side panel becomes a slide-up sheet. */
  /* flex: 1 1 0 - NOT the desktop "1 1 auto": this layout's only child is absolute, so its
     base size is 0 and it could never shrink when the readiness bar grows after first
     paint (the page then overflowed and clipped the bottom buttons). Grow-from-zero
     always takes exactly what is left. */
  html.mobile .layout { display: block; position: relative; min-height: 0; flex: 1 1 0; }
  html.mobile main.map-main { position: absolute; inset: 0; min-height: 0; }
  html.mobile aside.sidebar {
    position: fixed; left: 0; right: 0; bottom: 0; max-height: 78vh; z-index: 60; border-left: none;
    border-radius: 16px 16px 0 0; box-shadow: 0 -10px 36px rgba(0,0,0,0.35); padding: 0.8rem;
    transform: translateY(110%); transition: transform 0.2s ease-out;
  }
  html.mobile aside.sidebar.open { transform: none; }
  html.mobile .sheet-close { display: block; width: 100%; margin-bottom: 0.6rem; font-weight: 600; }
  html.mobile button, html.mobile select, html.mobile input[type=text], html.mobile input[type=number] { min-height: 44px; font-size: 1rem; }
  html.mobile .tabs { gap: 6px; }
  html.mobile .map-hint { display: none; }

  /* ---- Phone layout, portrait: three bands (2026-10-12) ----
       [ E-STOP + status ]  top    - always in thumb/eye reach
       [      MAP        ]  middle - 30% of the height; "Expand map" for placing goals
       [ joystick | btns ]  bottom - the rest, everything big
     On desktop .map-stage and .ctl-zone are invisible wrappers (see below), so
     nothing outside html.mobile changes. */
  html.mobile main.map-main { display: flex; flex-direction: column; background: #ffffff; }
  html.mobile .topleft-panel {
    position: static; order: 1; flex: 0 0 auto; width: auto; box-shadow: none; border-radius: 0;
    display: flex; gap: 0.5rem; align-items: stretch; padding: 0.4rem 0.5rem; border-bottom: 1px solid #d8dbe0;
  }
  html.mobile .topleft-panel .hint, html.mobile .topleft-panel #conn { display: none; }
  html.mobile .topleft-panel .estop-btn { flex: 1 1 52%; min-height: 66px; font-size: 1.15rem; margin: 0; }
  html.mobile .topleft-panel .amr-status-box { flex: 1 1 48%; min-height: 66px; padding: 0.4rem 0.6rem; font-size: 0.85rem; line-height: 1.3; margin: 0; }
  html.mobile .map-stage { position: relative; inset: auto; order: 2; flex: 0 0 30%; min-height: 150px; overflow: hidden; isolation: isolate; }
  html.mobile #tool-btn { display: none; }   /* the bottom button area has its own copy */
  html.mobile #recenter-btn { bottom: 0.4rem; right: 0.4rem; min-height: 40px; }
  html.mobile #map-expand-btn { display: block; }
  /* The zone is a size container so the joystick can be as big as the space
     allows and still round: width = the smaller of its column and the row height
     (zone height minus the 3rem speed row and the two gaps). */
  html.mobile .ctl-zone {
    order: 3; flex: 1 1 0; min-height: 0; display: grid; gap: 0.6rem; container-type: size;
    grid-template-columns: minmax(0, 1fr) 8.8rem; grid-template-rows: minmax(0, 1fr) 3rem;
    padding: 0.6rem 0.6rem calc(0.6rem + env(safe-area-inset-bottom));
  }
  html.mobile .joystick-base {
    display: block !important; position: relative; left: auto; bottom: auto; grid-column: 1; grid-row: 1;
    width: min(100%, calc(100cqh - 3.6rem)); height: auto; aspect-ratio: 1 / 1; justify-self: center; align-self: center;
    background: rgba(47,111,237,0.08); border: 3px solid #9aa3b0;
  }
  html.mobile .joystick-knob { width: 38%; height: 38%; }
  html.mobile .mbar {
    display: grid; grid-column: 2; grid-row: 1; grid-template-rows: repeat(4, minmax(0, 1fr)); gap: 0.45rem; min-height: 0;
  }
  html.mobile .mbar button { min-height: 0; font-size: 1.05rem; font-weight: 600; line-height: 1.15; padding: 0.3rem 0.4rem; }
  html.mobile #mb-tool { display: block; }
  html.mobile .ph-speed { display: block; grid-column: 1 / span 2; grid-row: 2; height: 3rem; }
  html.mobile .ph-speed label { display: flex; align-items: center; gap: 0.6rem; height: 100%; font-size: 0.95rem; font-weight: 600; white-space: nowrap; }
  html.mobile .ph-speed input[type=range] { flex: 1; height: 2.2rem; }
  /* Map expanded: the map takes the screen; the joystick and speed bar step aside. */
  html.mobile.map-expanded .map-stage { flex: 1 1 0; min-height: 0; }
  html.mobile.map-expanded .ctl-zone { flex: 0 0 auto; display: block; container-type: normal; padding: 0.4rem 0.5rem calc(0.4rem + env(safe-area-inset-bottom)); }
  html.mobile.map-expanded .joystick-base, html.mobile.map-expanded .ph-speed { display: none !important; }
  html.mobile.map-expanded .mbar { display: grid; grid-template-rows: none; grid-template-columns: repeat(4, 1fr); }
  html.mobile.map-expanded .mbar button { min-height: 52px; }
  /* Landscape phone: map on the left, E-stop + controls on the right. */
  @media (orientation: landscape) {
    html.mobile main.map-main { display: grid; grid-template-columns: 1.35fr 1fr; grid-template-rows: auto minmax(0, 1fr); }
    html.mobile .map-stage { grid-column: 1; grid-row: 1 / span 2; flex: none; min-height: 0; }
    html.mobile .topleft-panel { grid-column: 2; grid-row: 1; }
    html.mobile .ctl-zone { grid-column: 2; grid-row: 2; }
    html.mobile .topleft-panel .estop-btn, html.mobile .topleft-panel .amr-status-box { min-height: 52px; }
    html.mobile .ready-detail { display: none; }
    /* Short and wide: buttons 2 x 2 beside the joystick, smaller labels. */
    html.mobile .ctl-zone { grid-template-columns: minmax(0, 0.9fr) minmax(0, 1.3fr); gap: 0.4rem; padding: 0.4rem; grid-template-rows: minmax(0, 1fr) 2.4rem; }
    html.mobile .ph-speed { height: 2.4rem; }
    html.mobile .mbar { grid-template-columns: repeat(2, minmax(0, 1fr)); grid-template-rows: repeat(2, minmax(0, 1fr)); gap: 0.35rem; }
    html.mobile .mbar button { font-size: 0.85rem; line-height: 1.05; padding: 0.15rem 0.25rem; }
    html.mobile .ph-speed label { font-size: 0.8rem; gap: 0.4rem; }
    html.mobile .joystick-base { width: min(100%, calc(100cqh - 3.2rem)); }
    html.mobile.map-expanded main.map-main { grid-template-columns: 1fr; grid-template-rows: minmax(0, 1fr) auto auto; }
    html.mobile.map-expanded .map-stage { grid-column: 1; grid-row: 1; }
    html.mobile.map-expanded .topleft-panel { grid-column: 1; grid-row: 2; }
    html.mobile.map-expanded .ctl-zone { grid-column: 1; grid-row: 3; }
  }
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
      <label class="uitoggle" title="Switches this page to a phone-friendly layout (big buttons, full-screen map). The page reloads."><input type="checkbox" id="mobile-toggle"> Mobile view</label>
    </div>
  </header>
  <div id="ready-bar" class="ready-bar">
    <span id="ready-pill" class="ready-pill">CHECKING...</span>
    <span id="ready-detail" class="ready-detail">Waiting for the robot's status...</span>
    <span id="ready-checks" class="ready-checks"></span>
  </div>
  <div id="dock-bar" class="dock-bar">
    <span class="dock-label">Dock</span>
    <button id="dock-go" disabled title="Drive to the dock and reverse into it">Dock</button>
    <button id="dock-undock" disabled title="Drive forward off the dock">Undock</button>
    <button id="dock-teach" disabled title="Save this spot, 1.5 m in front of the V and facing it, as the dock waypoint">Teach waypoint</button>
    <button id="dock-save" disabled title="Save the docked position as this map's home">Save home</button>
    <button id="dock-pose" disabled title="Set the initial pose from the saved home (robot must be on the dock)">Pose from dock</button>
    <button id="dock-cancel" class="danger" hidden>Cancel dock</button>
    <span id="dock-status" class="dock-status">checking...</span>
  </div>

  <div class="layout">
    <main class="map-main">
      <div class="topleft-panel">
        <button id="estop-btn" class="estop-btn">EMERGENCY STOP</button>
        <div class="hint">Software stop only - does not cut motor power.</div>
        <div id="amr-status" class="amr-status-box unknown">No data yet</div>
        <div id="conn" class="hint">connecting...</div>
      </div>
      <div class="map-stage">
        <canvas id="map-canvas"></canvas>
        <div class="map-hint">Mouse: left-drag sets goal, right-drag pans, scroll zooms &middot;
        Touch: drag pans, pinch zooms, long-press-then-drag sets a goal</div>
        <button id="recenter-btn">Center on robot</button>
        <button id="tool-btn" title="What one finger does on the map: place a goal / initial pose, or move the map. Two fingers always move and zoom.">Touch: set goal</button>
        <button id="map-expand-btn" title="Make the map fill the screen to place a goal precisely; tap again to bring the joystick back.">Expand map</button>
        <span id="link-badge" class="link-badge" title="Round trip from this page to the robot, over the last 20 requests. Green is fine for driving; amber is sluggish; red is too slow to drive by hand.">Link ...</span>
      </div>
      <div class="ctl-zone">
        <div id="joystick-base" class="joystick-base"><div id="joystick-knob" class="joystick-knob"></div></div>
        <div id="mbar" class="mbar">
          <button id="mb-pose">Set initial pose</button>
          <button id="mb-cancel" class="danger">Cancel goal</button>
          <button id="mb-tool">Touch: set goal</button>
          <button id="mb-menu">Menu</button>
        </div>
        <div id="ph-speed" class="ph-speed">
          <label>Speed limit <input type="range" id="ph-speed-range" min="0.1" max="0.8" step="0.05" value="0.3"> <span id="ph-speed-val">0.30 m/s</span></label>
        </div>
      </div>
    </main>

    <aside class="sidebar" id="sidebar">
      <button id="sheet-close" class="sheet-close">Close menu</button>
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
            <button id="nav-restart-btn" title="Stops and starts nav2 again with the selected map - use this if it has frozen or stopped responding to goals.">Restart navigation</button>
            <button id="restart-all-btn" class="danger" title="Tears down EVERYTHING - navigation, the whole robot software and its container - and starts it all again from scratch, then navigation on the selected map. Takes about 2 minutes. This page shows progress.">Restart everything</button>
          </div>
          <div class="modes">
            <button id="nav-pose-mode-btn">Set initial pose</button>
          </div>
          <div class="hint">Click and drag on the map to set a goal. Drag direction sets heading.
          While "Set initial pose" is on, the next drag sets the robot's starting pose instead.</div>
          <div class="modes">
            <button id="nav-cancel-btn" class="danger" title="Cancels whatever goal nav2 is currently working on - use this before a restart if a goal has gone wrong.">Cancel goal</button>
          </div>
          <div id="nav-status" class="hint"></div>
          <div><span id="goal-status" class="badge unknown">Goal: none yet</span></div>
        </div>

        <fieldset class="wp-box">
          <legend>Waypoints &amp; loop</legend>
          <div class="modes">
            <button id="wp-add-btn">Add waypoint</button>
          </div>
          <div class="hint">Saved per map. "Add waypoint" then click-and-drag on the map like
          setting a goal - you'll be asked for a name. Tick the ones to include in a loop.</div>
          <div id="wp-list" class="wp-list"></div>
          <div class="modes">
            <label>Repeat <input id="loop-repeat" type="number" min="0" step="1" value="0" style="width:3.5em"></label>
            <span class="hint" style="margin:0">(0 = run forever)</span>
          </div>
          <div class="modes">
            <button id="loop-start-btn">Start loop</button>
            <button id="loop-stop-btn" class="danger">Stop loop</button>
          </div>
          <div id="loop-status" class="hint">Loop: idle</div>
        </fieldset>
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

// ---- Teleop sender (2026-10-12) ----
// Before: EVERY pointer-move / key event fired its own fetch('/teleop') - 60 to
// 120 a second from a phone thumb - with nothing limiting how many were in
// flight, and the STOP when you let go was a single request. Over a phone's WiFi
// those requests queue behind each other, so the robot obeyed commands that were
// already old, and the stop waited in the same queue (a robot hit something
// under phone control that day). Now there is ONE sender loop:
//   - 20 sends a second at most, and never more than one request in flight;
//     if the previous one hasn't returned, the next tick sends the NEWEST
//     command ("latest wins") instead of adding to a backlog;
//   - a stop is repeated for ~0.4 s, so one lost packet can't leave it moving;
//   - every request gives up after 400 ms and is timed (the Link badge);
//   - every command carries the page's clock so the server can refuse commands
//     that were held up in transit (see publish_teleop). The page itself never
//     decides a command is fine - the server and the robot's own timeouts do.
const TELEOP_PERIOD_MS = 50;
const TELEOP_TIMEOUT_MS = 400;
let teleopCmd = { linear: 0, angular: 0 };
let teleopStopsLeft = 0;
let teleopTimer = null;
let teleopBusy = false;
const rttHist = [];   // recent round trips in ms; a failed/timed-out request counts as 1000

function noteRtt(ms) {
  rttHist.push(ms === null ? 1000 : ms);
  if (rttHist.length > 20) rttHist.shift();
  updateLinkBadge();
}
function linkStats(n) {
  const a = rttHist.slice(-(n || 20)).sort((x, y) => x - y);
  return a.length ? { med: a[Math.floor(a.length / 2)], worst: a[a.length - 1], n: a.length } : null;
}
function updateLinkBadge() {
  const b = document.getElementById('link-badge');
  const s = linkStats();
  if (!b || !s) return;
  b.textContent = 'Link ' + Math.round(s.med) + ' ms (worst ' + Math.round(s.worst) + ')';
  b.className = 'link-badge ' + (s.worst > 300 ? 'bad' : s.worst > 150 ? 'warn' : 'ok');
}
// Too slow to drive by hand: stop sending motion until it recovers.
function linkTooSlow() {
  const s = linkStats(5);
  return !!s && s.n >= 5 && s.med > 350;
}

function teleopPublish(linear, angular) {
  teleopCmd = { linear, angular };
  if (linear === 0 && angular === 0) teleopStopsLeft = 8;
  if (!teleopTimer) teleopTimer = setInterval(teleopTick, TELEOP_PERIOD_MS);
}
async function teleopTick() {
  const moving = teleopCmd.linear !== 0 || teleopCmd.angular !== 0;
  if (!moving && teleopStopsLeft <= 0) { clearInterval(teleopTimer); teleopTimer = null; return; }
  if (teleopBusy) return;            // latest wins: the next tick sends the newest command
  if (!moving) teleopStopsLeft--;
  teleopBusy = true;
  const seq = ++teleopSeq;
  const t0 = performance.now();
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), TELEOP_TIMEOUT_MS);
  try {
    const res = await fetch('/teleop', {
      method: 'POST', signal: ctl.signal,
      body: JSON.stringify({ linear: teleopCmd.linear, angular: teleopCmd.angular, session: teleopSession, seq, t: t0 })
    });
    await res.json();
    noteRtt(performance.now() - t0);
  } catch (e) {
    noteRtt(null);
  } finally {
    clearTimeout(timer);
    teleopBusy = false;
  }
}
// Measure the link while not driving too (a cheap GET once a second), so the
// badge always shows the current state of the connection.
async function pingLink() {
  if (teleopBusy || teleopTimer) return;
  const t0 = performance.now();
  try { await (await fetch('/speed.json', { cache: 'no-store' })).json(); noteRtt(performance.now() - t0); }
  catch (e) { noteRtt(null); }
}
setInterval(pingLink, 1000);

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

// Phone layout (2026-10-12): the joystick is always on and has its OWN speed
// limit (default 0.3 m/s, the "Speed limit" slider under it), kept only in this
// browser. The shared limit (teleopLinearMax, which the RF remote also reads via
// /speed) is deliberately left alone, so using the phone never changes how fast
// the RF remote can drive. Turning is capped in proportion (2.5 rad/s per m/s),
// and never above the shared angular limit.
let phoneLinMax = 0.3;
try { const s = parseFloat(localStorage.getItem('amr_phone_speed')); if (s >= 0.1 && s <= 0.8) phoneLinMax = s; } catch (e) { /* default */ }
function joystickLimits() {
  if (window.AMR_MOBILE) return [phoneLinMax, Math.min(teleopAngularMax, phoneLinMax * 2.5)];
  return [teleopLinearMax, teleopAngularMax];
}
function joystickPublish() {
  const [nx, ny] = joystickVec;
  // A median round trip over 350 ms means the robot would react a good half
  // second late: refuse to drive (send stops) until the link recovers.
  if (window.AMR_MOBILE && linkTooSlow()) {
    teleopPublish(0, 0);
    document.getElementById('nav-status').textContent = 'LINK TOO SLOW to drive safely (' + Math.round(linkStats(5).med) + ' ms) - the robot is being held stopped. Move closer to the router or switch WiFi band.';
    return;
  }
  const [linMax, angMax] = joystickLimits();
  // Pushing the stick up (negative ny) drives forward; pushing right
  // (positive nx) turns right (angular.z negative, CW per REP-103).
  teleopPublish(-ny * linMax, -nx * angMax);
}

joystickBase.addEventListener('pointerdown', (evt) => {
  if (currentMode !== 'teleop' && !window.AMR_MOBILE) return;
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
joystickBase.addEventListener('lostpointercapture', endJoystick);   // any way the finger is lost = stop
window.addEventListener('blur', endJoystick);
document.addEventListener('visibilitychange', () => { if (document.hidden) endJoystick(); });  // phone locked / app switched
// Same resend reasoning as keyboard teleop - keep cmd_vel_mux's timeout fed
// while the stick is held still (pointermove doesn't fire without motion).
setInterval(() => {
  if ((currentMode === 'teleop' || window.AMR_MOBILE) && joystickActive) joystickPublish();
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
// 2026-10-12: also follow the map's own container. The window 'resize' event is not
// enough in the phone layout: the header wrapping, the readiness bar growing, the
// Expand map button and rotating the phone all change the map area without
// resizing the window, and a canvas left at its old size paints over the buttons
// below it.
if (window.ResizeObserver) { new ResizeObserver(() => resizeCanvas()).observe(canvas.parentElement); }

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
document.getElementById('nav-map-select').addEventListener('change', (e) => { showMap(e.target.value); loadWaypoints(); });
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

let canvasClickMode = 'goal';  // 'goal', 'initialpose' or 'waypoint' - see the mode buttons below
// 2026-10-12: pose mode is one-shot again, and the button locks for a short
// while after a pose is sent. It had been sticky (2026-10-09) so a second drag
// couldn't silently become a goal - but then, with the goal flow being the
// normal thing to do next, tapping "Set initial pose" again out of habit
// re-armed pose mode and the next goal drag overwrote the pose. The readiness
// bar now gives the feedback that sticky mode was standing in for, so: pose
// sent -> back to goal mode, button locked. It unlocks sooner if the bar says
// the pose must be redone (nothing set yet / LiDAR doesn't match the map), and
// asks first if the robot is already localized.
const POSE_LOCK_MS = 20000;
let poseLockUntil = 0;
function poseRetryAllowed() {
  const c = (typeof lastReady !== 'undefined' && lastReady.checks) || {};
  const st = typeof lastReady !== 'undefined' ? lastReady.state : 'unknown';
  return st === 'need_pose' || (st === 'aligning' && c.align != null && c.align < 0.30);
}
function refreshPoseButtons() {
  const locked = Date.now() < poseLockUntil && !poseRetryAllowed();
  const armed = canvasClickMode === 'initialpose';
  const left = Math.max(0, Math.ceil((poseLockUntil - Date.now()) / 1000));
  ['nav-pose-mode-btn', 'mb-pose'].forEach(id => {
    const b = document.getElementById(id);
    if (!b) return;
    b.disabled = locked;
    b.classList.toggle('active', armed);
    b.textContent = locked ? ('Pose set - locked ' + left + 's') : (armed ? 'Drag on the map: initial pose...' : 'Set initial pose');
  });
}
document.getElementById('nav-pose-mode-btn').addEventListener('click', () => {
  if (canvasClickMode !== 'initialpose') {
    if (Date.now() < poseLockUntil && !poseRetryAllowed()) return;
    if (typeof lastReady !== 'undefined' && lastReady.state === 'ready' &&
        !confirm('The robot is already localized (READY TO DRIVE).\\n\\nSet the initial pose again?')) return;
  }
  canvasClickMode = canvasClickMode === 'initialpose' ? 'goal' : 'initialpose';
  refreshPoseButtons();
});
document.getElementById('mb-pose').addEventListener('click', () => document.getElementById('nav-pose-mode-btn').click());
setInterval(refreshPoseButtons, 500);

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
document.getElementById('nav-restart-btn').addEventListener('click', () => {
  if (!document.getElementById('nav-map-select').value) {
    document.getElementById('nav-status').textContent = 'Choose a map first.';
    return;
  }
  // 2026-10-12: goes through the restart helper (progress overlay); falls back
  // to the old direct call if the helper isn't running - see startRestart().
  startRestart('navigation');
});

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
  const isWaypoint = canvasClickMode === 'waypoint';
  const status = document.getElementById('nav-status');
  if (isWaypoint) {
    // One-shot, same pattern as initial-pose mode below.
    canvasClickMode = 'goal';
    const btn = document.getElementById('wp-add-btn');
    btn.classList.remove('active');
    btn.textContent = 'Add waypoint';
    const name = prompt('Name this waypoint:');
    if (!name) { status.textContent = 'Waypoint not saved (no name given).'; return; }
    const map = document.getElementById('nav-map-select').value;
    try {
      const res = await fetch('/waypoints', {
        method: 'POST',
        body: JSON.stringify({ action: 'add', map, name, x: gx, y: gy, theta })
      });
      const data = await res.json();
      status.textContent = data.status === 'ok'
        ? `Waypoint "${name}" saved at x=${gx.toFixed(2)} y=${gy.toFixed(2)}.`
        : (data.detail || 'Could not save the waypoint.');
    } catch (e) {
      status.textContent = 'Failed to reach the server.';
    }
    loadWaypoints();
    return;
  }
  const url = isPose ? '/initial_pose' : '/goal';
  const body = JSON.stringify({ x: gx, y: gy, theta: theta });
  if (isPose) {
    // 2026-10-12: ONE request now (this used to send it five times, 250 ms
    // apart, and every copy made AMCL rebuild its particle cloud). The server
    // decides: if AMCL is already active it publishes the pose twice itself;
    // if AMCL is still starting it HOLDS the pose and sends it the moment
    // AMCL is up - before, a pose sent too early was silently dropped.
    let outcome = 'sent';
    try {
      const res = await fetch(url, { method: 'POST', body });
      const data = await res.json();
      outcome = data.outcome || 'sent';
    } catch (e) {
      status.textContent = 'Could not reach the robot to send the initial pose - try again.';
      return;
    }
    // Back to goal mode and lock the button (see POSE_LOCK_MS above).
    canvasClickMode = 'goal';
    poseLockUntil = Date.now() + POSE_LOCK_MS;
    refreshPoseButtons();
    status.textContent = outcome === 'queued'
      ? `Initial pose x=${gx.toFixed(2)} y=${gy.toFixed(2)} HELD - navigation is still starting, it will be sent automatically as soon as AMCL is ready (watch the bar at the top). Back in goal mode.`
      : `Initial pose sent: x=${gx.toFixed(2)} y=${gy.toFixed(2)}. Watch the bar at the top - it turns green READY TO DRIVE once the LiDAR matches the map. Back in goal mode; the pose button is locked for ${POSE_LOCK_MS / 1000} s (it unlocks sooner if the pose needs redoing).`;
    return;
  } else {
    if (!confirmReady('goal')) { status.textContent = 'Goal not sent.'; return; }
    // Exactly one send: bt_navigator treats every /goal_pose message as a new
    // navigation request and cancels whatever was running - resending here
    // would cancel and restart navigation repeatedly instead of helping it.
    try { await fetch(url, { method: 'POST', body }); } catch (e) { /* best-effort */ }
  }
  // 2026-10-09: pose mode used to be one-shot (reverted to 'goal' right
  // after sending), which silently turned a SECOND drag - meant to nudge or
  // retry the pose - into a navigation goal instead, with no error and no
  // visible sign anything was wrong. Pose mode now stays sticky until the
  // "Set initial pose" button is clicked again to turn it off, so you can
  // re-drag as many times as needed (e.g. while watching AMCL converge)
  // without it silently switching back to sending goals underneath you.
  status.textContent = `Goal sent: x=${gx.toFixed(2)} y=${gy.toFixed(2)}.`;
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
// 2026-10-12: reworked. Before, one finger PANNED and only a 350 ms
// long-press-then-drag placed a goal, so a normal swipe always moved the map
// and the goal/heading was very hard to set on a phone. Now:
//   one finger  = the "touch tool": 'place' (default) puts a goal / initial
//                 pose / waypoint at the touch-down point and the drag sets the
//                 heading, exactly like the mouse; 'pan' moves the map instead
//                 (button on the map toggles it);
//   two fingers = ALWAYS move the map (both fingers dragging) and zoom (pinch).
// A plain tap or a very short drag never sends anything - it needs a clear drag
// to set the heading - so brushing the screen can't send the robot off.
const PAN_THRESHOLD_PX = 12 * devicePixelRatio;
const MIN_TOUCH_DRAG_PX = 28 * devicePixelRatio;
let touchTool = 'place';          // 'place' | 'pan'
let touchState = null;

function refreshToolButtons() {
  document.querySelectorAll('#tool-btn, #mb-tool').forEach(b => {
    b.textContent = touchTool === 'place' ? 'Touch: set goal' : 'Touch: move map';
    b.classList.toggle('pan', touchTool === 'pan');
  });
}
document.querySelectorAll('#tool-btn, #mb-tool').forEach(b => b.addEventListener('click', () => {
  touchTool = touchTool === 'place' ? 'pan' : 'place';
  refreshToolButtons();
}));
refreshToolButtons();

function touchCanvasPoint(touch) {
  const rect = canvas.getBoundingClientRect();
  return [(touch.clientX - rect.left) * devicePixelRatio, (touch.clientY - rect.top) * devicePixelRatio];
}
function touchMid(t0, t1) {
  const rect = canvas.getBoundingClientRect();
  return [((t0.clientX + t1.clientX) / 2 - rect.left) * devicePixelRatio,
          ((t0.clientY + t1.clientY) / 2 - rect.top) * devicePixelRatio];
}

canvas.addEventListener('touchstart', (evt) => {
  evt.preventDefault();
  if (evt.touches.length >= 2) {
    const [t0, t1] = evt.touches;
    const [mx, my] = touchMid(t0, t1);
    touchState = { mode: 'pinch', dist: Math.hypot(t1.clientX - t0.clientX, t1.clientY - t0.clientY), midPx: mx, midPy: my };
    drag = null;                  // a second finger cancels any goal being placed
    return;
  }
  if (evt.touches.length === 1) {
    const [px, py] = touchCanvasPoint(evt.touches[0]);
    if (touchTool === 'place') {
      touchState = { mode: 'goal' };
      drag = { startPx: px, startPy: py, curPx: px, curPy: py };
    } else {
      touchState = { mode: 'pan', lastPx: px, lastPy: py };
    }
  }
}, { passive: false });

canvas.addEventListener('touchmove', (evt) => {
  evt.preventDefault();
  if (!touchState) return;
  if (touchState.mode === 'pinch' && evt.touches.length >= 2) {
    const [t0, t1] = evt.touches;
    const dist = Math.hypot(t1.clientX - t0.clientX, t1.clientY - t0.clientY);
    const [mx, my] = touchMid(t0, t1);
    const scale = view.scale * devicePixelRatio;
    // Move the map with the midpoint of the two fingers, then zoom about it.
    view.originX += (my - touchState.midPy) / scale;
    view.originY += (mx - touchState.midPx) / scale;
    zoomAt(mx, my, dist / touchState.dist);
    touchState.dist = dist; touchState.midPx = mx; touchState.midPy = my;
    return;
  }
  if (evt.touches.length !== 1) return;
  const [px, py] = touchCanvasPoint(evt.touches[0]);
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
  if (touchState && touchState.mode === 'goal' && drag) {
    const d = drag;
    drag = null;
    if (Math.hypot(d.curPx - d.startPx, d.curPy - d.startPy) < MIN_TOUCH_DRAG_PX) {
      document.getElementById('nav-status').textContent =
        'Nothing sent: drag from the spot to set the direction the robot should face (a tap is ignored).';
    } else {
      publishGoalFromDrag(d);
    }
  }
  if (evt.touches.length === 0) touchState = null;
}, { passive: false });
canvas.addEventListener('touchcancel', () => { drag = null; touchState = null; }, { passive: true });

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

// Goal status (2026-10-08) - from bt_navigator's action status, so it also
// reflects goals sent from RViz or the CLI, not only from this page.
const goalBadge = document.getElementById('goal-status');
const GOAL_LOOK = {
  accepted:  ['warn',    'Goal accepted'],
  executing: ['warn',    'Goal: moving'],
  canceling: ['warn',    'Goal: canceling'],
  succeeded: ['ok',      'Goal reached \u2713'],
  canceled:  ['unknown', 'Goal canceled'],
  aborted:   ['error',   'Goal FAILED \u2717'],
};
async function pollGoal() {
  try {
    const res = await fetch('/goal_status.json', { cache: 'no-store' });
    const g = await res.json();
    if (!g.received) {
      goalBadge.className = 'badge unknown';
      goalBadge.textContent = 'Goal: none yet';
      return;
    }
    const [cls, label] = GOAL_LOOK[g.status_name] || ['unknown', 'Goal: ' + g.status_name];
    let text = label;
    if (g.status_name === 'executing' && g.distance_remaining != null) {
      text += ` \u2014 ${g.distance_remaining.toFixed(2)} m left`;
    }
    if ((g.status_name === 'succeeded' || g.status_name === 'aborted') && g.nav_time != null) {
      text += ` in ${g.nav_time.toFixed(1)} s`;
    }
    if (g.recoveries) {
      text += ` (${g.recoveries} recover${g.recoveries === 1 ? 'y' : 'ies'})`;
    }
    goalBadge.className = 'badge ' + cls;
    goalBadge.textContent = text;
  } catch (e) {
    goalBadge.className = 'badge unknown';
    goalBadge.textContent = 'Goal: unknown';
  }
}

// --- Cancel goal (2026-10-09) ---
document.getElementById('nav-cancel-btn').addEventListener('click', async () => {
  const status = document.getElementById('nav-status');
  status.textContent = 'Canceling...';
  try {
    const res = await fetch('/cancel_goal', { method: 'POST' });
    const data = await res.json();
    status.textContent = data.detail || data.status;
  } catch (e) {
    status.textContent = 'Failed to reach the server.';
  }
});

// --- Waypoints (2026-10-09) ---
let lastWaypoints = [];
async function loadWaypoints() {
  const map = document.getElementById('nav-map-select').value;
  const list = document.getElementById('wp-list');
  if (!map) { list.innerHTML = '<div class="hint">Choose a map first.</div>'; lastWaypoints = []; return; }
  try {
    const res = await fetch(`/waypoints.json?map=${encodeURIComponent(map)}`, { cache: 'no-store' });
    const data = await res.json();
    lastWaypoints = data.waypoints || [];
    if (!lastWaypoints.length) {
      list.innerHTML = '<div class="hint">No waypoints saved for this map yet.</div>';
      return;
    }
    list.innerHTML = lastWaypoints.map(w => `
      <div class="wp-row" data-name="${w.name.replace(/"/g, '&quot;')}">
        <input type="checkbox" class="wp-select" checked>
        <span class="wp-name" title="${w.name.replace(/"/g, '&quot;')}">${w.name}</span>
        <button class="wp-go">Go</button>
        <button class="wp-del danger">×</button>
      </div>`).join('');
    list.querySelectorAll('.wp-go').forEach(btn => btn.addEventListener('click', async (e) => {
      const row = e.target.closest('.wp-row');
      const w = lastWaypoints.find(x => x.name === row.dataset.name);
      if (!w) return;
      if (!confirmReady(`goal "${w.name}"`)) { document.getElementById('nav-status').textContent = 'Goal not sent.'; return; }
      document.getElementById('nav-status').textContent = `Sending goal: ${w.name}`;
      try {
        await fetch('/goal', { method: 'POST', body: JSON.stringify({ x: w.x, y: w.y, theta: w.theta }) });
      } catch (err) { /* best-effort, same as the map-click goal sender */ }
    }));
    list.querySelectorAll('.wp-del').forEach(btn => btn.addEventListener('click', async (e) => {
      const row = e.target.closest('.wp-row');
      if (!confirm(`Delete waypoint "${row.dataset.name}"?`)) return;
      await fetch('/waypoints', { method: 'POST', body: JSON.stringify({ action: 'delete', map, name: row.dataset.name }) });
      loadWaypoints();
    }));
  } catch (e) {
    list.innerHTML = '<div class="hint">Could not load waypoints.</div>';
  }
}

document.getElementById('wp-add-btn').addEventListener('click', () => {
  if (!document.getElementById('nav-map-select').value) {
    document.getElementById('nav-status').textContent = 'Choose a map first.';
    return;
  }
  canvasClickMode = 'waypoint';
  const btn = document.getElementById('wp-add-btn');
  btn.classList.add('active');
  btn.textContent = 'Click and drag on the map...';
});

// --- Loop (2026-10-09) ---
function selectedWaypointNames() {
  return Array.from(document.querySelectorAll('.wp-row'))
    .filter(r => r.querySelector('.wp-select').checked)
    .map(r => r.dataset.name);
}
document.getElementById('loop-start-btn').addEventListener('click', async () => {
  const loopStatus = document.getElementById('loop-status');
  const map = document.getElementById('nav-map-select').value;
  const names = selectedWaypointNames();
  const repeat = parseInt(document.getElementById('loop-repeat').value, 10) || 0;
  if (!map) { loopStatus.textContent = 'Loop: choose a map first.'; return; }
  if (!names.length) { loopStatus.textContent = 'Loop: tick at least one waypoint above.'; return; }
  if (!confirmReady('waypoint loop')) { loopStatus.textContent = 'Loop: not started.'; return; }
  try {
    const res = await fetch('/loop', { method: 'POST', body: JSON.stringify({ action: 'start', map, names, repeat }) });
    const data = await res.json();
    loopStatus.textContent = 'Loop: ' + (data.detail || data.status);
  } catch (e) {
    loopStatus.textContent = 'Loop: failed to reach the server.';
  }
});
document.getElementById('loop-stop-btn').addEventListener('click', async () => {
  const loopStatus = document.getElementById('loop-status');
  try {
    const res = await fetch('/loop', { method: 'POST', body: JSON.stringify({ action: 'stop' }) });
    const data = await res.json();
    loopStatus.textContent = 'Loop: ' + (data.detail || data.status);
  } catch (e) {
    loopStatus.textContent = 'Loop: failed to reach the server.';
  }
});
async function pollLoop() {
  try {
    const res = await fetch('/loop_status.json', { cache: 'no-store' });
    const s = await res.json();
    const el = document.getElementById('loop-status');
    if (s.running) {
      const cyc = s.total_cycles ? `${s.cycle}/${s.total_cycles}` : `${s.cycle}`;
      el.textContent = `Loop: running - cycle ${cyc}, waypoint "${s.waypoint}" (${s.detail})`;
    } else {
      el.textContent = 'Loop: ' + (s.detail || 'idle');
    }
  } catch (e) { /* ignore - next poll will retry */ }
}

pollStatus();
pollPose();
pollScan();
pollGoal();
loadWaypoints();
pollLoop();
setInterval(pollStatus, 500);
setInterval(pollPose, 150);
setInterval(pollScan, 200);
setInterval(pollGoal, 500);
setInterval(pollLoop, 1000);

// ===== Readiness bar (2026-10-12) =====
// One line that says whether the robot can be driven: base software up ->
// navigation up -> initial pose set -> LiDAR matches the map -> READY.
let lastReady = { state: 'unknown' };
async function pollReadiness() {
  const bar = document.getElementById('ready-bar');
  try {
    const res = await fetch('/readiness.json', { cache: 'no-store' });
    const r = await res.json();
    lastReady = r;
    bar.className = 'ready-bar ' + (r.level || '');
    document.getElementById('ready-pill').textContent = r.title || 'CHECKING...';
    document.getElementById('ready-detail').textContent = r.detail || '';
    const c = r.checks || {};
    const nav = c.nav || {};
    const navUp = Object.values(nav).filter(v => v === 'active').length;
    document.getElementById('ready-checks').textContent = [
      'Robot ' + (c.base ? 'OK' : '--'),
      'Navigation ' + navUp + '/' + Object.keys(nav).length,
      'Localized ' + (c.localized ? 'yes' : 'no'),
      'LiDAR match ' + (c.align == null ? 'n/a' : Math.round(c.align * 100) + '%'),
    ].join('   |   ');
    // Draw the eye to the "Set initial pose" button exactly when it is needed.
    document.getElementById('nav-pose-mode-btn')
      .classList.toggle('needs-pose', r.state === 'need_pose' && canvasClickMode !== 'initialpose');
  } catch (e) {
    lastReady = { state: 'unknown' };
    bar.className = 'ready-bar error';
    document.getElementById('ready-pill').textContent = 'NO CONNECTION';
    document.getElementById('ready-detail').textContent = "Cannot reach the robot's web server - it may be restarting.";
    document.getElementById('ready-checks').textContent = '';
  }
}
// A goal sent before the bar says READY often goes nowhere (no pose set yet) or
// starts from a wrong pose. Ask first instead of failing silently; "unknown"
// (the bar itself is unavailable) never blocks.
function confirmReady(what) {
  if (lastReady.state === 'ready' || lastReady.state === 'unknown') return true;
  return confirm('The robot is not ready: ' + (lastReady.title || '') + '\\n' + (lastReady.detail || '') + '\\n\\nSend the ' + what + ' anyway?');
}
pollReadiness();
setInterval(pollReadiness, 1000);

// ===== Dock bar (2026-10-13) =====
// Dock / Undock / Save home / Pose from dock. The dock is the map's home position.
// The robot does all the driving itself (scripts/dock.py run by the server); the
// page only starts, shows progress and cancels. While a job runs the server ignores
// this page's joystick so the two never fight over the wheels.
function dockMap() { return document.getElementById('nav-map-select').value; }
function renderDock(d) {
  const job = d.job || { state: 'idle' };
  const running = job.state === 'running';
  const caps = d.caps || {};
  [['dock-go', 'dock'], ['dock-undock', 'undock'], ['dock-teach', 'teach'], ['dock-save', 'save'], ['dock-pose', 'pose']].forEach(([id, key]) => {
    const b = document.getElementById(id);
    const c = caps[key] || { ok: false, why: 'not available' };
    b.disabled = !c.ok;
    b.title = c.why || '';
  });
  document.getElementById('dock-cancel').hidden = !running;
  const st = document.getElementById('dock-status');
  let text, cls = '';
  const recent = job.finished && d.now && (d.now - job.finished) < 120;
  if (running) { text = (job.stage || 'Working') + ' - joystick locked; press Cancel dock or the E-stop to stop'; cls = 'run'; }
  else if (job.state === 'failed' && recent) { text = 'Dock stopped: ' + (job.detail || 'failed'); cls = 'fail'; }
  else if (job.state === 'done' && recent) { text = job.stage || 'Done'; }
  else if (d.docked) { text = 'On the dock' + (d.home ? ' - home saved' : ' - no home saved yet'); }
  else if (d.front) { text = 'Dock in view, ' + d.front.a.toFixed(1) + ' m ahead' + (d.wp ? ' - waypoint taught' : ' - no waypoint taught'); }
  else { text = (d.wp ? 'Dock waypoint taught.' : 'No dock waypoint taught.') + ' Dock not in view.'; }
  st.textContent = text;
  st.className = 'dock-status ' + cls;
}
async function pollDock() {
  try {
    const res = await fetch('/dock_status.json?map=' + encodeURIComponent(dockMap()), { cache: 'no-store' });
    renderDock(await res.json());
  } catch (e) {
    document.getElementById('dock-status').textContent = 'no connection';
  }
}
async function dockAction(action, ask) {
  const map = dockMap();
  if (action !== 'cancel' && !map) { alert('Choose the map for navigation first (Navigation tab).'); return; }
  if (ask && !confirm(ask)) return;
  const st = document.getElementById('dock-status');
  st.textContent = 'sending...';
  try {
    const res = await fetch('/dock', { method: 'POST', body: JSON.stringify({ action: action, map: map }) });
    const data = await res.json();
    st.textContent = 'Dock: ' + (data.detail || data.status);
  } catch (e) {
    st.textContent = 'Could not reach the server.';
  }
  setTimeout(pollDock, 600);
}
document.getElementById('dock-go').addEventListener('click', () => dockAction('dock',
  'The robot will now drive by itself to the dock and reverse into it. Keep a hand on the E-stop and keep the way clear. Start?'));
document.getElementById('dock-undock').addEventListener('click', () => dockAction('undock',
  'The robot will drive 0.3 m straight forward off the dock. Is the way ahead clear? Start?'));
document.getElementById('dock-teach').addEventListener('click', () => dockAction('teach_wp',
  'Save the robot position now as the DOCK WAYPOINT of this map (about 1.5 m in front of the V, facing it)? The robot will drive here before docking, and the home (docked position) is updated from what the LiDAR sees. Old files are kept as backups.'));
document.getElementById('dock-save').addEventListener('click', () => dockAction('save_home',
  'Save the robot position now as the HOME (dock) of this map? It replaces the old home (a backup is kept).'));
document.getElementById('dock-pose').addEventListener('click', () => dockAction('pose_from_dock',
  'Set the initial pose from the saved home? Only do this if the robot is really sitting on the dock.'));
document.getElementById('dock-cancel').addEventListener('click', () => dockAction('cancel', ''));
pollDock();
setInterval(pollDock, 1200);

// ===== Phone layout controls (2026-10-12) =====
// "Mobile view" reloads the page into the phone layout (full-screen map, big
// buttons, the side panel as a slide-up Menu). The head script applies it
// before first paint; ?ui=mobile / ?ui=desktop in the address also works and
// is remembered, so a bookmark or a QR code can open straight into it.
const mobileToggle = document.getElementById('mobile-toggle');
mobileToggle.checked = document.documentElement.classList.contains('mobile');
mobileToggle.addEventListener('change', () => {
  location.href = location.pathname + '?ui=' + (mobileToggle.checked ? 'mobile' : 'desktop');
});
const sidebarEl = document.getElementById('sidebar');
document.getElementById('mb-menu').addEventListener('click', () => sidebarEl.classList.add('open'));
document.getElementById('sheet-close').addEventListener('click', () => sidebarEl.classList.remove('open'));
document.getElementById('mb-cancel').addEventListener('click', () => document.getElementById('nav-cancel-btn').click());
// Phone joystick speed limit (browser-local; see joystickLimits()).
const phRange = document.getElementById('ph-speed-range');
const phVal = document.getElementById('ph-speed-val');
function showPhoneSpeed() { phRange.value = phoneLinMax; phVal.textContent = phoneLinMax.toFixed(2) + ' m/s'; }
phRange.addEventListener('input', () => {
  phoneLinMax = parseFloat(phRange.value);
  try { localStorage.setItem('amr_phone_speed', String(phoneLinMax)); } catch (e) { /* not remembered, still applied */ }
  showPhoneSpeed();
});
showPhoneSpeed();
// Expand map: the map takes the screen so a goal can be placed precisely; the
// joystick steps aside (and is released) until the map is shrunk again.
const expandBtn = document.getElementById('map-expand-btn');
expandBtn.addEventListener('click', () => {
  const on = document.documentElement.classList.toggle('map-expanded');
  expandBtn.textContent = on ? 'Shrink map' : 'Expand map';
  endJoystick();
  requestAnimationFrame(resizeCanvas);
});
if (document.documentElement.classList.contains('touch')) {
  const hintEl = document.querySelector('.map-hint');
  if (hintEl) {
    hintEl.textContent = 'Touch: drag from a spot to set the goal and its heading - two fingers move and zoom the map - the "Touch:" button lets one finger move the map instead.';
  }
}

// ===== Restart with progress (2026-10-12) =====
// The restart is done by amr_ctl.py on the robot HOST (port 8081), NOT by this
// web server: this server is part of the stack being restarted, so it cannot
// survive tearing the stack down. The helper keeps running and reports each
// stage; this page just shows them.
const CTL = location.protocol + '//' + location.hostname + ':8081';
let restartTimer = null;
let restartMode = '';
const rOverlay = document.getElementById('restart-overlay');

function renderJob(job) {
  const ul = document.getElementById('restart-steps');
  ul.innerHTML = '';
  (job.stages || []).forEach(s => {
    const li = document.createElement('li');
    li.className = s.state;
    const mark = document.createElement('span');
    mark.className = 'mark';
    mark.textContent = s.state === 'done' ? '✔' : s.state === 'failed' ? '✖' : s.state === 'running' ? '▶' : '•';
    const label = document.createElement('span');
    label.textContent = s.name;
    if (s.detail) {
      const det = document.createElement('span');
      det.className = 'det';
      det.textContent = s.detail;
      label.appendChild(det);
    }
    li.appendChild(mark);
    li.appendChild(label);
    ul.appendChild(li);
  });
}

async function pollJob() {
  clearTimeout(restartTimer);
  const msg = document.getElementById('restart-msg');
  try {
    const res = await fetch(CTL + '/ctl/job', { cache: 'no-store' });
    const job = await res.json();
    renderJob(job);
    document.getElementById('restart-sub').textContent =
      (job.mode === 'everything' ? 'Full restart' : 'Navigation restart') + ' - ' + Math.round(job.elapsed || 0) + ' s elapsed';
    if (job.state === 'running') { restartTimer = setTimeout(pollJob, 1000); return; }
    if (job.state === 'done') {
      msg.className = 'restart-msg ok';
      msg.textContent = 'Done. Navigation is up - set the initial pose when the bar says READY FOR INITIAL POSE.';
      setTimeout(() => { if (restartMode === 'everything') { location.reload(); } else { rOverlay.classList.remove('show'); } }, 2500);
    } else {
      msg.className = 'restart-msg err';
      msg.textContent = 'Restart failed: ' + (job.error || 'see the steps above') + ' - you can try again.';
      document.getElementById('restart-close-btn').hidden = false;
    }
  } catch (e) {
    // The helper itself didn't answer (network blip) - keep trying.
    msg.className = 'restart-msg';
    msg.textContent = 'Waiting for the restart helper...';
    restartTimer = setTimeout(pollJob, 2000);
  }
}

async function startRestart(mode) {
  const status = document.getElementById('nav-status');
  const map = document.getElementById('nav-map-select').value;
  if (!map) { status.textContent = 'Choose a map first.'; return; }
  if (mode === 'everything' && !confirm('Restart EVERYTHING?\\n\\nThis stops the robot, tears down navigation, all robot software and its container, and starts it all again. It takes about 2 minutes, and the robot cannot be driven meanwhile.')) return;
  let ok = false, err = '';
  try {
    const res = await fetch(CTL + '/ctl/restart', { method: 'POST', body: JSON.stringify({ mode, map }) });
    const j = await res.json();
    ok = !!j.ok;
    err = j.error || '';
  } catch (e) { err = 'restart helper not reachable'; }
  if (!ok) {
    if (mode === 'navigation' && err === 'restart helper not reachable') {
      // Helper not installed: the old direct call still works for navigation only.
      status.textContent = 'Restart helper not running - using the basic restart. Set the initial pose again once navigation is back up...';
      sendNavigation('restart');
      return;
    }
    status.textContent = 'Could not start the restart: ' + err + (err === 'restart helper not reachable' ? ' (the helper service amr-ctl must be installed on the robot - see docker/install_amr_ctl.sh).' : '');
    return;
  }
  restartMode = mode;
  document.getElementById('restart-title').textContent = mode === 'everything' ? 'Restarting everything' : 'Restarting navigation';
  document.getElementById('restart-msg').className = 'restart-msg';
  document.getElementById('restart-msg').textContent = 'Please wait - do not close this page.';
  document.getElementById('restart-close-btn').hidden = true;
  document.getElementById('restart-steps').innerHTML = '';
  rOverlay.classList.add('show');
  pollJob();
}
document.getElementById('restart-all-btn').addEventListener('click', () => startRestart('everything'));
document.getElementById('restart-close-btn').addEventListener('click', () => rOverlay.classList.remove('show'));
// If the page is (re)loaded while a restart is already running, pick it up.
fetch(CTL + '/ctl/job', { cache: 'no-store' }).then(r => r.json()).then(job => {
  if (job && job.state === 'running') {
    restartMode = job.mode;
    document.getElementById('restart-title').textContent = job.mode === 'everything' ? 'Restarting everything' : 'Restarting navigation';
    rOverlay.classList.add('show');
    pollJob();
  }
}).catch(() => { /* helper not installed - nothing to show */ });
</script>
<div id="restart-overlay" class="restart-overlay">
  <div class="restart-card">
    <h2 id="restart-title">Restarting</h2>
    <div id="restart-sub" class="sub"></div>
    <ul id="restart-steps" class="restart-steps"></ul>
    <div id="restart-msg" class="restart-msg"></div>
    <div class="modes"><button id="restart-close-btn" hidden>Close</button></div>
  </div>
</div>
</body>
</html>
"""


# 2026-10-09: HTTP/1.0 -> 1.1 on the handler below. Default BaseHTTPRequestHandler
# protocol is HTTP/1.0, which closes the TCP connection after every single
# response. The page polls ~14-17 times/sec from one tab (pose 150ms, scan
# 200ms, status+goal 500ms, loop 1s) - that was ~14-17 fresh TCP handshakes
# AND ~14-17 new OS threads/sec (ThreadingHTTPServer spawns one thread per
# CONNECTION, which under HTTP/1.0 means per request) on a board that is
# already short on CPU headroom during navigation. HTTP/1.1 lets fetch()
# reuse one connection for all of a tab's polling instead. Safe to turn on
# because every response here sets Content-Length explicitly (checked
# 2026-10-09 - a response with no Content-Length under keep-alive leaves the
# client hanging, waiting for a body that is never coming).
class HmiHTTPHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

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
        elif self.path == "/goal_status.json":
            with _goal_lock:
                data = dict(_latest_goal)
            self._send_json(data)
        elif self.path == "/readiness.json":
            with _readiness_lock:
                data = dict(_readiness)
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
        elif self.path.startswith("/waypoints.json"):
            query = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            name = query.get("map", [""])[0]
            if not _valid_map_name(name):
                self._send_json({"status": "error", "detail": "unknown map"}, status=404)
                return
            self._send_json({"waypoints": _load_waypoints(name)})
        elif self.path.startswith("/dock_status.json"):
            query = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            name = query.get("map", [""])[0]
            try:
                self._send_json(self.server.dock_mgr.status(name if _valid_map_name(name) else ""))
            except Exception as e:  # never let the dock panel take the page down
                self._send_json({"error": str(e), "job": {"state": "idle"}, "caps": {}})
        elif self.path == "/loop_status.json":
            with _loop_lock:
                data = dict(_loop_state)
            self._send_json(data)
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
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            with open(fpath, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        # urlparse().path: the page is also opened as /?ui=mobile (see the
        # head script), and an exact match on self.path would 404 that.
        elif urllib.parse.urlparse(self.path).path in ("/", "/index.html"):
            body = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.send_header("Content-Length", "0")
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
        elif self.path == "/cancel_goal":
            ok = self.server.hmi_node.cancel_navigation()
            self._send_json({"status": "ok" if ok else "error",
                              "detail": "cancel sent" if ok else
                                        "nav2's action server is not up (is navigation running?)"},
                             status=200 if ok else 503)
        elif self.path == "/waypoints":
            try:
                data = json.loads(raw.decode("utf-8"))
                action, map_name, name = data["action"], data["map"], data.get("name", "")
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad waypoint payload: {e}"}, status=400)
                return
            if not _valid_map_name(map_name):
                self._send_json({"status": "error", "detail": "unknown map"}, status=400)
                return
            waypoints = _load_waypoints(map_name)
            if action == "add":
                if not _valid_waypoint_name(name):
                    self._send_json({"status": "error",
                                      "detail": "name: 1-60 plain characters"}, status=400)
                    return
                try:
                    x, y, theta = float(data["x"]), float(data["y"]), float(data["theta"])
                except (KeyError, ValueError, TypeError):
                    self._send_json({"status": "error", "detail": "bad x/y/theta"}, status=400)
                    return
                waypoints = [w for w in waypoints if w["name"] != name]  # upsert by name
                waypoints.append({"name": name, "x": x, "y": y, "theta": theta})
                _save_waypoints(map_name, waypoints)
                self._send_json({"status": "ok"})
            elif action == "delete":
                kept = [w for w in waypoints if w["name"] != name]
                _save_waypoints(map_name, kept)
                self._send_json({"status": "ok", "deleted": len(waypoints) != len(kept)})
            else:
                self._send_json({"status": "error", "detail": f"unknown action '{action}'"}, status=400)
        elif self.path == "/loop":
            try:
                data = json.loads(raw.decode("utf-8"))
                action = data["action"]
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad loop payload: {e}"}, status=400)
                return
            if action == "start":
                map_name = data.get("map", "")
                names = data.get("names") or []
                try:
                    repeat = int(data.get("repeat", 0))
                except (TypeError, ValueError):
                    repeat = 0
                if not _valid_map_name(map_name):
                    self._send_json({"status": "error", "detail": "choose a map first"}, status=400)
                    return
                ok, detail = _start_loop(self.server.hmi_node, map_name, names, repeat)
                self._send_json({"status": "ok" if ok else "error", "detail": detail},
                                 status=200 if ok else 400)
            elif action == "stop":
                ok, detail = _stop_loop()
                self._send_json({"status": "ok", "detail": detail})
            else:
                self._send_json({"status": "error", "detail": f"unknown action '{action}'"}, status=400)
        elif self.path == "/dock":
            try:
                data = json.loads(raw.decode("utf-8"))
                action = data["action"]
                map_name = data.get("map", "")
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad dock payload: {e}"}, status=400)
                return
            if action != "cancel" and not _valid_map_name(map_name):
                self._send_json({"status": "error", "detail": "choose a map first"}, status=400)
                return
            ok, detail = self.server.dock_mgr.act(action, map_name)
            self._send_json({"status": "ok" if ok else "error", "detail": detail}, status=200 if ok else 400)
        elif self.path == "/initial_pose":
            try:
                data = json.loads(raw.decode("utf-8"))
                x, y, theta = float(data["x"]), float(data["y"]), float(data["theta"])
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad initial pose payload: {e}"}, status=400)
                return
            outcome = self.server.hmi_node.request_initial_pose(x, y, theta)
            self._send_json({"status": "ok", "outcome": outcome})
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
            elif action == "restart":
                # 2026-10-08: nav2 occasionally wedges (most often the
                # planner_server/AMCL bringup deadlock documented in
                # PENDING_ISSUES.md - stuck until an initial pose arrives, or
                # a stale PID file left over from an unclean stop). Until now
                # that meant asking someone with SSH to run stop+start by
                # hand every time. Same two scripts, run back to back -
                # stop_navigation.sh is safe to call even if nothing is
                # running (see its own header comment).
                if not _valid_map_name(map_name):
                    self._send_json({"status": "error", "detail": "choose a map first"}, status=400)
                    return
                try:
                    subprocess.run(["bash", f"{docker_dir}/stop_navigation.sh"],
                                    capture_output=True, text=True, timeout=20)
                except subprocess.TimeoutExpired:
                    pass  # best-effort - start_navigation.sh will still refuse on a stale PID file
                cmd = ["bash", f"{docker_dir}/start_navigation.sh", map_name]
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
            session, seq, t_ms = data.get("session"), data.get("seq"), data.get("t")
            outcome = self.server.hmi_node.publish_teleop(linear, angular, session, seq, t_ms)
            self._send_json({"status": "ok", "accepted": outcome == "ok", "outcome": outcome})
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
            self.send_header("Content-Length", "0")
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

    def _goal_prev_id():
        with _goal_lock:
            return _latest_goal.get("id")

    def _ready_state():
        with _readiness_lock:
            return _readiness.get("state")

    def _loop_running():
        with _loop_lock:
            return bool(_loop_state.get("running"))

    server.dock_mgr = dock_manager.DockManager({
        "get_scan": node.get_raw_scan,
        "ready_state": _ready_state,
        "map_pose": node.get_map_pose,
        "request_pose": node.request_initial_pose,
        "publish_goal": node.publish_goal,
        "goal_prev_id": _goal_prev_id,
        "wait_goal": _loop_wait_for_goal,
        "cancel_nav": node.cancel_navigation,
        "loop_running": _loop_running,
        "set_busy": lambda v: setattr(node, "dock_busy", v),
    })
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
