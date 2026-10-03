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

"Open RViz2" button (2026-10-01): spawns a plain native RViz2 window on
THIS MACHINE's own screen (wherever this server process is running) - not
embedded in the page, no VNC, none of the earlier self-healing-watchdog
complexity. One-shot: click launches it if it's not already running: if you
close the window yourself, that's it, nothing relaunches it - click the
button again if you want it back. Only meaningful when this server runs on
a machine with a display attached; opening the HMI from a phone or another
computer on the network still only opens RViz2 on the HOST machine's screen.

Usage:
    source /opt/ros/humble/setup.bash
    source <workspace>/install/setup.bash   # for message bindings
    python3 status_web_server.py [--port 8080]

    Then open http://localhost:8080 - works alongside any ros2_control
    launch (diffbot.launch.py / diffbot_no_ekf.launch.py) for /mcu_status and
    pose, and alongside cmd_vel_mux.py for mode selection to have any effect.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

_LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
_VALID_MODES = {"none", "teleop"}  # the only two states the page itself can toggle - see setMode()'s comment
_LATCHED_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

# Snap-package environment pollution fix (see the RViz2 "undefined symbol:
# __libc_pthread_init" crash this project hit before): VS Code, itself
# installed as a Snap, leaks these vars into child GUI processes launched
# from its integrated terminal. Strip them before spawning rviz2, since this
# server may itself be running from that same polluted shell.
_SNAP_POLLUTED_VARS = (
    "LOCPATH", "GTK_PATH", "GTK_EXE_PREFIX", "GIO_MODULE_DIR",
    "GTK_IM_MODULE_FILE", "GSETTINGS_SCHEMA_DIR", "XDG_DATA_DIRS",
)

_status_lock = threading.Lock()
_latest_status = {"received": False}

_active_lock = threading.Lock()
_latest_active = {"received": False}

_rviz_lock = threading.Lock()
_rviz_proc = None  # subprocess.Popen, once launched - one-shot, no watchdog

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


def _launch_rviz2():
    """One-shot: spawn a native RViz2 window on this machine's own display
    if one isn't already running. No watchdog, no auto-relaunch - closing
    the window is just closing it (see this file's docstring for why that
    matters: an earlier version fought the user over this)."""
    global _rviz_proc
    with _rviz_lock:
        if _rviz_proc is not None and _rviz_proc.poll() is None:
            return {"status": "already_running", "detail": "RViz2 is already open on this machine."}
        try:
            rviz_config = os.path.join(
                get_package_share_directory("calixto-ros-bot"), "description", "rviz", "rviz_config.rviz"
            )
            env = {k: v for k, v in os.environ.items() if k not in _SNAP_POLLUTED_VARS}
            _rviz_proc = subprocess.Popen(["rviz2", "-d", rviz_config], env=env,
                                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return {"status": "launched", "detail": "RViz2 opening on this machine's screen."}
        except Exception as e:
            return {"status": "error", "detail": f"Failed to launch rviz2: {e}"}


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
        self.create_subscription(LaserScan, "/scan", self._on_scan, _SCAN_QOS)
        self._mode_pub = self.create_publisher(String, "/hmi/cmd_vel_mode", _LATCHED_QOS)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        # Plain Twist (2026-10-01), not TwistStamped - twist_mux (the sole
        # subscriber on this topic) only supports plain geometry_msgs/Twist
        # inputs in the installed version (4.3.0), no stamped variant. A
        # type mismatch here means the two endpoints never connect at all
        # (ROS2 requires an exact type match, topic name alone isn't
        # enough) - confirmed on hardware: keyboard teleop silently
        # produced zero motion despite the mode/topic/subscriber count all
        # looking correct, because of exactly this.
        self._teleop_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        # Software-only "EMERGENCY STOP" (2026-10-02) - see twist_mux.yaml's
        # software_stop entry for exactly what this does and doesn't
        # guarantee (not a real E-stop - no firmware involved, nothing
        # happens if ROS2/the network is down).
        self._estop_pub = self.create_publisher(Twist, "/cmd_vel_estop", 10)
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

    def publish_goal(self, x: float, y: float, theta: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"  # no map/localization yet - see this file's docstring
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        self._goal_pub.publish(msg)
        self.get_logger().info(f"Published /goal_pose: x={x:.2f} y={y:.2f} theta={theta:.2f}")

    def publish_teleop(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._teleop_pub.publish(msg)

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
  .badges { display: flex; gap: 0.5rem; flex-wrap: wrap; }
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
    <h1>Calixto AMR</h1>
    <div class="badges">
      <button id="rviz2-btn" title="Opens a native RViz2 window on the machine running this server - not this browser">Open RViz2</button>
      <span id="active-badge" class="badge unknown">Source: unknown</span>
      <span id="pose-badge" class="badge unknown">Pose: unknown</span>
    </div>
  </header>

  <div class="layout">
    <main class="map-main">
      <canvas id="map-canvas"></canvas>
      <div class="map-hint">Mouse: left-drag sets goal, right-drag pans, scroll zooms &middot;
      Touch: drag pans, pinch zooms, long-press-then-drag sets a goal</div>
      <button id="recenter-btn">Center on robot</button>
      <div id="joystick-base" class="joystick-base"><div id="joystick-knob" class="joystick-knob"></div></div>
    </main>

    <aside class="sidebar">
      <button id="estop-btn" class="estop-btn">EMERGENCY STOP</button>
      <div class="hint">Software-only - commands zero velocity at the highest priority over
      ROS2, but does NOT de-energize the motors (unlike the physical E-stops - wheels still
      resist being pushed by hand) and does nothing if the network/ROS2 stack is down. Use the
      physical E-stop buttons for anything safety-critical.</div>

      <h2>AMR Status</h2>
      <div id="amr-status" class="amr-status-box unknown">No data yet</div>
      <div id="conn" class="hint">connecting...</div>

      <h2>Drive mode</h2>
      <div class="hint">Priority is fixed, not selectable: the RF remote always wins when it's
      actively being used, then this page's keyboard/joystick, then navigation goals - whichever
      of those is actually sending a command drives the robot. The only thing to toggle here is
      whether <em>this browser tab's</em> keyboard/joystick is live.</div>
      <div class="modes">
        <button data-mode="teleop">Keyboard control: OFF</button>
      </div>
      <div id="mode-panel" class="mode-panel"></div>

      <h2>RF remote</h2>
      <div class="mode-panel">
        <strong>Button mapping</strong> (always active, highest priority - no toggle needed):
        <table class="remote-map"><tbody>
          <tr><td>Top</td><td>Drive forward</td></tr>
          <tr><td>Down</td><td>Drive backward</td></tr>
          <tr><td>Left</td><td>Pivot left (CCW)</td></tr>
          <tr><td>Right</td><td>Pivot right (CW)</td></tr>
          <tr><td>Centre</td><td>Stop</td></tr>
        </tbody></table>
        <div class="hint">Run alongside: <code>python3 web_hmi/remote_ros_node.py</code>. Stops
        automatically if the remote goes silent for &gt;1.5s.</div>
      </div>

      <h2>Navigation</h2>
      <div class="mode-panel">
        Nav2 isn't running on the real robot yet (needs LIDAR integration first), but goal-setting
        is already wired up and always active: click and drag on the map to the left to publish a
        goal on /goal_pose (drag direction sets heading). It just has nothing listening yet.
      </div>
    </aside>
  </div>

<script>
const activeBadge = document.getElementById('active-badge');
const poseBadge = document.getElementById('pose-badge');
const connEl = document.getElementById('conn');
const amrStatusEl = document.getElementById('amr-status');
const modeButtons = document.querySelectorAll('button[data-mode]');
const modePanelEl = document.getElementById('mode-panel');
const rviz2Btn = document.getElementById('rviz2-btn');

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

rviz2Btn.addEventListener('click', async () => {
  const prevText = rviz2Btn.textContent;
  rviz2Btn.textContent = '...';
  rviz2Btn.disabled = true;
  try {
    const res = await fetch('/launch_rviz2', { method: 'POST' });
    const data = await res.json();
    rviz2Btn.title = data.detail;
  } catch (e) {
    rviz2Btn.title = 'Failed to reach the server';
  } finally {
    rviz2Btn.textContent = prevText;
    rviz2Btn.disabled = false;
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

async function pollActive() {
  try {
    const res = await fetch('/active.json', { cache: 'no-store' });
    const data = await res.json();
    activeBadge.className = 'badge ' + (data.received && data.source !== 'none' ? 'ok' : 'unknown');
    activeBadge.textContent = data.received ? `Source: ${data.source}` : 'Source: unknown';
  } catch (e) {
    activeBadge.className = 'badge unknown';
    activeBadge.textContent = 'Source: unknown';
  }
}

// Exact key bindings from ros2 run teleop_twist_keyboard teleop_twist_keyboard
// (its own printed banner) - i/,/j/l drive, u/o/m/. are diagonals, k or any
// other key stops.
const TELEOP_PANEL = `<strong>Keyboard teleop</strong> - drives straight from this page, no terminal needed.
    Just press the keys below (this browser tab needs focus, but not any particular element in it):
    <div class="keygrid">
      <span data-key="u">u</span><span data-key="i">i</span><span data-key="o">o</span>
      <span data-key="j">j</span><span data-key="k">k</span><span data-key="l">l</span>
      <span data-key="m">m</span><span data-key=",">,</span><span data-key=".">.</span>
    </div>
    i/, = forward/backward, j/l = pivot left/right, u/o/m/. = diagonal, k or releasing = stop.
    <div class="hint">Same bindings as <code>ros2 run teleop_twist_keyboard</code> - captured by this
    page's own keydown/keyup handlers and published on /cmd_vel_teleop instead.</div>`;

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
  setJoystickVisible(mode === 'teleop');
  try {
    await fetch('/mode', { method: 'POST', body: mode });
  } catch (e) {
    activeBadge.textContent = 'Failed to set mode';
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
const TELEOP_LINEAR_SPEED = 0.5;   // m/s - matches teleop_twist_keyboard's own default
const TELEOP_ANGULAR_SPEED = 1.0;  // rad/s - matches teleop_twist_keyboard's own default
let teleopKey = null;

function teleopPublish(linear, angular) {
  fetch('/teleop', { method: 'POST', body: JSON.stringify({ linear, angular }) }).catch(() => {});
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
  teleopPublish(binding[0] * TELEOP_LINEAR_SPEED, binding[1] * TELEOP_ANGULAR_SPEED);
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
    teleopPublish(binding[0] * TELEOP_LINEAR_SPEED, binding[1] * TELEOP_ANGULAR_SPEED);
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
  teleopPublish(-ny * TELEOP_LINEAR_SPEED, -nx * TELEOP_ANGULAR_SPEED);
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
      ctx.fillStyle = 'rgba(224, 70, 50, 0.65)';
      const a0 = latestScan.angle_min, da = latestScan.angle_increment;
      const rmin = latestScan.range_min, rmax = latestScan.range_max;
      for (let i = 0; i < latestScan.ranges.length; i++) {
        const r = latestScan.ranges[i];
        if (r === null || r < rmin || r > rmax) continue;
        const angle = latestPose.theta + a0 + i * da;
        const wx = latestPose.x + r * Math.cos(angle);
        const wy = latestPose.y + r * Math.sin(angle);
        const [px, py] = worldToPx(wx, wy);
        ctx.beginPath(); ctx.arc(px, py, 1.5 * devicePixelRatio, 0, 7); ctx.fill();
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
  try {
    await fetch('/goal', { method: 'POST', body: JSON.stringify({ x: gx, y: gy, theta: theta }) });
  } catch (e) { /* best-effort */ }
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
    latestPose = data;
    if (!view.initialized) { centerOnRobot(); view.initialized = true; }
    trail.push([data.x, data.y]);
    if (trail.length > 400) trail.shift();
    const age = (Date.now() / 1000) - data.stamp;
    poseBadge.className = 'badge ' + (age > 1.0 ? 'unknown' : 'ok');
    poseBadge.textContent = `Pose: ${data.x.toFixed(2)}, ${data.y.toFixed(2)} (${data.frame_id})`;
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
pollActive();
pollPose();
pollScan();
setInterval(pollStatus, 500);
setInterval(pollActive, 500);
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
            self._send_json(data)
        elif self.path == "/scan.json":
            with _scan_lock:
                data = dict(_latest_scan)
            self._send_json(data)
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
        elif self.path == "/teleop":
            try:
                data = json.loads(raw.decode("utf-8"))
                linear, angular = float(data["linear"]), float(data["angular"])
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self._send_json({"status": "error", "detail": f"bad teleop payload: {e}"}, status=400)
                return
            self.server.hmi_node.publish_teleop(linear, angular)
            self._send_json({"status": "ok"})
        elif self.path == "/estop":
            self.server.hmi_node.publish_estop()
            self._send_json({"status": "ok"})
        elif self.path == "/launch_rviz2":
            self._send_json(_launch_rviz2())
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
