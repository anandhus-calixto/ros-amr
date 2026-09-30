#!/usr/bin/env python3
"""Minimal status-only web HMI - Phase 1 (2026-09-30).

Subscribes to /mcu_status (published by calixto-ros-bot's
DiffBotSystemHardware - see hardware/diffbot_system.cpp) via rclpy, and
serves it as a simple polled JSON endpoint plus a static HTML page that
polls it. No new system dependencies beyond what's already required -
just rclpy (already needed for this whole workspace) and Python's stdlib
http.server.

This is a deliberately minimal first phase of a larger web HMI plan (see
the design discussion this was born from): prove the real E-stop/bumper/
fault status data reaches a browser before adding RViz-equivalent 3D
visualization or command-source switching (remote/teleop/nav). Later
phases will very likely use rosbridge_suite + roslibjs instead, since
those need genuine two-way ROS2<->browser communication (visualization
subscriptions AND publishing commands) that a simple polled HTTP endpoint
isn't suited for - this script is intentionally read-only and
single-purpose, not a foundation to keep extending in place.

Usage:
    source /opt/ros/humble/setup.bash
    source <workspace>/install/setup.bash   # for the diagnostic_msgs Python bindings
    python3 status_web_server.py [--port 8080]

    Then open http://localhost:8080 in a browser. Works alongside any
    ros2_control launch (diffbot.launch.py / diffbot_no_ekf.launch.py) that
    has /mcu_status being published - this script only subscribes, it
    doesn't launch or depend on any particular launch file.
"""
import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

_LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}

_status_lock = threading.Lock()
_latest_status = {"received": False}


def _level_to_int(level) -> int:
    """DiagnosticStatus.level is a ROS 'byte' - rclpy hands it back as a
    length-1 bytes object in some message versions, a plain int in others."""
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else 0
    return int(level)


class McuStatusSubscriber(Node):
    def __init__(self):
        super().__init__("mcu_status_web_bridge")
        self.create_subscription(DiagnosticArray, "/mcu_status", self._on_status, 10)
        self.get_logger().info("Subscribed to /mcu_status - waiting for data...")

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


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Calixto AMR - Status</title>
<style>
  :root { color-scheme: light dark; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    max-width: 640px; margin: 2rem auto; padding: 0 1rem;
    background: #111318; color: #e8e8ea;
  }
  h1 { font-size: 1.3rem; }
  #conn { font-size: 0.85rem; color: #888; margin-bottom: 1rem; }
  .banner {
    padding: 0.9rem 1rem; border-radius: 8px; font-weight: 600;
    margin-bottom: 1rem; font-size: 1.05rem;
  }
  .banner.ok    { background: #1f4d2e; color: #8ce99a; }
  .banner.warn  { background: #4d3f1f; color: #ffd43b; }
  .banner.error { background: #4d1f1f; color: #ff6b6b; }
  .banner.unknown { background: #2a2d34; color: #888; }
  table { width: 100%; border-collapse: collapse; margin-top: 0.5rem; }
  td, th { text-align: left; padding: 0.4rem 0.6rem; border-bottom: 1px solid #2a2d34; }
  th { color: #888; font-weight: 500; font-size: 0.85rem; }
  .val-active { color: #ff6b6b; font-weight: 600; }
  .val-ok { color: #8ce99a; }
</style>
</head>
<body>
  <h1>Calixto AMR - MCU Status</h1>
  <div id="conn">connecting...</div>
  <div id="banner" class="banner unknown">No data yet</div>
  <table id="values"><tbody></tbody></table>

<script>
const bannerEl = document.getElementById('banner');
const connEl = document.getElementById('conn');
const valuesEl = document.getElementById('values').querySelector('tbody');

function levelClass(levelName) {
  if (levelName === 'OK') return 'ok';
  if (levelName === 'WARN') return 'warn';
  if (levelName === 'ERROR') return 'error';
  return 'unknown';
}

function isActiveValue(v) {
  return v === 'ACTIVE' || v === 'TRIPPED' || v === 'true';
}

async function poll() {
  try {
    const res = await fetch('/status.json', { cache: 'no-store' });
    const data = await res.json();
    if (!data.received) {
      connEl.textContent = 'Waiting for /mcu_status ...';
      bannerEl.className = 'banner unknown';
      bannerEl.textContent = 'No data yet';
      return;
    }
    const age = (Date.now() / 1000) - data.stamp;
    connEl.textContent = `hardware_id: ${data.hardware_id}  |  last update ${age.toFixed(1)}s ago` +
      (age > 2.0 ? '  (STALE - is the stack still running?)' : '');

    bannerEl.className = 'banner ' + levelClass(data.level_name);
    bannerEl.textContent = `${data.level_name}: ${data.message}`;

    valuesEl.innerHTML = '';
    for (const [key, value] of Object.entries(data.values)) {
      const tr = document.createElement('tr');
      const tdKey = document.createElement('td');
      tdKey.textContent = key;
      const tdVal = document.createElement('td');
      tdVal.textContent = value;
      tdVal.className = isActiveValue(value) ? 'val-active' : 'val-ok';
      tr.appendChild(tdKey);
      tr.appendChild(tdVal);
      valuesEl.appendChild(tr);
    }
  } catch (e) {
    connEl.textContent = 'Cannot reach status server: ' + e;
    bannerEl.className = 'banner unknown';
  }
}

poll();
setInterval(poll, 500);
</script>
</body>
</html>
"""


class StatusHTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # keep the console quiet - this is a status page, not a web server demo

    def do_GET(self):
        if self.path == "/status.json":
            with _status_lock:
                body = json.dumps(_latest_status).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    rclpy.init()
    node = McuStatusSubscriber()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), StatusHTTPHandler)
    print(f"Status web page: http://localhost:{args.port}  (Ctrl+C to stop)")
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
