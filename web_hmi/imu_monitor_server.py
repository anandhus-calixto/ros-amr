#!/usr/bin/env python3
"""BNO055 IMU verification page (2026-10-01).

Standalone bring-up/verification tool for the BNO055 IMU wired to the
i.MX8M Plus board's I2C5 (expansion connector J22, address 0x28) and read
by the `bno055` ROS2 driver (github.com/flynneva/bno055). This is
deliberately separate from status_web_server.py - it's a hardware bring-up
aid, not part of the robot's normal driving HMI, and has no dependency on
the rest of that page's state (mcu_status/pose/drive-mode).

Subscribes to:
  - /bno055/imu (sensor_msgs/Imu) - orientation quaternion, angular
    velocity, linear acceleration.
  - /bno055/mag (sensor_msgs/MagneticField)
  - /bno055/temp (sensor_msgs/Temperature)
  - /bno055/calib_status (std_msgs/String) - JSON string, e.g.
    '{"sys": 0, "gyro": 3, "accel": 3, "mag": 3}', each 0-3.

Each callback stamps the value with the server's own wall-clock time
(time.time()), not the message's own header stamp - the page uses that
stamp to detect staleness (driver crashed / I2C died / node not running)
independent of any clock sync between browser and board.

Usage (inside the ros2_humble container, after `ros2 launch bno055
bno055.launch.py` is already running):
    source /opt/ros/humble/setup.bash
    python3 imu_monitor_server.py [--port 8090]

    Then open http://<board-ip>:8090 from any device on the LAN.
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String

_imu_lock = threading.Lock()
_latest_imu = {"received": False}

_mag_lock = threading.Lock()
_latest_mag = {"received": False}

_temp_lock = threading.Lock()
_latest_temp = {"received": False}

_calib_lock = threading.Lock()
_latest_calib = {"received": False}


def _quat_to_euler_deg(x, y, z, w):
    """ZYX Euler (roll/pitch/yaw), degrees - standard aerospace convention,
    matching what an artificial-horizon-style display expects."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class ImuMonitorNode(Node):
    def __init__(self):
        super().__init__("imu_monitor_server")
        self.create_subscription(Imu, "/bno055/imu", self._on_imu, 10)
        self.create_subscription(MagneticField, "/bno055/mag", self._on_mag, 10)
        self.create_subscription(Temperature, "/bno055/temp", self._on_temp, 10)
        self.create_subscription(String, "/bno055/calib_status", self._on_calib, 10)
        self.get_logger().info("imu_monitor_server: subscribed to /bno055/*")

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = _quat_to_euler_deg(q.x, q.y, q.z, q.w)
        data = {
            "received": True,
            "recv_time": time.time(),
            "quat": {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
            "euler_deg": {"roll": roll, "pitch": pitch, "yaw": yaw},
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
        }
        with _imu_lock:
            _latest_imu.clear()
            _latest_imu.update(data)

    def _on_mag(self, msg: MagneticField):
        data = {
            "received": True,
            "recv_time": time.time(),
            "field": {
                "x": msg.magnetic_field.x,
                "y": msg.magnetic_field.y,
                "z": msg.magnetic_field.z,
            },
        }
        with _mag_lock:
            _latest_mag.clear()
            _latest_mag.update(data)

    def _on_temp(self, msg: Temperature):
        data = {"received": True, "recv_time": time.time(), "temperature": msg.temperature}
        with _temp_lock:
            _latest_temp.clear()
            _latest_temp.update(data)

    def _on_calib(self, msg: String):
        try:
            parsed = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            parsed = {}
        data = {"received": True, "recv_time": time.time(), **parsed}
        with _calib_lock:
            _latest_calib.clear()
            _latest_calib.update(data)


HTML_PAGE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BNO055 IMU Monitor</title>
<style>
  body { margin: 0; background: #11151a; color: #e8ecf1; font-family: system-ui, sans-serif; }
  #wrap { max-width: 860px; margin: 0 auto; padding: 16px; }
  h1 { font-size: 1.1rem; font-weight: 600; margin: 0 0 12px; display: flex; align-items: center; gap: 10px; }
  #staleDot { width: 12px; height: 12px; border-radius: 50%; background: #3ecf6a; flex-shrink: 0; }
  #staleDot.stale { background: #e24d4d; animation: blink 1s infinite; }
  @keyframes blink { 50% { opacity: 0.3; } }
  #staleMsg { font-size: 0.85rem; color: #9aa4b2; }
  #staleMsg.stale { color: #e24d4d; font-weight: 600; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  @media (max-width: 640px) { .grid { grid-template-columns: 1fr; } }
  .card { background: #1a2029; border-radius: 10px; padding: 14px 16px; }
  .card h2 { font-size: 0.8rem; text-transform: uppercase; letter-spacing: 0.04em; color: #9aa4b2; margin: 0 0 10px; }
  .row { display: flex; justify-content: space-between; padding: 3px 0; font-variant-numeric: tabular-nums; font-size: 0.92rem; }
  .row span:first-child { color: #9aa4b2; }
  canvas { display: block; margin: 0 auto; background: #000; border-radius: 50%; }
  .calibRow { display: flex; gap: 8px; flex-wrap: wrap; }
  .badge { flex: 1; min-width: 70px; text-align: center; padding: 8px 4px; border-radius: 6px; font-size: 0.85rem; }
  .badge .label { display: block; font-size: 0.7rem; color: rgba(0,0,0,0.6); text-transform: uppercase; }
  .badge .val { display: block; font-size: 1.3rem; font-weight: 700; }
  .c0 { background: #e24d4d; } .c1 { background: #e2a04d; } .c2 { background: #e2d34d; color:#000;} .c3 { background: #3ecf6a; }
</style>
</head>
<body>
<div id="wrap">
  <h1><span id="staleDot"></span>BNO055 IMU Monitor <span id="staleMsg"></span></h1>
  <div class="grid">
    <div class="card">
      <h2>Attitude</h2>
      <canvas id="horizon" width="220" height="220"></canvas>
      <div class="row"><span>Roll</span><span id="roll">-</span></div>
      <div class="row"><span>Pitch</span><span id="pitch">-</span></div>
      <div class="row"><span>Yaw</span><span id="yaw">-</span></div>
    </div>
    <div class="card">
      <h2>Calibration (0-3, 3 = fully calibrated)</h2>
      <div class="calibRow">
        <div class="badge" id="cal-sys"><span class="label">Sys</span><span class="val">-</span></div>
        <div class="badge" id="cal-gyro"><span class="label">Gyro</span><span class="val">-</span></div>
        <div class="badge" id="cal-accel"><span class="label">Accel</span><span class="val">-</span></div>
        <div class="badge" id="cal-mag"><span class="label">Mag</span><span class="val">-</span></div>
      </div>
      <h2 style="margin-top:14px">Temperature</h2>
      <div class="row"><span>Board temp</span><span id="temp">-</span></div>
    </div>
    <div class="card">
      <h2>Orientation quaternion</h2>
      <div class="row"><span>x</span><span id="qx">-</span></div>
      <div class="row"><span>y</span><span id="qy">-</span></div>
      <div class="row"><span>z</span><span id="qz">-</span></div>
      <div class="row"><span>w</span><span id="qw">-</span></div>
    </div>
    <div class="card">
      <h2>Angular velocity (rad/s)</h2>
      <div class="row"><span>x</span><span id="gx">-</span></div>
      <div class="row"><span>y</span><span id="gy">-</span></div>
      <div class="row"><span>z</span><span id="gz">-</span></div>
      <h2 style="margin-top:14px">Linear acceleration (m/s&sup2;)</h2>
      <div class="row"><span>x</span><span id="ax">-</span></div>
      <div class="row"><span>y</span><span id="ay">-</span></div>
      <div class="row"><span>z</span><span id="az">-</span></div>
    </div>
    <div class="card">
      <h2>Magnetic field (&mu;T)</h2>
      <div class="row"><span>x</span><span id="mx">-</span></div>
      <div class="row"><span>y</span><span id="my">-</span></div>
      <div class="row"><span>z</span><span id="mz">-</span></div>
    </div>
  </div>
</div>
<script>
const canvas = document.getElementById('horizon');
const ctx = canvas.getContext('2d');
function drawHorizon(rollDeg, pitchDeg) {
  const w = canvas.width, h = canvas.height, cx = w/2, cy = h/2, r = w/2 - 2;
  ctx.clearRect(0, 0, w, h);
  ctx.save();
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI*2); ctx.clip();
  ctx.translate(cx, cy);
  ctx.rotate(-rollDeg * Math.PI / 180);
  const pitchPx = Math.max(-r, Math.min(r, pitchDeg * 2.2));
  ctx.fillStyle = '#2b6cb0'; ctx.fillRect(-w, -h*2 + pitchPx, w*2, h*2);
  ctx.fillStyle = '#8a5a2b'; ctx.fillRect(-w, pitchPx, w*2, h*2);
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(-w, pitchPx); ctx.lineTo(w, pitchPx); ctx.stroke();
  ctx.restore();
  ctx.strokeStyle = '#ffcc00'; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(cx - 30, cy); ctx.lineTo(cx - 8, cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx + 8, cy); ctx.lineTo(cx + 30, cy); ctx.stroke();
  ctx.beginPath(); ctx.arc(cx, cy, 3, 0, Math.PI*2); ctx.fillStyle = '#ffcc00'; ctx.fill();
  ctx.strokeStyle = 'rgba(255,255,255,0.4)'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI*2); ctx.stroke();
}
function fmt(v, d=2) { return (typeof v === 'number') ? v.toFixed(d) : '-'; }
function calibClass(v) { return 'badge c' + (typeof v === 'number' ? v : 0); }

let lastImuRecv = null;
let staleStreak = 0;

async function poll() {
  try {
    const [imu, calib, temp, mag] = await Promise.all([
      fetch('/imu.json').then(r => r.json()),
      fetch('/calib.json').then(r => r.json()),
      fetch('/temp.json').then(r => r.json()),
      fetch('/mag.json').then(r => r.json()),
    ]);

    const dot = document.getElementById('staleDot');
    const msg = document.getElementById('staleMsg');
    if (imu.received) {
      if (imu.recv_time === lastImuRecv) {
        staleStreak++;
      } else {
        staleStreak = 0;
        lastImuRecv = imu.recv_time;
      }
    } else {
      staleStreak = 999;
    }
    const isStale = staleStreak >= 5; // ~1s of identical/no data at 200ms poll
    dot.className = isStale ? 'stale' : '';
    msg.className = isStale ? 'stale' : '';
    msg.textContent = isStale
      ? (imu.received ? 'NO NEW DATA - driver/node may have died' : 'NOT RECEIVING - is bno055 launched?')
      : 'live';

    if (imu.received) {
      document.getElementById('roll').textContent = fmt(imu.euler_deg.roll) + '°';
      document.getElementById('pitch').textContent = fmt(imu.euler_deg.pitch) + '°';
      document.getElementById('yaw').textContent = fmt(imu.euler_deg.yaw) + '°';
      document.getElementById('qx').textContent = fmt(imu.quat.x, 4);
      document.getElementById('qy').textContent = fmt(imu.quat.y, 4);
      document.getElementById('qz').textContent = fmt(imu.quat.z, 4);
      document.getElementById('qw').textContent = fmt(imu.quat.w, 4);
      document.getElementById('gx').textContent = fmt(imu.angular_velocity.x);
      document.getElementById('gy').textContent = fmt(imu.angular_velocity.y);
      document.getElementById('gz').textContent = fmt(imu.angular_velocity.z);
      document.getElementById('ax').textContent = fmt(imu.linear_acceleration.x);
      document.getElementById('ay').textContent = fmt(imu.linear_acceleration.y);
      document.getElementById('az').textContent = fmt(imu.linear_acceleration.z);
      drawHorizon(imu.euler_deg.roll, imu.euler_deg.pitch);
    }
    if (calib.received) {
      for (const k of ['sys', 'gyro', 'accel', 'mag']) {
        const el = document.getElementById('cal-' + k);
        el.className = calibClass(calib[k]);
        el.querySelector('.val').textContent = (k in calib) ? calib[k] : '-';
      }
    }
    if (temp.received) document.getElementById('temp').textContent = fmt(temp.temperature, 1) + ' °C';
    if (mag.received) {
      document.getElementById('mx').textContent = fmt(mag.field.x * 1e6);
      document.getElementById('my').textContent = fmt(mag.field.y * 1e6);
      document.getElementById('mz').textContent = fmt(mag.field.z * 1e6);
    }
  } catch (e) {
    document.getElementById('staleDot').className = 'stale';
    document.getElementById('staleMsg').className = 'stale';
    document.getElementById('staleMsg').textContent = 'page cannot reach server';
  }
}
setInterval(poll, 200);
poll();
</script>
</body>
</html>
"""


class ImuHTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _send_json(self, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/imu.json":
            with _imu_lock:
                data = dict(_latest_imu)
            self._send_json(data)
        elif self.path == "/mag.json":
            with _mag_lock:
                data = dict(_latest_mag)
            self._send_json(data)
        elif self.path == "/temp.json":
            with _temp_lock:
                data = dict(_latest_temp)
            self._send_json(data)
        elif self.path == "/calib.json":
            with _calib_lock:
                data = dict(_latest_calib)
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8090)
    args = ap.parse_args()

    rclpy.init()
    node = ImuMonitorNode()

    def _spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
        except Exception as e:
            print(f"[imu_monitor] ROS spin thread crashed: {e!r}", file=sys.stderr, flush=True)
        os._exit(0)

    threading.Thread(target=_spin, daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), ImuHTTPHandler)
    print(f"IMU monitor page: http://0.0.0.0:{args.port}  (Ctrl+C to stop)")
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
