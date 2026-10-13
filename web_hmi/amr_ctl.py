#!/usr/bin/env python3
"""amr-ctl - restart helper that lives OUTSIDE the ROS stack (2026-10-12).

Why this exists
---------------
The web UI (status_web_server.py) runs inside the ROS stack it is asked to
restart, so its own "Restart" could never really tear the stack down: when it
worked it killed its own page, and when nav2 or the stack was wedged the
request just hung and the page looked frozen. This small server runs on the
robot HOST under systemd (amr-ctl.service), independent of the container and
of the stack, so it survives the restart it performs and can report each
step to the page (the page polls it on port 8081 and shows a progress card).

What it does
------------
POST /ctl/restart  {"mode": "navigation" | "everything", "map": "<map name>"}
    navigation : stop nav2 -> start nav2 on <map> -> wait until its nodes are active
    everything : stop nav2 -> stop the robot software (amr-stack) -> DESTROY the
                 ros2_humble container's processes (docker restart: nothing
                 survives, zombies included) -> start amr-stack -> wait for the
                 motor controller / LiDAR / odometry -> start nav2 -> wait.
    Navigation is started with start_navigation.sh's default params file.
GET  /ctl/job      progress of the current / last restart (stages, state, error)
GET  /ctl/status   is the helper itself up, is amr-stack active, is the container up

Every step has a timeout and an escalation (graceful stop -> kill), and the
first step that cannot be completed ends the job as "failed" with the reason,
instead of hanging. Only one restart runs at a time.

Trust model: no authentication - same as the web UI itself (anyone on the LAN
who can open the page can already e-stop and teleop the robot). Port 8081.

Install (on the robot, as root):  bash docker/install_amr_ctl.sh
Standard library only - nothing to pip install.
"""
import argparse
import json
import re
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CONTAINER = "ros2_humble"
DOCKER_DIR = "/ros2_ws/src/ros-amr/docker"      # path INSIDE the container
UI_URL = "http://127.0.0.1:8080"                 # the web UI (container uses host networking)
MAP_RE = re.compile(r"^[A-Za-z0-9_.-]+$")
# Leftover navigation processes. The [x] bracket trick stops pkill -f from
# matching the shell that is running pkill (whose command line contains this text).
NAV_KILL = ("[n]av2_|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|"
            "[s]moother_server|[l]ifecycle_manager|[v]elocity_smoother|[a]mcl|[m]ap_server|[w]aypoint_follower")

_lock = threading.Lock()
_job = {"id": 0, "state": "idle", "mode": "", "stages": [], "error": None, "started": 0.0, "finished": 0.0}


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def run(cmd, timeout):
    """(returncode, combined output). 124 = timed out."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return 124, f"timed out after {timeout}s"
    except OSError as e:
        return 127, str(e)


def dx(shell_cmd, timeout):
    """Run a command inside the ROS container (login shell, so ROS is sourced)."""
    return run(["docker", "exec", CONTAINER, "bash", "-lc", shell_cmd], timeout)


def readiness():
    """The web UI's own view of the robot (None if the UI isn't answering)."""
    try:
        with urllib.request.urlopen(UI_URL + "/readiness.json", timeout=3) as r:
            return json.loads(r.read().decode())
    except Exception:
        return None


def container_running():
    rc, out = run(["docker", "inspect", "-f", "{{.State.Running}}", CONTAINER], 10)
    return rc == 0 and out.strip() == "true"


# ---------------------------------------------------------------- stages
# Each returns (ok, detail) and may call ctx["say"](text) to update the page.

def st_stop_navigation(ctx):
    if not container_running():
        return True, "container not running - nothing to stop"
    rc, out = dx(f"bash {DOCKER_DIR}/stop_navigation.sh", 35)
    if rc == 0:
        return True, "stopped"
    # stop_navigation.sh hung or failed (it did on 2026-10-09): kill what is left.
    ctx["say"]("stop script failed - killing leftover navigation processes")
    dx(f"pkill -9 -f '{NAV_KILL}'; rm -f /tmp/navigation.pids; true", 15)
    return True, "stop script failed (" + out.splitlines()[-1][:60] + ") - killed the leftovers" if out else "killed the leftovers"


def st_stop_stack(ctx):
    rc, out = run(["systemctl", "stop", "amr-stack"], 90)
    if rc != 0:
        ctx["say"]("graceful stop failed - forcing it")
        run(["systemctl", "kill", "-s", "SIGKILL", "amr-stack"], 10)
        run(["systemctl", "stop", "amr-stack"], 30)
    state = run(["systemctl", "is-active", "amr-stack"], 10)[1]
    return state != "active", f"amr-stack {state}"


def st_destroy(ctx):
    rc, out = run(["docker", "restart", "-t", "5", CONTAINER], 90)
    if rc != 0:
        return False, "docker restart failed: " + out[-120:]
    for i in range(30):
        if container_running():
            time.sleep(2)
            # docker restart keeps the container's files: marker files left by
            # processes that no longer exist would make the page think mapping /
            # navigation is still active (the page treats /tmp/mapping.pids
            # existing as "mapping is running"). Everything they pointed at is dead.
            dx("rm -f /tmp/mapping.pids /tmp/navigation.pids", 10)
            return True, "container restarted - no old process survives, stale markers cleared"
        time.sleep(1)
    return False, "container did not come back"


def st_start_stack(ctx):
    rc, out = run(["systemctl", "start", "amr-stack"], 30)
    return rc == 0, "amr-stack started" if rc == 0 else out[-120:]


def st_wait_base(ctx):
    deadline = time.time() + 200
    t0 = time.time()
    while time.time() < deadline:
        r = readiness()
        waited = int(time.time() - t0)
        if r is None:
            ctx["say"](f"waiting for the web UI to come up ({waited}s)")
        elif r.get("state") not in ("base_down", "unknown"):
            return True, f"robot software up after {waited}s"
        else:
            ctx["say"](f"waiting for: {r.get('detail', '')[:80]} ({waited}s)")
        time.sleep(3)
    return False, "robot software did not come up within 200 s (see: journalctl -u amr-stack)"


def st_start_navigation(ctx):
    if not ctx["map"]:
        return True, "no map selected - navigation not started"
    rc, out = dx(f"bash {DOCKER_DIR}/start_navigation.sh {ctx['map']}", 180)
    return rc == 0, (out.splitlines()[-1][:100] if out else "started") if rc == 0 else ("start script failed: " + out[-140:])


def st_wait_navigation(ctx):
    if not ctx["map"]:
        return True, "skipped"
    deadline = time.time() + 180
    t0 = time.time()
    while time.time() < deadline:
        r = readiness()
        waited = int(time.time() - t0)
        if r and r.get("state") in ("need_pose", "aligning", "ready"):
            return True, f"navigation active after {waited}s"
        ctx["say"]((r.get("detail", "")[:90] if r else "waiting for the web UI") + f" ({waited}s)")
        time.sleep(3)
    return False, "navigation nodes did not all become active within 180 s"


STAGES = {
    "navigation": [
        ("Stop navigation", st_stop_navigation),
        ("Start navigation", st_start_navigation),
        ("Wait for navigation to be active", st_wait_navigation),
    ],
    "everything": [
        ("Stop navigation", st_stop_navigation),
        ("Stop the robot software", st_stop_stack),
        ("Destroy every leftover process", st_destroy),
        ("Start the robot software", st_start_stack),
        ("Wait for motor controller, LiDAR and odometry", st_wait_base),
        ("Start navigation", st_start_navigation),
        ("Wait for navigation to be active", st_wait_navigation),
    ],
}


def run_job(mode, map_name):
    stages = STAGES[mode]
    with _lock:
        _job.update(id=_job["id"] + 1, state="running", mode=mode, error=None, started=time.time(), finished=0.0,
                    stages=[{"name": n, "state": "pending", "detail": ""} for n, _ in stages])
    log(f"restart job started: mode={mode} map={map_name!r}")
    for i, (name, fn) in enumerate(stages):
        def say(text, i=i):
            with _lock:
                _job["stages"][i]["detail"] = text
        with _lock:
            _job["stages"][i]["state"] = "running"
        try:
            ok, detail = fn({"map": map_name, "say": say})
        except Exception as e:  # a bug here must still end the job cleanly
            ok, detail = False, f"unexpected error: {e}"
        log(f"  [{i + 1}/{len(stages)}] {name}: {'ok' if ok else 'FAILED'} - {detail}")
        with _lock:
            _job["stages"][i].update(state="done" if ok else "failed", detail=detail)
            if not ok:
                _job.update(state="failed", error=f"{name}: {detail}", finished=time.time())
                return
    with _lock:
        _job.update(state="done", finished=time.time())
    log("restart job finished OK")


def start_job(mode, map_name):
    with _lock:
        if _job["state"] == "running":
            return False, "a restart is already running"
    threading.Thread(target=run_job, args=(mode, map_name), daemon=True).start()
    time.sleep(0.2)  # let the job mark itself running before the page's first poll
    return True, ""


def snapshot():
    with _lock:
        j = json.loads(json.dumps(_job))
    end = time.time() if j["state"] == "running" else (j["finished"] or time.time())
    j["elapsed"] = max(0.0, end - j["started"]) if j["started"] else 0.0
    return j


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # keep the journal quiet - the job log is what matters
        pass

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")

    def _send(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Content-Length", "0")
        self._cors()
        self.end_headers()

    def do_GET(self):
        if self.path == "/ctl/job":
            self._send(snapshot())
        elif self.path == "/ctl/status":
            self._send({"ok": True, "amr_stack": run(["systemctl", "is-active", "amr-stack"], 5)[1],
                        "container_running": container_running(), "job": snapshot()["state"]})
        else:
            self._send({"ok": False, "error": "not found"}, 404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        if self.path != "/ctl/restart":
            self._send({"ok": False, "error": "not found"}, 404)
            return
        try:
            data = json.loads(raw.decode() or "{}")
            mode = data.get("mode")
            map_name = data.get("map", "") or ""
        except (ValueError, UnicodeDecodeError):
            self._send({"ok": False, "error": "bad request body"}, 400)
            return
        if mode not in STAGES:
            self._send({"ok": False, "error": "mode must be 'navigation' or 'everything'"}, 400)
            return
        if map_name and not MAP_RE.match(map_name):
            self._send({"ok": False, "error": "bad map name"}, 400)
            return
        ok, err = start_job(mode, map_name)
        self._send({"ok": ok, "error": err}, 200 if ok else 409)


def main():
    ap = argparse.ArgumentParser(description="AMR restart helper (runs on the robot host)")
    ap.add_argument("--port", type=int, default=8081)
    args = ap.parse_args()
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    srv.daemon_threads = True
    log(f"amr-ctl listening on :{args.port}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
