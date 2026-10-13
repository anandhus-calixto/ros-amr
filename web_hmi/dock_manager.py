"""Dock / undock / home position for the web page (2026-10-13).

The dock IS the home position. Docking itself is live - scripts/dock.py finds the
V-shaped dock with the LiDAR every time, so the dock can be anywhere. What is
stored per map (maps/<map>.dock.json) is only the pose of the robot when it sits
docked, so the page can (a) set the initial pose from the dock and (b) drive to a
spot in front of the dock with Nav2 before docking. Moving the dock = re-teach:
set the pose by hand, Dock, then "Save home" (the old file is kept as .bak).

This module is plain Python (no rclpy at import time); the web server hands it
small callbacks ("hooks") so it never touches the server's globals.
"""
import collections
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(_HERE, "..", "scripts")
_MAPS_DIR = os.path.join(_HERE, "..", "maps")
DOCK_SCRIPT = os.path.join(_SCRIPTS, "dock.py")

HALF_LEN = 0.39                      # robot centre -> rear edge
DOCKED_MIN, DOCKED_MAX = 0.088, 0.118    # rear edge -> V tip while docked (stop point is ~0.10)
DOCKED_TIP_FROM_CENTRE = HALF_LEN + 0.101
APPROACH_TIP_DIST = 1.5              # Nav2 brings the robot to 1.5 m from the V tip, facing it (when no waypoint was taught)
LIVE_MIN, LIVE_MAX = 0.78, 2.5       # axis distance range dock.py can start from
UNDOCK_MAX_REAR = 0.30               # undock is offered while the rear V is within this

_dm = None
_dm_mtime = 0.0


def _dock_module():
    """scripts/dock.py imported lazily for its V detector (numpy only at call time)."""
    global _dm, _dm_mtime
    mt = os.path.getmtime(DOCK_SCRIPT)
    if _dm is None:
        if _SCRIPTS not in sys.path:
            sys.path.insert(0, _SCRIPTS)
        import dock as m
        _dm = m
        _dm_mtime = mt
    elif mt != _dm_mtime:           # dock.py was edited: pick up the new detector without a server restart
        import importlib
        _dm = importlib.reload(_dm)
        _dm_mtime = mt
    return _dm


# ----------------------------------------------------------------- home file
def _home_path(map_name):
    return os.path.join(_MAPS_DIR, map_name + ".dock.json")


def load_home(map_name):
    try:
        with open(_home_path(map_name)) as f:
            d = json.load(f)
        return d if all(k in d for k in ("x", "y", "theta")) else None
    except (OSError, ValueError):
        return None


def save_home(map_name, x, y, theta):
    os.makedirs(_MAPS_DIR, exist_ok=True)
    path = _home_path(map_name)
    if os.path.isfile(path):
        try:
            os.replace(path, path + ".bak")        # keep the previous home so a bad save can be undone
        except OSError:
            pass
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"x": x, "y": y, "theta": theta, "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "note": "pose of the robot (base_link) in the map frame while docked, rear to the dock"}, f, indent=2)
    os.replace(tmp, path)


def _wp_path(map_name):
    return os.path.join(_MAPS_DIR, map_name + ".dockwp.json")


def load_wp(map_name):
    """The taught dock waypoint: a map pose ~1.5 m in front of the V, robot facing it."""
    try:
        with open(_wp_path(map_name)) as f:
            d = json.load(f)
        return d if all(k in d for k in ("x", "y", "theta")) else None
    except (OSError, ValueError):
        return None


def save_wp(map_name, x, y, theta, a):
    os.makedirs(_MAPS_DIR, exist_ok=True)
    path = _wp_path(map_name)
    if os.path.isfile(path):
        try:
            os.replace(path, path + ".bak")
        except OSError:
            pass
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"x": x, "y": y, "theta": theta, "v_axis_dist": a, "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "note": "robot (base_link) pose in the map frame, facing the V, taught by the user"}, f, indent=2)
    os.replace(tmp, path)


def approach_pose(home):
    """Pose in front of the dock: tip + APPROACH_TIP_DIST along the dock axis, robot facing the dock."""
    th = home["theta"]
    shift = APPROACH_TIP_DIST - DOCKED_TIP_FROM_CENTRE
    return (home["x"] + shift * math.cos(th), home["y"] + shift * math.sin(th), th + math.pi)


# --------------------------------------------------------------- dock vision
def view(scan):
    """What the LiDAR sees of the dock right now: front fit / rear fit (or None)."""
    out = {"front": None, "rear": None, "live_ok": False, "behind_ok": False, "docked": False, "rear_near": False}
    if scan is None:
        return out
    m = _dock_module()
    f = m.detect_v(scan, "front")
    r = m.detect_v(scan, "rear")
    if f:
        out["front"] = {"a": f["a"], "lat": f["lat"], "psi": math.degrees(f["psi"])}
        out["live_ok"] = LIVE_MIN <= f["a"] <= LIVE_MAX and abs(math.degrees(f["psi"])) <= 35
    if r:
        dr = r["tx"] - HALF_LEN
        out["rear"] = {"dr": dr, "lat": r["lat"], "psi": math.degrees(r["psi"])}
        out["docked"] = DOCKED_MIN <= dr <= DOCKED_MAX and abs(r["lat"]) < 0.05 and abs(math.degrees(r["psi"])) < 8
        out["rear_near"] = dr <= UNDOCK_MAX_REAR
        out["behind_ok"] = (not f) and 0.74 <= r["tx"] <= LIVE_MAX      # 0.74: the robot sits ~0.79 m from the tip after a 0.3 m undock and abs(math.degrees(r["psi"])) <= 35
    return out


_STAGE_MARKS = (            # substring of a dock.py output line -> what the page shows
    ("BEHIND the robot", "Turning to face the dock"),
    ("turning to look", "Looking for the dock"),
    ("FOUND the V", "Found the dock"),
    ("start:", "Looking at the dock"),
    ("align:", "Lining up with the dock"),
    ("ALIGNED", "Lined up - turning around"),
    ("SPUN", "Turned around"),
    ("blind reverse", "Reversing toward the dock"),
    ("REVERSED", "Checking the rear view of the dock"),
    ("REAR VIEW", "Creeping in slowly"),
    ("creep:", "Creeping in slowly"),
    ("STOPPED", "Stopped at the dock"),
    ("DOCKED", "Docked"),
    ("undocking", "Driving off the dock"),
    ("UNDOCKED", "Undocked"),
)


class DockManager:
    """hooks (all callables, supplied by the web server):
        get_scan()                    latest LaserScan message or None
        ready_state()                 readiness state string ('ready', 'need_pose', ...)
        map_pose()                    robot pose in the map frame (x, y, theta) or None
        request_pose(x, y, theta)     set the initial pose
        publish_goal(x, y, theta)     Nav2 goal
        goal_prev_id() / wait_goal(prev_id, stop_event) / cancel_nav()
        loop_running()                a waypoint loop is running
        set_busy(bool)                tell the server to ignore page joystick while a job runs
    """

    def __init__(self, hooks):
        self.h = hooks
        self.lock = threading.Lock()
        self.job = {"state": "idle", "kind": "", "stage": "", "detail": "", "log": [], "started": 0.0, "finished": 0.0}
        self._log = collections.deque(maxlen=40)
        self._proc = None
        self._stop = threading.Event()
        self._view = (0.0, None)

    # ---- status for the page
    def _cached_view(self):
        t, v = self._view
        if time.monotonic() - t < 0.7 and v is not None:
            return v
        v = view(self.h["get_scan"]())
        self._view = (time.monotonic(), v)
        return v

    def status(self, map_name):
        with self.lock:
            job = dict(self.job)
            job["log"] = list(self._log)[-6:]
        running = job["state"] == "running"
        ready = self.h["ready_state"]()
        home = load_home(map_name) if map_name else None
        wp = load_wp(map_name) if map_name else None
        v = {} if running else self._cached_view()
        caps = {}
        if running:
            why = "a dock job is running"
            caps = {k: (False, why) for k in ("dock", "undock", "save", "pose", "teach")}
        else:
            if self.h["loop_running"]():
                caps["dock"] = (False, "a waypoint loop is running")
            elif v.get("live_ok"):
                caps["dock"] = (True, "the dock is in view - will dock directly")
            elif v.get("behind_ok"):
                caps["dock"] = (True, "the dock is behind the robot - will turn around, then dock")
            elif v.get("rear_near"):
                caps["dock"] = (False, "the robot is right at the dock - use Undock first, or it is already docked")
            elif (wp or home) and ready == "ready":
                caps["dock"] = (True, "will drive to the dock waypoint first, then dock")
            elif wp or home:
                caps["dock"] = (False, "the dock is not in view and the robot is not READY to drive to it")
            else:
                caps["dock"] = (False, "the dock is not in view (face it from 0.8-2.5 m) and no dock waypoint is taught for this map")
            front = v.get("front")
            if front and v.get("live_ok") and 1.0 <= front["a"] <= 2.2:
                caps["teach"] = (True, "save this spot (about %.1f m from the V, facing it) as the dock waypoint" % front["a"]) if ready == "ready" \
                    else (False, "the robot must be READY (localized) to teach the waypoint")
            else:
                caps["teach"] = (False, "face the V from 1.0-2.2 m (1.5 m is best) so the dock is in view")
            caps["undock"] = (True, "drive 0.3 m forward off the dock") if v.get("rear_near") else (False, "the dock is not right behind the robot")
            if not v.get("docked"):
                caps["save"] = (False, "only possible while docked")
                caps["pose"] = (False, "only possible while docked")
            else:
                caps["save"] = (True, "save this as the home position of the map") if ready == "ready" else (False, "the robot must be READY (localized) to save home")
                caps["pose"] = (True, "set the initial pose from the saved home") if home else (False, "no home saved for this map yet")
        return {"now": time.time(), "job": job, "home": home, "wp": wp, "docked": bool(v.get("docked")), "front": v.get("front"), "rear": v.get("rear"),
                "caps": {k: {"ok": ok, "why": why} for k, (ok, why) in caps.items()}}

    # ---- actions
    def act(self, action, map_name):
        """Returns (ok, message)."""
        with self.lock:
            if self.job["state"] == "running" and action != "cancel":
                return False, "a dock job is already running"
        if action == "cancel":
            return self.cancel()
        st = self.status(map_name)
        cap = st["caps"].get({"dock": "dock", "undock": "undock", "save_home": "save", "pose_from_dock": "pose", "teach_wp": "teach"}.get(action, ""), None)
        if cap is None:
            return False, "unknown action"
        if not cap["ok"]:
            return False, cap["why"]
        if action == "save_home":
            return self._save_home(map_name)
        if action == "teach_wp":
            return self._teach_wp(map_name)
        if action == "pose_from_dock":
            h = load_home(map_name)
            self.h["request_pose"](h["x"], h["y"], h["theta"])
            return True, "initial pose set from the saved home"
        self._stop.clear()
        threading.Thread(target=self._run_job, args=(action, map_name, st), daemon=True).start()
        return True, "started"

    def _median_map_pose(self):
        samples = []
        for _ in range(6):
            p = self.h["map_pose"]()
            if p:
                samples.append(p)
            time.sleep(0.2)
        if len(samples) < 4:
            return None
        xs = sorted(s[0] for s in samples)
        ys = sorted(s[1] for s in samples)
        c = sorted(math.cos(s[2]) for s in samples)
        sn = sorted(math.sin(s[2]) for s in samples)
        return xs[len(xs) // 2], ys[len(ys) // 2], math.atan2(sn[len(sn) // 2], c[len(c) // 2])

    def _save_home(self, map_name):
        p = self._median_map_pose()
        if not p:
            return False, "no map pose from localization"
        save_home(map_name, *p)
        return True, "home saved: x=%.2f y=%.2f heading=%.0f deg" % (p[0], p[1], math.degrees(p[2]))

    def _teach_wp(self, map_name):
        """Save the waypoint AND work out the docked (home) pose from where the V is seen right now, so one
        tap re-teaches everything after the dock was moved."""
        fits = []
        for _ in range(5):
            v = view(self.h["get_scan"]())
            if v["live_ok"]:
                fits.append(v["front"])
            time.sleep(0.2)
        if len(fits) < 4:
            return False, "the dock is not steadily in view"
        med = lambda k: sorted(f[k] for f in fits)[len(fits) // 2]
        a, lat, psi = med("a"), med("lat"), math.radians(med("psi"))
        p = self._median_map_pose()
        if not p:
            return False, "no map pose from localization"
        x, y, th = p
        e = -psi                                           # dock.py's dock-axis angle in the robot frame
        nrx, nry = -math.cos(e), math.sin(e)               # unit vector tip -> robot, robot frame
        prx, pry = math.sin(e), math.cos(e)                # sideways unit vector
        along = a - DOCKED_TIP_FROM_CENTRE
        dx, dy = -along * nrx + lat * prx, -along * nry + lat * pry
        c, s = math.cos(th), math.sin(th)
        hx, hy = x + c * dx - s * dy, y + s * dx + c * dy
        hth = math.atan2(math.sin(th + math.pi - e), math.cos(th + math.pi - e))
        save_wp(map_name, x, y, th, a)
        save_home(map_name, hx, hy, hth)
        return True, "dock waypoint and home saved: waypoint x=%.2f y=%.2f heading=%.0f deg (V %.2f m ahead)" % (x, y, math.degrees(th), a)

    def cancel(self):
        self._stop.set()
        self.h["cancel_nav"]()
        p = self._proc
        if p and p.poll() is None:
            try:
                p.send_signal(signal.SIGINT)         # dock.py stops the robot in its finally block
            except OSError:
                pass
            threading.Timer(3.0, self._kill_proc, args=(p,)).start()
        return True, "cancelling"

    @staticmethod
    def _kill_proc(p):
        if p.poll() is None:
            try:
                p.kill()
            except OSError:
                pass

    # ---- job thread
    def _set(self, **kw):
        with self.lock:
            self.job.update(kw)

    def _run_job(self, kind, map_name, st):
        self._log.clear()
        self._set(state="running", kind=kind, stage="Starting", detail="", started=time.time(), finished=0.0)
        self.h["set_busy"](True)
        ok, detail = False, ""
        try:
            self.h["get_scan"]()                      # starts the raw-scan subscription if it is not running
            if kind == "dock":
                vw = view(self.h["get_scan"]())
                if not (vw["live_ok"] or vw["behind_ok"]):
                    wp = load_wp(map_name)
                    if wp:
                        x, y, th = wp["x"], wp["y"], wp["theta"]
                    else:
                        x, y, th = approach_pose(load_home(map_name))
                    self._set(stage="Driving to the dock waypoint (Nav2)")
                    prev = self.h["goal_prev_id"]()
                    self.h["publish_goal"](x, y, th)
                    res = self.h["wait_goal"](prev, self._stop)
                    if self._stop.is_set():
                        raise _Cancelled()
                    if res != "succeeded":
                        raise _Failed("could not reach the dock approach point (%s)" % res)
                    time.sleep(2.0)                      # dock.py looks around by itself if the V is not in view
                ok, detail = self._run_script(["run", "--until", "creep"], "DOCKED")
            else:
                ok, detail = self._run_script(["undock"], "UNDOCKED")
        except _Cancelled:
            ok, detail = False, "cancelled"
        except _Failed as e:
            ok, detail = False, str(e)
        except Exception as e:
            ok, detail = False, "unexpected error: %s" % e
        finally:
            self.h["set_busy"](False)
        if self._stop.is_set() and not ok:
            detail = "cancelled"
        self._set(state="done" if ok else "failed", stage="Docked" if (ok and kind == "dock") else ("Undocked" if ok else "Stopped"),
                  detail=detail, finished=time.time())

    def _run_script(self, args, success_word):
        cmd = [sys.executable, "-u", DOCK_SCRIPT] + args
        self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        seen_success, last_abort = False, ""
        for line in self._proc.stdout:
            line = line.rstrip()
            if not line or "XMLPARSER" in line:
                continue
            self._log.append(line[:160])
            if success_word in line:
                seen_success = True
            if line.startswith("ABORT:"):
                last_abort = line[6:].strip()
            for mark, label in _STAGE_MARKS:
                if mark in line:
                    self._set(stage=label)
                    break
        rc = self._proc.wait()
        if rc == 0 and seen_success:
            return True, ""
        if self._stop.is_set():
            return False, "cancelled"
        return False, last_abort or ("the dock script exited with code %s" % rc)


class _Cancelled(Exception):
    pass


class _Failed(Exception):
    pass
