#!/usr/bin/env python3
"""dock.py - reverse-in auto-docking onto the V-shaped charging dock (2026-10-13).

The dock is seen by the LiDAR as a shallow V (two ~17 cm arms, ~34 cm wide,
~6 cm deep). It is found by SHAPE (its intensity is the same as a wall's).
The robot's charging terminal is on its back, so it docks in reverse:

  align   face the V, drive (LiDAR feedback) to a standoff centred on the dock axis
  spin    turn 180 deg in place (odometry)
  reverse blind reverse (odometry + heading hold) to rear-edge->V-tip = 0.30 m
  verify  check the REAR LiDAR view agrees (distance / sideways / angle)
  creep   reverse on the rear view, very slowly, steering on the V, stop ~1 cm
          past first touch (fitted rear-edge->tip 9.9 cm; first touch is ~10.9)

Measured basis (see memory docking-project-facts): LiDAR is 0.31 m forward of
base_link, rotation 0 (laser 0 = front); body half-length 0.39; the rear view is
unreliable at 0.6 m but clean from 0.3 m to contact; reverse drift 0.4 cm/0.9 m;
a 180 deg spin lands within ~1 deg.

Safety: commands go to /cmd_vel_teleop (priority 100, 0.3 s timeout - if this
script or the scan stops, twist_mux releases the robot by itself). The web UI
joystick / E-stop still override. Speed caps are enforced in drive(). Any
failure or Ctrl-C stops the robot. KEEP A HAND ON THE PHYSICAL E-STOP.

Usage (inside the ROS container):
  python3 scripts/dock.py detect [--view front|rear|both]   no motion
  python3 scripts/dock.py run --until align|spin|reverse|verify|creep
  python3 scripts/dock.py creep      (already rear-facing ~0.3 m from the V)
  python3 scripts/dock.py undock [--distance 0.3]
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

LASER_X = 0.31            # LiDAR position ahead of base_link centre
HALF_LEN = 0.39           # centre -> front/rear edge
MIN_I = 30                # intensity floor (drops scan-seam ghosts, I = 4-7)

# approach geometry
STANDOFF_A = 0.99         # robot centre -> V tip when facing the dock: rear edge then 0.60 m from it after the spin
A_MIN = 0.80              # never creep closer than this while still facing the dock
BLIND_END = 0.30          # blind reverse stops with rear-edge->tip at this distance
TOUCH_STOP = 0.104        # stop here (the robot then coasts ~0.5 cm): 1st run stopped at 9.2 cm from a 9.9 target and was ~0.5 cm too compressed
HARD_MIN = 0.085          # closer than this: stop and abort (spring end stop is ~8.9)
LATERAL_OK = 0.012        # align: sideways error accepted (m); the final creep corrects the rest
HEADING_OK = math.radians(1.5)

V_FRONT = 0.05            # m/s caps
V_REAR_FAR = 0.03
V_REAR_NEAR = 0.02
V_REAR_LAST = 0.015       # last 3 cm before the stop
W_MAX = 0.25              # rad/s while driving
W_SPIN = 0.40


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ------------------------------------------------------------------ detection
def view_points(scan, mode):
    """Scan points in a 'view frame' centred on the robot with +x pointing at the dock.
    mode 'front' = robot frame; 'rear' = robot frame rotated by pi."""
    n = len(scan.ranges)
    ang = scan.angle_min + np.arange(n) * scan.angle_increment
    r = np.asarray(scan.ranges, dtype=float)
    inten = np.asarray(scan.intensities, dtype=float) if len(scan.intensities) == n else np.full(n, 99.0)
    ok = np.isfinite(r) & (r > 0.05) & (r < 4.0) & (inten >= MIN_I)
    if mode == "rear":
        ok &= np.abs(np.abs(ang) - math.pi) > math.radians(1.5)      # scan seam sits on the rear tip
    x = r * np.cos(ang) + LASER_X
    y = r * np.sin(ang)
    if mode == "rear":
        x, y = -x, -y
    sel = ok & (x > HALF_LEN + 0.02) & (x < 2.6) & (np.abs(y) < np.minimum(1.6, 0.7 * x))      # +-35 deg cone
    return np.column_stack([x[sel], y[sel]])


def _clusters(P):
    P = P[np.argsort(P[:, 1])]
    cuts = [0]
    for i in range(1, len(P)):
        dy = P[i, 1] - P[i - 1, 1]
        if dy > 0.09 or abs(P[i, 0] - P[i - 1, 0]) > 0.05 + 1.5 * dy:      # 0.09: the rear view drops +-1.5 deg at the scan seam; the x allowance grows with dy for a slanted arm
            cuts.append(i)
    cuts.append(len(P))
    return [P[cuts[i]:cuts[i + 1]] for i in range(len(cuts) - 1)]


def _fit_cluster(C):
    n = len(C)
    if n < 24 or not 0.20 <= np.ptp(C[:, 1]) <= 0.44:
        return None
    best = None
    for k in range(10, n - 10):
        L, R = C[:k], C[k:]
        fl = np.polyfit(L[:, 1], L[:, 0], 1)
        fr = np.polyfit(R[:, 1], R[:, 0], 1)
        e = ((L[:, 0] - np.polyval(fl, L[:, 1])) ** 2).sum() + ((R[:, 0] - np.polyval(fr, R[:, 1])) ** 2).sum()
        if best is None or e < best[0]:
            best = (e, k, fl, fr)
    e, k, fl, fr = best
    mL, cL = fl
    mR, cR = fr
    aL, aR = math.degrees(math.atan(mL)), math.degrees(math.atan(mR))
    # Turning the robot shifts one arm's angle up and the other's down by the same amount, so the SUM stays ~45 deg
    # (interior angle 135) whatever the viewing angle: that is the shape test. Each arm alone may range widely (one arm
    # even tilts past edge-on when the robot is >20 deg off the dock axis, as in an S-curve line-up).
    if not (-30 <= aL <= 58) or not (-30 <= -aR <= 58) or not (30 <= aL - aR <= 58):
        return None
    ty = (cR - cL) / (mL - mR)
    tx = mL * ty + cL
    lenL = math.hypot(np.ptp(C[:k, 0]), np.ptp(C[:k, 1]))
    lenR = math.hypot(np.ptp(C[k:, 0]), np.ptp(C[k:, 1]))
    depth = tx - min(C[0, 0], C[-1, 0])
    rms = math.sqrt(e / n)
    if not (0.10 <= lenL <= 0.24 and 0.10 <= lenR <= 0.24 and 0.03 <= depth <= 0.16 and rms < 0.008):
        return None
    ev = (math.atan(mL) + math.atan(mR)) / 2            # dock lateral line vs robot y axis
    # robot centre in dock frame: along the axis (a) and sideways (l); heading error psi = -ev
    a = tx * math.cos(ev) - ty * math.sin(ev)
    lat = tx * math.sin(ev) + ty * math.cos(ev)
    return dict(tx=tx, ty=ty, e=ev, a=a, lat=lat, psi=-ev, rms=rms, n=n, width=float(np.ptp(C[:, 1])),
                arms=(aL, -aR))


def detect_v(scan, mode):
    P = view_points(scan, mode)
    best = None
    for C in _clusters(P):
        f = _fit_cluster(C)
        if f and (best is None or f["rms"] < best["rms"]):
            best = f
    return best


def describe(f):
    if not f:
        return "no V"
    return ("tip %.3f m ahead, %+.1f cm sideways | axis dist %.3f  lateral %+.1f cm  heading err %+.1f deg | "
            "arms %.0f/%.0f deg width %.0f cm rms %.1f mm (%d pts)" % (
                f["tx"], 100 * f["ty"], f["a"], 100 * f["lat"], math.degrees(f["psi"]),
                f["arms"][0], f["arms"][1], 100 * f["width"], 1000 * f["rms"], f["n"]))


# --------------------------------------------------------------------- runner
class Dock:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("dock")
        self.scan = None
        self.scan_count = 0
        self.odom = None
        self.n.create_subscription(LaserScan, "/scan", self._on_scan,
                                   QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.n.create_subscription(Odometry, "/diffbot_base_controller/odom", self._on_odom, 10)
        self.pub = self.n.create_publisher(Twist, "/cmd_vel_teleop", 10)

    def _on_scan(self, m):
        self.scan = m
        self.scan_count += 1

    def _on_odom(self, m):
        self.odom = m

    # -- plumbing
    def spin(self, t):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)

    def wait_scan(self, timeout=0.5):
        c = self.scan_count
        end = time.time() + timeout
        while self.scan_count == c and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)
        return self.scan_count != c

    def drive(self, v, w):
        t = Twist()
        t.linear.x = float(max(-0.06, min(0.06, v)))
        t.angular.z = float(max(-0.45, min(0.45, w)))
        self.pub.publish(t)

    def stop(self):
        for _ in range(10):
            self.pub.publish(Twist())
            self.spin(0.03)

    def yaw(self):
        q = self.odom.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def xy(self):
        p = self.odom.pose.pose.position
        return p.x, p.y

    def startup(self):
        end = time.time() + 12
        while (self.scan is None or self.odom is None) and time.time() < end:
            self.spin(0.1)
        self.spin(0.5)
        if self.scan is None or self.odom is None:
            raise Abort("no scan / odometry")
        if abs(self.odom.twist.twist.linear.x) > 0.02 or abs(self.odom.twist.twist.angular.z) > 0.02:
            raise Abort("robot is moving at start")

    def median_fit(self, mode, count=5, timeout=4.0):
        fits = []
        end = time.time() + timeout
        while len(fits) < count and time.time() < end:
            if self.wait_scan():
                f = detect_v(self.scan, mode)
                if f:
                    fits.append(f)
        if len(fits) < max(3, count - 2):
            return None
        keys = ("tx", "ty", "e", "a", "lat", "psi", "rms", "width")
        out = {k: float(np.median([f[k] for f in fits])) for k in keys}
        out["n"] = fits[-1]["n"]
        out["arms"] = fits[-1]["arms"]
        return out

    # -- stages
    def st_align(self):
        """Get onto the dock's centre line facing the V, WITHOUT a long run-in: turn to face a point on the
        line (STANDOFF_A from the V tip), drive straight there, turn to face along the line, re-measure.
        Turns and straight drives use odometry only (spin within ~1 deg, drift 0.4 cm/0.9 m), so the
        sideways error is fixed in one move whatever the start; a second pass trims what is left."""
        f = self.median_fit("front")
        if not f:
            rr = self.median_fit("rear", 5, 3.0)
            if rr and 0.74 <= rr["tx"] <= 2.5 and abs(math.degrees(rr["psi"])) <= 35:
                print("the dock is BEHIND the robot (%.2f m): turning around first" % rr["tx"])
                self.st_spin()
                f = self.median_fit("front")
        if not f:
            f = self.st_search()
        if not f:
            raise Abort("cannot see the V anywhere around the robot (it must be 0.8-2.4 m away)")
        print("start:", describe(f))
        if not (A_MIN - 0.02 <= f["a"] <= 2.4):
            raise Abort("V axis distance %.2f m is outside %.2f-2.4 m: move the robot" % (f["a"], A_MIN - 0.02))
        if abs(math.degrees(f["psi"])) > 35:
            raise Abort("robot is more than 35 deg off the dock axis")
        for attempt in range(4):
            a, lat, psi = f["a"], f["lat"], f["psi"]
            if abs(lat) <= LATERAL_OK and abs(psi) <= HEADING_OK and abs(a - STANDOFF_A) <= 0.04:
                break
            if attempt == 3:
                raise Abort("could not line up after 3 moves (lateral %+.1f cm, heading %+.1f deg, axis %.2f m)"
                            % (100 * lat, math.degrees(psi), a))
            ev = f["e"]
            nx, ny = -math.cos(ev), math.sin(ev)
            sx, sy = f["tx"] + STANDOFF_A * nx, f["ty"] + STANDOFF_A * ny      # the line-up point, robot frame
            dist = math.hypot(sx, sy)
            yaw_axis = self.yaw() + psi                                         # odometry yaw that faces along the axis
            if dist >= 0.012:
                beta = math.atan2(sy, sx)
                if abs(beta) <= math.pi / 2:
                    theta, sign = beta, 1
                else:
                    theta, sign = wrap(beta + math.pi), -1
                self.check_clear(beta, dist, (f["tx"], f["ty"]), theta, wrap(psi - theta))
                print("  align: pass %d: lateral %+.1f cm, heading %+.1f deg, axis %.2f m -> turn %+.0f deg, %s %.2f m, face the dock"
                      % (attempt + 1, 100 * lat, math.degrees(psi), a, math.degrees(theta),
                         "drive" if sign > 0 else "reverse", dist))
                self.turn_by(theta)
                self.drive_dist(dist, sign)
            else:
                self.check_clear(0.0, 0.0, (f["tx"], f["ty"]), 0.0, psi)
                print("  align: pass %d: lateral %+.1f cm, heading %+.1f deg -> face the dock" % (attempt + 1, 100 * lat, math.degrees(psi)))
            self.turn_by(wrap(yaw_axis - self.yaw()))
            f = self.median_fit("front")
            if not f:
                raise Abort("lost the V after moving to the line-up point")
        self.stop()
        fin = self.median_fit("front", 5)
        if not fin:
            raise Abort("lost the V after aligning")
        print("ALIGNED:", describe(fin))
        return fin

    def st_search(self):
        """The V is not in view: turn on the spot in 50 deg steps (one full circle at most) until it is."""
        print("the V is not in view: turning to look for it")
        self.check_turn_clear()
        for k in range(7):
            self.turn_by(math.radians(50))
            f = self.median_fit("front", 3, 1.5)
            if f and A_MIN - 0.02 <= f["a"] <= 2.4 and abs(math.degrees(f["psi"])) <= 35:
                print("FOUND the V after turning %d deg" % (50 * (k + 1)))
                return self.median_fit("front")
        return None

    def check_turn_clear(self):
        self.spin(0.3)
        x, y = self.scan_xy()
        keep = ~((np.abs(x) < 0.43) & (np.abs(y) < 0.30))
        x, y = x[keep], y[keep]
        hit = self._turn_hits(x, y, (0.0, 0.0), 0.0, 2 * math.pi)
        if hit.any():
            raise Abort("something is only %.2f m from the robot's centre: not turning to look" % float(np.hypot(x, y)[hit].min()))

    def scan_xy(self):
        m = self.scan
        n = len(m.ranges)
        ang = m.angle_min + np.arange(n) * m.angle_increment
        r = np.asarray(m.ranges, dtype=float)
        inten = np.asarray(m.intensities, dtype=float) if len(m.intensities) == n else np.full(n, 99.0)
        ok = np.isfinite(r) & (r > 0.05) & (r < 6.0) & (inten >= 10)
        return (r * np.cos(ang) + LASER_X)[ok], (r * np.sin(ang))[ok]

    @staticmethod
    def _turn_hits(x, y, pos, h0, dh):
        """Scan points that the robot body (+4 cm) touches while it turns in place at pos from heading h0 by dh."""
        px, py = x - pos[0], y - pos[1]
        hit = np.zeros(len(x), dtype=bool)
        steps = max(1, int(abs(dh) / math.radians(2)))
        for k in range(steps + 1):
            h = h0 + dh * k / steps
            c, s_ = math.cos(h), math.sin(h)
            lx = c * px + s_ * py
            ly = -s_ * px + c * py
            hit |= (np.abs(lx) < HALF_LEN + 0.04) & (np.abs(ly) < 0.28)
        return hit

    def check_clear(self, beta, dist, tip, theta, delta):
        """Refuse to move if anything is in the way of the planned moves: turn by theta, drive dist along beta
        (robot frame), turn by delta at the line-up point."""
        self.spin(0.3)
        x, y = self.scan_xy()
        keep = np.hypot(x - tip[0], y - tip[1]) > 0.45                    # the dock itself is not an obstacle
        keep &= ~((np.abs(x) < 0.43) & (np.abs(y) < 0.30))                # the robot's own body / mounts
        x, y = x[keep], y[keep]
        sx, sy = dist * math.cos(beta), dist * math.sin(beta)
        for pos, h0, dh, what in (((0.0, 0.0), 0.0, theta, "turning at the start"), ((sx, sy), theta, delta, "turning at the line-up point")):
            hit = self._turn_hits(x, y, pos, h0, dh)
            if hit.any():
                raise Abort("something is within reach of the robot while %s (%.2f m from its centre)" % (what, float(np.hypot(x - pos[0], y - pos[1])[hit].min())))
        ux, uy = math.cos(beta), math.sin(beta)
        sa = x * ux + y * uy
        c = -x * uy + y * ux
        blocked = (sa > 0.35) & (sa < dist + HALF_LEN + 0.15) & (np.abs(c) < 0.32)
        if blocked.any():
            raise Abort("something is in the way %.2f m along the %.2f m move to the line-up point" % (float(sa[blocked].min()), dist))

    def turn_by(self, delta):
        """Rotate in place by delta (rad, +ccw) using odometry; one trim pass for the coast."""
        for _ in range(2):
            goal = wrap(self.yaw() + delta)
            t0 = time.time()
            while time.time() - t0 < 40:
                err = wrap(goal - self.yaw())
                if abs(err) < 0.012:
                    break
                self.drive(0, math.copysign(min(W_SPIN, max(0.06, 1.0 * abs(err))), err))
                self.spin(0.05)
            self.stop()
            self.spin(1.0)
            delta = wrap(goal - self.yaw())
            if abs(delta) < 0.02:
                break

    def drive_dist(self, dist, sign):
        """Straight drive of dist (m), forward (sign +1) or reverse (-1), holding the heading."""
        x0, y0 = self.xy()
        ref = self.yaw()
        t0 = time.time()
        while time.time() - t0 < dist / 0.03 + 8:
            x, y = self.xy()
            rem = dist - math.hypot(x - x0, y - y0)
            if rem <= 0.004:
                break
            v = 0.05 if rem > 0.06 else 0.03
            w = max(-0.1, min(0.1, 1.0 * wrap(ref - self.yaw())))
            self.drive(sign * v, w)
            self.spin(0.05)
        self.stop()
        self.spin(1.0)

    def st_spin(self):
        th_prev = self.yaw()
        acc = 0.0
        t0 = time.time()
        tw_w = W_SPIN
        while time.time() - t0 < 25:
            self.drive(0, tw_w)
            self.spin(0.05)
            th = self.yaw()
            acc += wrap(th - th_prev)
            th_prev = th
            if acc >= math.pi - 0.06:                # stop a little early: the controller coasts
                break
        self.stop()
        self.spin(1.5)
        th = self.yaw()
        acc += wrap(th - th_prev)
        print("SPUN: odometry says %+.1f deg in %.0f s" % (math.degrees(acc), time.time() - t0))
        if abs(math.degrees(acc) - 180) > 6:
            raise Abort("spin ended %.1f deg from 180" % (math.degrees(acc) - 180))

    def st_reverse(self, dist):
        if not 0.05 <= dist <= 0.9:
            raise Abort("blind reverse distance %.2f m is out of range" % dist)
        x0, y0 = self.xy()
        th_ref = self.yaw()
        t0 = time.time()
        print("blind reverse %.3f m at 0.05 m/s" % dist)
        while time.time() - t0 < dist / 0.03 + 6:
            x, y = self.xy()
            d = math.hypot(x - x0, y - y0)
            if d >= dist - 0.005:
                break
            w = max(-0.1, min(0.1, 1.0 * wrap(th_ref - self.yaw())))
            self.drive(-0.05, w)
            self.spin(0.05)
        self.stop()
        self.spin(1.0)
        x, y = self.xy()
        print("REVERSED: odometry %.3f m (target %.3f)" % (math.hypot(x - x0, y - y0), dist))

    def st_verify(self):
        f = self.median_fit("rear", 7, 5.0)
        if not f:
            raise Abort("rear view cannot see the V")
        dr = f["tx"] - HALF_LEN
        print("REAR VIEW: rear edge -> tip %.3f m | %s" % (dr, describe(f)))
        if abs(dr - BLIND_END) > 0.04:
            raise Abort("rear distance %.3f differs from expected %.2f by more than 4 cm" % (dr, BLIND_END))
        if abs(f["lat"]) > 0.02 or abs(math.degrees(f["psi"])) > 3.0:
            raise Abort("rear view says lateral %+.1f cm / heading %+.1f deg: too far off to creep in" % (100 * f["lat"], math.degrees(f["psi"])))
        return f

    def st_creep(self):
        t0 = time.time()
        lost = 0
        touch = 0
        last_print = 0
        LH = 0.25
        while time.time() - t0 < 60:
            if not self.wait_scan():
                raise Abort("scan stopped")
            f = detect_v(self.scan, "rear")
            if not f:
                lost += 1
                self.drive(0, 0)
                if lost > 9:
                    raise Abort("lost the V for >0.6 s")
                continue
            lost = 0
            dr = f["tx"] - HALF_LEN
            if dr < HARD_MIN:
                raise Abort("rear edge only %.1f cm from the V tip: past the spring end stop" % (100 * dr))
            if dr <= TOUCH_STOP:
                touch += 1
                self.drive(0, 0)
                if touch >= 2:
                    break
                continue
            touch = 0
            ev = f["e"]
            nx, ny = -math.cos(ev), math.sin(ev)
            s = max(0.05, f["a"] - LH)
            h = math.atan2(f["ty"] + s * ny, f["tx"] + s * nx)
            v = V_REAR_FAR if dr > 0.15 else (V_REAR_NEAR if dr > TOUCH_STOP + 0.03 else V_REAR_LAST)
            if abs(h) > 0.35:
                raise Abort("steering error %.0f deg while creeping in: stop and check" % math.degrees(h))
            w = max(-0.1, min(0.1, 1.2 * h))
            self.drive(-v, w)                       # robot reverses; yaw rate is the same in the rotated view frame
            if time.time() - last_print > 0.7:
                last_print = time.time()
                print("  creep: rear edge->tip %.1f cm | lat %+.1f cm head %+.1f deg | v -%.2f w %+.2f" % (
                    100 * dr, 100 * f["lat"], math.degrees(f["psi"]), v, w))
        else:
            raise Abort("creep timed out")
        self.stop()
        self.spin(1.0)
        fin = self.median_fit("rear", 5)
        print("STOPPED:", describe(fin) if fin else "V not fitted")
        if fin:
            print("   rear edge -> tip %.1f cm (stop target %.1f cm, first touch ~10.9 cm)" % (100 * (fin["tx"] - HALF_LEN), 100 * TOUCH_STOP))
        print("DOCKED by position only (no charge sensing yet) - check the terminals by eye.")


    def st_undock(self, dist=0.3):
        """Drive straight forward off the dock (the terminal is on the back, so forward is away from it)."""
        self.spin(0.5)
        m = self.scan
        n = len(m.ranges)
        ang = m.angle_min + np.arange(n) * m.angle_increment
        r = np.asarray(m.ranges, dtype=float)
        inten = np.asarray(m.intensities, dtype=float) if len(m.intensities) == n else np.full(n, 99.0)
        ok = np.isfinite(r) & (r > 0.05) & (r < 6.0) & (inten >= 10)
        x = r * np.cos(ang) + LASER_X
        y = r * np.sin(ang)
        blocked = ok & (x > HALF_LEN + 0.02) & (x < HALF_LEN + dist + 0.30) & (np.abs(y) < 0.50)
        if blocked.any():
            k = int(np.argmin(np.where(blocked, x, 9.0)))
            raise Abort("something is %.2f m ahead of the front edge: not undocking" % (x[k] - HALF_LEN))
        x0, y0 = self.xy()
        th_ref = self.yaw()
        t0 = time.time()
        print("undocking %.2f m forward at 0.05 m/s" % dist)
        while time.time() - t0 < dist / 0.03 + 6:
            xx, yy = self.xy()
            if math.hypot(xx - x0, yy - y0) >= dist - 0.005:
                break
            w = max(-0.1, min(0.1, 1.0 * wrap(th_ref - self.yaw())))
            self.drive(0.05, w)
            self.spin(0.05)
        self.stop()
        self.spin(1.0)
        xx, yy = self.xy()
        print("UNDOCKED: odometry %.3f m forward" % math.hypot(xx - x0, yy - y0))


class Abort(Exception):
    pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("detect", help="print what the LiDAR sees, no motion")
    d.add_argument("--view", choices=["front", "rear", "both"], default="both")
    d.add_argument("--seconds", type=float, default=4.0)
    u = sub.add_parser("undock", help="drive straight forward off the dock (the robot MOVES)")
    u.add_argument("--distance", type=float, default=0.3)
    sub.add_parser("creep", help="robot already rear-facing ~0.3 m from the V: verify, then creep in (the robot MOVES)")
    r = sub.add_parser("run", help="run the docking stages (the robot MOVES)")
    r.add_argument("--until", choices=["align", "spin", "reverse", "verify", "creep"], default="align")
    args = ap.parse_args()

    dk = Dock()
    try:
        dk.startup()
        if args.cmd == "detect":
            end = time.time() + args.seconds
            views = ["front", "rear"] if args.view == "both" else [args.view]
            while time.time() < end:
                if dk.wait_scan():
                    for v in views:
                        print("%-5s %s" % (v, describe(detect_v(dk.scan, v))))
                    time.sleep(0.4)
            return 0
        if args.cmd == "undock":
            dk.st_undock(max(0.2, min(1.0, args.distance)))
            return 0
        if args.cmd == "creep":
            dk.st_verify()
            dk.st_creep()
            return 0
        order = ["align", "spin", "reverse", "verify", "creep"]
        upto = order.index(args.until)
        fin = dk.st_align()
        if upto >= 1:
            dist = fin["a"] - HALF_LEN - BLIND_END
            dk.st_spin()
        if upto >= 2:
            dk.st_reverse(dist)
        if upto >= 3:
            dk.st_verify()
        if upto >= 4:
            dk.st_creep()
        print("finished stage '%s'" % args.until)
        return 0
    except Abort as e:
        print("ABORT:", e)
        return 2
    except KeyboardInterrupt:
        print("interrupted")
        return 130
    finally:
        try:
            dk.stop()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
