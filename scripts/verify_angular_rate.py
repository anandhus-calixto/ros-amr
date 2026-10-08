#!/usr/bin/env python3
"""Check that an angular velocity commanded over ROS2 is what the robot actually turns at.

Why this is not as simple as comparing /cmd_vel with /odom
----------------------------------------------------------
/diffbot_base_controller/odom's angular.z is computed BY diff_drive_controller
from the encoder counts and its own wheel_separation/wheel_radius constants -
the same constants it used to turn your command into wheel setpoints. So
comparing command against that odometry mostly re-checks the controller's own
arithmetic. If wheel_separation is wrong, or a wheel slips, the command and
that odometry agree perfectly while the robot physically turns at some other
rate. It is a real check of the encoder/motor loop, and a useless check of
whether the ROBOT turns at 0.3 rad/s.

The BNO055 gyro is the independent witness. It measures the chassis rotating,
knows nothing about wheels, encoders or wheel_separation, and so is the one
signal that catches wheel slip and wrong geometry. That is why this script
treats GYRO as the verdict and the rest as diagnosis.

What each row of the report tells you
-------------------------------------
  commanded    what /cmd_vel carried (the ask)
  wheels       per-wheel rad/s from /joint_states, and the chassis rate they
               imply: w = r * (w_R - w_L) / L. Disagrees with "commanded" =>
               the MCU/motor loop is not hitting its setpoint.
  encoder odom diff_drive_controller's own odometry. Tracks "wheels" by
               construction; shown to confirm nothing odd sits between them.
  GYRO         what the chassis actually did. This is the answer.

Reading the result
------------------
  gyro ~= commanded                 -> the chain is correct, you are done.
  wheels ~= commanded, gyro LOWER   -> wheels are turning as asked but the
                                       robot is not: wheel slip, or
                                       wheel_separation/wheel_radius is wrong.
                                       Both agree with each other because both
                                       come from the same two constants.
  wheels BELOW commanded            -> the MCU/motor is not delivering its
                                       setpoint. Check which wheel in the
                                       per-wheel numbers - this robot has a
                                       known ~20% right-wheel shortfall in
                                       reverse (see config/nav2_params.yaml).
  gyro ~= commanded, wheels off     -> suspect the encoder scaling, not the drive.

A gyro-vs-commanded ratio that is consistently off by one constant factor is
usually geometry, not the motors: wheel_separation enters the command->wheel
conversion linearly, so a 10% error there is a flat 10% error in turn rate.

Safety
------
Commands go to cmd_vel_teleop (twist_mux priority 100), NOT straight to
/cmd_vel, so the physical remote (150) and the software E-stop (200) both still
override this script while it runs. twist_mux's 0.5 s timeout means the robot
stops by itself within half a second if this process dies or is Ctrl-C'd. The
script also ramps up and down rather than stepping, and publishes zero on exit.

THE ROBOT WILL SPIN IN PLACE. Put it somewhere with clearance before running.

Usage (on the robot, inside the container)
------------------------------------------
    source /opt/ros/humble/setup.bash
    python3 verify_angular_rate.py --rate 0.3 --duration 8

    # watch only, command nothing - drive from the web UI or send a nav goal:
    python3 verify_angular_rate.py --passive --duration 15

    # sweep several rates in one go:
    for w in 0.2 0.3 0.5 0.8; do python3 verify_angular_rate.py --rate $w; done

Ground truth that needs no sensors at all
-----------------------------------------
Mark the floor, run --rate 0.5 --duration 25, and count turns with a stopwatch.
Measured rate = 2*pi*turns / seconds. If that disagrees with the gyro, the gyro
scaling is itself wrong - worth knowing before trusting any of the above.
"""
import argparse
import math
import statistics
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Imu, JointState

# Must match config/diffbot_controllers.yaml - if these are wrong, the whole
# point of the gyro comparison is to reveal it.
WHEEL_SEPARATION = 0.384   # effective track, calibrated - see diffbot_controllers.yaml
WHEEL_RADIUS = 0.085

CMD_TOPIC = "cmd_vel_teleop"   # twist_mux input, priority 100
PUBLISH_HZ = 20.0              # twist_mux timeout is 0.5 s; stay well inside it
RAMP_S = 1.0                   # ease in/out instead of stepping the motors


class AngularRateCheck(Node):
    def __init__(self, rate, duration, passive, linear=False, ramp=RAMP_S):
        super().__init__("verify_angular_rate")
        self.target = rate
        self.duration = duration
        self.passive = passive
        # linear=True drives forward (m/s) instead of spinning (rad/s). There is
        # no independent sensor for forward speed - the gyro only sees rotation -
        # so in linear mode the encoders verify the MOTOR LOOP and only a tape
        # measure verifies wheel_radius. Said plainly in the report.
        self.linear = linear
        self.ramp = ramp

        # Samples collected only while the command is at full value, so the
        # ramp and the spin-down never pollute the averages.
        self.lock = threading.Lock()
        self.collecting = False
        self.moving = False
        self.cmd, self.gyro, self.odom = [], [], []
        self.cmd_lin, self.odom_lin = [], []
        self.wl, self.wr = [], []
        # Safety abort. The bumper stop itself lives in MCU firmware
        # (STATUS_BUMPER_LATCHED, safety.c) and does not need us - this is a
        # second, softer layer that also stops COMMANDING as soon as the MCU
        # reports a trip, instead of pushing against a firmware brake.
        self.abort = None
        # First/last odometry pose of the whole move, so the reported distance
        # is the real path length including the ramps - which is what a tape
        # measure on the floor actually spans.
        self.p0 = None
        self.p1 = None
        # Closed-loop turn mode: integrate the gyro and stop when IT says we
        # have turned the requested angle. The point is to then look at the
        # robot: if the gyro is honest the chassis really did that many turns,
        # and if it over-reads the robot stops visibly short.
        self.turns = 0.0
        self.angle = 0.0          # integrated gyro angle, radians
        self._last_imu_t = None

        sensor_qos = QoSProfile(depth=50)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(TwistStamped, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Odometry, "/diffbot_base_controller/odom", self._on_odom, 10)
        self.create_subscription(Imu, "/bno055/imu", self._on_imu, sensor_qos)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(DiagnosticArray, "/mcu_status", self._on_mcu, 10)

        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)

    def _record(self, bucket, value):
        with self.lock:
            if self.collecting:
                bucket.append(value)

    def _on_cmd(self, msg):
        self._record(self.cmd, msg.twist.angular.z)
        self._record(self.cmd_lin, msg.twist.linear.x)

    def _on_odom(self, msg):
        self._record(self.odom, msg.twist.twist.angular.z)
        self._record(self.odom_lin, msg.twist.twist.linear.x)
        p = msg.pose.pose.position
        if self.moving:
            if self.p0 is None:
                self.p0 = (p.x, p.y)
            self.p1 = (p.x, p.y)

    def _on_mcu(self, msg):
        for st in msg.status:
            for kv in st.values:
                if kv.key == "bumper" and kv.value.upper() == "TRIPPED":
                    self.abort = "bumper TRIPPED"
                elif kv.key == "bumper_latched" and kv.value.lower() == "true":
                    self.abort = "bumper LATCHED"
                elif kv.key in ("front_estop", "back_estop") and kv.value.lower() not in ("ok", ""):
                    self.abort = kv.key + " = " + kv.value

    def _on_imu(self, msg):
        self._record(self.gyro, msg.angular_velocity.z)
        # Integrate on the message's own stamps, not wall clock, so network or
        # scheduling hiccups do not inflate the angle.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.moving and self._last_imu_t is not None:
            dt = t - self._last_imu_t
            if 0.0 < dt < 0.5:          # ignore gaps and any stamp going backwards
                self.angle += msg.angular_velocity.z * dt
        self._last_imu_t = t

    def _on_joints(self, msg):
        try:
            li = msg.name.index("left_wheel_joint")
            ri = msg.name.index("right_wheel_joint")
        except ValueError:
            return
        if len(msg.velocity) <= max(li, ri):
            return
        self._record(self.wl, msg.velocity[li])
        self._record(self.wr, msg.velocity[ri])

    def _send(self, value):
        t = Twist()
        if self.linear:
            t.linear.x = value
        else:
            t.angular.z = value
        self.pub.publish(t)

    def run(self):
        period = 1.0 / PUBLISH_HZ
        if self.passive:
            print("PASSIVE: commanding nothing, just listening for "
                  "{:.0f}s. Drive the robot now.".format(self.duration))
            with self.lock:
                self.collecting = True
            time.sleep(self.duration)
            with self.lock:
                self.collecting = False
            return

        if self.turns:
            return self._run_turns(period)

        print("Commanding {:+.3f} {} on /{} for {:.1f}s "
              "(ramp {:.1f}s each end). Ctrl-C stops."
              .format(self.target, "m/s" if self.linear else "rad/s",
                      CMD_TOPIC, self.duration, self.ramp))

        self.moving = True
        start = time.time()
        while True:
            now = time.time() - start
            if now < self.ramp:
                omega = self.target * (now / self.ramp)
            elif now < self.ramp + self.duration:
                omega = self.target
                with self.lock:
                    # Drop the first half-second at full speed: the motors are
                    # still settling and those samples read low.
                    settle = min(0.5, self.duration / 3.0)
                    self.collecting = now > self.ramp + settle
            elif now < self.ramp + self.duration + self.ramp:
                with self.lock:
                    self.collecting = False
                k = (now - self.ramp - self.duration) / self.ramp
                omega = self.target * (1.0 - k)
            else:
                break
            if self.abort:
                print("\n*** ABORT: {} - stopping ***".format(self.abort))
                break
            self._send(omega)
            time.sleep(period)

        for _ in range(int(PUBLISH_HZ)):   # ~1s of explicit zero
            self._send(0.0)
            time.sleep(period)
        self.moving = False

    def _run_turns(self, period):
        """Spin until the INTEGRATED GYRO says we have done self.turns rotations."""
        target = abs(self.turns) * 2.0 * math.pi
        sign = 1.0 if self.target >= 0 else -1.0
        print("Closed loop on the gyro: spinning until it has integrated "
              "{:.2f} turns ({:.0f} deg)".format(abs(self.turns), math.degrees(target)))
        print("at {:+.2f} rad/s. Note where the robot is pointing NOW.\n"
              .format(self.target))

        self.moving = True
        self.angle = 0.0
        t0 = time.time()
        omega = 0.0
        ramping_down = False

        while True:
            if self.abort:
                print("\n*** ABORT: {} - stopping ***".format(self.abort))
                break
            done = abs(self.angle)
            # Start easing off early: the robot keeps turning while it slows,
            # and that coast is part of what the gyro will have integrated.
            coast = 0.5 * abs(omega) * self.ramp
            if not ramping_down and done + coast >= target:
                ramping_down = True
                t_rd = time.time()
            if ramping_down:
                k = (time.time() - t_rd) / self.ramp
                if k >= 1.0:
                    break
                omega = sign * abs(self.target) * (1.0 - k)
            else:
                el = time.time() - t0
                omega = (sign * abs(self.target) *
                         (min(1.0, el / self.ramp) if self.ramp > 0 else 1.0))
                with self.lock:
                    self.collecting = el > self.ramp + 0.5
            self._send(omega)
            time.sleep(period)
            if time.time() - t0 > 400:
                print("\n*** timeout guard hit - stopping ***")
                break

        with self.lock:
            self.collecting = False
        for _ in range(int(PUBLISH_HZ)):
            self._send(0.0)
            time.sleep(period)
        time.sleep(0.5)           # let the last gyro samples land
        self.moving = False

        deg = math.degrees(abs(self.angle))
        print("\n" + "=" * 62)
        print("GYRO INTEGRATED: {:.1f} deg  ({:.3f} turns) in {:.0f}s"
              .format(deg, deg / 360.0, time.time() - t0))
        print("=" * 62)
        print("\nNOW LOOK AT THE ROBOT. It should be back on its starting heading.")
        print("  landed on its mark      -> the gyro is honest.")
        print("                             wheel_separation is really ~0.379 m")
        print("                             (effective track is narrower than the tape).")
        print("  stopped ~{:.0f} deg SHORT  -> the gyro over-reads by ~5.6%."
              .format(deg * 0.0545))
        print("                             your 0.400 m tape is right; fix the gyro scale.")
        print("  anything else           -> tell me the angle and I will work it out.")
        return

    def stop(self):
        for _ in range(5):
            self._send(0.0)
            time.sleep(0.02)

    def _report_linear(self, stat, wl_m, wr_m, wl_n, wr_n):
        """Forward-speed report.

        Deliberately does NOT claim a ground truth. The gyro measures rotation
        only, and integrating IMU linear acceleration over a couple of seconds
        is far too noisy to call a verdict. So this verifies the half of the
        chain that CAN be verified from on-board data - command -> wheel
        setpoint -> encoder - and tells you to reach for a tape measure for
        the other half, which is wheel_radius.
        """
        cmd_m, cmd_sd, cmd_n = stat(self.cmd_lin)
        odo_m, odo_sd, odo_n = stat(self.odom_lin)
        ref = cmd_m if cmd_n else (None if self.passive else self.target)

        if not ref:
            print("\nNothing was commanded during the window - nothing to check.")
            return 0

        print("\n" + "=" * 62)
        print("{:<18}{:>10}{:>10}{:>8}{:>14}".format(
            "signal", "mean m/s", "stdev", "n", "vs commanded"))
        print("-" * 62)

        def row(label, m, sd, n):
            if not n:
                print("{:<18}{:>10}{:>10}{:>8}{:>14}".format(label, "-", "-", 0, "-"))
                return
            print("{:<18}{:>10.4f}{:>10.4f}{:>8}{:>12.1f} %".format(
                label, m, sd, n, 100.0 * m / ref))

        row("commanded", cmd_m, cmd_sd, cmd_n)
        if wl_n and wr_n:
            implied = WHEEL_RADIUS * (wr_m + wl_m) / 2.0
            row("wheels->chassis", implied, 0.0, min(wl_n, wr_n))
        row("encoder odom", odo_m, odo_sd, odo_n)
        print("=" * 62)

        if wl_n and wr_n:
            want = ref / WHEEL_RADIUS
            print("per-wheel rad/s   left {:+.3f}   right {:+.3f}   "
                  "(both want {:+.3f})".format(wl_m, wr_m, want))
            if abs(want) > 1e-6:
                lp, rp = 100.0 * wl_m / want, 100.0 * wr_m / want
                print("                  left {:.0f}% of setpoint, right {:.0f}% "
                      "-> imbalance {:+.0f}%".format(lp, rp, rp - lp))
                if abs(rp - lp) > 5:
                    print("                  NOTE: a left/right mismatch makes the "
                          "robot curve, not just\n                  run slow - this is "
                          "the known wheel gap, now measured going forward.")

            err = 100.0 * (implied - ref) / ref
            print("\nMOTOR LOOP: wheels delivered {:.4f} m/s against {:.4f} "
                  "commanded ({:+.1f}%).".format(implied, ref, err))
            if abs(err) <= 5:
                print("  Within 5% - the MCU is executing the commanded speed.")
            else:
                print("  Off by {:.0f}% - the MCU/motor is not hitting its "
                      "setpoint.".format(abs(err)))

        dist = None
        if self.p0 and self.p1:
            dist = math.hypot(self.p1[0] - self.p0[0], self.p1[1] - self.p0[1])
            print("\nODOMETRY DISTANCE over the whole move (incl. ramps): "
                  "{:.4f} m".format(dist))
        if dist:
            print("\nNOT VERIFIED on-board: whether that equals real ground speed.")
            print("  Measure the actual distance with a tape. If it reads short/long "
                  "by a fixed\n  ratio, wheel_radius ({} m) is wrong - everything "
                  "above would still agree,\n  because it all derives from that same "
                  "number.".format(WHEEL_RADIUS))
        return 0

    def report(self):
        def stat(xs):
            if not xs:
                return None, None, 0
            m = statistics.fmean(xs)
            sd = statistics.pstdev(xs) if len(xs) > 1 else 0.0
            return m, sd, len(xs)

        wl_m, _, wl_n = stat(self.wl)
        wr_m, _, wr_n = stat(self.wr)

        if self.linear:
            return self._report_linear(stat, wl_m, wr_m, wl_n, wr_n)

        cmd_m, cmd_sd, cmd_n = stat(self.cmd)
        gyr_m, gyr_sd, gyr_n = stat(self.gyro)
        odo_m, odo_sd, odo_n = stat(self.odom)

        if not gyr_n:
            print("\nNo gyro samples - is /bno055/imu publishing? "
                  "Without it this test cannot conclude anything.")
            return 1
        if not cmd_n:
            print("\nNo /cmd_vel samples. In passive mode that just means "
                  "nothing was driving; otherwise twist_mux may be blocking "
                  "cmd_vel_teleop (a higher-priority source still active?).")

        # In passive mode with nothing driving there is no "commanded" value
        # to compare against, and falling back to --rate would invent one and
        # report a meaningless -100%.
        ref = cmd_m if cmd_n else (None if self.passive else self.target)

        print("\n" + "=" * 62)
        print("{:<16}{:>10}{:>10}{:>8}{:>16}".format(
            "signal", "mean", "stdev", "n", "vs commanded"))
        print("-" * 62)

        def row(label, m, sd, n):
            if not n:
                print("{:<16}{:>10}{:>10}{:>8}{:>16}".format(label, "-", "-", 0, "-"))
                return
            ratio = ("{:>13.1f} %".format(100.0 * m / ref)
                     if ref not in (None, 0) else "{:>16}".format("-"))
            print("{:<16}{:>10.4f}{:>10.4f}{:>8}{}".format(label, m, sd, n, ratio))

        row("commanded", cmd_m, cmd_sd, cmd_n)
        if wl_n and wr_n:
            implied = WHEEL_RADIUS * (wr_m - wl_m) / WHEEL_SEPARATION
            row("wheels->chassis", implied, 0.0, min(wl_n, wr_n))
        row("encoder odom", odo_m, odo_sd, odo_n)
        row("GYRO (truth)", gyr_m, gyr_sd, gyr_n)
        print("=" * 62)

        if wl_n and wr_n:
            want = (ref * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS if ref else 0.0
            print("per-wheel rad/s   left {:+.3f} (want {:+.3f})   "
                  "right {:+.3f} (want {:+.3f})"
                  .format(wl_m, -want, wr_m, want))
            if abs(want) > 1e-6:
                print("                  left {:.0f}% of setpoint, "
                      "right {:.0f}% of setpoint"
                      .format(100.0 * abs(wl_m) / abs(want),
                              100.0 * abs(wr_m) / abs(want)))

        if not ref:
            print("\nNothing was commanded during the window, so there is "
                  "nothing to verify against.\n  Drive the robot while this "
                  "runs, or use --rate to have it command the turn itself.")
            return 0

        if ref:
            err = 100.0 * (gyr_m - ref) / ref
            print("\nVERDICT: chassis turned at {:.4f} rad/s against {:.4f} "
                  "commanded ({:+.1f}%).".format(gyr_m, ref, err))
            if abs(err) <= 5:
                print("  Within 5% - the commanded rate is being executed.")
            elif abs(err) <= 15:
                print("  Off by {:.0f}%. Check the per-wheel numbers above: if both "
                      "wheels hit their\n  setpoints, suspect wheel_separation "
                      "({} m) or wheel_radius ({} m)\n  rather than the motors."
                      .format(abs(err), WHEEL_SEPARATION, WHEEL_RADIUS))
            else:
                print("  Off by {:.0f}% - that is too much to be calibration noise. "
                      "Compare the\n  per-wheel setpoint percentages: a single low "
                      "wheel points at the drive,\n  two matching wheels point at "
                      "the geometry constants.".format(abs(err)))
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--rate", type=float, default=0.3,
                    help="angular velocity to command, rad/s (default 0.3). "
                         "Negative turns the other way - worth testing both, "
                         "this robot is not symmetric.")
    ap.add_argument("--duration", type=float, default=8.0,
                    help="seconds to hold the rate (default 8)")
    ap.add_argument("--passive", action="store_true",
                    help="command nothing; only observe while you drive")
    ap.add_argument("--linear", action="store_true",
                    help="drive FORWARD at --rate m/s instead of spinning at "
                         "rad/s. No independent sensor exists for this, so the "
                         "report verifies the motor loop and asks for a tape "
                         "measure to confirm wheel_radius.")
    ap.add_argument("--turns", type=float, default=0.0,
                    help="closed loop: spin until the INTEGRATED GYRO reports "
                         "this many 360 deg turns, then stop. Compare where the "
                         "robot actually points against where the gyro thinks "
                         "it does - this is what validates the gyro itself.")
    ap.add_argument("--ramp", type=float, default=RAMP_S,
                    help="ramp-up/down seconds (default %(default)s; use ~0.3 "
                         "for very short runs)")
    args = ap.parse_args()

    rclpy.init()
    node = AngularRateCheck(args.rate, args.duration, args.passive,
                            linear=args.linear, ramp=args.ramp)
    node.turns = args.turns
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    time.sleep(1.0)   # let subscriptions match before measuring

    rc = 1
    try:
        node.run()
        rc = 0 if args.turns else node.report()
    except KeyboardInterrupt:
        print("\ninterrupted - stopping")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
