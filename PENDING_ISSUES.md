# Pending Issues

Running list of known gaps/temporary fixes that need real follow-up. Newest on top.

## 2026-10-09 — CPU optimization backlog: IMU-to-MCU, costmap update rates

Found while chasing the goal-reach stutter (same day as the web_hmi keep-alive
fix). Neither of these is applied yet - both need either hardware/firmware
work outside this repo, or a deliberate decision about obstacle-avoidance
behaviour, so they are left here to revisit rather than rushed in.

### 1. Move the BNO055 IMU off the i.MX8MP, onto the motor-control MCU

Measured 2026-10-09: the `bno055` driver node costs 36.7% of one core on its
own - ~9.2% of the i.MX8MP's total 4-core capacity, the single biggest
consumer after the web UI. It is NOT doing fusion math - the BNO055 chip has
its own onboard Cortex-M0 and computes the orientation quaternion itself. The
i.MX8MP's cost is pure driver overhead: polling I2C and publishing a ROS
message at ~65 Hz, when EKF (the only consumer) only runs its filter at 10 Hz.

The motor driver is already CANopen-profile controlled via an i.MX RT MCU
(Cortex-M7, plenty of headroom for a simple I2C poll) with its own telemetry
channel back to the i.MX8MP (encoders, bumper, E-stop, status - see
hardware/include/calixto-ros-bot/mcu_comms.hpp). Folding BNO055 reads into
that SAME channel, instead of running a dedicated driver node on the i.MX8MP,
would move the I2C polling cost off the SBC entirely - the i.MX8MP side would
just be parsing a few more fields out of a packet it already receives, not
running a whole separate 65 Hz node. Expected to recover close to the full
9.2%, likely slightly under it (an Imu message still needs constructing and
publishing from whatever the MCU hands over).

In context: during an active nav2 goal the board runs at ~75.5% CPU busy
(~24.5% headroom). Freeing ~9 points of total capacity would take that to
roughly ~66.5% busy / ~33.5% headroom - a ~35-40% increase in the margin that
was actually tight today.

What is needed, none of which lives in this repo:
  * Confirm the physical wiring - is BNO055 on the i.MX8MP's own I2C bus
    right now, or already reachable from the MCU's?
  * MCU firmware work: read the BNO055 registers, add orientation/angular
    velocity/linear acceleration fields to the existing telemetry frame
    format (mcu_frame_protocol.h - the MCU firmware's own repo, not this
    one). Must not introduce jitter into the real-time motor control loop -
    run it as a lower-priority task, not inline with motor timing.
  * i.MX8MP side: extend diffbot_system.cpp's existing telemetry parsing to
    also read the new fields and publish Imu, instead of launching the
    separate bno055 driver node.

### 2. RETRACTED 2026-10-09 (same day) - costmap rates were never backwards

Originally claimed global_costmap updates at 5.0 Hz and local_costmap at only
1.0 Hz, and proposed swapping them. That was wrong - a line-number mismatch
while grepping without context (both sections share a `update_frequency:`
line, and the wrong one got attributed to each). The actual config, verified
by reading both `ros__parameters` blocks directly:

    local_costmap:  update_frequency 5.0 Hz   (correct - close range, fast)
    global_costmap: update_frequency 1.0 Hz   (correct - far range, slow)

This is already the sensible way round, not backwards. No change needed, none
applied. Checked against the actual behaviour tree
(navigate_to_pose_w_replanning_and_recovery.xml, stock, not customised here):
`RateController hz="1.0"` governs how often the global planner replans, which
matches global_costmap's 1 Hz exactly - already in sync. local_costmap at
5 Hz vs the controller reading it every cycle at 20 Hz is a 4x gap, not the
20x this entry originally implied, and 5 Hz is nav2's own standard default
for a local costmap - not a misconfiguration.

A same-day 7-day reminder (routine trig_01W68RSD8NPvFdri8zidgTeM) was created
and then corrected to drop this item once the error was caught.

## 2026-10-09 — Waypoint loop runs in the web server, not nav2's waypoint_follower

The web UI's new Waypoints & loop section (status_web_server.py) does NOT use
nav2's own `nav2_waypoint_follower` / `/follow_waypoints` action. Deliberate
trade-off, not an oversight:

  * This board is already CPU-tight for navigation - see the 2026-10-08 entry
    on sim_time/vtheta_samples having to be sized down after the controller
    missed its 20 Hz deadline ~38x per goal at the "obvious" settings.
    waypoint_follower would be a 9th long-running process, plus another
    lifecycle node for lifecycle_manager to bring up and hold a bond to -
    one more thing that can fail to activate the same way planner_server
    already does while waiting on an initial pose.
  * Instead the loop is implemented in status_web_server.py itself: a
    background thread calls the same publish_goal() → /goal_pose that the
    page's own click-and-drag goal already uses, and watches the goal-status
    tracking added the same day (bt_navigator's own action-status topic) to
    know when each leg finishes before sending the next.
  * One failed/aborted/canceled waypoint stops the WHOLE loop, rather than
    skipping ahead to the next one. A loop is meant to run unattended, and
    "something went wrong, so it kept going and made it worse" is a worse
    failure mode than "it stopped and is waiting for you" - especially given
    the goal-abort spirals this same day that prompted the Cancel goal button
    and the Restart navigation button in the first place.

Revisit if nav2_waypoint_follower's CPU/process cost is ever actually
measured and found acceptable - it has genuine features this reimplementation
does not (waypoint task executors, pause-at-waypoint, etc.).

## 2026-10-08 — RViz freezes: tf2 BufferCore deadlock (root cause found, mitigated not fixed)

RViz2 hangs permanently after anywhere from ~1 to ~7 minutes, the window stops
redrawing, GNOME shows "rviz2 is not responding", and force-quitting it is what
produced every `exit code -9` ("crash") seen on 2026-10-07. The kill was the
symptom; the hang is the bug.

### Root cause

An AB-BA deadlock between two mutexes inside the SAME `tf2::BufferCore`
object. Caught with gdb while hung (needs `sudo sysctl -w
kernel.yama.ptrace_scope=0`, default 1 blocks attaching):

    Thread 1 (GUI)          waits futex 0x...bf0
      VisualizationManager::onUpdate -> Executor::spin_some_impl
       -> execute_subscription (LaserScan)
        -> TFFrameTransformer::waitForTransform
         -> tf2_ros::Buffer::waitForTransform
          -> tf2::BufferCore::addTransformableRequest   <-- blocked

    Thread 16 (TF listener) waits futex 0x...ca0
      TransformListener::subscription_callback
       -> BufferCore::setTransform -> setTransformImpl
        -> BufferCore::testTransformableRequests        <-- blocked

The two futex addresses are 176 bytes apart, i.e. two members of one
BufferCore, taken in opposite orders. It never recovers.

Trigger: a message arrives whose transform to the fixed frame is not yet
available, so the display's TF message filter registers a "transformable
request" - exactly the `Message Filter dropping message ... queue is full`
lines that appear in every RViz log here. The TF listener then services those
requests while inserting a transform, and the two collide.

Upstream tf2 bug, not a config error. `ros-humble-tf2` 0.25.23 is installed and
apt has no newer candidate, so there is nothing to upgrade to yet. Re-check
after a ROS distro update.

### What this is NOT (all tested and ruled out on 2026-10-08)

  * Not the network. It deadlocks identically replaying a LOCAL bag with zero
    sockets to the robot. WiFi jitter (laptop->robot avg 23 ms, max 141 ms, vs
    ~5-6 ms from each machine to the router) makes it MORE FREQUENT, because
    late transforms mean more pending requests - but it is not the cause.
  * Not the GPU. Moving RViz to the NVIDIA GTX 1650 via
    `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` removed the
    harmless Ogre/Mesa "active samplers" GLSL error but changed nothing else.
  * Not XWayland. The session was switched to Xorg (`loginctl show-session`
    Type=x11) and it still deadlocked. qtwayland5 is pointless here - there is
    no Wayland compositor running, so `wl_display` connection is refused.
  * Not OOM (`memory.events` oom_kill 0), not the kernel (journal completely
    silent at every hang), not costmap display load (disabled both, still hung).

### How to run RViz without it hanging

The deadlock is reached through TF MESSAGE FILTERS, and the LaserScan display
is by far the worst offender because it runs at 15 Hz while the map and costmap
displays run at ~1.7 Hz. Dropping LaserScan is roughly a 10x cut in exposure.

IMPORTANT: this is mitigation, not immunity. The Map and costmap displays use
TF message filters too, so they can still reach the same lock pair - just much
less often.

  * For costmap/plan work, use the config that has no LaserScan at all:

        ros2 launch calixto-ros-bot rviz_remote.launch.py \
            rviz_config:=$(ros2 pkg prefix calixto-ros-bot)/share/calixto-ros-bot/rviz/costmap_analysis.rviz

    (`rviz/costmap_analysis.rviz`: Grid, RobotModel, Map, LocalCostmap,
    GlobalCostmap, GlobalPlan, LocalPlan, Footprint; fixed frame map.)

  * If the LiDAR scan IS needed, `rviz/lidar_robot_odom.rviz` uses fixed frame
    `odom` instead of `map`, so the scan needs only odom->laser_frame, which is
    always present. Partial help: it narrows the window but scan drops still
    occur, so requests still get registered.

  * Add the NVIDIA offload env vars. They do not prevent the hang but they do
    remove the GLSL error and give smoother rendering:

        export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

  * For deep analysis prefer `docker/record_bag.sh` on the robot + `ros2 bag
    play` locally over long live sessions - less exposure, and you can scrub
    and repeat. NOTE: that script currently MISSES `/map` and other
    TRANSIENT_LOCAL topics, because rosbag2 on Humble subscribes VOLATILE by
    default. Needs a QoS override file before those bags are useful.

### Catching it again

`scripts/`-less helper lives in the session scratchpad, but the method is:
set `kernel.yama.ptrace_scope=0`, poll `/proc/<pid>/wchan` once a second, and
when it stays `futex_wait_queue` for ~25 s run
`gdb -p <pid> -batch -ex "thread apply all bt 25"`. Normal operation cycles
through `do_poll`/`0`, so a long solid futex streak is a real hang.

## 2026-10-07 — LiDAR body filter: check CPU overhead once mapping and navigation run

The laser_filters box filter (`config/scan_box_filter.yaml`, run as step 8 in
`docker/run_full_stack.sh`) removes LiDAR returns on the robot body: the four
load poles and the wiring inside the chassis. Its box is x ±0.40 m, y ±0.25 m
in base_link, with a 1 cm margin. Mapping and the web UI read `/scan_filtered`.

**Measured so far (idle stack, robot still):**
- Filter: about 3% of one core, instantaneous. The earlier Python version
  (`scripts/scan_body_filter.py`) used about 20%, so it was replaced.
- Whole board: load about 2.6 on 4 cores (about 65%), temperatures 51–53 °C,
  no throttling. Top consumers: `bno055` about 39%, web UI about 32%, EKF about 10%.

**To check later:**
- Re-measure the filter's CPU while mapping and while navigation run. Those add
  more load than the filter does.
- Confirm the box still removes the poles after any change to the robot's
  dimensions or the LiDAR position (`description/robot_core.xacro`, `lidar.xacro`).
- Check the `bno055` load, which is the largest single consumer.

## 2026-10-03 — Web keyboard/joystick keeps moving after release on a laggy network (partly fixed, on hold)

Observed by the owner: when the network lags, the wheels keep moving after a
key is released. Cause: the browser sends commands as separate HTTP POSTs,
which queue and arrive late and out of order. A stale motion command arriving
after the stop restarts motion for another 0.5 s (the twist_mux teleop timeout).

Reproduced on the board with a simulated laggy client (random 0–2 s per request,
`teleop_lag_test.py`, not committed). Before the fix, stale motion arrived up to
1.6 s after release and the robot moved for about 2 s.

**Partly fixed (uncommitted, on the board):** every browser command now carries
a session id and sequence number, and `status_web_server.py`'s `publish_teleop`
drops anything older than the newest accepted command. After the fix, the robot
stops about 0.5 s after the last accepted command, roughly the lag plus 0.5 s.

**Still open (held for now):**
- The stop itself travels over the same laggy link, so it can't arrive sooner than the link allows.
- Options: (1) lower the twist_mux teleop timeout from 0.5 s to 0.3 s in `config/twist_mux.yaml`, which shortens the tail but leaves less margin for the 200 ms resend; (2) have the browser cancel queued motion requests on release (helps only requests not yet sent).
- Not yet tested with a real browser over the real Wi-Fi link. The lag test was simulated in software.
- Change in `web_hmi/status_web_server.py` (sequence check + browser tagging) is uncommitted.

## 2026-10-02 — Wheel direction invert flipped between two restarts (watch for recurrence)

After the owner rewired the board onto a flat surface, the right wheel
span backward when driven straight. Fixed with a ROS2-side per-wheel
invert (`invert_left`/`invert_right` in `diffbot.ros2_control.xacro` -
see that file's own comment and `hardware/diffbot_system.cpp`'s cfg_
comment for the full mechanism) - deliberately a config value, not a
firmware change, per the owner's explicit instruction.

**Then it flipped again on its own.** Restarted the board (no deliberate
rewiring in between) and the *exact same* `invert_right=true` config,
sending the *exact same* commanded signs as the verified-working test,
produced the *opposite* real-world result (right wheel backward again).
Toggled to `invert_right=false`, confirmed correct, survived one more
restart since (2026-10-02).

**Ruled out**: CANopen node-ID auto-discovery non-determinism - checked
`src/motor_can/motor_config.h` in the firmware repo directly,
`DS_AXIS_A_NODE_ID`/`DS_AXIS_B_NODE_ID` are fixed (1/2), not discovered
at boot, so this isn't a "random enumeration order" software issue.

**Leading theory, not confirmed**: a loose or unkeyed motor/CAN
connector that can physically reseat in either of two valid orientations
depending on handling/vibration during a restart - would explain
identical software behaving differently across power cycles with no
deliberate rewiring.

**What to check next time this comes up**: has `invert_right` stayed
`false` (correct) across several more restarts/power cycles with nothing
physically touched? If yes for a good while, probably settled - consider
this resolved. If it flips again on its own, that confirms the loose-
connector theory - the real fix at that point is physical (secure/key
the connector) rather than yet another config toggle, since a config
value can't reliably compensate for a connection that isn't stable.

## 2026-10-01 — Wheel-slip detection via IMU (deferred until floor deployment)

Owner's request: detect a wheel slipping/losing traction (e.g. dropping
into a gutter) as a new exceptional/safety case, using the IMU as an
independent cross-check against wheel encoder data (wheels can report
"motion" via encoders while the chassis isn't actually moving/rotating
that way - a slipping or stuck wheel). Deliberately **not implemented
yet** - owner wants to do this once the robot is actually deployed on
the floor, since the real-world thresholds (how much wheel-vs-IMU
mismatch is normal noise vs. real slip) can't be meaningfully tuned on
a bench.

Design sketch for whenever this is picked up again:
- **Rotational slip**: compare wheel-odometry-implied yaw rate
  (`/diffbot_base_controller/odom` twist.angular.z) against the BNO055's
  own raw gyro (`/bno055/imu` angular_velocity.z, not the EKF-fused
  output - fusing already trusts wheel data, so comparing against the
  fused estimate wouldn't be an independent check). Sustained mismatch
  beyond some threshold/duration = flag.
- **Linear slip/stall** ("stuck in a gutter"): steady-state accelerometer
  comparison doesn't work (constant velocity = ~zero acceleration either
  way, slipping or not) - instead watch for a *commanded ramp-up* in
  wheel velocity with no corresponding kick in the IMU's gravity-
  compensated linear acceleration *magnitude* (orientation-independent,
  since the IMU's final mounting angle on the chassis isn't nailed down
  yet either).
- Publish as a new diagnostic (same `diagnostic_msgs/DiagnosticArray`
  pattern as `/mcu_status`) so it can plug into the HMI's AMR Status box
  the same way E-stop/bumper/CAN-fault already do.
- Thresholds will need real tuning on the actual floor surface(s) this
  robot runs on - don't hardcode guessed values and assume they're right.

## 2026-10-01 — RF remote control integrated, two Docker/USB gotchas left open

RF remote now fully wired (see README's "RF remote control" section for
the full writeup and the `ch341` kernel module fix). Two follow-ups not
done yet, low priority unless they start actually causing problems:

- `docker/run_container.sh` doesn't bind-mount `/dev` into the container,
  so `/dev/serial/by-id/` stable paths aren't visible in-container - only
  raw `/dev/ttyUSB0`-style paths work, which can renumber if a second
  USB-serial adapter is ever added. Fix: add `-v /dev:/dev` if/when that
  becomes a real problem.
- Hot-plugged USB devices (or a driver loaded after container start) don't
  appear inside this `--privileged` container until it's restarted - not
  obviously fixable without the `-v /dev:/dev` change above either.

## 2026-10-01 — Encoder counts/rev mismatch (temporary ROS-side fix applied)

**What's wrong:** `description/diffbot.ros2_control.xacro`'s `enc_counts_per_rev`
was documented as `4096` (L2DB4830 hub-motor, 12-bit magnetic encoder, direct
drive — a specific, sourced spec, not a guess). Two independent physical
measurements say otherwise:

- Wheel marked, motor spun at a slow constant command, rotations counted by
  eye against the *exact* encoder tick delta read back over UART from the
  i.MX RT MCU (`/bno055` unrelated — this is the `lpuart6` MCU link).
- Run 1: 25282/25297 ticks (left/right) over 4.5 counted wheel rotations →
  ~5618-5622 ticks/rev.
- Run 2: 50529/50563 ticks over ~9.0 counted wheel rotations → ~5614-5618
  ticks/rev.
- Both runs agree closely: **~5618 ticks per real wheel revolution**, not 4096.

**Leading theory (not confirmed):** this is a direct-drive hub motor, so a
mechanical gear ratio doesn't make sense as the explanation. More likely:
the DS20270C drive has an internal position-scaling object (left over from
this exact drive's *previous life* on a heavy-duty pallet-shuttle
application, per the owner) that silently rescales raw encoder counts
before they're ever reported over CAN/telemetry — independent of the
encoder's true physical resolution. Not yet verified against the drive's
actual CiA402 object dictionary.

**Status / what's been done:**
- `enc_counts_per_rev` changed `4096` → `5618` in
  `description/diffbot.ros2_control.xacro`, **ROS2 side only, temporary**,
  per the owner's explicit instruction. This fixes odometry position (and
  therefore derived velocity, since `diffbot_system.cpp` differentiates
  position rather than trusting the firmware's own RPM-based
  `measured_vel_left/right` telemetry field) to roughly the right real-world
  scale.
- Firmware (`calixto-amr-imx-rt-app`) deliberately **left untouched** — owner
  wants this fixed properly at the source next time the i.MX RT is
  reflashed, not patched twice in two places. See that repo's
  `src/CLAUDE.md` for the corresponding gap entry.
- **Command path was checked and is NOT affected** — `diffbot_system.cpp`
  passes `cmd_vel_left/right` straight through without involving
  `enc_counts_per_rev` at all, and the firmware's CANopen velocity command
  appears to already be real-world-RPM-referenced (9 RPM commanded produced
  almost exactly 4.5 real rotations in 30s, matching physical expectation).
  Only the *tick-to-position* conversion was wrong.

**Still needed:**
- Root-cause the actual drive-side scaling (check the DS20270C's object
  dictionary directly, e.g. position factor / encoder resolution objects)
  instead of just compensating for the symptom.
- Re-measure more precisely once that's understood (current ~5618 has some
  uncertainty from eyeballing partial rotations by eye — fine for rough
  odometry, not for anything that needs real precision).
- Once root-caused, fix it properly in firmware (or wherever the real
  scaling lives) and revert this ROS2-side value back to whatever is
  actually correct for that fix.
- **Reminder requested by the owner**: revisit this when the i.MX RT
  flasher next connects.

## 2026-10-01 — ros-amr repo is missing source that was never committed

Discovered while setting up the i.MX8M Plus Docker container (fresh clone
exposed what local-only work had never reached GitHub). Untracked in git,
confirmed build-blocking or needed, currently only present as local files:
`calixto-ros-bot.xml`, `hardware/include/calixto-ros-bot/wheel.hpp`,
`description/`, `gazebo/`, `rviz/`, `worlds/`, `hooks/`, most of
`config/*.yaml`, most of `launch/*.py`, and an empty `maps/` directory (git
can't track an empty dir — needs a `.gitkeep`). Decision on committing these
(and the new `docker/` folder) is still pending.

Separately flagged, not yet decided:
- `test_commands.txt` — looks like personal scratch/testing notes, probably
  should stay untracked/excluded rather than committed.
- `"config/nav2_params (copy).yaml"` — differs from `nav2_params.yaml` only
  in `rotate_to_heading_angular_vel` (1.6 vs 1.0, comment says "was 1.8 -
  less overshoot") — looks like an intentional tuning experiment. Keep,
  discard, or merge into the real file?

## 2026-10-01 — CAN bus access from inside the ros2_humble container

Never actually verified whether `can0` is reachable/usable from inside the
Docker container on the i.MX8M Plus (host-level `can0` was confirmed fine
independently). Quick check, just never circled back to it.

## 2026-10-01 — bno055 + IMU monitor don't survive a board/container reboot

Both `ros2 launch bno055 bno055.launch.py` and `imu_monitor_server.py` were
started by hand (`docker exec -d ...`), not wired into the container's
persistent startup — confirmed they do NOT come back after a board reboot
(container itself does, via `--restart=always`, but these two don't).
Needs a real supervised-startup mechanism (e.g. baked into the image's
entrypoint, or a systemd/supervisor unit inside the container) once the
bring-up/testing phase is done and this is ready to be "always on."

## Earlier, already acknowledged as deferred (not forgotten, just not yet)

- Docker `--privileged` device-access model: intentionally simple for now,
  revisit with an explicit `--device` list once cameras/LIDAR are physically
  connected and the final hardware list is known.
- Real `twist_mux` (added to the Dockerfile to replace the host's hand-rolled
  `cmd_vel_mux.py`) has not been exercised end-to-end in an actual multi-
  source arbitration test on this board yet.
- i.MX95/SPI transition: i.MX8M Plus + UART (`lpuart6` ↔ UART3) is the
  explicitly-interim transport; i.MX95 + SPI is the ~3-4 week target (owner
  estimate, 2026-10-01). Tracked in detail in
  `calixto-amr-imx-rt-app/src/CLAUDE.md` Gap #22 and
  `calixto-amr-imx-rt-app/calixto-amr-info.md`'s Transport status note -
  cross-referenced here since it's directly relevant to the encoder-scaling
  item above (both get revisited at the same "flasher reconnects" moment).
