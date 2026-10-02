# Pending Issues

Running list of known gaps/temporary fixes that need real follow-up. Newest on top.

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
