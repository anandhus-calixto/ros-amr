## Calixto-ros-bot

This is the repo of calixto's amr proto. 
version:
ROS HUMBLE, Gazebo 11.10, 

*TO SLAM (Mapping )*

1. ros2 launch calixto-ros-bot launch_sim.launch.py world:=src/calixto-ros-bot/gazebo/maze.world

2. rviz2 

3. ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/calixto-ros-bot/config/mapper_params_online_async.yaml use_sim_time:=true

[ Add new map, laser_scan , assign topics ]

4. ros2 run teleop_twist_keyboard teleop_twist_keyboard

drive around and create map

5.  Save map using rviz pluggin, serial and pgm

6. change the mapper_param file #mapping to #localisation and map path below it, start at dock true

*TO EXECUTE AUTONOMOUS NAVIGATION PROCESS*

1. To run the Gazebo simulation launch file :

ros2 launch calixto-ros-bot launch_sim.launch.py world:=src/calixto-ros-bot/gazebo/maze.world

2. To run the Rviz2 for Nav2 :
rviz2

3. To launch the slam_toolbox with existing map :

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/calixto-ros-bot/config/mapper_params_online_async.yaml use_sim_time:=true

(this method will map and can be used for the navigation purpose also ; map+nav)

4. To launch the Nav2 navigation stack :

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

5. To run the teleop keyboard for navigation (if req.) :

ros2 run teleop_twist_keyboard teleop_twist_keyboard


***** NAV2 with AMCL*****
1. ros2 run twist_mux twist_mux --ros-args --params-file ./src/calixto-ros-bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped  

// required in the possible future for autonomous + remote driving , mux the input from var sources

2. ros2 launch calixto-ros-bot launch_sim.launch.py world:=src/calixto-ros-bot/gazebo/maze.world

3. ros2 launch nav2_bringup bringup_launch.py     map:=/path/to/map.yaml     use_sim_time:=true     params_file:=/home/anandhu-sudha/ws_ros2/src/calixto-ros-bot/config/nav2_params.yaml

4. rviz2

// load with saved config.

5. ros2 launch nav2_bringup localization_launch.py     map:=/home/anandhu-sudha/ws_ros2/src/calixto-ros-bot/maps/my_map.yaml     use_sim_time:=true     map_subscribe_transient_local:=true

set the initial pose, set the cost map and all, set the target location.

## Real hardware: Web HMI

Everything above is the Gazebo-simulation workflow. For the real robot
(i.MX RT1176 MCU over serial + an RF remote), there's a single-command
bring-up and a browser-based HMI instead - no RViz2/terminal juggling
required for day-to-day driving.

**Quick start** (after a `colcon build` in this workspace):

```bash
./run_hmi.sh
```

This sources ROS2 + the workspace and runs `launch/hmi_bringup.launch.py`,
which starts:
- The control stack (`ros2_control` + `diff_drive_controller`, no EKF -
  see `diffbot_no_ekf.launch.py`'s own header for why).
- `cmd_vel_mux.py` - merges remote/keyboard/nav cmd_vel sources by priority
  (`config/cmd_vel_mux.yaml`) into the one `/cmd_vel` the controller reads.
- `remote_ros_node.py` - reads the RF remote, publishes `/cmd_vel_remote`.
- `web_hmi/status_web_server.py` - the browser HMI itself.

Then open `http://<this-machine's-LAN-IP>:8080` from any device on the same
network (or `http://localhost:8080` on the machine itself).

**In the HMI:**
- **AMR Status** - a single clear-or-error summary (E-stop/bumper-latch/
  CAN-fault/comms-timeout/drive-fault), not a raw checklist. Front and back
  E-stop are distinguished, and more than one issue can show at once.
- **Drive mode** - Remote / Keyboard / Navigation. Leave none selected and
  whichever is actually publishing wins by priority (remote highest); pick
  one to force it exclusively.
  - *Keyboard* mode captures `teleop_twist_keyboard`'s exact key bindings
    (`u i o` / `j k l` / `m , .`) directly in the page - no separate
    terminal - plus an on-screen joystick (mouse or touch) for driving
    from a phone.
  - *Navigation* mode has no autonomy yet (needs LIDAR/IMU + Nav2 on the
    real robot), but goal-setting is already wired: drag on the map (or
    long-press-then-drag on touch) to publish a goal on `/goal_pose`.
- **Map view** - a live 2D canvas driven by `/diffbot_base_controller/odom`
  (world-fixed, the robot moves within it - pan/zoom independently with
  mouse or touch), not an embedded RViz2. A real RViz2 embed via VNC was
  tried and abandoned - see `web_hmi/status_web_server.py`'s docstring.
- **Open RViz2** button - opens a native RViz2 window on whichever machine
  is running the HMI server (one-shot, not embedded in the page).

**Serial devices**: `hmi_bringup.launch.py` defaults to the MCU and RF
remote's stable `/dev/serial/by-id/...` paths instead of `/dev/ttyUSB0`/`1`,
which renumber across reboots/replugs. If either physical device is ever
swapped, check `ls /dev/serial/by-id/` and pass the new path, e.g.
`./run_hmi.sh serial_port:=/dev/serial/by-id/...`.
