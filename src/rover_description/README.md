# Rescue Rover — `rover_ws` recovery snapshot

This is a faithful rebuild of the `rover_description` package as it stood when the
SSD failed. Everything here you wrote and reasoned through yourself; this is just
your work restored.

## How to restore

1. Put the workspace back in your home folder:
   ```
   ~/rover_ws/
   └── src/
       └── rover_description/
   ```
   (Unzip so that `rover_ws/src/rover_description/` ends up at `~/rover_ws/src/rover_description/`.)

2. Build and source (the overlay is NOT auto-sourced — you chose to source it by hand):
   ```
   cd ~/rover_ws
   colcon build
   source install/setup.bash
   ```

## Package contents

- `urdf/rover.urdf.xacro` — full robot: sea-blue chassis, left/right driven wheels
  (`wheel` macro), front/rear casters (`caster` macro), `laser_frame`, and the
  `gz-sim-diff-drive-system` DiffDrive plugin (wheel_separation 0.24, wheel_radius 0.05,
  topic `cmd_vel`, odom at 30 Hz). Wheels centred at x=0; casters at ±0.12 for turn-in-place.
- `launch/display.launch.py` — processes the xacro, starts robot_state_publisher,
  rviz2 (loads the saved config), and joint_state_publisher_gui.
- `rviz/view_robot.rviz` — RobotModel + TF, Fixed Frame `base_link`.
- `setup.py` / `package.xml` / `setup.cfg` — ament_python package config; data_files
  installs urdf, launch, and rviz into the share space.

## Where we were (state at failure)

The differential-drive base is complete and drives in Gazebo. Confirmed working:
- Spawns and settles stably in Gazebo (collision + inertial validated).
- Drives via raw `gz topic` AND via ROS `/cmd_vel` through the bridge.
- Bridge command (ROS → Gazebo, hence the `]`):
  ```
  ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
  ```

## To drive it again (3–4 terminals, source overlay in each)

```
# T1 — Gazebo
gz sim empty.sdf

# T2 — robot_state_publisher (+ rviz, jsp_gui)
ros2 launch rover_description display.launch.py

# T3 — spawn the rover into Gazebo
ros2 run ros_gz_sim create -topic robot_description -name rover -z 0.1

# T4 — bridge, then drive
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## THE NEXT STEP (where we paused)

Bridge **odometry and TF** out of the DiffDrive plugin so the rover knows where it is
(needed for SLAM). Before bridging, discover the real Gazebo topic names — with the sim
running and the rover spawned:

```
gz topic -l
```

Look for the odometry topic and the TF topic the plugin publishes (you set `cmd_vel`
explicitly, but left these as defaults, so they'll likely be `/model/rover/...`).
Report the exact names and we build the bridge specs for them.

TF-edge ownership target for SLAM:
- `map → odom`        → SLAM (later)
- `odom → base_link`  → DiffDrive odometry (bridging next)
- `base_link → *`     → robot_state_publisher (already done)

Note: when running with Gazebo, set `use_sim_time:=true` on the ROS nodes, and switch the
RViz Fixed Frame from `base_link` to `odom` once odometry/TF is bridged.

## Roadmap remaining (your domain)

odometry/TF bridge → simulated LiDAR on `laser_frame` + bridge `/scan` → SLAM
(slam_toolbox, build from source for Lyrical; lifecycle node, needs configure→activate)
→ risk heatmap → A* path planning → rebuild as 4-wheel skid-steer → RL navigation.
