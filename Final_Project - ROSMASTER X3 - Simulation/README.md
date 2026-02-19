# ROSMASTER X3 Simulation (Gazebo + Nav2 + QR Detection)

This section describes the full simulation environment developed **before receiving the real robot**.
The objective was to validate navigation, perception and mission logic in Gazebo, then transfer the same software stack to the physical ROSMASTER X3.

The simulation reproduces the competition arena and allows:

* Mapping (SLAM)
* Localization (AMCL)
* Autonomous navigation (Nav2)
* QR code detection
* Mission testing

---

# Workspace Structure

```
trc_ws/src/
 ├── 2025-ROS-Ressources
 ├── maps
 │    ├── ma_carte_yes.pgm
 │    └── ma_carte_yes.yaml
 ├── my_nav_pkg
 │    ├── config/nav2_params.yaml
 │    ├── launch/x3_nav.launch.py
 │    ├── rviz/robot_view.rviz
 ├── qr_scanners
 │    └── qr_node.py
 ├── robot_bringup
 │    └── bringup.launch.py
 ├── trc_navigation
 │    └── qr_navigator_node.py
```

---

# Build the Workspace

```
cd ~/trc_ws
colcon build --symlink-install
source install/setup.bash
```

Launch the simulation:

```
ros2 launch sim_trc sim_trc_no_gpu.launch.py
```

This launch file:

* Starts Gazebo with the arena world
* Spawns the ROSMASTER X3 robot
* Loads robot description
* Starts joystick teleoperation
* Starts QR detection node

---

# Gazebo Troubleshooting

If Gazebo refuses to start:

```
[Err] Unable to start server[bind: Address already in use]
```

Kill existing processes:

```
pkill -f gzserver
pkill -f gzclient
```

---

# QR Code Detection

The robot uses the front camera to detect QR codes placed in the arena.

Dependencies:

```
sudo apt install ros-humble-cv-bridge
pip install opencv-python pyzbar
```

Run the node:

```
ros2 run qr_scanners qr_node
```

Features:

* Detects QR codes in real time
* Saves detected images
* Logs detections into CSV file
* Associates QR with robot position

Output files:

```
data/
 ├── qr_log.csv
 └── images/
```

---

# Mapping (SLAM)

Install SLAM Toolbox:

```
sudo apt install ros-humble-slam-toolbox
```

Start simulation:

```
ros2 launch sim_trc sim_trc_no_gpu.launch.py
```

Start SLAM:

```
ros2 launch slam_toolbox online_async_launch.py
```

Control robot manually:

```
ros2 run joy joy_node
ros2 run x3_control yahboom_joy_X3.py
```

Save the generated map:

```
ros2 run nav2_map_server map_saver_cli -f ~/trc_ws/src/maps/ma_carte_yes
```

Generated files:

```
ma_carte_yes.pgm
ma_carte_yes.yaml
```

---

# Autonomous Navigation (Nav2)

## Localization

```
ros2 launch nav2_bringup localization_launch.py map:=./src/maps/ma_carte_yes.yaml use_sim_time:=true
```

## Navigation

```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscriber_transient_local:=true params_file:=./src/my_nav_pkg/config/nav2_params.yaml
```

## Visualization

```
rviz2
```

Configure RViz:

* Map → `/map`
* LaserScan → `/scan`
* RobotModel → `/robot_description`
* Path → `/plan`
* TF → enabled

Initialize robot:

* 2D Pose Estimate
* 2D Goal Pose

---

# Launching the Combined Navigation

```
ros2 launch my_nav_pkg x3_nav.launch.py
```

A startup delay was added using `TimerAction` to avoid a known ROS2 timing issue:

Problem:
AMCL and sensors start before TF and simulation clock are stable → messages dropped.

Solution:
Delay navigation nodes until Gazebo and TF are ready.

---

# Robot Position Monitoring

Get pose from TF:

```
ros2 run tf2_ros tf2_echo odom base_link
```

Get localization pose:

```
ros2 topic echo /amcl_pose
```

---

# QR Locations (Competition Map)

The robot must visit and identify QR-coded districts.

| QR   | District     | Type        |
| ---- | ------------ | ----------- |
| QR_1 | Morocco Mall | Commercial  |
| QR_2 | Erevan       | Commercial  |
| QR_3 | Toamasina    | Industrial  |
| QR_4 | Arkadia      | Industrial  |
| QR_5 | Nabi Yaar    | Commercial  |
| QR_6 | Tampouy      | Residential |
| QR_7 | BeauSejour   | Residential |

Coordinates were measured in simulation and later reused on the real robot.

---

# Final Objective

The simulation environment allowed full validation of:

* Mapping accuracy
* Localization stability
* Path planning reliability
* Perception pipeline (QR detection)

This ensured a smooth transition from simulation to the physical ROSMASTER X3 robot with minimal modifications.
