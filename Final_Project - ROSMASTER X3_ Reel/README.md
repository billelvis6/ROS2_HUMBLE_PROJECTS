# ROSMASTER X3 – Tekbot Robotics Challenge 2025

Autonomous System Deployment on Raspberry Pi 5

---

# Overview

This project documents the complete deployment of a **Yahboom ROSMASTER X3** robot for the Tekbot Robotics Challenge 2025.

The system architecture combines:

* Ubuntu 24.04 (Host OS on Raspberry Pi 5)
* Ubuntu 22.04 (via Distrobox container)
* ROS2 Humble
* LiDAR RPLidar
* Astra Pro Camera
* Nav2 Autonomous Navigation
* AMCL Localization
* Custom competition algorithms

The objective was to build a fully autonomous mobile robot capable of:

* Sensor integration (Camera + LiDAR + IMU)
* Real-time localization
* Autonomous navigation
* Mission execution in competition mode

---

# System Architecture

The software stack is separated into 3 logical layers:

## 1️⃣ Robot Hardware Layer (On Raspberry Pi)

Launch file: `robot_real_launch.py`

Responsibilities:

* Start Yahboom motors & IMU
* Launch RPLidar (4s delayed start)
* Launch Astra Camera (6s delayed start)
* Apply TF correction (180° LiDAR rotation)

This layer initializes all physical hardware components.

---

## 2️⃣ Control & Vision Layer (PC Side)

Launch files:

* `pc_controly_launch.py`
* `pc_control_launch.py`

Modes:

* **Manual Mode**

  * Teleop (keyboard or joystick)
  * RViz visualization

* **Vision Mode**

  * Teleop
  * RViz
  * QR detection node (`qr_node`)

---

## 3️⃣ Autonomous Navigation Layer

Launch file: `x3_nav.launch.py`

Components:

* Map loading (`map_win.yaml`)
* AMCL Localization
* Nav2 BT Navigator
* Planner Server (NavFn – Dijkstra)
* Custom Costmaps configuration

Robot dimensions:

* Width: 45 cm
* Clearance margin: 20 cm

The system performs:

* Global path planning
* Local obstacle avoidance
* Autonomous mission execution

---

# Installation Guide

## 1️⃣ Ubuntu 24 on Raspberry Pi 5

Install using Raspberry Pi Imager:
https://www.raspberrypi.com/software/

Machine name:

```
trc2k25
```

Then:

```bash
sudo apt update
sudo apt upgrade
sudo apt install nano usbutils build-essential git
sudo apt install nlohmann-json3-dev
```

---

## 2️⃣ Install Distrobox

```bash
sudo add-apt-repository ppa:michel-slm/distrobox
sudo apt update
sudo apt upgrade
sudo apt install distrobox -y
```

Create Ubuntu 22 container:

```bash
distrobox create --name team_name --image ubuntu:22.04 --hostname team_name
distrobox enter team_name
```

Inside Ubuntu 22:

```bash
sudo apt update
sudo apt upgrade
sudo apt install nano usbutils build-essential git
sudo apt install nlohmann-json3-dev
```

---

## 3️⃣ Install ROS2 Humble

Follow:
https://roboticsbackend.com/install-ros2-on-raspberry-pi/

Install RViz:

```bash
sudo apt install ros-humble-rviz2
```

Clone competition repository:

```bash
git clone https://github.com/TekBot-Robotics-Challenge/2025-ROS-Ressources.git -b development
cd 2025-ROS-Ressources
source configure.sh
```

Copy packages into:

```
~/trc_ws/src
```

Build:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

# Drivers Installation

## Astra Camera

Install dependencies from:
drivers/ros2_astra_camera

On Ubuntu 24:

```bash
cd ~/trc_ws/src/drivers/ros2_astra_camera/astra_camera/scripts
sudo mkdir -p /etc/udev/rules.d/
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Rebuild on Ubuntu 22:

```bash
colcon build --symlink-install
```

---

## RPLidar

Copy udev rules:

```bash
sudo cp usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Test:

```bash
ros2 launch rplidar_ros view_rplidar_a1_launch.py
```

---

# ROS Communication Test (PC ↔ Raspberry Pi)

Both devices on same WiFi.

Add in ~/.bashrc:

```bash
export ROS_DOMAIN_ID=team_id
```

Test:

On Raspberry Pi:

```bash
ros2 topic pub -r 1 /msg std_msgs/msg/String "data: 'Hello Tekbot'"
```

On PC:

```bash
ros2 topic echo /msg
```

---

# ROSMASTER X3 Driver

Install driver:

```bash
pip3 install pyserial
pip3 install .
```

Test bringup:

Terminal 1:

```bash
ros2 run yahboomcar_bringup Mcnamu_driver_X3
```

Terminal 2:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

---

# Super Launchers (Competition Mode)

To simplify startup and avoid CPU overload, we created high-level launch files:

### super_one.launch.py

Hardware + Manual control

### super_two.launch.py

Hardware + Vision

### super_three.launch.py

Hardware + Navigation + Test algorithm

### super_four.launch.py

Full autonomous competition mode

Includes:

* Timed startup (TimerAction)
* Hardware stabilization
* Complete mission execution

---

# Why This Architecture is Robust

* CPU Load management on Raspberry Pi
* Modular testing capability
* Stable Nav2 configuration
* All required BT plugins loaded
* Clear separation between hardware and intelligence

---

# Final Result

The ROSMASTER X3 robot is capable of:

* Sensor fusion (LiDAR + Camera)
* Real-time localization
* Safe navigation
* Fully autonomous mission execution

This project demonstrates advanced skills in:

* Embedded Linux systems
* ROS2 architecture
* Autonomous robotics
* Real robot deployment
* Competition-level system integration

---

# Author

B2MS CleanTech
Tekbot Robotics Challenge 2025 by Bill-Elvis SOMAKOU

