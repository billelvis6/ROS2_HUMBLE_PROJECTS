# Sensor Data Evaluation (ROS2 – C++)

## Overview

This project demonstrates the **publisher–subscriber communication mechanism in ROS2** using a simulated environmental sensor system.

A ROS2 package named `sensor_data_evaluation` was developed to simulate and evaluate sensor measurements in real time.

The system publishes environmental data periodically, then verifies whether the received values respect predefined ranges.

Developed with **ROS2 Humble** and **C++ (rclcpp)**.

---

## Objectives

* Understand ROS2 architecture
* Implement topic-based communication
* Simulate real-world sensor measurements
* Validate received data
* Use launch files to orchestrate nodes

---

## ROS2 Concepts Used

| Concept     | Usage                                          |
| ----------- | ---------------------------------------------- |
| Node        | Independent processes (Publisher & Subscriber) |
| Topic       | `/sensor_data`                                 |
| Message     | `std_msgs/String`                              |
| Timer       | Periodic publishing                            |
| Launch file | Start multiple nodes                           |

---

## Package Structure

```
sensor_data_evaluation/
 ├── src/
 │    ├── sensor_node_publisher.cpp
 │    └── sensor_node_subscriber.cpp
 ├── launch/
 │    └── sensor_data_eval_launch.xml
 ├── CMakeLists.txt
 └── package.xml
```

---

## Publisher Node

**Node name:** `sensor_node_publisher`
**Publishing topic:** `/sensor_data`
**Publishing rate:** 0.5 seconds

The node generates random environmental data:

| Parameter   | Range              |
| ----------- | ------------------ |
| Temperature | 15°C – 35°C        |
| Humidity    | 30% – 70%          |
| Pressure    | 950 hPa – 1050 hPa |

Example log:

```
Publishing: T = 23°C, H = 45%, P = 1002hPa
```

---

## Subscriber Node

**Node name:** `sensor_node_subscriber`
**Subscribed topic:** `/sensor_data`

The subscriber:

* Receives sensor data
* Parses values
* Checks if values are inside acceptable ranges
* Displays validation result

Example output:

```
Data received: T = 23°C, H = 45%, P = 1002hPa : correct values
```

---

## Build Instructions

```
cd ~/workspace
colcon build --symlink-install
source install/setup.bash
```

Run the project:

```
ros2 launch sensor_data_evaluation sensor_data_eval_launch.xml
```

---

## Testing Tools

Useful ROS2 commands:

```
ros2 topic list
ros2 topic echo /sensor_data
rqt_graph
rqt_topic
```

These tools verify communication between nodes.

---

## Dependencies

* ament_cmake
* rclcpp
* std_msgs
* ros2launch

ROS2 Distribution:

```
ROS2 Humble
```

---

## Multi-Machine Communication (Bonus)

The publisher and subscriber were tested on **two separate machines** connected to the same network.

Requirements:

* Same ROS2 distribution
* Same network
* Firewall disabled
* Same `ROS_DOMAIN_ID`

This validates ROS2 distributed DDS communication.

---

## Limitations

* Data transmitted as formatted string instead of structured message
* Integer values only
* XML launch file

---

## Possible Improvements

* Custom ROS2 message type
* Floating-point data
* Python launch file
* QoS configuration
* Unit tests

---

## Conclusion

This project validates the fundamental ROS2 communication workflow and demonstrates a basic distributed robotic software architecture.

It represents a first step toward more advanced robotic systems such as sensor fusion, localization and autonomous navigation.

---

**Author:** B2MS CleanTech Team by Bill-Elvis SOMAKOU
**Year:** 2025
