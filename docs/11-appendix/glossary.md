---
sidebar_position: 22
---

# 📚 Glossary of Terms

This glossary defines **ROS2 terminology**, **robotics concepts**, and **project-specific abbreviations** used in the **LIMO Pro + MyCobot + YOLO** documentation.

import Admonition from '@theme/Admonition';

---

## 🔄 ROS2 Terminology

| Term | Definition |
|------|------------|
| **Node** | A single executable in ROS2 that performs computation (e.g., `object_detector`, `mission_manager`). |
| **Topic** | A named bus over which nodes exchange messages asynchronously (e.g., `/yolo/annotated`, `/target_pose`). |
| **Publisher/Subscriber** | ROS2 communication pattern where publishers send messages on a topic and subscribers receive them. |
| **Service** | A synchronous request/response communication method between ROS2 nodes. |
| **Action** | A goal-oriented communication pattern for long-running tasks (e.g., `NavigateToPose`). |
| **Parameter** | Configurable values for nodes, passed via YAML or CLI. |
| **Launch File** | Script that starts multiple nodes with specific configurations. |
| **TF (Transform)** | A ROS2 library for keeping track of coordinate frames over time. |
| **TimerAction** | ROS2 launch feature for delaying node startup to ensure proper sequencing. |

---

## 🤖 Robotics Concepts

| Term | Definition |
|------|------------|
| **SLAM** | Simultaneous Localization and Mapping — building a map of an environment while tracking robot position. |
| **AMCL** | Adaptive Monte Carlo Localization — estimates robot position on a known map. |
| **Nav2** | Navigation stack in ROS2 for autonomous movement. |
| **Waypoint Navigation** | Moving the robot along a predefined set of coordinates. |
| **Localization** | Determining the robot’s position within a map. |
| **Object Detection** | Identifying and locating objects in sensor data (e.g., YOLO model in RGB-D feed). |
| **Manipulation** | Physical interaction with objects using a robotic arm. |
| **Gripper** | End-effector used to grasp or release objects. |

---

## 🛠 Project-Specific Terms

| Term | Definition |
|------|------------|
| **Mission Manager** | The central node controlling exploration, detection interrupts, navigation, and pick/drop sequences. |
| **Pick Node** | Node that commands the MyCobot arm to grasp an object. |
| **Drop Node** | Node that commands the MyCobot arm to release an object. |
| **Pose Setter** | Node that publishes the initial AMCL pose to automate localization. |
| **Full System Launch** | The main launch file (`full_system.launch.py`) that starts all subsystems in the correct sequence. |
| **Approach Distance** | Distance the robot stops from a detected object to allow safe arm movement. |
| **Static TF** | A fixed transform between two coordinate frames (e.g., camera to base link). |

---

## 🔤 Technical Abbreviations

| Abbreviation | Full Form |
|--------------|-----------|
| **RGB-D** | Red-Green-Blue + Depth camera |
| **FPS** | Frames per Second |
| **DOF** | Degrees of Freedom (robot arm joints) |
| **FOV** | Field of View |
| **CPU** | Central Processing Unit |
| **GPU** | Graphics Processing Unit |
| **IP** | Internet Protocol address |
| **SSID** | Service Set Identifier (Wi-Fi network name) |

---

<Admonition type="tip" title="Pro Tip">
If you come across a term in the logs or code you don’t understand, check here first before searching online — this glossary is tailored to **this exact project**.
</Admonition>
