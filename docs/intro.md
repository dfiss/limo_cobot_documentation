---
sidebar_position: 2
---

# Product Structure: LIMO COBOT

Welcome to the **Product Structure** documentation for the **LIMO COBOT platform** — an integrated modular robotic system combining mobile autonomy, precision manipulation, and AI-powered perception.

## Modular Components Breakdown

LIMO COBOT comprises the following key components:

![LIMO COBOT Overview](/img/product.png)

---

### 1. 🚗 Base Platform – **LIMO (Mobile Robot Chassis)**

- **Manufacturer**: AgileX Robotics  
- **Function**: Mobile base with onboard compute, power, and sensor integrations  
- **Steering Modes**:
  - Differential
  - Ackermann
  - Omni-directional
  - Tracked  
- **Wheel Type**: Rubber all-terrain with independent suspension  
- **I/O Ports**: USB, HDMI, Ethernet, CAN, UART  
- **Battery**: 4S Li-ion with BMS (~40 min runtime)  
- **Model**: LIMO PRO V2.0  
- **Application**: Supports SLAM, navigation, autonomous driving algorithms

---

### 2. 🦾 Robotic Arm – **Elephant Robotics myCobot 280 (6-DoF)**

- **Reach**: 280 mm  
- **Payload**: 250 g  
- **Repeatability**: ±0.5 mm  
- **Weight**: ~850 g  
- **Material**: Aluminum alloy + ABS plastic  
- **Degrees of Freedom**: 6  
- **Connectivity**: USB / Serial  
- **Mount**: Swappable end-effector (grippers, suction, tools)  
- **Use Case**: Pick-and-place, human gestures, lab automation, object sorting

---

### 3. 🧠 Compute Module – **NVIDIA Jetson Orin Nano**

- **CPU**: ARM Cortex-A78AE  
- **GPU**: Ampere architecture with 1024 CUDA cores  
- **AI Performance**: Up to 40 TOPS  
- **Memory**: 8 GB LPDDR5  
- **Interfaces**: GPIO, I²C, UART, SPI, M.2  
- **OS**: Ubuntu 20.04 with NVIDIA JetPack SDK  
- **Purpose**: AI inference (YOLO, pose estimation), ROS2 nodes, sensor fusion

---

### 4. 🔎 Depth Camera – **Orbbec DaBai**  

- **Sensor Type**: Stereo depth + RGB  
- **Depth Resolution**: 1280×720  
- **RGB Resolution**: 1920×1080  
- **Depth Range**: Up to 8 meters  
- **Field of View**: ~90°  
- **Interface**: USB 3.0  
- **Applications**: Obstacle avoidance, 3D mapping, hand tracking, human interaction

---

### 5. 🧭 Inertial Measurement Unit – **IMU (e.g., MPU6050)**

- **Sensors**:
  - 3-Axis Gyroscope
  - 3-Axis Accelerometer  
- **Purpose**: Orientation, angular velocity, acceleration  
- **Usage**: Pose estimation via sensor fusion (IMU + LiDAR + Odometry)

---

### 6. 🧠 LiDAR Sensor – **EAI T-mini Pro**

- **Type**: 2D 360° LiDAR Scanner  
- **Range**: 0.1 – 12 meters  
- **FOV**: 360°  
- **Scan Frequency**: Up to 15 Hz  
- **Angular Resolution**: ≤ 0.25°  
- **Interface**: USB / UART  
- **Use Case**: SLAM, obstacle detection, autonomous navigation

---

## 🔧 Physical Integration Layout

The components are compactly and logically mounted on the LIMO chassis:

- **Robotic Arm**: Top-front mounted — ensures optimal reachability  
- **Depth Camera (Orbbec DaBai)**: Front-mounted — eye-level for human-like perception  
- **LiDAR**: Top-mounted — unobstructed 360° scanning  
- **IMU**: Internally centered — stable motion tracking  
- **Jetson Orin Nano**: Internally mounted with thermal design considerations  

---

## 🧱 Capability Stack Overview

| Layer               | Modules                         | Functional Role                                      |
|--------------------|----------------------------------|------------------------------------------------------|
| **Perception**      | Orbbec DaBai, LiDAR              | Vision, mapping, depth sensing, SLAM                 |
| **Intelligence**    | NVIDIA Orin Nano                 | AI computation, decision-making, sensor processing   |
| **Mobility**        | LIMO Chassis                     | Locomotion, terrain traversal, autonomous driving    |
| **Manipulation**    | myCobot 280                      | Grasping, actuation, interaction                     |
| **Sensing & Feedback** | IMU, LiDAR                  | Pose estimation, obstacle awareness, localization    |

---
