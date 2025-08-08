---
sidebar_position: 20
---

# 🛠 Hardware Specifications

This section lists the complete hardware specifications for the **LIMO Pro + MyCobot + YOLO** platform, including datasheets, performance notes, and compatibility information.

import Admonition from '@theme/Admonition';

---

## 🚗 LIMO Pro Mobile Base

**Type:** 4-wheel differential drive robot platform  
**Manufacturer:** AgileX Robotics  
**Key Specs:**
- Dimensions: ~322 × 280 × 130 mm
- Weight: ~8 kg
- Max Speed: 1.5 m/s
- Payload: ~5 kg
- Drive Modes: Ackermann, Differential, Mecanum, Tracked (varies by model)
- Power: 24V lithium battery, ~2–4 hrs runtime
- Sensors: Integrated IMU, wheel encoders
- ROS2 Support: Fully supported via `limo_bringup` package

**Datasheet:** [AgileX LIMO Pro Datasheet](https://www.agilex.ai/)

---

## 🤖 MyCobot 280 (Wi-Fi Controlled)

**Type:** 6-DOF collaborative robotic arm  
**Manufacturer:** Elephant Robotics  
**Key Specs:**
- Payload: 250 g
- Reach: 280 mm
- Repeatability: ±0.5 mm
- Communication: Wi-Fi (converted from USB in this project)
- Control: `pymycobot` Python API
- Power: 12V DC
- End Effector: 2-finger parallel gripper (electric)

**Special Modifications in This Project:**
- USB board failure replaced with Wi-Fi control board
- Auto-IP detection in `full_system.launch.py`
- Voice feedback during pick/drop sequences

**Datasheet:** [MyCobot 280 Datasheet](https://www.elephantrobotics.com/)

---

## 🎥 Orbbec DaBai Depth Camera

**Type:** RGB-D camera  
**Key Specs:**
- RGB Resolution: 1920 × 1080 @ 30 FPS
- Depth Resolution: 640 × 480 @ 30 FPS
- Depth Range: 0.15–4.0 m
- FOV: ~87° × 58° × 95°
- Interface: USB 3.0
- ROS2 Driver: `orbbec_camera`

**Usage in This Project:**
- RGB feed for YOLO object detection
- Depth feed for 3D position estimation
- Static TF to `base_link` published in `full_system.launch.py`

---

## 💻 Onboard Computer

**Suggested Config for Stable Performance:**
- CPU: Quad-core ARM or x86 (e.g., Intel i5 / Jetson Orin Nano)
- RAM: 8 GB minimum (16 GB recommended)
- Storage: 64+ GB microSD or SSD
- OS: Ubuntu 20.04 LTS
- ROS2: Foxy Fitzroy

---

## 🧩 Compatibility Information

- **ROS2 Version:** Foxy Fitzroy (tested & stable)
- **Python Version:** 3.8+
- **GPU Acceleration:** Optional but recommended for YOLO inference
- **Networking:** Requires stable 2.4 GHz Wi-Fi for MyCobot connectivity
- **Nav2 Stack:** Compatible with maps generated from Cartographer or RTAB-Map

<Admonition type="tip" title="Tip">
Keep spare cables, a second microSD card with a pre-flashed OS, and an external power bank for field testing.
</Admonition>
