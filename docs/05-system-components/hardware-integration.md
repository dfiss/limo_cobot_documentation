---
sidebar_position: 13
title: Hardware Integration
---

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# 🛠 Hardware Integration

This section explains how to connect, configure, and calibrate all physical components of your LIMO + MyCobot system for seamless operation with ROS2.

---

## 📷 Camera Integration

The **Orbbec DaBai depth camera** provides both RGB and depth data for object detection and 3D localization.

**Connection:**
- Mount securely to the LIMO base (avoid vibration).
- Connect via USB 3.0 to the robot computer.

**Bringup:**
```bash
ros2 launch orbbec_camera dabai.launch.py
```

**Verify:**
```bash
ros2 topic list | grep camera
```

You should see:
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`

<Admonition type="tip" title="Pro Tip">
Keep the lens clean and avoid direct sunlight for consistent depth readings.
</Admonition>

---

## 🚗 Robot Platform Setup

The **LIMO Pro base** provides mobility, odometry, and sensor input (LiDAR, encoders).

**Connection:**
1. Power on using the main switch.
2. Connect via Ethernet/Wi-Fi to the onboard computer.

**Bringup:**
```bash
ros2 launch limo_bringup limo_start.launch.py
```

**Verify movement:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Check that `/cmd_vel` commands result in movement.

---

## 🦾 Arm Configuration

The **MyCobot arm** is used for manipulation tasks (pick & drop).

**Connection:**
1. Mount on the LIMO base in a stable position.
2. Connect via Wi-Fi or USB (depending on model).

**Bringup:**
```bash
ros2 launch mycobot_ros2 mycobot.launch.py
```

**Test gripper:**
```bash
ros2 topic pub /gripper_control std_msgs/String "data: 'open'"
ros2 topic pub /gripper_control std_msgs/String "data: 'close'"
```

<Admonition type="info" title="Tip">
Always start the arm in its **home position** to avoid unexpected collisions.
</Admonition>

---

## 🎯 Sensor Calibration

### **LiDAR**
1. Ensure LiDAR is level.
2. Verify data:
   ```bash
   ros2 topic echo /scan
   ```
3. Check in RViz for alignment.

### **Depth Camera**
1. Adjust exposure and white balance if needed.
2. Test depth data in RViz using DepthCloud plugin.

### **Arm Pose Offsets**
- Use calibration jigs or manual measurement to adjust grasp positions.

---

## 🛠 Hardware Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Camera not detected | Bad USB cable or driver issue | Try different cable/port, reinstall driver |
| Robot not moving | Motor lock, low battery, or wrong mode | Check battery, release motor lock |
| Arm jerks during movement | Wrong joint speed or PID settings | Tune motion parameters |
| Depth readings inaccurate | Reflective/absorptive surfaces in view | Adjust angle or lighting |
| LiDAR scan offset | Sensor mount misaligned | Realign and re-tighten screws |

<Admonition type="warning" title="Caution">
Always power off the robot before connecting or disconnecting major components.
</Admonition>

---

## 📚 Learn More

- [Orbbec Camera ROS2 Driver](https://github.com/orbbec/OrbbecSDK_ROS2)
- [LIMO ROS2 Bringup](https://github.com/agilexrobotics/limo_ros2)
- [MyCobot ROS2 Control](https://github.com/elephantrobotics/mycobot_ros2)
- [ROS2 Sensor Calibration](https://docs.ros.org/en/foxy/Tutorials/TF2/TF2-and-Time.html)

---

## 🎯 Next Steps

- [System Components Overview](./overview.md)
- [Pose Setter Node](./pose-setter.md)
- [Launch Files and System Bringup](./launch-files.md)
