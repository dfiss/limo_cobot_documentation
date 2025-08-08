---
sidebar_position: 11
title: Pose Setter Node
---

import Admonition from '@theme/Admonition';

# 🎯 Pose Setter Node

See the [source code](https://github.com/krish-rRay23/LIMO_COBOT_PROJECT/tree/main/src) for implementation details.

The **Pose Setter Node** ensures that the robot's internal localization system starts from a **known, accurate position** on the map.  
It publishes an **initial pose** to Nav2's AMCL (Adaptive Monte Carlo Localization) or other localization systems.

---

## 🗺 Pose Management System

When Nav2 starts, it needs to know **where the robot is** in the map coordinate frame.  
If the pose is wrong, navigation will fail or produce unsafe paths.

**Pose Setter Node functions:**
1. Publishes `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) with predefined coordinates.
2. Ensures AMCL aligns the robot with the saved map.
3. Can be triggered manually or run automatically at startup.

```mermaid
flowchart LR
    PoseSetter["Pose Setter Node"] --> InitialPose["/initialpose"]
    InitialPose --> AMCL["Localization Node<br/>(AMCL/RTAB-Map)"]
    AMCL --> Nav2["Navigation Stack"]
```

---

## 🔄 Coordinate Transformations

ROS2 uses the **TF2 transform system** to keep track of coordinate frames:
- **map** → global reference frame
- **odom** → short-term odometry frame
- **base_link** → robot's base frame

When you set an initial pose, you are placing `base_link` in the **map** frame.

**Example transform chain after pose set:**
```
map → odom → base_link → camera_link / arm_link
```

---

## 📏 Calibration Procedures

**One-time setup:**
1. Place the robot at your desired "base position" in the real world.
2. In RViz, use the **2D Pose Estimate** tool to align robot on the map.
3. Echo `/amcl_pose`:
   ```bash
   ros2 topic echo /amcl_pose
   ```
4. Copy the position (x, y) and orientation quaternion (or yaw) into your `pose_setter.py` file:
   ```python
   BASE_POSE = (1.9525, -9.8651, 108.25)  # x, y, yaw in degrees
   ```

---

## ▶️ Usage Examples

### **1. Run Pose Setter at Startup**
```bash
ros2 run pose_setter pose_setter_node
```
This will automatically publish the stored base pose.

### **2. Update Base Pose via Parameters**
```bash
ros2 run pose_setter pose_setter_node --ros-args -p base_x:=2.0 -p base_y:=-8.0 -p base_yaw:=90.0
```

### **3. Verify in RViz**
Open RViz and check robot position aligns with the map.

---

## 🛠 Troubleshooting Guide

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Robot starts in wrong location | Incorrect pose values or wrong map frame | Recalibrate in RViz, update BASE_POSE |
| Pose doesn't update in Nav2 | AMCL not running or /initialpose not received | Ensure AMCL node is active |
| Robot drifts after setting pose | Poor localization quality or sensor noise | Improve map quality, check LiDAR alignment |
| Node crashes on start | Invalid parameter types | Ensure numbers are floats, yaw in degrees |

<Admonition type="tip" title="Pro Tip">
Always set the pose **before sending navigation goals** to avoid planning errors.
</Admonition>

---

## 📚 Learn More

- [ROS2 Navigation Initial Pose](https://navigation.ros.org/tutorials/docs/navigation2_with_keepout_filter.html)
- [TF2 Transform Basics](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)

---

## 🎯 Next Steps

- [Navigation & Waypoint Planning](../04-core-concepts/navigation.md)
- [Mission Manager Node](./mission-manager.md)
- [System Integration](../04-core-concepts/system-integration.md)
