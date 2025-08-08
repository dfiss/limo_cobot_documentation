---
sidebar_position: 2
---

# 🐞 Debugging Guide

This section explains **how to systematically debug** each major subsystem in your LIMO Autonomous System — object detection, navigation, arm control — and which diagnostic tools to use for fast root-cause analysis.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

---

## 🧩 How to Debug Each Subsystem

### 1. Object Detection (`object_detector.py`)
**Goal:** Verify YOLO is publishing `/yolo/annotated`, `/object_found`, and `/target_pose`.

**Checks:**
```bash
# Is the node running?
ros2 node list | grep yolo_nav_detector

# Is camera data arriving?
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw

# Is YOLO producing output?
ros2 topic echo /object_found -n 1
ros2 topic echo /target_pose -n 1
```

**Key Log Lines to Expect:**
```python
✅ YOLO detector node initialized with synchronized inputs.
[📍 DETECTED] <class> 2D(...) 3D(...)
[🌍 POSE] Published /target_pose → map(...)
```

**If It Fails:**
- No detections → check `conf` threshold in code.
- Depth invalid → verify `/camera/depth/image_raw` encoding is `16UC1`.
- TF errors → confirm `static_transform_publisher` is launched before YOLO.

### 2. Navigation (Nav2 + mission_manager.py)
**Goal:** Ensure robot can accept goals and move.

**Checks:**
```bash
# Nav2 actions exist
ros2 action list | grep navigate_to_pose

# Send a manual goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

**Key Log Lines:**
```
🟢 Exploration node ready.
🚀 Navigating to: (x, y, yaw)
✅ Goal accepted.
🏁 Goal reached.
```

**If It Fails:**
- `❌ Nav2 server not ready!` → Increase `TimerAction` delay before Mission Manager starts.
- `❌ Goal rejected.` → Check map frame alignment and initial pose (`pose_setter.py`).

### 3. Arm Control (pick_node.py / drop_node.py)
**Goal:** Confirm MyCobot arm moves through pick/drop sequences.

**Checks:**
```bash
# Directly run pick node
ros2 run mycobot_arm pick_node --ros-args -p m5_ip:=192.168.137.75

# Directly run drop node
ros2 run mycobot_arm drop_node --ros-args -p m5_ip:=192.168.137.75
```

**Key Log Lines:**
```
🤖 Connected to MyCobot over Wi-Fi
🧲 Grabbed the object!
✅ Pick sequence complete!
🪣 Dropped the object!
✅ Drop sequence complete!
```

**If It Fails:**
- Socket errors → check Wi-Fi hotspot SSID/password.
- Motion off → verify joint angles in code match your hardware setup.

---

## 🔍 Diagnostic Tools

| Tool | Purpose | Command |
|------|---------|---------|
| `ros2 topic list` | List all topics | `ros2 topic list` |
| `ros2 topic echo <topic>` | See topic messages | `ros2 topic echo /amcl_pose -n1` |
| `ros2 topic hz <topic>` | Check publish rate | `ros2 topic hz /camera/color/image_raw` |
| `ros2 node info <node>` | Node pubs/subs list | `ros2 node info /yolo_nav_detector` |
| `ros2 action list` | See available actions | `ros2 action list` |
| `tf2_tools` | View TF tree | `ros2 run tf2_tools view_frames.py` |
| `rviz2` | Visual debugging | `rviz2` |

<Admonition type="tip" title="Pro Tip">
When debugging, run **one subsystem at a time** — camera + YOLO, then Nav2 alone, then Mission Manager — before launching `full_system.launch.py`.
</Admonition>

---

## 🛠 Step-by-Step Debug Flow

### 1. Camera First
- Launch only the camera node.
- Verify RGB + depth topics are publishing at good rates.

### 2. Add YOLO
- Run `yolo_detector`.
- Confirm detections and `/target_pose` output.

### 3. Add Nav2
- Start Nav2 with known map.
- Publish initial pose manually or via `pose_setter.py`.
- Send a test goal.

### 4. Add Mission Manager
- Confirm it receives `/target_pose` and interrupts exploration.

### 5. Test Arm
- Run pick/drop independently.
- Check Wi-Fi IP detection from `full_system.launch.py`.

### 6. Full System
- Launch `full_system.launch.py`.
- Watch logs for sequence:
  ```
  YOLO → Target Pose → Nav2 → Pick → Return → Drop
  ```

---

## 🧠 Prevention Strategies During Debugging

- Use **RViz** to confirm robot pose, goals, and detections visually.
- Save mission logs:
  ```bash
  ros2 launch nav_handler full_system.launch.py | tee mission.log
  ```
- Version your configs (`map.yaml`, `nav2_params.yaml`, TF static transforms).
- Avoid overlapping launches — don't start two nodes that publish the same topic.

---

**Next: ♻ Reset & Recovery**
