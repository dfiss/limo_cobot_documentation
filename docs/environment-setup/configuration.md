---
sidebar_position: 4
title: Configuration
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';
import Details from '@theme/Details';

# ⚙️ Configuration

This section explains how to configure your LIMO Robotics system for smooth operation and robust development.

---

## 🌍 Environment Variables Setup

LIMO's launch system is designed for **auto-detection** and minimal manual configuration.  
**Most variables are handled for you!** But for advanced use, here's what you might need:

| Variable | Purpose                                | Default / Example            | Where Set                |
|----------|----------------------------------------|------------------------------|--------------------------|
| `M5_IP`  | MyCobot Arm WiFi IP address            | `192.168.137.75` (auto-set)  | Launch file/env variable |
| `ROS_DOMAIN_ID` | ROS2 Domain for network isolation | Usually `0`                  | `.bashrc` or terminal    |
| `MAP_FILE` | Map file path for navigation         | `/home/agilex/krish_ws/maps/map11.yaml` | Launch file           |
| `YOLO_WEIGHTS` | Path to YOLO weights             | Set in `object_detector.py`  | Config file/launch       |

<Tabs>
  <TabItem value="How to Set" label="How to Set" default>
  - Temporary (current terminal):  
    ```bash
    export M5_IP=192.168.137.75
    ```
  - Persistent (all terminals):  
    Add to `~/.bashrc` or `~/.bash_profile`
    ```bash
    echo 'export M5_IP=192.168.137.75' >> ~/.bashrc
    source ~/.bashrc
    ```
  </TabItem>
  <TabItem value="Do I Need This?" label="Do I Need This?">
    In most cases, **the launch files set everything for you**!  
    Only set variables manually if you're debugging, developing, or using custom hardware.
  </TabItem>
</Tabs>

---

## 🚀 Launch Files Configuration

- **Main launch:**  
  `full_system.launch.py`  
  - **Auto-detects MyCobot IP on your subnet**
  - Brings up robot, camera, navigation, object detector, and mission manager
  - Uses `TimerAction` to ensure components start in the correct order (critical for real robots!)

```python title="Example: full_system.launch.py"
limo_start = IncludeLaunchDescription(...)
camera_launch = IncludeLaunchDescription(...)
nav2 = ExecuteProcess(...)
pose_setter = Node(...)
yolo_node = Node(...)
mission_manager = Node(..., additional_env={"M5_IP": detected_ip})
```

<Admonition type="info" title="Pro Tip">
Launching with:
```bash
ros2 launch nav_handler full_system.launch.py
```
**automatically configures IPs, starts all nodes, and opens up RQT for visual feedback.**
</Admonition>

---

## 🔌 Hardware Connections

**LIMO Pro robot:** battery charged, safety on

**MyCobot Arm:**
- Connect to WiFi "MyCobotWiFi2.4G", password: `mycobt123`
- Arm's IP is detected by code—no need for static assignment!

**Orbbec DaBai Camera:**
- Plug into USB 3.0 (blue port)
- Detected by `orbbec_camera` launch

**PC/Laptop:**
- Must be on the same network/subnet as robot and arm

<Details summary="Quick Checklist">
- [ ] Robot powered on
- [ ] MyCobot hotspot running and arm on
- [ ] Camera connected
- [ ] PC on same network
- [ ] Use `ros2 launch nav_handler full_system.launch.py` to start everything
</Details>

---

## 🎚️ Parameter Tuning

Many nodes support parameter tuning to optimize performance:

**YOLO detector:**
- Detection confidence (`conf=0.5`)
- Depth thresholds (`MIN_DEPTH_METERS`, `MAX_DEPTH_METERS`)
- Approach distance to object (`APPROACH_DIST`)
- Camera intrinsics (auto-set from camera info)

**Navigation:**
- Map file, costmap configs, waypoint lists
- Initial pose (set by `pose_setter.py`)

<Tabs>
  <TabItem value="How to Tune" label="How to Tune" default>
    Edit parameters in the relevant Python or YAML config file:
    ```python title="object_detector.py"
    MIN_DEPTH_METERS = 0.15
    MAX_DEPTH_METERS = 4.0
    APPROACH_DIST = 0.30
    ```
    Or use ROS2 param system for launch-time tuning.
  </TabItem>
  <TabItem value="Where to Find Parameters" label="Where to Find Parameters">
    - Object detection: `object_detector.py`
    - Arm control: `pick_node.py`, `drop_node.py`
    - Navigation: `nav2_params.yaml`
    - Launch-time: `full_system.launch.py`
  </TabItem>
</Tabs>

---

## 🛑 Common Configuration Pitfalls

- Forgetting to source your workspace (`source install/setup.bash`)
- Not connecting all hardware to the same network
- Hardcoding IP addresses (use auto-detect!)
- Editing vendor launch files instead of copying to your own workspace

<Admonition type="caution" title="Real-World Advice">
**Never hardcode IPs or parameters unless absolutely necessary. The launch system is designed to auto-configure as much as possible—trust it!**
</Admonition>

---

## 🎯 Next Steps

- [Step-by-Step System Usage](../usage-guide/full-workflow)
- [Advanced Parameter Tuning](../advanced-usage/waypoint-management)
- [Troubleshooting & FAQ](../troubleshooting/faq)
