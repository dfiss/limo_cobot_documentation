---
sidebar_position: 6
title: First Run
---

import Admonition from '@theme/Admonition';

# 🟢 First Run

Everything installed? Great — let's make sure your **LIMO Pro + MyCobot + YOLO** system is alive, communicating, and ready for its first mission.

---

## 🚀 Launch the Full System

From your workspace root (e.g., `~/krish_ws/`):

```bash
source install/setup.bash
ros2 launch full_system full_system.launch.py
```

---

## 🔄 What Happens During Startup

The `full_system.launch.py` file uses `TimerAction` sequencing to bring up subsystems in the correct order:

### 1. **LIMO Base Bringup** (`limo_start.launch.py`)
- Initializes motors, IMU, and base communication
- **Expected log:**
```
[limo_bringup] [INFO] LIMO base initialized and ready.
```

### 2. **Orbbec Camera Node** (`dabai.launch.py`)
- Starts RGB-D streaming on `/camera/color/image_raw` and `/camera/depth/image_raw`
- **Expected log:**
```
[orbbec_camera] [INFO] Camera connected. Streaming at 1920x1080 RGB, 640x480 Depth.
```

### 3. **YOLO Object Detector** (`yolo_detector`)
- Loads your trained weights
- Publishes annotated images to `/yolo/annotated` and target poses to `/target_pose`
- **Expected log:**
```
[yolo_nav_detector] [INFO] ✅ YOLO detector node initialized with synchronized inputs.
```

### 4. **Nav2 Navigation Stack** (`limo_nav2.launch.py`)
- Loads map (`map11.yaml`) and starts planners/controllers
- **Expected log:**
```
[nav2_controller] [INFO] Nav2 bringup complete. Ready for navigation goals.
```

### 5. **Pose Setter** (`pose_setter.py`)
- Automatically publishes the robot's initial AMCL pose
- **Expected log:**
```
[pose_setter] [INFO] ✅ Initial pose published at (-0.78, -0.23, 0.14 rad)
```

### 6. **Mission Manager** (`mission_manager.py`)
- Begins waypoint exploration and waits for YOLO interrupts
- **Expected log:**
```
[mission_manager] [INFO] 🟢 Exploration node ready.
[mission_manager] [INFO] 🚀 Navigating to: (0.37, -0.10, -13.3°)
```

### 7. **RQT Image View** (optional)
- GUI for visualizing camera feeds

---

## ✅ How to Verify Everything is Working

### 1. Check ROS2 Nodes
```bash
ros2 node list
```
You should see:
```
/yolo_nav_detector
/mission_manager
/pose_setter
/camera_node
/amcl
/nav2_controller
/nav2_planner
/bt_navigator
/waypoint_follower
/lifecycle_manager
```

### 2. Check Topic List
```bash
ros2 topic list
```
Expected key topics:
```
/camera/color/image_raw
/camera/depth/image_raw
/yolo/annotated
/target_pose
/cmd_vel
/odom
/scan
/map
/amcl_pose
/initialpose
```

### 3. View Camera Feed
```bash
rqt_image_view
```
- Select `/camera/color/image_raw` for raw camera feed
- Select `/yolo/annotated` to see bounding boxes and labels

### 4. Monitor System Status
```bash
ros2 topic echo /diagnostics
```

---

## 🧠 What to Expect on the First Run

### Phase 1: System Initialization (0-30 seconds)
- All nodes start up sequentially
- Camera begins streaming
- YOLO loads model weights
- Nav2 stack initializes with map
- Initial pose is set automatically

### Phase 2: Exploration Mode (30+ seconds)
- **Robot will start moving** along predefined waypoints
- At each waypoint, it may **rotate in place** to scan for objects
- YOLO continuously processes camera frames
- Navigation logs show goal acceptance and progress

### Phase 3: Object Detection & Mission Execution
If YOLO detects a target object, Mission Manager will:
1. **Interrupt exploration**
2. **Navigate to object location**
3. **Run pick sequence** (MyCobot arm extension)
4. **Return to base/drop zone**
5. **Run drop sequence** (MyCobot arm retraction)
6. **Resume exploration**

---

## 📋 Expected Log Messages

### Successful Detection:
```
[📍 DETECTED] cup 2D(312,245) 3D(0.45,0.12,0.89)
[mission_manager] [INFO] 🎯 Target detected! Interrupting exploration.
```

### Navigation Success:
```
[mission_manager] [INFO] ✅ Goal accepted.
[mission_manager] [INFO] 🏁 Goal reached.
```

### Pick/Drop Actions:
```
[pick_node] [INFO] 🧲 Grabbed the object!
[drop_node] [INFO] 🪣 Dropped the object!
```

### Voice Feedback:
- System uses `espeak` for audio confirmation
- You'll hear spoken updates like "Target detected", "Moving to object", "Mission complete"

---

## 🛠️ Troubleshooting First Run Issues

### Camera Not Starting
```bash
# Check if camera is detected
lsusb | grep -i orbbec
# Restart camera node specifically
ros2 launch orbbec_camera dabai.launch.py
```

### Navigation Not Working
```bash
# Check if map is loaded
ros2 topic echo /map -n 1
# Verify initial pose is set
ros2 topic echo /amcl_pose -n 1
```

### YOLO Not Detecting Objects
```bash
# Check if YOLO node is running
ros2 node info /yolo_nav_detector
# View raw camera feed to verify objects are visible
rqt_image_view
```

### MyCobot Not Responding
```bash
# Check USB connection
ls /dev/ttyUSB* /dev/ttyACM*
# Test arm connection
ros2 run mycobot_280 test_connection.py
```

---

## 🔍 Advanced Monitoring

### Real-time Performance Monitoring
```bash
# Monitor system resources
htop
# Check ROS2 performance
ros2 run rqt_graph rqt_graph
```

### Detailed Logging
```bash
# Run with debug logging
ros2 launch full_system full_system.launch.py --ros-args --log-level debug
```

### Save Logs for Analysis
```bash
# Create log directory
mkdir -p ~/logs/$(date +%Y%m%d_%H%M%S)
# Launch with logging
ros2 launch full_system full_system.launch.py 2>&1 | tee ~/logs/$(date +%Y%m%d_%H%M%S)/full_system.log
```

---

## 📊 System Health Checks

### Before Each Run
- [ ] Battery levels (LIMO Pro > 20%, MyCobot > 30%)
- [ ] Camera lens clean and unobstructed
- [ ] Area clear of obstacles
- [ ] Map file accessible and up-to-date
- [ ] YOLO weights file present and valid

### During Operation
- [ ] All nodes remain active (`ros2 node list`)
- [ ] No error messages in logs
- [ ] Camera feed shows clear images
- [ ] Robot responds to navigation commands
- [ ] Object detection working (test with known objects)

---

<Admonition type="tip" title="Pro Tip">
Create a launch alias for easier startup:

```bash
echo "alias start_robot='cd ~/krish_ws && source install/setup.bash && ros2 launch full_system full_system.launch.py'" >> ~/.bashrc
source ~/.bashrc
```

Now you can start everything with just `start_robot`!
</Admonition>

<Admonition type="warning" title="Safety First">
- Always have the emergency stop button accessible
- Ensure adequate space (minimum 3x3 meters) for robot operation
- Keep the area clear of people and pets during autonomous operation
- Monitor the robot closely during first runs
</Admonition>

<Admonition type="info" title="Next Steps">
Once your first run is successful, check out:
- [Mission Manager Configuration](./mission_manager.md) - Customize waypoints and behavior
- [Object Detector Tuning](./object_detector.md) - Improve detection accuracy
- [Advanced Navigation](./advanced_navigation.md) - Fine-tune path planning
</Admonition>

---

## 🎯 Success Criteria

Your first run is successful when:
- ✅ All nodes start without errors
- ✅ Camera feed displays in rqt_image_view
- ✅ Robot moves to first waypoint
- ✅ YOLO detections appear in `/yolo/annotated` topic
- ✅ Voice feedback confirms system status
- ✅ Robot completes at least one exploration cycle

Ready to run your first autonomous mission? Let's go! 🚀