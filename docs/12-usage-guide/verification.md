---
sidebar_position: 11
title: Verification
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

# ✅ Verification

This section ensures that your **LIMO Autonomous System** is healthy, all components are working correctly, and performance matches expectations before you start a mission.

---

## 🩺 System Health Checks

Run these **before** launching the full system:

| Check | Command | Expected Result |
|-------|---------|-----------------|
| ROS 2 Environment | `printenv \| grep ROS` | Correct distro (`foxy`) and workspace sourced |
| LIMO Power | *Physical* | Battery ≥ 50%, LEDs normal |
| MyCobot Arm | `ping 192.168.137.75` *(or auto-detected IP)* | Response < 10 ms |
| Camera | `ros2 topic list` | `/camera/color/image_raw` and `/camera/depth/image_raw` available |
| Nav2 Server | `ros2 action list` | `/navigate_to_pose` listed |

<Admonition type="tip" title="Fast Health Check">
Run:
```bash
ros2 topic list | grep camera && ros2 action list | grep navigate
```
If you see both camera topics and navigation action, you're good to go.
</Admonition>

---

## 🔍 Component Verification

<Tabs>
<TabItem value="base" label="LIMO Base">
```bash
ros2 topic echo /odom
```
Should show changing **x, y, theta** when moving the robot manually.
</TabItem>

<TabItem value="camera" label="Orbbec Camera">
```bash
ros2 run rqt_image_view rqt_image_view
```
Check that RGB and depth streams are visible and depth values make sense.
</TabItem>

<TabItem value="yolo" label="YOLO Object Detector">
```bash
ros2 run object_detector yolo_detector
```
Expected log:
```
✅ YOLO detector node initialized with synchronized inputs.
[📍 DETECTED] target_object 2D(320,240) 3D(0.42, 0.15, 0.80)
```
</TabItem>

<TabItem value="nav2" label="Nav2 Navigation">
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: { header: {frame_id: map}, pose: { position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0} }}}"
```
Robot should start moving to the target.
</TabItem>

<TabItem value="arm" label="MyCobot Arm">
```bash
ros2 run mycobot_arm pick_node
```
Arm should move to **center → pickup → grab → center** sequence.
</TabItem>
</Tabs>

---

## 📜 Log Monitoring

Monitor logs in real time while launching:
```bash
ros2 launch nav_handler full_system.launch.py | tee startup.log
```

Look for these key lines:
- **Pose Setter:** `✅ Initial pose published at (...)`
- **Mission Manager:** `🟢 Exploration node ready.`
- **YOLO:** `Object detected.`
- **Nav2:** `Begin navigating from current location...`
- **Pick/Drop:** `✅ Pick sequence complete!` / `✅ Drop sequence complete!`

<Admonition type="danger" title="If You See Errors">
If you encounter `❌ Nav2 server not ready!` or `YOLO detector not receiving camera frames`, stop and re-check startup order.
</Admonition>

---

## 📈 Performance Validation

Run a short mission test:

1. Launch system (`full_system.launch.py`).
2. Let the robot navigate to at least 1 waypoint.
3. Place a detectable object in view of the camera.
4. Verify:
   - Exploration pauses on detection.
   - Robot navigates directly to object.
   - Pick sequence runs successfully.
   - Robot returns to base and drops the object.

**Success Criteria:**
- Navigation smoothness (no abrupt stops unless object detected).
- Pick success rate ≥ 80% in controlled conditions.
- Drop zone accuracy within ±5 cm.

---

## 🧠 Suggested Automation

You can automate verification using:
```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /odom
```
This ensures camera publishes at ≥ 15 FPS and odometry updates ≥ 10 Hz.

---

## 🎯 Next Steps

- [Mission Manager Node](../05-system-components/mission-manager.md)
- [System Components Overview](../05-system-components/overview.md)
- [Troubleshooting Guide](../07-troubleshooting/common-errors.md)
