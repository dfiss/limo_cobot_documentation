---
sidebar_position: 3
---

# 📍 Advanced Usage — Waypoint Management

This section explains how to **add, modify, optimize, and dynamically update waypoints** for your LIMO's exploration routes while keeping the detection–pick–drop logic intact.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

---

## 🗂 Current Waypoint Structure

Your `mission_manager.py` stores waypoints as:
```python
WAYPOINTS = [
    (0.37247, -0.10103, -13.26),
    (1.05778, -0.28532, -13.34),
    (1.71948, -0.40092, -11.97),
    (2.63431, -0.54106, 7.45),
    (3.70974, -0.48306, -12.09),
    (3.96764, -0.49506, -44.46)
]
```

Each waypoint is a tuple:
```
(x_position, y_position, yaw_degrees)
```

---

## ➕ Adding New Waypoints

### 1. Find Coordinates in RViz
- Click **2D Nav Goal**.
- Read `/amcl_pose`:

```bash
ros2 topic echo /amcl_pose
```

Note `x`, `y`, and convert quaternion to yaw (in degrees).

### 2. Add to the List

```python
WAYPOINTS.append((new_x, new_y, new_yaw))
```

### 3. Rebuild Workspace

```bash
colcon build --packages-select nav_handler
source install/setup.bash
```

### 4. Test in Simulation or Small Area
Run `full_system.launch.py` and ensure robot visits the new point in order.

<Admonition type="tip" title="Best Practice">
Place new waypoints so the robot can **rotate and scan** without colliding with obstacles.
</Admonition>

---

## ✏️ Modifying Existing Routes

1. Simply change `x`, `y`, `yaw` in the `WAYPOINTS` list.
2. Keep sequential flow logical (avoid sharp backtracking).
3. If you want to skip certain points temporarily:

```python
WAYPOINTS = WAYPOINTS[:3]  # Only first 3 waypoints
```

---

## ⚡ Waypoint Optimization

### Reduce Travel Time:
Order points to minimize path length (use TSP solver if many points).

### Improve Coverage:
Ensure scanning arcs overlap slightly so no blind zones remain.

### Include Rotation Angles:
Choose yaw so the camera faces open space for best object detection.

Example optimized layout:

```python
WAYPOINTS = [
    (-0.78, -0.23, 7.9),
    (-0.49, 0.00, -33.3),
    (-0.22, -0.15, -55.0),
    (0.15, -0.62, -29.2),
    (1.31, -0.65, -9.5),
    (2.20, -0.61, 2.0),
    (3.71, -0.53, -3.5)
]
```

---

## 🔄 Dynamic Waypoint Updates

You can update waypoints on the fly without restarting the system:

<Tabs>
<TabItem value="service" label="1. Using a ROS 2 Service">

- Create a service `/update_waypoints` that updates the `WAYPOINTS` list in `mission_manager`.
- Call service from terminal or another node.

</TabItem>
<TabItem value="topic" label="2. Using a Topic">

- Subscribe to `/waypoint_update` in `mission_manager.py`.
- Publish a list of waypoints during runtime to override current path.

</TabItem>
</Tabs>

<Admonition type="info" title="Important">
Dynamic updates will only take effect when the robot is in **IDLE** or after completing its current navigation goal. Object detection interruptions still override waypoints immediately.
</Admonition>

---

## ✅ Success Indicators

- Robot visits all intended waypoints in order.
- No collision warnings in Nav2 logs.
- Scanning occurs at each waypoint.
- Detection interruptions still trigger correct pick–drop sequence.

---

**Next: ⚙️ Hardware Upgrades & Integrations**
