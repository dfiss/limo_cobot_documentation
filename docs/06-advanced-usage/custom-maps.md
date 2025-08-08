---
sidebar_position: 1
---

# 🗺 Advanced Usage — Custom Maps

This section explains how to **create, customize, and optimize maps** for your LIMO robot using **RTAB-Map** or **Cartographer**, so navigation is more precise and exploration covers your environment efficiently.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

---

## 🧩 Why Custom Maps?

Your robot's **navigation precision** directly depends on the quality of its map.  
Custom maps allow you to:
- Adapt to new environments.
- Remove clutter and unnecessary details.
- Fine-tune navigation for **tight spaces** and **complex layouts**.
- Improve **localization stability** for Nav2.

---

## 🗺 RTAB-Map Customization

<Tabs>
<TabItem value="record" label="1. Record a Map">

```bash
ros2 launch rtabmap_ros rtabmap.launch.py \
   use_sim_time:=false \
   rgb_topic:=/camera/color/image_raw \
   depth_topic:=/camera/depth/image_raw \
   camera_info_topic:=/camera/color/camera_info \
   frame_id:=base_link
```

Drive the robot manually via:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room
```

</TabItem>
<TabItem value="customize" label="2. Customize Map Settings">

- Edit `rtabmap` parameters:
  - **`Grid/CellSize`** → smaller for higher resolution.
  - **`Optimizer/Strategy`** → `1` for TORO, `2` for G2O (better loop closure).
  - **`RGBD/LoopClosureThreshold`** → adjust for more aggressive relocalization.
- Save parameters in:

```bash
~/.ros/rtabmap.ini
```

</TabItem>
<TabItem value="optimize" label="3. Optimize After Mapping">

- Use `rtabmap-databaseViewer` to:
  - Remove ghost points.
  - Merge duplicate nodes.
  - Manually close loops for better accuracy.

```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

</TabItem>
</Tabs>

---

## 🧭 Cartographer Configuration

<Tabs>
<TabItem value="launch" label="1. Launch Cartographer">

```bash
ros2 launch limo_bringup limo_cartographer.launch.py \
   use_sim_time:=false \
   configuration_basename:=limo_cartographer.lua
```

</TabItem>
<TabItem value="edit" label="2. Key Parameters">

Edit your `.lua` config:
- **`TRAJECTORY_BUILDER_2D.min_range` / `max_range`** — adjust for sensor range.
- **`POSE_GRAPH.optimize_every_n_nodes`** — lower for faster optimization.
- **`MAP_RESOLUTION`** — lower value for higher detail.

</TabItem>
<TabItem value="save" label="3. Save Map">

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_cartographer_map
```

</TabItem>
</Tabs>

---

## 🖌 Map Editing & Optimization

You can clean and modify maps for better navigation:

1. **Remove noise & artifacts** using GIMP or Photoshop.
2. **Fill small gaps** in walls to prevent Nav2 from "cutting corners."
3. **Remove furniture** if it has been moved since mapping.
4. **Ensure the robot's starting location** (from pose_setter.py) is in a free space.

Example cleanup workflow:

```bash
sudo apt install gimp
gimp ~/maps/my_room.pgm
```

Edit → Save → Update .yaml metadata file with correct resolution and origin.

---

## 🏗 Creating a Custom Map from Scratch

### 1. Prepare Environment
Remove unnecessary obstacles; ensure good lighting for vision-based SLAM.

### 2. Launch Mapping Tool
Choose RTAB-Map or Cartographer as above.

### 3. Drive Methodically
- Cover every area.
- Rotate at intersections for loop closure.

### 4. Save & Optimize
Clean the .pgm, update .yaml, and store in ~/maps/.

### 5. Update Launch File
Point Nav2 to your new map:

```bash
ros2 launch limo_bringup limo_nav2.launch.py map:=/home/agilex/maps/my_room.yaml
```

<Admonition type="tip" title="Pro Tip">
If your environment changes often, create **multiple maps** for different configurations and switch them depending on the task.
</Admonition>

---

**Next: 📍 Customizing Waypoints**
