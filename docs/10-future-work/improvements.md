---
sidebar_position: 18
---

# 🧩 Upgrade Implementation Guide

This manual turns the broad upgrade ideas into **concrete, reproducible steps** for the **LIMO Pro + MyCobot + YOLO** platform.  
Each section follows a **prereqs → install → configure → test → rollback** pattern so you can iterate safely.

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Link from '@docusaurus/Link';

---

<Admonition type="warning" title="Safety First">
Test upgrades on a **copy of your workspace** and keep your robot **on a stand** (wheels lifted) when validating motion changes.  
Always back up your **maps** and **params** before editing files.
</Admonition>

<Admonition type="tip" title="Recommended Workspace Layout">
Use a clean overlay workspace (example: `~/krish_ws`):

```
krish_ws/
├── src/
│   ├── nav_handler/          # mission_manager, pose_setter, etc.
│   ├── object_detector/      # YOLO detector node(s)
│   ├── mycobot_arm/         # pick_node, drop_node
│   ├── full_system/         # launch files, params, utils
│   └── rtabmap_configs/     # rtabmap/cartographer configs
├── maps/
├── params/
└── scripts/
```
</Admonition>

---

## 1) Advanced Mapping & Exploration (RTAB‑Map)

<Tabs groupId="rtab" defaultValue="install">
<TabItem value="install" label="Install & Prereqs">

**Goal:** Replace Cartographer with **RTAB‑Map** to enable **live mapping + autonomous exploration** (no prebuilt map).

**Prereqs**
- Camera topics available:  
  - `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/depth/image_raw`
- Static TF from `base_link` → `camera_link` (already in your bringup)

**Install**
```bash
sudo apt update
sudo apt install ros-foxy-rtabmap-ros
```

</TabItem>

<TabItem value="launch" label="Create RTAB‑Map Launch">

Create: `full_system/launch/rtabmap_explore.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('rtabmap_ros'), 'launch', 'rtabmap.launch.py'])
        ]),
        launch_arguments={
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'frame_id': 'camera_link',
            'approx_sync': 'true',   # more tolerant sync
        }.items()
    )

    explorer = Node(
        package='rtabmap_ros',
        executable='rtabmap-explore',
        name='rtabmap_explore',
        output='screen',
        parameters=[{'use_action_for_goal': True}],
        remappings=[
            ('/map', '/rtabmap/map'),
            ('/move_base_simple/goal', '/goal_pose'),
        ]
    )

    return LaunchDescription([
        rtabmap,
        TimerAction(period=5.0, actions=[explorer]),
    ])
```

Optional config: `rtabmap_configs/rtabmap_params.yaml` (loop closure, voxel size, memory).

</TabItem>

<TabItem value="wire" label="Wire Into System">

**Option A:** Swap Cartographer in `full_system.launch.py` with the RTAB‑Map include.

**Option B:** Provide a second bringup for exploration days:
```bash
ros2 launch full_system rtabmap_explore.launch.py
```

Keep your static TF publisher (e.g., `base_link` → `camera_link`) as in your current launch.

</TabItem>

<TabItem value="test" label="Test & Rollback">

**Test**
1. Bring up base, camera, and Nav2.
2. Launch `rtabmap_explore.launch.py`.
3. In RViz, confirm `/rtabmap/map` grows as the robot moves; check `/rtabmap/info` for loop closures.

**Rollback**
1. Stop RTAB‑Map and explorer.
2. Relaunch Cartographer mapping or your original stack.

</TabItem>
</Tabs>

---

## 2) YOLO: Accuracy, Multi‑Class, GPU Offload

<Tabs groupId="yolo" defaultValue="dataset">
<TabItem value="dataset" label="Dataset & Training Workflow">

**Goal:** Increase robustness to 99%+ via more data and better coverage.

**Data**
- Capture in your real environment (lighting, occlusion, distance).
- Balance classes (target ≥ 1k images/class for strong generalization).
- Maintain train/val/test splits (e.g., 80/10/10).

**Training Options**
- Roboflow Train or local Ultralytics runs (preferred for reproducibility).

**Export**
- Produce a single `.pt` model for deployment into `object_detector.py`.

</TabItem>

<TabItem value="script" label="Standardized Trainer Script">

Create: `scripts/train_yolo.py`

```python
#!/usr/bin/env python3
from ultralytics import YOLO
import argparse

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--data', required=True, help='data.yaml path')
    p.add_argument('--model', default='yolov8n.pt', help='base model')
    p.add_argument('--img', type=int, default=640)
    p.add_argument('--epochs', type=int, default=100)
    p.add_argument('--project', default='runs/detect')
    p.add_argument('--name', default='custom')
    p.add_argument('--device', default='0')   # '0' for GPU, 'cpu' for CPU
    args = p.parse_args()

    model = YOLO(args.model)
    model.train(data=args.data, imgsz=args.img, epochs=args.epochs,
                project=args.project, name=args.name, device=args.device)

if __name__ == "__main__":
    main()
```

Run on a GPU box for speed; copy the resulting weights back to the robot.

</TabItem>

<TabItem value="filter" label="Runtime Class Filtering (ROS Param)">

Add dynamic class filtering in `object_detector.py`:

```python
# __init__
self.declare_parameter('allowed_classes', [])  # example: ["bottle","cup"]
self.allowed_classes = set(
    self.get_parameter('allowed_classes').get_parameter_value().string_array_value
)

# after getting cls_name
if self.allowed_classes and cls_name not in self.allowed_classes:
    return

# Optional: per-class action map (pick/skip/custom)
self.declare_parameter('class_actions', ['bottle:pick', 'cup:skip'])
pairs = self.get_parameter('class_actions').get_parameter_value().string_array_value
self.class_action = dict(x.split(':', 1) for x in pairs)
action = self.class_action.get(cls_name, 'pick')
# attach action to a side topic or encode in header for mission_manager to read
```

Run:
```bash
ros2 run object_detector yolo_detector --ros-args \
  -p allowed_classes:="['bottle','cup']" \
  -p class_actions:="['bottle:pick','cup:skip']"
```

</TabItem>

<TabItem value="gpu" label="GPU Offload & Throughput">

**GPU Setup**
- Use CUDA-enabled PyTorch + Ultralytics.
- Set `device=0` in `train_yolo.py` and detector runtime.
- Target ≥15 FPS for smooth detect→navigate loops.

**Node Optimizations**
- Keep `ApproximateTimeSynchronizer` slop minimal.
- Cache TF transforms for short windows (≤ 250ms) to avoid repetitive lookups.

</TabItem>

<TabItem value="test" label="Validation & Rollback">

**Validate**
- Visualize `/yolo/annotated` for bounding boxes + overlays.
- Confirm `/target_pose` publishes only for allowed classes.
- Capture a quick confusion matrix on your validation set.

**Rollback**
- Restore previous `.pt` weight and remove param overrides.

</TabItem>
</Tabs>

---

## 3) Multi‑Object Queue (Mission Manager)

<Tabs groupId="queue" defaultValue="logic">
<TabItem value="logic" label="Logic & Debounce">

**Goal:** Detect and collect several objects before returning to base.

**Steps**
1. Replace single target state with a queue of `PoseStamped`:
   ```python
   self.targets = deque()
   ```
2. Debounce by time (e.g., ignore duplicate detections for 3s).
3. Deduplicate by distance (e.g., ignore if within 0.4m of last queue item).
4. Control flow:
   - If queue not empty → pause exploration → navigate to `targets[0]`.
   - On arrival → pick → pop-left → continue.
   - When empty → return to base → drop.

**Notes**
- Keep your existing interrupt guard so exploration doesn't collide with object navigation.

</TabItem>

<TabItem value="snippet" label="Code Snippet (Concept)">

```python
from collections import deque

self.targets = deque()
self.TARGET_MIN_DIST = 0.4     # meters
self.TARGET_COOLDOWN = 3.0     # seconds
self._last_target_stamp = 0.0

def object_detected_callback(self, msg):
    now = self.get_clock().now().nanoseconds / 1e9
    if now - self._last_target_stamp < self.TARGET_COOLDOWN:
        return
    self._last_target_stamp = now

    if self.targets:
        last = self.targets[-1].pose.position
        dx = msg.pose.position.x - last.x
        dy = msg.pose.position.y - last.y
        if (dx*dx + dy*dy) ** 0.5 < self.TARGET_MIN_DIST:
            return

    self.targets.append(msg)
    self.get_logger().info(f"➕ Queued target #{len(self.targets)}")
    if self.state in [State.IDLE, State.NAVIGATING, State.SCANNING]:
        self.pause_exploration_and_go_next()

def pause_exploration_and_go_next(self):
    self.cancel_current_nav_goal()
    self.stop_scanning()
    if self.targets:
        self.navigate_to_pose_msg(self.targets[0])
        self.state = State.NAVIGATING_TO_OBJECT
```

</TabItem>

<TabItem value="test" label="Test & Rollback">

**Test**
- Place 2–3 objects in different areas.
- Verify the robot queues all, picks sequentially, then returns to base once.

**Rollback**
- Ignore new detections after first target (restore single-target behavior).

</TabItem>
</Tabs>

---

## 4) Vector Memory (User Recognition)

<Tabs groupId="face" defaultValue="enroll">
<TabItem value="enroll" label="Enroll Users">

**Goal:** Deliver objects to the correct person.

**Enroll Script**
- Create `scripts/enroll_user.py` to capture 10–20 face images and compute an average embedding for each user.
- Store as `{ "name": [embedding floats...] }` in JSON or as rows in SQLite.

**Tips**
- Capture under typical room lighting.
- Keep faces frontal and slightly off-axis samples.

</TabItem>

<TabItem value="recognize" label="Recognition Node">

**Node Behavior**
- Subscribes to `/camera/color/image_raw`.
- Extracts face embedding (e.g., `face_recognition` / `dlib`).
- Finds nearest embedding below a threshold → publishes `/current_user` (String).
- Optional: publish `/current_user_pose` if you add a person-tracking node.

**Mission Hook**
- After pick, wait up to N seconds to observe `/current_user`.
- If matched: navigate near user's last seen location and drop.
- If not found: fallback to base drop.

</TabItem>

<TabItem value="test" label="Test & Rollback">

**Test**
- Enroll yourself, stand in view, confirm `/current_user` matches your name.
- Trigger pick → confirm delivery attempts to your position.

**Rollback**
- Disable recognition subscriber; revert to base drop only.

</TabItem>
</Tabs>

---

## 5) Voice Commands (Offline)

<Tabs groupId="voice" defaultValue="stt">
<TabItem value="stt" label="Speech‑to‑Text Node">

**Goal:** Hands‑free commands like "start mission", "pause", "return to base".

**Approach**
- Use `vosk` (light, offline) or `whisper.cpp` (more accurate, heavier).
- Publish intents to `/voice_cmd` (String or custom msg).

**Example Grammar**
- `start` → begin exploration
- `pause` → cancel current nav goal
- `return` → return to base and drop

</TabItem>

<TabItem value="mission" label="Integrate with Mission Manager">

- Subscribe to `/voice_cmd` in `mission_manager`.
- Map recognized intents to state transitions.
- Provide audible feedback using your `espeak` helper so the user knows it worked.

</TabItem>

<TabItem value="test" label="Test & Rollback">

**Test**
- Speak commands and verify state changes in logs and RViz (markers/hud).

**Rollback**
- Disable `/voice_cmd` subscriber; keep TTS only.

</TabItem>
</Tabs>

---

## 6) Web Dashboard (Browser Control)

<Tabs groupId="webui" defaultValue="stack">
<TabItem value="stack" label="Stack & Topics">

**Stack**
- `rosbridge_server` (WebSocket bridge)
- `web_video_server` (camera stream)
- Optional: Foxglove Studio for a professional dashboard

**Controls & Feeds**
- Buttons → publish to `/ui_cmd` (start, pause, return).
- Live map → subscribe `/map`.
- Camera → MJPEG stream.
- Object queue → subscribe your queue topic or reuse `/target_pose`.

</TabItem>

<TabItem value="html" label="Minimal HTML Concept">

Serve a basic page that:
- connects to ROS via `roslibjs`,
- shows action buttons,
- embeds MJPEG stream for camera,
- lists queued targets.

</TabItem>

<TabItem value="test" label="Test & Rollback">

**Test**
- Access from a LAN device; confirm you can start/pause/return and see live camera/map.

**Rollback**
- Stop `rosbridge_server` & `web_video_server`; remove page.

</TabItem>
</Tabs>

---

## 7) Performance & Reliability

<Tabs groupId="perf" defaultValue="nav2">
<TabItem value="nav2" label="Nav2 Tuning Checklist">

- Reduce `max_vel_theta` for precise turns in tight labs.
- Increase `planner_frequency` for responsiveness.
- Tune controller plugin (DWB or RPP) for smoother trajectories.
- Validate AMCL params (laser noise, update rates).
- Track changes in `params/nav2_params.yaml` for quick rollbacks.

</TabItem>

<TabItem value="tf" label="TF/Math Optimizations">

**Cache TF Lookups in `object_detector.py`:**

```python
# pseudo-cache
if (src_frame, 'map') in self._tf_cache and time.time() - self._tf_cache_time < 0.25:
    tf = self._tf_cache[(src_frame, 'map')]
else:
    tf = self.tf_buffer.lookup_transform('map', src_frame, rclpy.time.Time())
    self._tf_cache[(src_frame, 'map')] = tf
    self._tf_cache_time = time.time()
```

Keep depth sampling to a small ROI around bbox center; avoid per-pixel heavy math.

</TabItem>

<TabItem value="proc" label="Process Layout & Logging">

- Run the detector in a separate process / executor.
- Use multi-threaded executors for SLAM + Nav2.
- Limit log spam in tight loops; use throttled logging to reduce overhead.

</TabItem>

<TabItem value="bench" label="Benchmarks & Rollback">

**Benchmarks**
- Detector FPS and latency from `/target_pose` → nav goal acceptance.
- Bag runs before/after each tuning round; graph with `rqt_plot`.

**Rollback**
- Keep `params/` snapshots; use `git checkout` to revert quickly.

</TabItem>
</Tabs>

---

## 8) System Health & Diagnostics (Recommended)

Add a lightweight health node that publishes to `/diagnostics`:
- CPU, RAM, temperature, battery (if available), MyCobot connection status.
- Mirror key states in RViz (MarkerArray) and the Web UI.
- Speak alerts via `espeak` when thresholds exceed (e.g., "CPU 85 percent").

---

## 9) Test Plans (Per Upgrade)

**RTAB‑Map**
- Map grows live; restart → confirm persistence (if saving DB is enabled).

**YOLO**
- Validate with a confusion matrix; field-test: 20 detections with 0–1 false positive.

**Multi‑Object Queue**
- Place 3 targets; verify sequential pickup without intermediate base returns.

**User Recognition**
- 3 enrolled users; match rate ≥ 95% within 2 m in typical lighting.

**Voice/Web UI**
- Misrecognition rate < 5% after grammar tweaks; remote controls gated by a "safe mode" switch.

---

## 10) Rollback & Recovery

- Use feature branches: `feature/rtabmap`, `feature/multiqueue`, etc.
- Version all `params/` and `full_system/launch/` files.
- Keep a stable bringup (`full_mission_stable.launch.py`) to recover instantly.

<Admonition type="success" title="Design Once, Extend Forever">
Keep new behavior **param‑driven** and **node‑decoupled**. When you add multi‑robot, cloud model updates, or docking later, you won't need rewrites.
</Admonition>