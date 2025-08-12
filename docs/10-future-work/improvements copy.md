---
sidebar_position: 18
---

# 🛠️ Customization & Upgrades

The **LIMO Pro + MyCobot + YOLO** platform is not just plug-and-play — it’s designed to be **modified, extended, and upgraded**.  
This page is your **hands-on guide** for changing how the robot works, adding new capabilities, and pushing it beyond its default setup.

import Admonition from '@theme/Admonition';

---

## 🗺 Mapping & Exploration

### 1. Switch from Cartographer to RTAB-Map
- **Why:** RTAB-Map enables **real-time autonomous exploration** without pre-building maps.
- **How:**  
  1. Install `rtabmap_ros`:  
     ```bash
     sudo apt install ros-foxy-rtabmap-ros
     ```
  2. Replace Cartographer launch in `full_system.launch.py` with `rtabmap.launch.py`.
  3. Use `rtabmap_explore` to roam unmapped spaces automatically.
- **Hint:** RTAB-Map is heavier on resources — keep other processes light for smoother performance.

### 2. Tune SLAM Parameters
- **Why:** Better loop closure, less drift, higher mapping accuracy.
- **How:**  
  Edit parameters in `cartographer.lua` or `rtabmap_params.yaml`:
  - Increase loop closure frequency
  - Adjust voxel size for your space
- **Hint:** Test changes in a small area before applying to full missions.

---

## 🎯 Object Detection & Recognition

### 3. Improve YOLO Accuracy
- **Why:** Current model trained with only ~150 images; accuracy can reach 99%+ with a richer dataset.
- **How:**  
  - Use **Roboflow** to expand dataset & annotate more images.
  - Fine-tune via a custom `train_yolo.py` script in ROS2.
  - Replace the `.pt` file in `object_detector.py`.
- **Hint:** Capture images in the same lighting and environment the robot will operate in.

### 4. Add More Object Classes
- **Why:** Allow robot to identify multiple object types and act differently for each.
- **How:**  
  - Add new labeled classes to the YOLO dataset.
  - Retrain, then modify `object_detector.py` to filter `cls_name`.
- **Hint:** Use a class-to-action mapping table in Python to control the arm’s behavior per object.

### 5. Enable Configurable Classes at Runtime
- **Why:** Change detection targets without retraining.
- **How:**  
  Pass `--classes` to YOLO predict or use a ROS2 parameter for active detection classes.
- **Hint:** Add a dynamic parameter server node so you can change classes mid-mission.

---

## 🧠 Memory & AI Enhancements

### 6. Add Vector Memory for User Recognition
- **Why:** Robot learns the user’s face and returns thrown objects only to them.
- **How:**  
  - Integrate `face_recognition` or `dlib` with a local embedding database.
  - Link recognition results to mission logic in `mission_manager.py`.
- **Hint:** Store embeddings in JSON/SQLite for quick retrieval; keep images under consistent lighting.

### 7. AI Path Planning
- **Why:** Optimize pickup order when multiple objects are detected.
- **How:**  
  - Use A* or Dijkstra with dynamically updated object positions.
  - Or integrate `stable-baselines3` RL agent to learn optimal routes.
- **Hint:** Keep a fallback static waypoint list in case AI planner fails.

---

## 🤝 Multi-Robot Coordination

### 8. Collaborative Missions
- **Why:** Multiple LIMOs share the workload, mapping, and object collection.
- **How:**  
  - Share maps via `map_server` and DDS multicast.
  - Create a master mission manager node to divide waypoints.
- **Hint:** Assign each robot a namespace (`/robot1`, `/robot2`) to avoid topic collisions.

---

## 🖥 UI & User Interaction

### 9. Browser-Based Dashboard
- **Why:** Control and monitor the robot remotely.
- **How:**  
  - Install `rosbridge_server` + `web_video_server`.
  - Build a web app with `roslibjs` or integrate Foxglove Studio.
- **Hint:** Expose only safe commands in remote mode.

### 10. Voice-Enabled Conversation
- **Why:** Control robot hands-free and add interactive personality.
- **How:**  
  - Use `vosk` or `whisper.cpp` for offline speech-to-text.
  - Map recognized commands to ROS2 actions.
- **Hint:** Add audio feedback so the user knows the robot understood the command.

---

## ⚡ Performance Optimizations

### 11. Nav2 & Motion Tuning
- **Why:** Smoother navigation, fewer localization errors.
- **How:**  
  Adjust velocity/acceleration in `nav2_params.yaml`:
  - Lower `max_vel_theta` for precise turning.
  - Fine-tune `planner_frequency` for better responsiveness.
- **Hint:** Record bag files before/after tuning to measure improvements.

### 12. Offload Heavy Processing
- **Why:** Reduce CPU load for smoother multi-node operation.
- **How:**  
  - Run YOLO on an edge GPU (e.g., Jetson).
  - Use nodelets or multi-threaded executors.
- **Hint:** Cache TF transforms in `object_detector.py` to cut processing time.

---

<Admonition type="success" title="Pro Tip">
Upgrades don’t have to be huge — even small tweaks (like better SLAM tuning) can boost mission success rates dramatically.
</Admonition>
