---
sidebar_position: 1
---

# 🧯 Common Errors

This page catalogs the **most frequent failures** in the LIMO system, how to recognize them (by logs or symptoms), **step-by-step fixes**, and **how to prevent** them next time.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

---

## 🪵 Most Common Errors — By Log Message

<Tabs>
<TabItem value="nav2-not-ready" label="❌ Nav2 server not ready!">

**Where it appears:** `mission_manager.py`

**Why it happens:** Nav2 is still starting when Mission Manager tries to send a goal.

**Fix (step-by-step):**
1. Stop everything (`Ctrl+C`).
2. Start Nav2 alone and wait for "Lifecycle nodes are active."
   ```bash
   ros2 launch limo_bringup limo_nav2.launch.py map:=/path/to/map.yaml
   ```
3. If stable, re-run full launch. If using `full_system.launch.py`, increase the `TimerAction` delay for Mission Manager by +3–5s.

**Prevention:**
- Keep Mission Manager start ≥ 10s after Nav2 (`TimerAction`).
- Avoid CPU spikes (close browsers/RViz while starting).

</TabItem>
<TabItem value="goal-rejected" label="❌ Goal rejected.">

**Where:** `mission_manager.py` during `navigate_to_pose`

**Causes:** invalid frame, unreachable goal, AMCL not localized, or costmap thinks the goal is in an obstacle.

**Fix:**
1. Confirm frames:
   ```bash
   ros2 run tf2_tools view_frames.py && evince frames.pdf
   ```
2. Check goal is in map:
   - RViz → "2D Pose Estimate" → confirm robot is roughly correct.
3. Try a nearby reachable goal (clear of obstacles).
4. Clear costmaps:
   ```bash
   ros2 service call /global_costmap/clear_entirely_std_srvs/Empty {}
   ros2 service call /local_costmap/clear_entirely_std_srvs/Empty {}
   ```

**Prevention:**
- Use valid map + correct origin/resolution in .yaml.
- Run `pose_setter.py` only after Nav2 + map server are up.

</TabItem>
<TabItem value="no-camera" label="YOLO: no frames / depth invalid">

**Where:** `object_detector.py`

**Symptoms/logs:** No `/yolo/annotated`, warnings like `[SKIP] No valid depth at (cx,cy)`.

**Fix:**
1. Verify topics exist:
   ```bash
   ros2 topic list | grep -E "camera|image|depth"
   ```
2. Ensure depth encoding is `16UC1`; reconfigure camera if needed.
3. Confirm intrinsics publishing:
   ```bash
   ros2 topic echo /camera/color/camera_info -n1
   ```
4. If depth alignment is off, re-seat camera or adjust static TF (camera→base_link).

**Prevention:**
- Keep camera TF fixed and correct (`static_transform_publisher`).
- Clean lens; avoid reflective floors (depth dropouts).

</TabItem>
<TabItem value="tf-fail" label="[❌ TF FAIL] Even 'now' failed">

**Where:** `object_detector.py` (map transform)

**Why:** TF tree incomplete or wrong frame names; timestamp mismatch.

**Fix:**
1. Inspect TF:
   ```bash
   ros2 run tf2_tools view_frames.py && evince frames.pdf
   ```
2. Ensure `depth_msg.header.frame_id` equals your camera link (e.g., `camera_link`).
3. Start the `static_transform_publisher` before YOLO node.
4. Retry with reduced transform timeout (already handled in code) after frames are steady.

**Prevention:**
- Launch the static TF node early (`TimerAction` before detector).

</TabItem>
<TabItem value="arm-offline" label="MyCobot: connection / no motion">

**Where:** `pick_node.py` / `drop_node.py`

**Symptoms:** No movement, or socket error.

**Fix:**
1. Ensure hotspot `MyCobotWiFi2.4G` / pass `mycobt123` is ON.
2. Ping detected IP (from auto-scan) or fallback:
   ```bash
   ping 192.168.137.75
   ```
3. Run arm node directly with IP:
   ```bash
   ros2 run mycobot_arm pick_node --ros-args -p m5_ip:=192.168.137.75
   ```
4. Power-cycle the arm; re-run.

**Prevention:**
- Keep the arm on stable power; avoid sharing the 2.4GHz band with heavy traffic.

</TabItem>
<TabItem value="pose-not-set" label="No initial pose set">

**Where:** Nav2/AMCL feels "lost"

**Logs:** Goals fail or robot spins in place.

**Fix:**
1. Confirm `pose_setter` log:
   ```
   ✅ Initial pose published at (...)
   ```
2. In RViz, manually use "2D Pose Estimate" once, then retry goals.

**Prevention:**
- Delay `pose_setter.py` ~2–3s after Nav2 + map server (already in launch).

</TabItem>
</Tabs>

---

## 🧭 Most Common Errors — By Symptom

<Tabs>
<TabItem value="no-move" label="Robot doesn't move">

**Likely causes:** Nav2 not ready, invalid goal, emergency stop.

**Checklist:**
1. `/navigate_to_pose` exists?
   ```bash
   ros2 action list | grep navigate
   ```
2. Costmaps valid?
3. Battery > 50%?
4. Try a short 0.5m forward goal in free space.

</TabItem>
<TabItem value="drift" label="Robot drifts / wrong localization">

**Likely causes:** bad map origin/resolution, initial pose error, wheel slip.

**Fix:**
1. Re-publish initial pose (RViz).
2. Verify map YAML resolution & origin.
3. Remap space if environment changed substantially.

</TabItem>
<TabItem value="no-detect" label="No detections">

**Likely causes:** wrong model path, too-high confidence, poor lighting.

**Fix:**
1. Confirm model path in `object_detector.py`.
2. Lower `conf` to 0.4–0.5.
3. Improve lighting; ensure object is within 0.3–3.5m.

</TabItem>
<TabItem value="approach-bad" label="Robot stops too far/close to object">

**Likely causes:** camera→base_link offset incorrect.

**Fix:**
1. Adjust `CAMERA_TO_BASE_LINK` in detector (e.g., 0.10–0.18m).
2. Recalibrate static TF and re-test.

</TabItem>
<TabItem value="arm-miss" label="Arm misses object / bad grip">

**Likely causes:** pose mismatch vs. gripper reach, wrong pickup pose.

**Fix:**
1. Tweak pickup angles in `pick_node.py` (pickup joints).
2. Ensure approach leaves clearance (no base collision).
3. Slow joint speed a bit for accuracy (e.g., 30–40).

</TabItem>
</Tabs>

---

## 🧰 Step‑by‑Step Solutions (Triage Playbooks)

### 1) Nav2 Fail → Fast Recovery
1. `Ctrl+C` all nodes.
2. Start map + Nav2 only; wait stable.
3. Publish initial pose (RViz or `pose_setter.py`).
4. Send a tiny test goal near the robot.
5. If OK, launch full stack.

### 2) YOLO/Depth Fail → Fast Recovery
1. Check topics exist (`/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`).
2. Validate TF: `camera_link` → `base_link` exists.
3. Restart camera node; then restart detector.
4. If still flaky, reduce `conf`, verify exposure/lighting.

### 3) Arm Not Responding
1. Verify Wi‑Fi connection; ping IP.
2. Run `pick_node` standalone with `m5_ip` param.
3. Power-cycle arm; re-run launch.

---

## 🛡 Prevention Strategies

- **Staggered Launch**: Use `TimerAction` gaps (Camera → Nav2 → Pose Setter → Mission Manager → RQT).
- **Map Hygiene**: Remap after hardware changes (camera height, LiDAR swap, furniture move).
- **Stable TFs**: Keep a single source of truth for `camera_link` ↔ `base_link` (`static` publisher), launched early.
- **Resource Headroom**: Avoid starting RViz, Foxglove, and heavy terminals all at once on the robot; use a remote machine when possible.
- **Battery & Space**: >50% charge; ensure clear space for 270° scans and arm motions.
- **Model Management**: Store trained weights with versioned folders; update `object_detector.py` path intentionally.

---

## 🧪 Quick Health Commands

```bash
# Topics & rates
ros2 topic list
ros2 topic hz /camera/color/image_raw
ros2 topic echo /amcl_pose -n1

# Actions & services
ros2 action list | grep navigate
ros2 service list | grep costmap

# TF sanity check
ros2 run tf2_tools view_frames.py

# Logs with highlighting
ros2 launch nav_handler full_system.launch.py | grep -E "WARN|ERROR|FAIL|❌|🛑|❌|⚠️"
```

<Admonition type="info" title="Tip">
Keep a **known-good mission.log** from a successful run. Diff against it when things break to spot regressions fast.
</Admonition>

---

**Next: 🧑‍💻 Developer's Corner**
