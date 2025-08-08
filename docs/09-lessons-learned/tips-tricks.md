---
sidebar_position: 15
---

# 💡 Tips & Tricks

Whether you’re just starting with the **LIMO Pro + MyCobot + YOLO** system or already deep into development, these tips will help you **work smarter, debug faster, and optimize performance**.

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

---

## 🚀 Quick Wins for New Users

<Tabs>
<TabItem value="setup" label="Setup Tips">

- Always develop in a **separate colcon workspace** — never in the robot’s source workspace.
- Keep **two SD cards**: one in use, one as a bootable backup.
- Store your **map files, YOLO weights, and config YAMLs** in a Git repo for easy restoration.

</TabItem>

<TabItem value="operation" label="Operation Tips">

- Launch with `full_system.launch.py` to ensure correct startup order.
- If something fails, **disable non-essential nodes** and re-run the failing one in isolation.
- Use **voice feedback** logs (`say()` in nodes) to monitor progress without watching the terminal.

</TabItem>
</Tabs>

---

## ⚡ Performance Optimization

- **YOLO Speed**:  
  - Reduce image resolution if latency is high.  
  - Increase `conf` threshold in `object_detector.py` to filter noise.
- **Navigation Smoothness**:  
  - Tune Nav2 parameters in `nav2_params.yaml` — especially acceleration and max velocity.
  - Avoid overloading the CPU by running visualization tools (`rviz2`, `rqt_image_view`) on a remote PC.
- **Arm Control**:  
  - Wi-Fi control avoids USB bottlenecks and improves reliability.
  - Use **IP auto-detection** logic from `full_system.launch.py` to remove connection delays.

---

## 🔄 Workflow Improvements

<Admonition type="tip" title="Use Modular Testing">
Run and verify each subsystem individually before full integration:
1. **YOLO Detection** — run `object_detector` alone, check `/yolo/annotated`.
2. **Navigation** — run Nav2 with `pose_setter.py`, verify waypoint traversal.
3. **Arm Control** — test `pick_node.py` and `drop_node.py` without navigation.
</Admonition>

- Use `TimerAction` in launch files to stagger startup and avoid race conditions.
- Keep a **"safe" default map** and a **"development" map** — switch depending on your testing environment.

---

## ⏱ Time-Saving Techniques

- **Parameterize everything** — IPs, file paths, thresholds — so you can change them via CLI instead of editing code.
- Use `ros2 topic echo` and `ros2 topic hz` to quickly check if a topic is alive and healthy.
- For fast restarts, kill all nodes:
  ```bash
  pkill -9 -f ros2
