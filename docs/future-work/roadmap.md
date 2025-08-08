---
sidebar_position: 17
---

# 🗺️ Project Roadmap

The **LIMO Pro + MyCobot + YOLO** system is already functional and production-grade — but there’s plenty of room to make it **faster, smarter, and more versatile**.  
This roadmap outlines **short-term tasks, long-term vision, and planned features**.

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

---

## 🎯 Short-Term Goals (0–3 Months)

<Tabs>
<TabItem value="stability" label="System Stability">

- Fine-tune Nav2 parameters for smoother motion in tight spaces.
- Improve YOLO detection speed by optimizing image resolution & confidence threshold.
- Expand interrupt logic in `mission_manager.py` for multi-object detection without manual restart.

</TabItem>

<TabItem value="usability" label="Usability">

- Add CLI launch profiles:
  - **explore-only**
  - **detection-only**
  - **full-mission**
- Implement visual HUD in RViz2 showing:
  - Current state (exploring, navigating, picking, dropping)
  - Waypoint progress
  - Last detected object coordinates

</TabItem>
</Tabs>

---

## 🚀 Long-Term Vision (6–12 Months)

- **Multi-Object Handling**  
  Detect, queue, and collect multiple objects in one mission cycle.
- **Dynamic Mapping**  
  Use SLAM to update the map in real-time while still running detection.
- **Web Control UI**  
  Remote monitoring and manual override via browser dashboard.
- **AI Path Planning**  
  Integrate machine learning to choose optimal object collection order based on environment changes.
- **Multi-Robot Coordination**  
  Have multiple LIMOs share maps and divide pickup tasks.

---

## 🆕 Planned Features

| Feature | Benefit | Priority |
|---------|---------|----------|
| Configurable object classes in YOLO | Adapt to new object types without retraining | High |
| Automatic gripper calibration | Reduces failed picks | Medium |
| Voice-command control | Hands-free operation | Medium |
| System health diagnostics node | Early detection of hardware faults | High |

---

## ⏳ Timeline Estimates

| Timeframe | Goals |
|-----------|-------|
| **0–1 Month** | Nav2 tuning, YOLO optimization, basic CLI profiles |
| **2–3 Months** | RViz HUD, improved interrupt handling |
| **4–6 Months** | Multi-object handling, web UI prototype |
| **6–12 Months** | Dynamic mapping, AI path planning, multi-robot setup |

---

<Admonition type="tip" title="How to Contribute">
If you want to help with a roadmap item, check the **Developer’s Corner** for contribution guidelines and open tasks.
</Admonition>
