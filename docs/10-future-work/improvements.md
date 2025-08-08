---
sidebar_position: 19
---

# 🔧 Planned Improvements

The **LIMO Pro + MyCobot + YOLO** platform works reliably today — but there’s still plenty of room to **make it faster, smarter, and easier to use**.  
This page outlines improvements we’re targeting across mapping, detection, UI/UX, and performance.

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

---

## 🗺 Mapping Improvements

<Tabs>
<TabItem value="cartographer" label="Cartographer Tuning">

- Adjust scan matching parameters for better loop closure.
- Reduce map drift in long missions by fine-tuning `TRAJECTORY_BUILDER` settings.
- Store multiple environment maps for quick switching.

</TabItem>

<TabItem value="dynamic" label="Dynamic Mapping">

- Integrate **online SLAM** to update maps while running detection.
- Merge updated maps without interrupting the mission flow.

</TabItem>
</Tabs>

---

## 🎯 Detection Enhancements

- Add **configurable detection classes** so YOLO can be retargeted without retraining.
- Optimize inference by:
  - Lowering input resolution when FPS is critical.
  - Running detection on a dedicated GPU thread if available.
- Add **multi-object queuing** so detected targets are stored and collected in sequence.

<Admonition type="tip" title="Pro Tip">
Use `/yolo/annotated` to quickly visually confirm detection accuracy before a mission.
</Admonition>

---

## 🖥 UI/UX Improvements

- **RViz HUD Overlay** showing:
  - Current robot state (exploring, navigating, picking, dropping)
  - Waypoint progress
  - Last detected object position
- Browser-based **Web Dashboard** for:
  - Remote control
  - Live camera feed
  - Mission state management
- Voice-command interface for key commands (start, pause, return to base).

---

## ⚡ Performance Optimizations

- Fine-tune Nav2 velocity/acceleration parameters for smoother motion.
- Reduce CPU load by running heavy image processing on a **separate nodelet** or edge GPU.
- Cache TF transforms in `object_detector.py` to speed up coordinate conversions.
- Optimize `TimerAction` delays in `full_system.launch.py` for faster startup without race conditions.

---

## ✅ Next Steps

| Area | Action | Target |
|------|--------|--------|
| Mapping | Tune Cartographer params | 1–2 weeks |
| Detection | Configurable classes & multi-object queue | 1–2 months |
| UI/UX | RViz HUD & basic web UI | 2–3 months |
| Performance | Nav2 tuning & TF caching | Continuous |

---

<Admonition type="success" title="Goal">
These improvements aim to cut mission time by **20–30%** while improving reliability and user control.
</Admonition>
