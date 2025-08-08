---
sidebar_position: 13
---

# 📖 Project Story — Lessons Learned

Every great robotics project is as much about **learning from mistakes** as it is about achieving milestones.  
Here’s the journey of building the **LIMO Pro + MyCobot + YOLO** system — the challenges, breakthroughs, and key decisions.

import Admonition from '@theme/Admonition';
import ThemedImage from '@theme/ThemedImage';

---

## 🚀 How It Started

I began with **basic ROS2 knowledge** — enough to follow tutorials but not enough to handle a real, complex robot.  
The first week was **pure ideation**:
- Defined the system’s end goal: autonomous exploration → detect → navigate → pick → return → drop.
- Drafted the **workflow and roadmap**.

**Lesson:** *“Knowing the steps is not the same as knowing how to execute them in the real world.”*

---

## ⚡ The First Mistake — Workspace Chaos

In my eagerness, I made a **critical error**:  
I built my first test node **inside the robot’s source workspace** and ran `colcon build` there.

Result?  
- Robot’s configs were overwritten.  
- Core files got corrupted.  
- Required a **pre-flashed SD card** from the manufacturer to recover.

<Admonition type="danger" title="Never Again">
Never build inside the source workspace.  
Always use an **external colcon workspace** for custom development.
</Admonition>

---

## 🛠️ Restart & Smart Development

While waiting for the SD card, I studied documentation, planned better, and decided:
- Keep **everything modular** — separate packages for detection, navigation, arm, mission control.
- Never touch outdated or unused robot files — clone fresh from GitHub when needed.
- Build the system in **isolated, tested components**.

---

## 🎯 Major Milestones

### **1. Object Detection**
- Switched to **YOLOv8** with a **custom dataset**.
- Trained the model to detect only relevant objects.
- Added depth integration and map-frame transforms for precise navigation targeting.

### **2. Arm Control**
- MyCobot’s USB board was faulty — instead of replacing it, I **converted it to Wi-Fi control**.
- Wrote auto-IP detection logic in `full_system.launch.py` so users don’t need to manually configure IPs.

### **3. Navigation**
- Created robust maps with **Cartographer**.
- Automated pose initialization using a fixed AMCL pose in `pose_setter.py`.
- Designed **waypoints + rotational scans** to ensure full area coverage.

### **4. Mission Manager**
- The “brain” node:  
  - Orchestrates all components.
  - Handles interrupts from YOLO detection without breaking Nav2 goals.
  - Integrates voice feedback for key events.

---

## 🧠 Key Decisions & Rationale

| Decision | Why It Mattered |
|----------|-----------------|
| **Modular package structure** | Prevented one node from breaking the whole system. |
| **TimerAction in launch files** | Ensured correct startup order, avoiding race conditions. |
| **Voice feedback** | Allowed hands-free monitoring during development. |
| **Custom YOLO training** | Reduced false positives and increased pick success rate. |
| **Base pose automation** | Removed manual localization step, speeding up deployments. |

---

## 📌 What I Learned

<Admonition type="tip" title="Top Lessons">
- Don’t rush — build, test, and integrate **one module at a time**.
- Always back up working configurations before making big changes.
- Hardware issues are as important as software — solve them creatively.
- Clear system architecture from day one saves months later.
</Admonition>

---

<ThemedImage
  alt="Project Timeline"
  sources={{
    light: '/img/project_timeline_light.svg',
    dark: '/img/project_timeline_dark.svg',
  }}
/>

