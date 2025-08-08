---
sidebar_position: 14
---

# ⚠️ Common Mistakes & How to Avoid Them

Robotics development is full of surprises — some pleasant, many painful.  
Here are the **most common mistakes** made during this project, how they were fixed, and what you can do to avoid them.

import Admonition from '@theme/Admonition';

---

## 🛑 Mistake 1 — Building Inside the Source Workspace

**What Happened:**  
In the first week, I ran `colcon build` inside the **robot’s factory source workspace** instead of an external one.  
This overwrote configs and broke the system.

**How It Was Fixed:**  
- Requested a **pre-flashed SD card** from the vendor.
- Reinstalled ROS2 + original robot packages.
- Created a separate `~/krish_ws` workspace for all custom code.

<Admonition type="tip" title="Lesson Learned">
Always develop in a **dedicated external workspace** — never touch the vendor’s source workspace.
</Admonition>

---

## 🔄 Mistake 2 — Doing Everything in One Node

**What Happened:**  
Early attempts combined detection, navigation, and arm control in a single giant node.  
This caused **clashes in state handling** and made debugging impossible.

**How It Was Fixed:**  
- Split the system into **modular packages**:
  - `object_detector`
  - `nav_handler` (Mission Manager + Pose Setter)
  - `mycobot_arm` (Pick & Drop)
- Used ROS2 topics and actions for communication.

**Lesson:** Modular design means **one node crash ≠ total system failure**.

---

## 📡 Mistake 3 — Hardware Bottlenecks

**What Happened:**  
The MyCobot arm’s USB control board failed, breaking wired control.

**How It Was Fixed:**  
- Flashed the arm’s firmware for **Wi-Fi control**.
- Wrote **IP auto-detection** logic in `full_system.launch.py` to connect without manual setup.

**Lesson:** Hardware failures can be solved creatively — and sometimes upgraded in the process.

---

## 🗺 Mistake 4 — Poor Map & Pose Initialization

**What Happened:**  
Manual AMCL initialization was slow and error-prone, causing bad navigation.

**How It Was Fixed:**  
- Created `pose_setter.py` to **automatically publish the correct initial pose** after Nav2 starts.
- Tuned Nav2 parameters to match the working environment.

**Lesson:** Automating localization removes an entire category of human error.

---

## 🧮 Mistake 5 — Navigation & Detection Race Conditions

**What Happened:**  
When YOLO detected an object mid-navigation, Nav2 would keep executing the old goal, leading to collisions or missed pickups.

**How It Was Fixed:**  
- Added **interrupt logic** in `mission_manager.py` to:
  - Cancel current goals.
  - Stop scanning.
  - Navigate to the detected object.
- Used threading to delay and control transitions.

**Lesson:** Robust **state management** is essential for multi-component robotics systems.

---

## 📝 Tips to Avoid These Mistakes

- [ ] **Modularize early** — split detection, navigation, and manipulation from the start.
- [ ] **Back up configs** before testing major changes.
- [ ] **Automate setup** (pose setting, IP detection) wherever possible.
- [ ] **Sequence startup** with `TimerAction` to avoid race conditions.
- [ ] **Test in parts** before running the full workflow.

<Admonition type="tip" title="Pro Tip">
If something breaks unexpectedly, **disable all non-essential nodes** and run the system in isolation to find the problem fast.
</Admonition>
