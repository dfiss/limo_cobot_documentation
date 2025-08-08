---
sidebar_position: 16
---

# 🪖 War Stories

Every robotics project has its **battle scars** — moments when everything broke, but creative thinking brought it back to life.  
Here are the most memorable problem-solving moments from the **LIMO Pro + MyCobot + YOLO** journey.

import Admonition from '@theme/Admonition';

---

## 💥 The Workspace Catastrophe

**The Situation:**  
In week one, I built a test node **inside the robot’s source workspace** and ran `colcon build`.  
This **overwrote factory configs**, corrupted files, and left the LIMO Pro unbootable.

**The Fix:**  
- Requested a **pre-flashed SD card** from the manufacturer.  
- Switched to a clean external workspace for all future development.  

<Admonition type="tip" title="Lesson Learned">
Treat the vendor’s source workspace like **read-only firmware** — never modify it directly.
</Admonition>

---

## 🔌 The MyCobot USB Failure

**The Situation:**  
The MyCobot arm’s USB control board stopped working. A replacement was expensive and would take weeks.

**The Fix:**  
- Flashed the arm firmware for **Wi-Fi mode**.  
- Wrote auto-IP detection in `full_system.launch.py` to connect instantly on startup.  
- Standardized Wi-Fi SSID/Password:  
