---
sidebar_position: 3
---

# 🔄 Reset & Recovery Guide

When working with the **LIMO Pro + MyCobot + YOLO** system, unexpected errors, crashes, or misconfigurations can happen.  
This guide gives you **step-by-step reset and recovery procedures** to get your robot back online without corrupting its core setup.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

---

## 🛑 Emergency Shutdown

If your robot behaves unexpectedly (e.g., spinning uncontrollably, arm collision risk):

1. **Press the Robot's E-Stop Button** – instantly cuts motor power.
2. **Kill All ROS2 Nodes**:
   ```bash
   pkill -9 -f ros2
   ```
3. **Turn Off the Main Power Switch** on the LIMO Pro.
4. **Disconnect Arm Power** (if MyCobot is active and unsafe).

<Admonition type="danger" title="Immediate Action">
Never try to "catch" the robot arm by hand while motors are active — it can cause injury or damage.
</Admonition>

---

## ♻️ Full System Reset

Sometimes, you need to reset software state without reflashing SD cards.

<Tabs>
<TabItem value="quick" label="Quick Reset">

For minor glitches (e.g., wrong pose, map drift):

```bash
# Kill all running nodes
pkill -9 -f ros2

# Restart full system
ros2 launch full_system full_system.launch.py
```

✅ This reloads `pose_setter`, Nav2, YOLO, and Mission Manager in the correct sequence.

</TabItem>
<TabItem value="full" label="Full Reset">

For major issues (navigation failing, robot unresponsive):

1. **Shutdown ROS2**:
   ```bash
   pkill -9 -f ros2
   ```

2. **Power cycle the robot** (turn off, wait 5s, turn on).

3. **Reconnect MyCobot over Wi-Fi**:
   - SSID: `MyCobotWiFi2.4G`
   - Pass: `mycobot123`

4. **Re-run the main launch file**:
   ```bash
   ros2 launch full_system full_system.launch.py
   ```

</TabItem>
</Tabs>

---

## 🧹 Workspace & Config Recovery

If ROS2 packages break or configs get corrupted:

<Admonition type="warning" title="Golden Rule">
Never build inside the **source workspace** of the robot. Always use an **external colcon workspace** for your code.
</Admonition>

1. **Backup your current workspace**:
   ```bash
   tar -czvf backup_ws_$(date +%F).tar.gz ~/krish_ws
   ```

2. **If broken**:
   ```bash
   rm -rf ~/krish_ws
   mkdir -p ~/krish_ws/src
   ```

3. **Re-clone your project** from GitHub.

4. **colcon build** only inside your workspace.

---

## 💾 Data Recovery (Maps, Logs, Params)

To restore custom maps, YOLO weights, or parameter configs after a reset:

| Data Type | Default Path | Backup Method |
|-----------|-------------|---------------|
| RTAB-Map / Cartographer Maps | `~/maps/*.yaml`, `.pgm` | Copy to USB / Git repo |
| YOLO Model Weights | `~/krish_ws/runs/detect/.../best.pt` | Cloud or Git LFS |
| Nav2 Params | `~/krish_ws/src/nav2_params.yaml` | Git commit |

---

## 🚑 Worst-Case Recovery (SD Card Reflash)

If nothing works:

1. Contact vendor or use your pre-flashed SD card backup.
2. Flash using **balenaEtcher** or **Raspberry Pi Imager**.
3. Boot robot and set up your workspace again.

---

## ✅ Post-Recovery Checklist

- ☐ All ROS2 nodes launch without errors.
- ☐ MyCobot connects automatically over Wi-Fi.
- ☐ Navigation goals work.
- ☐ YOLO detects objects in RViz / image topic.
- ☐ Mission Manager resumes full workflow.

<Admonition type="tip" title="Quick Tip">
Keep two microSD cards — one in use, one as a ready-to-go backup.
</Admonition>

---

**Next: Usage Guide → Full Workflow**
