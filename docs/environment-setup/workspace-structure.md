---
sidebar_position: 1
title: Workspace Setup
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

# 🏁 Workspace Setup

Welcome!  
Before you do anything else, it's **critical** to set up your development environment the right way—this one step will save you countless hours of pain.

<Admonition type="info" title="TL;DR for Pros">
  - **NEVER build inside the vendor's source workspace** (`/opt/ros/...`, `/home/agilex/robot_ws`, etc).
  - **ALWAYS create your own external workspace** (e.g., `~/limo_ws`) for development, builds, and experiments.
  - **Only copy config/launch files from vendor if needed.**
</Admonition>

---

## 📦 Create a Safe ROS2 Workspace

<Tabs>
  <TabItem value="Ubuntu 20.04+" label="Ubuntu (Recommended)" default>

```bash
# 1. Create your external workspace
mkdir -p ~/limo_ws/src
cd ~/limo_ws

# 2. Add your project code (clone or copy)
# Example:
git clone https://github.com/your-org/limo-robotics.git src/limo_robotics

# 3. Build (DON'T build in /opt/ros or vendor folders!)
colcon build
```

  </TabItem>
  <TabItem value="Other OS" label="Other OS / WSL2">

LIMO is officially supported on **Ubuntu 20.04+** only. For Windows/Mac: use WSL2, a VM, or dual boot. Do all dev work in a Linux environment.

  </TabItem>
</Tabs>

## 🗂️ Directory Layout

```bash
~/limo_ws/
 ├── src/               # your code only!
 ├── build/             # auto-generated, never touch!
 ├── install/
 ├── log/
```

Keep it clean:
- Only your code in `src/`
- Never modify vendor/original robot folders
- All new packages, configs, and launches go here

## 📝 Best Practices Checklist

- ✅ Never touch vendor's workspaces.
- ✅ Always use your own `~/limo_ws`
- ✅ Backup your workspace regularly (USB, cloud, etc)
- ✅ Don't delete or overwrite robot configs
- ✅ If in doubt, copy, don't move

<Admonition type="caution" title="Real-World Lesson">
If you build in the source/vendor workspace, you can easily break your robot and lose all your configs. <b>Restore requires a fresh SD card image and wastes days. Trust me—set up your own workspace!</b>
</Admonition>

## 🔄 If Things Break: Recovery

- **Workspace corrupted?** Wipe your workspace and re-clone the repo.
- If you broke the vendor workspace, re-flash SD card (ask vendor for image).
- Backup often!
- Never delete vendor files.

## 🎯 Next Steps

1. [Environment Setup & Best Practices](../environment-setup/configuration.md)
2. [Core Concepts: ROS2 & Robotics 101](../04-core-concepts/ros2-basics.md)