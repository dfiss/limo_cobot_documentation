---
sidebar_position: 1
---

# 🛠️ Contributing to LIMO Cobot

We welcome contributions from developers, researchers, and robotics enthusiasts who want to improve the **LIMO Pro + MyCobot + YOLO** system.  
This section explains **how to add new features, submit code, and collaborate effectively**.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';
import Link from '@docusaurus/Link';

---

## 🚀 How to Contribute New Features

1. **Fork the Repository**  
   Click **Fork** on GitHub to create your copy.

2. **Clone Your Fork**
   ```bash
   git clone https://github.com/YOUR-USERNAME/limo-cobot.git
   cd limo-cobot
   ```

3. **Create a Feature Branch**
   ```bash
   git checkout -b feature/my-new-feature
   ```

4. **Develop in an External Workspace**  
   Never modify the robot's source workspace directly — always use an external colcon workspace.

5. **Follow Project Structure**
   - Nodes in dedicated packages (`object_detector`, `nav_handler`, `mycobot_arm`, etc.)
   - Launch files in `launch/` with proper `TimerAction` sequencing.
   - Configs in `config/` (maps, `nav2_params.yaml`, YOLO settings).
   - Avoid hardcoded paths — use ROS2 parameters instead.

---

## 📤 Code Submission Process

<Tabs>
<TabItem value="git" label="GitHub Workflow">

1. **Push your feature branch:**
   ```bash
   git push origin feature/my-new-feature
   ```

2. **Open a Pull Request (PR) on GitHub:**
   - Describe what you changed and why.
   - Reference any related issues.
   - Include test logs or screenshots.

</TabItem>
<TabItem value="ros" label="ROS Package Testing">

Before submitting, make sure your changes:

1. **Build without errors:**
   ```bash
   colcon build --symlink-install
   ```

2. **Pass basic runtime checks:**
   ```bash
   ros2 launch full_system full_system.launch.py
   ```

3. **Don't break Mission Manager logic** — especially interrupt handling between exploration, detection, and arm control.

</TabItem>
</Tabs>

---

## 🔍 Review Procedures

All contributions are reviewed for:

- **Code Quality**: Follows PEP8 for Python, uses meaningful variable names.
- **Safety**: No unsafe robot motions, respects e-stop behavior.
- **Modularity**: No "god nodes" — separate logic into focused packages.
- **Logging & Feedback**: Use `self.get_logger()` for debug info, and `say()` for user feedback where relevant.
- **Performance**: No unnecessary CPU/GPU load (e.g., avoid overly high YOLO input resolution).

---

## 📜 Community Guidelines

<Admonition type="tip" title="We value:">
- Clean, well-documented code.
- Respect for other contributors' time.
- Testing before submission.
- Clear commit messages (`feat: add drop node sequence improvements`).
- Avoiding direct commits to `main` branch.
</Admonition>

---

## 💬 Need Help While Contributing?

- **Ask in our [GitHub Discussions](https://github.com/krish-rRay23/limo_cobot_documentation/discussions)**
- **Join the [Discord Server](#)**
- **Use [ROS Discourse](https://discourse.ros.org/): ROS Community Forum**

<Link 
  className="button button--primary button--lg" 
  to="/docs/developer/code-style"
>
  📖 Read Next: Code Style Guidelines
</Link>

---

**Next: Code Style Guidelines**
