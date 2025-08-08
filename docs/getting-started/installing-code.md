---
sidebar_position: 3
title: Installing Project Code
---

import Admonition from '@theme/Admonition';

#  Installing Project Code

Let's get your development environment and LIMO Cobot codebase set up **the right way**—no risky workspace mistakes!

---

##  1. Clone the Repository

```bash
# Make a new directory for your robotics workspace (if you haven't)
mkdir -p ~/limo_ws/src
cd ~/limo_ws/src

# Clone the full project repo
git clone https://github.com/your-org/limo-robotics.git
cd limo-robotics
```

<Admonition type="tip" title="Pro tip">
Never clone or build in the robot's source workspace! Always use your own <code>~/limo_ws/src/</code> folder for development.
</Admonition>

##  2. Install Dependencies

<Admonition type="info" title="ROS2 Dependencies">
Install all required ROS2 packages and Python libs:

```bash
# Make sure you have rosdep
sudo apt update
sudo apt install python3-rosdep

# Initialize rosdep if not done already
sudo rosdep init || true
rosdep update

# Install all dependencies from your workspace root
cd ~/limo_ws
rosdep install --from-paths src --ignore-src -r -y
```
</Admonition>

<Admonition type="info" title="Python Packages">
From the repo root:

```bash
pip3 install -r requirements.txt
```
</Admonition>

##  3. Build the Workspace

```bash
cd ~/limo_ws
colcon build
```

Always source your workspace after building:

```bash
source install/setup.bash
```

##  4. Verify Installation

Run the basic launch to test:

```bash
ros2 launch full_system.launch.py
```

Watch for:
- No errors on launch
- Robot, arm, and camera nodes connect and announce
- Logs say: "Exploration initialized"

<Admonition type="tip" title="Troubleshooting">
If you see errors, check:
<ul>
  <li>Did you <code>source install/setup.bash</code>?</li>
  <li>Is your Python version  3.8?</li>
  <li>Are all USB devices and cameras plugged in and detected?</li>
</ul>
See <a href="../troubleshooting/faq.md">Troubleshooting & FAQ</a> for common fixes.
</Admonition>
