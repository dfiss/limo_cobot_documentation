---
sidebar_position: 3
---

# 🆕 Adding New Nodes & Packages

This guide explains **how to create new ROS2 nodes and packages** for the **LIMO Pro + MyCobot + YOLO** ecosystem, follow architectural best practices, integrate them into the workflow, and test them safely.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';
import Link from '@docusaurus/Link';

---

## 📦 Creating a New Package

We use ROS2 Python packages for most nodes in this project.

```bash
cd ~/krish_ws/src
ros2 pkg create --build-type ament_python my_new_node
```

**Structure:**

```bash
my_new_node/
 ├── package.xml
 ├── setup.py
 ├── resource/my_new_node
 ├── my_new_node/__init__.py
 ├── my_new_node/my_new_node.py
 └── launch/
```

---

## 🧩 Node Architecture Guidelines

<Admonition type="tip" title="Keep it Modular">
Each node should have **one core responsibility** (e.g., detection, navigation, arm control) and communicate via ROS2 topics/services/actions.
</Admonition>

**Required structure inside a node:**

### 1. Initialization
- Node name
- Parameters
- Publishers/Subscribers/Action clients
- Logging start message

### 2. Core Logic
- Main loop or callbacks
- State handling (if applicable)

### 3. Shutdown Handling
- Stop motors, close connections, save state

**Example skeleton:**

```python
import rclpy
from rclpy.node import Node

class MyNewNode(Node):
    def __init__(self):
        super().__init__('my_new_node')
        self.get_logger().info("Node initialized.")

    def main_loop(self):
        pass  # Core logic here

def main(args=None):
    rclpy.init(args=args)
    node = MyNewNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 🔗 Integration Procedures

To integrate a new node:

### 1. Add to Launch File
Use `launch_ros.actions.Node` in `full_system.launch.py` with proper `TimerAction` so it starts after dependencies are ready.

### 2. Respect Existing Topics
- If publishing `/target_pose`, ensure format matches `PoseStamped` from `object_detector.py`.
- Avoid name collisions with existing topics.

### 3. Parameterize IPs and Paths
- For hardware nodes (like MyCobot), accept IP from `--ros-args -p m5_ip:=....`
- Never hardcode file paths — use ROS2 params.

### 4. Logging & Voice Feedback
- Use `self.get_logger().info()` for state changes.
- For user feedback, follow the `say()` function pattern in `pick_node.py` and `mission_manager.py`.

---

## 🧪 Testing New Nodes

<Tabs>
<TabItem value="sim" label="Simulation First">

- Use **RViz2** and fake data to test without risking hardware.
- If it's a detection node, feed it saved images.
- If it's a navigation node, test in Gazebo before the real robot.

</TabItem>
<TabItem value="hardware" label="Hardware Deployment">

1. **Ensure the workspace is clear** of obstacles.
2. **Launch only your new node + dependencies:**
   ```bash
   ros2 run my_new_node my_new_node
   ```
3. **Watch logs in real-time:**
   ```bash
   ros2 topic echo /your_topic
   ```

</TabItem>
</Tabs>

---

## ✅ Pre-Merge Checklist for New Nodes

- ☐ Package builds with `colcon build --symlink-install`
- ☐ Code follows [Code Style Guidelines](/docs/developer/code-style)
- ☐ Node starts without errors or warnings
- ☐ Integration tested with `full_system.launch.py`
- ☐ Does not break Mission Manager or YOLO detection logic

<Link 
  className="button button--primary button--lg" 
  to="/docs/developer/launch-config-files"
>
  📖 Read Next: Launch & Config File Creation
</Link>

---

**Next: Launch Configuration Files**
