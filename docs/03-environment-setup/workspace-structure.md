---
sidebar_position: 1
title: ROS2 Workspace Mastery
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

# 🚀 ROS2 Workspace Mastery

Welcome to the **definitive guide** for setting up a bulletproof ROS2 development environment!  
This is your **minimal-but-complete** pattern for Python nodes, launch files, and professional packaging.

<Admonition type="tip" title="🎯 TL;DR for Pros">
  - **NEVER build inside vendor workspaces** (`/opt/ros/...`, `/home/agilex/robot_ws`)
  - **ALWAYS create your own external workspace** (`~/krish_ws`) 
  - **Use proper ament_python packaging** with glob patterns for launch files
  - **Follow the 10-step workflow** below for maximum productivity
</Admonition>

---

## 🛠️ Step 0: One-Time Shell Setup

<Tabs>
  <TabItem value="ubuntu" label="Ubuntu 20.04+ (Recommended)" default>

```bash
# Assuming Ubuntu 20.04 + ROS2 Foxy already installed
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

  </TabItem>
  <TabItem value="humble" label="ROS2 Humble">

```bash
# For ROS2 Humble users
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

  </TabItem>
</Tabs>

---

## 🏗️ Step 1: Create the Workspace Foundation

```bash
mkdir -p ~/krish_ws/src
cd ~/krish_ws
colcon build    # builds (empty) workspace just fine
source install/setup.bash
echo "source ~/krish_ws/install/setup.bash" >> ~/.bashrc
```

<Admonition type="info" title="💡 Pro Tip">
Notice we're using `~/krish_ws` instead of the generic `~/limo_ws`. This creates a personal namespace that won't conflict with vendor workspaces!
</Admonition>

---

## 📦 Step 2: Create a Python Package (ament_python)

```bash
cd ~/krish_ws/src
ros2 pkg create --build-type ament_python demo_pkg
```

This generates the perfect structure:

```
demo_pkg/
  ├── package.xml
  ├── resource/demo_pkg
  ├── setup.cfg
  ├── setup.py
  └── demo_pkg/
      └── __init__.py
```

---

## 🤖 Step 3: Add Your First Node

Create `demo_pkg/demo_pkg/talker.py`:

```python
import rclpy
from rclpy.node import Node

class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.timer = self.create_timer(1.0, self.tick)
        self.count = 0

    def tick(self):
        self.get_logger().info(f"Hello #{self.count}")
        self.count += 1

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

<Admonition type="warning" title="⚠️ Critical">
Make sure `demo_pkg/demo_pkg/__init__.py` exists (can be empty). Without this, Python won't recognize it as a package!
</Admonition>

---

## 🚀 Step 4: Add Launch Files + Glob Magic

Create `demo_pkg/launch/talker.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="demo_pkg", executable="talker", name="talker")
    ])
```

Now update `demo_pkg/setup.py` with **glob patterns** for automatic launch file discovery:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'demo_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ament index
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        # package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # 🎯 install ALL launch/*.py automatically!
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krish',
    maintainer_email='you@example.com',
    description='Demo package with a talker node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'talker = demo_pkg.talker:main',
        ],
    },
)
```

Configure dependencies in `package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>demo_pkg</name>
  <version>0.0.1</version>
  <description>Demo package with a talker node</description>
  <maintainer email="you@example.com">krish</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <!-- runtime dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## 🔧 Step 5: Build & Source

```bash
cd ~/krish_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

<Admonition type="tip" title="🎯 Symlink Install">
The `--symlink-install` flag is **crucial** for development! It creates symbolic links instead of copying files, so your code changes are reflected immediately without rebuilding.
</Admonition>

---

## ▶️ Step 6: Run Your Node (Two Ways)

<Tabs>
  <TabItem value="direct" label="Direct Execution" default>

```bash
ros2 run demo_pkg talker
```

  </TabItem>
  <TabItem value="launch" label="Via Launch File">

```bash
ros2 launch demo_pkg talker.launch.py
```

  </TabItem>
</Tabs>

---

## 🔄 Step 7: Add Publisher-Subscriber Nodes

Let's create a complete communication example!

### Publisher Node: `demo_pkg/demo_pkg/talker_pub.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerPub(Node):
    def __init__(self):
        super().__init__("talker_pub")
        self.pub = self.create_publisher(String, "chatter", 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.i = 0

    def tick(self):
        msg = String()
        msg.data = f"hello {self.i}"
        self.pub.publish(msg)
        self.get_logger().info(f"sent: {msg.data}")
        self.i += 1

def main():
    rclpy.init()
    node = TalkerPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Subscriber Node: `demo_pkg/demo_pkg/listener.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.sub = self.create_subscription(String, "chatter", self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f"got: {msg.data}")

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Update Entry Points in `setup.py`

```python
entry_points={
    'console_scripts': [
        'talker = demo_pkg.talker:main',
        'talker_pub = demo_pkg.talker_pub:main',
        'listener = demo_pkg.listener:main',
    ],
}
```

### Multi-Node Launch: `demo_pkg/launch/chat.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="demo_pkg", executable="talker_pub", name="talker_pub"),
        Node(package="demo_pkg", executable="listener", name="listener"),
    ])
```

---

## ⚙️ Step 8: Parameters & Remapping Patterns

### Using Parameters

```python
# In launch file
Node(
    package="demo_pkg",
    executable="talker_pub",
    parameters=[{'rate_hz': 2.0}],  # inline dict
)

# Or from YAML file
Node(
    package="demo_pkg",
    executable="talker_pub", 
    parameters=[os.path.join(
        get_package_share_directory('demo_pkg'), 'config', 'params.yaml'
    )]
)
```

### Topic Remapping

```python
Node(
    package="demo_pkg",
    executable="listener",
    remappings=[('chatter', 'chatter_renamed')]
)
```

---

## 🔄 Step 9: The Perfect Dev Loop

```bash
# 1. Edit your code
# 2. Quick rebuild with symlinks
colcon build --symlink-install
source install/setup.bash

# 3. Test immediately
ros2 launch demo_pkg chat.launch.py
```

### When Things Get Weird 🐛

```bash
# Nuclear option - fresh start
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## ⚡ Step 10: Common Gotchas (Save Hours!)

<Admonition type="danger" title="🚨 Critical Requirements">
- **Must have** `resource/<pkgname>` directory
- **Must have** `pkgname/__init__.py` file  
- **Keep consistent**: package name = folder name = entry_points name
- **Always rebuild** after changing `setup.py` or `package.xml`
- **Glob magic**: Drop any `.py` file into `launch/` and rebuild - it's auto-installed!
</Admonition>

---

## 🗂️ Final Directory Structure

```
~/krish_ws/
├── src/
│   └── demo_pkg/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/demo_pkg
│       ├── launch/
│       │   ├── talker.launch.py
│       │   └── chat.launch.py
│       └── demo_pkg/
│           ├── __init__.py
│           ├── talker.py
│           ├── talker_pub.py
│           └── listener.py
├── build/              # auto-generated
├── install/            # auto-generated  
└── log/               # auto-generated
```

---

## 🎯 Quick Commands Reference

| Action | Command |
|--------|---------|
| **Build workspace** | `colcon build --symlink-install` |
| **Source workspace** | `source install/setup.bash` |
| **Run node directly** | `ros2 run demo_pkg talker` |
| **Launch multi-node** | `ros2 launch demo_pkg chat.launch.py` |
| **Clean rebuild** | `rm -rf build/ install/ log/ && colcon build` |
| **Install dependencies** | `rosdep install --from-paths src --ignore-src -r -y` |

<Admonition type="success" title="🎉 You're Ready!">
This workflow is **production-tested** and will scale from simple demos to complex robotics systems. The glob patterns, symlink installs, and proper packaging mean you can focus on **building cool robots** instead of fighting the build system!
</Admonition>

---

## 🔗 Next Steps

1. [Core ROS2 Concepts](../core-concepts/ros2-basics.md) - Dive deeper into nodes, topics, and services
2. [System Integration](../core-concepts/system-integration.md) - Connect your nodes to the LIMO robot
3. [Launch Configuration](../developer/launch-config-files.md) - Advanced launch file patterns#   U p d a t e d 
 
 # Forced update
