---
sidebar_position: 6
title: ROS2 Basics
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';
import Details from '@theme/Details';

# 🚀 ROS2 Basics

Welcome to **ROS2**—the backbone of your robotics development!  
Here's a crisp, practical overview so you can hit the ground running, build safely, and actually understand what's going on.

---

## 🧩 What is ROS2?

ROS2 (Robot Operating System 2) is a **modular robotics middleware** for building robot applications—from simulation to real-world deployment.  
It provides a standard for nodes, messaging, sensors, actuators, and distributed computation.

---

## 🛠️ ROS2 Workspace Structure

**A typical ROS2 workspace looks like this:**
```bash
~/limo_ws/
 ├── src/        # All your source code (packages, nodes, launch, config)
 ├── build/      # Auto-generated build files (never touch)
 ├── install/    # Installed executables, setup scripts
 ├── log/        # Build logs
```

<Admonition type="warning" title="Important">
Always develop in your own workspace (`~/limo_ws`), never in vendor or `/opt/ros` folders!
</Admonition>

---

## 📦 Creating a Workspace

```bash
# 1. Create the workspace and src folder
mkdir -p ~/limo_ws/src
cd ~/limo_ws

# 2. Build (initial empty build is fine)
colcon build

# 3. Source it (activate environment variables)
source install/setup.bash
```

Add this to your `~/.bashrc` for convenience:

```bash
echo "source ~/limo_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🏗️ ROS2 Core Concepts

| Term | What it is | Example |
|------|------------|---------|
| **Node** | A single process/module | `object_detector`, `mission_manager` |
| **Topic** | Pub/Sub message channel | `/cmd_vel`, `/image_raw`, `/target_pose` |
| **Message** | Data packet exchanged between nodes | `geometry_msgs/Twist`, `sensor_msgs/Image` |
| **Service** | Synchronous request/response between nodes | Resetting a sensor, start/stop commands |
| **Action** | Asynchronous long-running task (with feedback) | Navigation goal, arm movement |
| **Launch** | Script to start multiple nodes at once | `full_system.launch.py` |
| **Parameter** | Config variable, can be set at runtime | Navigation speed, sensor thresholds |

---

## 📡 Pub/Sub: Topics & Messages

Nodes communicate using **topics** (publish/subscribe pattern).

- **Publisher**: Sends messages on a topic.
- **Subscriber**: Listens for messages on a topic.

**Example:**  
`/cmd_vel` (velocity commands) uses `geometry_msgs/Twist` messages.

```python
# Publish a Twist message (move forward)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.1, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

---

## 🤖 Running Nodes & Launch Files

### Running a Node Directly
```bash
ros2 run <package_name> <executable>
# Example:
ros2 run object_detector object_detector_node
```

### Using Launch Files
```bash
ros2 launch <package_name> <launch_file>
# Example:
ros2 launch nav_handler full_system.launch.py
```

---

## 📝 Parameters

Parameters let you adjust node behavior at runtime (e.g., detection thresholds, map files).

Set with launch files, YAML configs, or the command line.

```bash
ros2 param set /node_name param_name value
```

---

## 🔧 Useful ROS2 Commands

| Command | Description |
|---------|-------------|
| `ros2 topic list` | List all topics |
| `ros2 topic echo <topic>` | Print messages published on a topic |
| `ros2 node list` | List all running nodes |
| `ros2 service list` | List available services |
| `ros2 param list` | List all parameters |
| `ros2 run ...` | Run a single node |
| `ros2 launch ...` | Launch multiple nodes |

---

## 🗺️ Visualization: RViz & RQT

**RViz**: 3D visualization of maps, robot pose, sensors, goals, etc.

```bash
rviz2
```

**RQT**: Graphical tools for inspecting topics, TF trees, node graphs, and more

```bash
rqt_graph     # Show node-topic relationships
rqt_tf_tree   # Visualize TF frames
```

---

## 📚 Creating & Building Packages

Create a new package:

```bash
cd ~/limo_ws/src
ros2 pkg create --build-type ament_python <package_name>
```

Build all packages:

```bash
cd ~/limo_ws
colcon build
source install/setup.bash
```

---

## 🚦 Safety & Shutdown

- Always stop all nodes with **Ctrl+C** before disconnecting hardware
- Shut down the robot before removing the SD card or power

---

## 🤓 Learn More

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [LIMO Pro ROS2 User Manual](https://github.com/agilexrobotics/limo_ros2)

---

## 📚 Next Steps

- [Workspace Setup](../03-environment-setup/ros2-workspace)
- [Environment Setup & Best Practices](../03-environment-setup/configuration)
- [Backup & Restore](../03-environment-setup/backup-restore)
