---
sidebar_position: 4
---

# 🚀 Launch & Config Files

In the **LIMO Pro + MyCobot + YOLO** system, launch files and configuration files are **critical** for ensuring every node starts in the correct order, with the right parameters, and without conflicts.  
This guide explains **how to write, structure, and manage them**.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

---

## 📝 Writing Custom Launch Files

We use **Python-based ROS2 launch files** to control startup.

**Example minimal launch file:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            output='screen',
            parameters=[{'example_param': 'value'}]
        )
    ])
```

---

## ⏱ Startup Sequencing with TimerAction

Our `full_system.launch.py` uses `TimerAction` so nodes launch only after dependencies are ready:

```python
from launch.actions import TimerAction

TimerAction(period=2.0, actions=[nav2_node]),  # Start Nav2 after 2s
TimerAction(period=3.0, actions=[pose_setter]), # Pose setter after Nav2
TimerAction(period=10.0, actions=[mission_manager]), # Mission Manager last
```

**Why?**
- Nav2 must load the map before `pose_setter` runs.
- `mission_manager` must wait until localization and YOLO detection are ready.

---

## ⚙️ Configuration File Formats

We use **YAML** for parameters like `nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    min_particles: 500
    max_particles: 2000
    update_min_a: 0.2
    update_min_d: 0.25
```

**Tips:**
- Use `ros__parameters` root key for ROS2.
- Keep one YAML per node for clarity.
- Store in `config/` inside the package.

---

## 🔄 Parameter Management

You can pass parameters in three ways:

<Tabs>
<TabItem value="inline" label="Inline in Launch">

```python
Node(
    package="my_package",
    executable="my_node",
    parameters=[{'m5_ip': '192.168.137.75'}]
)
```

</TabItem>
<TabItem value="yaml" label="From YAML">

```python
Node(
    package="nav2_bringup",
    executable="bringup_launch.py",
    parameters=["config/nav2_params.yaml"]
)
```

</TabItem>
<TabItem value="cli" label="From CLI">

```bash
ros2 run my_package my_node --ros-args -p m5_ip:=192.168.137.75
```

</TabItem>
</Tabs>

---

## 💡 Launch File Best Practices

<Admonition type="tip" title="Do This">
- Use **FindPackageShare** so your launch files work from any workspace.
- Keep launch files **modular** — one per subsystem (YOLO, navigation, arm).
- Use `SetLaunchConfiguration` for values reused across nodes.
- Log key events so the startup process is traceable.
</Admonition>

<Admonition type="danger" title="Avoid">
- Hardcoding absolute paths — always resolve relative to the package share.
- Starting all nodes at once — it causes race conditions.
- Embedding too much logic in launch files — keep them orchestration-only.
</Admonition>

---

## ✅ Checklist Before Committing

- ☐ Launch file runs without errors.
- ☐ Config YAML loads correctly (no tab characters — YAML only allows spaces).
- ☐ Dependencies start in the right order.
- ☐ Works both from `ros2 launch` and as part of `full_system.launch.py`.

---

## 📎 Related

- [ROS2 Launch File API Docs](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Parameter Files in ROS2](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)

---

**Next: Parameter Tuning & Optimization**
