---
sidebar_position: 2
---

# ✏️ Code Style & Standards

Consistent code style ensures that the **LIMO Pro + MyCobot + YOLO** project remains **readable, maintainable, and safe**.  
This guide defines **standards, documentation rules, testing requirements, and best practices**.

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

---

## 📐 Style & Formatting

We follow **PEP 8** for Python with a few ROS2-specific conventions:

- **4-space indentation** (no tabs).
- **Snake_case** for variables/functions: `object_detected`, `navigate_to_pose_msg()`.
- **PascalCase** for class names: `PickNode`, `ExplorationNode`.
- **ALL_CAPS** for constants: `WAYPOINTS`, `PICK_TIMEOUT`.
- Imports grouped: standard libs → ROS2 → project modules.

```python
# ✅ Example
import math
import time

import rclpy
from rclpy.node import Node

from .utils import quat_from_yaw
```

<Admonition type="danger" title="Avoid">
- Long functions (>50 lines) — split into smaller methods.
- Magic numbers — use constants or parameters.
- Hardcoded paths — pass via ROS2 parameters.
</Admonition>

---

## 📄 Documentation Requirements

Every node, class, and function must have docstrings explaining its purpose.

**Example:**

```python
class PickNode(Node):
    """
    Controls the MyCobot arm to pick an object.

    Sequence:
    1. Open gripper
    2. Move to center
    3. Move to pickup pose
    4. Close gripper
    5. Return to center
    """
```

- Use inline comments to explain non-obvious math (e.g., yaw calculations, TF transforms).
- In launch files, comment why a delay exists (`TimerAction`), not just the value.

---

## 🧪 Testing Guidelines

<Tabs>
<TabItem value="unit" label="Unit Tests">

- Test individual functions (e.g., `quat_from_yaw()`).
- Mock ROS2 publishers/subscribers when possible.
- Use `pytest` for automated testing.

</TabItem>
<TabItem value="integration" label="Integration Tests">

- Run `colcon build && colcon test` before merging.
- Test end-to-end:
  1. Start `full_system.launch.py`
  2. Verify YOLO detects in `/yolo/annotated`
  3. Confirm `/target_pose` triggers navigation.

</TabItem>
<TabItem value="hardware" label="Hardware Tests">

- Test arm motions with clear workspace to prevent collisions.
- For navigation, run in a safe, obstacle-free area first.

</TabItem>
</Tabs>

---

## 💡 Best Practices

- **Fail safely** — if a node crashes, it should not leave motors running.
- **Voice feedback** for key states (pick complete, drop complete, navigation start).
- **Use ROS2 parameters** instead of editing code for configuration.
- **Log at appropriate levels:**
  - `self.get_logger().info()` → normal operation
  - `self.get_logger().warn()` → recoverable issue
  - `self.get_logger().error()` → critical failure
- **Keep launch dependencies modular** — avoid one giant launch file for everything.

---

## ✅ Pre-Merge Checklist

- ☐ Code builds with `colcon build --symlink-install`
- ☐ No PEP8 violations (`flake8` clean)
- ☐ All tests pass
- ☐ Code reviewed by at least one other contributor
- ☐ No debug prints left (`print()` → use logger)

<Admonition type="tip" title="Pro Tip">
Run `black . && isort .` before committing to auto-format your Python code.
</Admonition>

---

**Next: Testing Framework**
