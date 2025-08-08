---
sidebar_position: 4
---

# ❓ Frequently Asked Questions (FAQ)

Got questions? Here are the most common ones we've encountered while developing and running the **LIMO Pro + MyCobot + YOLO** system.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';
import Link from '@docusaurus/Link';

---

## ⚡ Quick Answers

<Tabs>
<TabItem value="object-detection" label="Object Detection">

**Q:** YOLO is not detecting objects — what should I check?  
**A:**  
- Confirm camera topics are active:  
  ```bash
  ros2 topic list | grep camera
  ```
- Check YOLO weights path in `object_detector.py` matches your trained model.
- Ensure lighting is adequate and the object is within depth sensor range.

</TabItem>
<TabItem value="navigation" label="Navigation Issues">

**Q:** Robot spins in place or doesn't move to goal.  
**A:**
- Make sure `pose_setter` published the initial pose (check logs).
- Verify `nav2_params.yaml` matches your map.
- Reset AMCL and re-run navigation:
  ```bash
  ros2 lifecycle set /amcl configure
  ros2 lifecycle set /amcl activate
  ```

</TabItem>
<TabItem value="arm" label="Arm Control">

**Q:** MyCobot is not responding.  
**A:**
- Ensure Wi-Fi is connected to:
  ```
  SSID: MyCobotWiFi2.4G
  PASS: mycobot123
  ```
- Run IP detection logic from `full_system.launch.py` to confirm the arm is found.
- Power cycle the arm if connection fails.

</TabItem>
</Tabs>

---

## 💡 Common Problems & Solutions

| Problem | Possible Cause | Quick Fix |
|---------|----------------|-----------|
| Nav2 server not ready | Launch order issue | Increase delay in `full_system.launch.py` TimerAction |
| Pick/Drop timeout | Arm motion blocked or Wi-Fi lag | Clear workspace, re-run arm node with stable signal |
| Map drift | Poor AMCL initialization | Re-publish `/initialpose` from `pose_setter.py` |
| YOLO lag | Low GPU/CPU resources | Lower image resolution or increase confidence threshold |

---

## 🌍 Community Resources

- [ROS2 Documentation](https://docs.ros.org/en/foxy/)
- [Nav2 Navigation Stack](https://navigation.ros.org/)
- [Ultralytics YOLO Docs](https://docs.ultralytics.com/)
- [MyCobot Developer Hub](https://docs.elephantrobotics.com/)

---

## 📣 Where to Ask for Help

- **GitHub Discussions**: [Project Discussion Board](https://github.com/krish-rRay23/limo_cobot_documentation/discussions)
- **Discord Server**: [Join our real-time chat here](#)
- **ROS Discourse**: [ROS Community Forum](https://discourse.ros.org/)

<Admonition type="tip" title="Pro Tip">
When asking for help, always share:
- ROS2 version (`ros2 --version`)
- Full launch command used
- Relevant log output (error + 10 lines before it)
</Admonition>

---

**Next: Developer Guide → Contributing**
