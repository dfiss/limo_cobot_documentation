---
sidebar_position: 4
---

# 🛠 Advanced Usage — Hardware Upgrades

This section covers **compatible hardware upgrades** for your LIMO system, how to integrate them into your ROS 2 setup, the performance gains you can expect, and how to troubleshoot common upgrade issues.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

---

## 🔌 Compatible Hardware Upgrades

| Component | Upgrade Option | Benefit |
|-----------|----------------|---------|
| **Camera** | Intel RealSense D435i, ZED 2i, Orbbec Astra+ | Higher depth accuracy, wider FOV, better low-light performance |
| **LiDAR** | RPLIDAR A3, Hokuyo UST-10LX | Improved SLAM, faster map updates |
| **Arm** | MyCobot Pro 600, UFactory xArm 6 | Higher payload, reach, and precision |
| **Compute** | NVIDIA Jetson Orin Nano / Xavier NX | Faster YOLO inference, improved multi-node performance |
| **Networking** | Wi-Fi 6 or Gigabit Ethernet module | Lower latency for remote monitoring |
| **Power** | Higher capacity Li-ion battery pack | Extended mission runtime |

<Admonition type="tip" title="Pro Tip">
If you upgrade your camera or LiDAR, **always remap and recalibrate** your environment before running autonomous missions.
</Admonition>

---

## 🔄 Integration Procedures

<Tabs>
<TabItem value="camera" label="Upgrading the Camera">

1. **Install Drivers**  
   - Intel RealSense:  
     ```bash
     sudo apt install ros-foxy-realsense2-camera
     ```
   - ZED:  
     Install SDK from [stereolabs.com](https://www.stereolabs.com/).

2. **Update Launch Files**  
   - Replace `orbbec_camera/dabai.launch.py` with the correct camera node.
   - Update topics in `object_detector.py`:
     ```python
     rgb_topic = '/camera/color/image_raw'
     depth_topic = '/camera/depth/image_raw'
     ```

3. **Recalibrate** in RViz:
   - Use `tf2_ros` static transform publisher to set correct camera → `base_link` offset.

</TabItem>
<TabItem value="lidar" label="Upgrading the LiDAR">

1. **Install ROS 2 Driver Package**  
   For RPLIDAR:  
   ```bash
   sudo apt install ros-foxy-rplidar-ros
   ```

2. **Update SLAM Launch**  
   Edit RTAB-Map or Cartographer launch to use `/scan` from new LiDAR.

3. **Test**:
   ```bash
   ros2 topic echo /scan
   ```

</TabItem>
<TabItem value="compute" label="Upgrading Compute Unit">

1. Install OS & ROS 2 on new board.
2. Copy your workspace:
   ```bash
   rsync -avz krish_ws/ user@new_board:/home/user/
   ```
3. Rebuild:
   ```bash
   colcon build --symlink-install
   ```
4. Test YOLO inference speed with:
   ```bash
   yolo detect predict model=best.pt source=sample.jpg
   ```

</TabItem>
</Tabs>

---

## 🚀 Performance Improvements

- **Camera Upgrade** → Better depth accuracy = more precise `/target_pose`
- **LiDAR Upgrade** → Faster loop closures and cleaner maps
- **Compute Upgrade** → Lower detection latency, higher navigation update rate
- **Battery Upgrade** → 30–50% longer missions
- **Network Upgrade** → More stable RQT/remote teleop feeds

---

## 🛠 Upgrade Troubleshooting

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| Camera node crashes | Driver mismatch | Install correct ROS 2 driver for your camera |
| Depth image blank | Wrong topic names | Update `object_detector.py` to new depth topic |
| Nav2 fails after LiDAR upgrade | Map resolution mismatch | Remap environment with new sensor |
| YOLO slower after upgrade | Wrong CUDA setup | Reinstall NVIDIA drivers & torch with GPU support |
| RQT feed freezes | Network bottleneck | Switch to wired Gigabit Ethernet for testing |

<Admonition type="danger" title="Important">
After **any hardware change**, clear your old maps and re-run full mapping to ensure accurate localization.
</Admonition>

---

**Next: 🛡 Troubleshooting & FAQ**
