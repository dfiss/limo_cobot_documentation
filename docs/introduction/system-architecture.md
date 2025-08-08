---
sidebar_position: 3
title: System Architecture Diagram
---

# 🏗️ System Architecture Diagram

This diagram shows how every part of the LIMO Cobot system fits together—hardware, ROS2 nodes, and data flow.  
Everything is modular, robust, and designed for easy debugging and upgrades.

```mermaid
graph TD
    subgraph Hardware
      A1[LIMO Pro Base]
      A2[MyCobot Arm]
      A3[Orbbec Camera]
    end
    subgraph "ROS2 Nodes"
      B1[Mission Manager<br/>mission_manager.py]
      B2[Object Detector<br/>object_detector.py]
      B3[Pick Node<br/>pick_node.py]
      B4[Drop Node<br/>drop_node.py]
      B5[Pose Setter<br/>pose_setter.py]
    end
    subgraph Navigation
      C1[Nav2 Stack]
      C2[SLAM/Cartographer]
    end

    %% Hardware connections
    A1 -->|/cmd_vel, /amcl_pose| B1
    A2 <-->|Socket WiFi| B3
    A2 <-->|Socket WiFi| B4
    A3 -->|RGB+Depth| B2

    %% Core workflow
    B1 -->|Waypoint Goals| C1
    B2 -->|/target_pose| B1
    B1 -->|Pick/Drop Calls| B3
    B1 -->|Pick/Drop Calls| B4
    B5 -->|/initialpose| C1
    C1 -->|Feedback/Status| B1
    C2 -->|/map| C1

    %% Data flow highlights
    B2 -->|/yolo/annotated| User[User Interface]
    C1 -->|/cmd_vel| A1
```

## 🔍 Key Components Explained

### Hardware Layer
- **LIMO Pro Base**: The mobile platform providing autonomous navigation capabilities
- **MyCobot Arm**: 6-DOF robotic arm for object manipulation via WiFi control
- **Orbbec Camera**: RGB-D sensor for object detection and 3D perception

### ROS2 Node Layer
- **Mission Manager**: Orchestrates the entire workflow and coordinates between nodes
- **Object Detector**: YOLOv8-powered detection with 3D pose estimation
- **Pick Node**: Handles object grasping operations with the MyCobot arm
- **Drop Node**: Manages object placement and drop-off operations
- **Pose Setter**: Initial robot localization and pose management

### Navigation Stack
- **Nav2 Stack**: Advanced path planning and obstacle avoidance
- **SLAM/Cartographer**: Real-time mapping and localization

## 🔄 Data Flow Summary

1. **Mission Manager** sends waypoint goals to **Nav2**
2. **Object Detector** processes camera data and publishes target poses
3. **Navigation** moves the robot while avoiding obstacles
4. **Pick/Drop Nodes** execute manipulation tasks via WiFi
5. **Continuous feedback** ensures robust operation and error recovery
