---
sidebar_position: 1
title: Introduction to LIMO Cobot
---

import Admonition from '@theme/Admonition';

# 📚 Introduction to LIMO Cobot

Welcome to your all-in-one guide for the **LIMO Cobot** project—a full-stack, real-world robotics platform that brings together **autonomous exploration**, **object detection**, and **mobile manipulation** into a single, production-ready system.

<Admonition type="info" title="What is LIMO Cobot?">
<strong>LIMO Cobot</strong> is an autonomous mobile manipulation robot based on the AgileX LIMO Pro platform, featuring a MyCobot arm, YOLOv8 object detection, and a custom ROS2 control stack. It can autonomously explore, find, pick, and deliver objects—making it perfect for research, teaching, and practical automation demos.
</Admonition>

## 🛠️ What does it actually do?

- Explores a mapped environment completely on its own
- Uses advanced **YOLOv8** AI to detect target objects in 3D
- Plans and executes safe paths to each object (even with obstacles)
- Picks up the object with a MyCobot arm (via wireless control!)
- Returns to base, drops the object, and resumes exploring
- **All steps are voice-enabled**: the robot talks you through every stage

## 💡 Why was this built?

Building a working mobile manipulation robot isn’t just about code—it’s about real-world lessons, hardware hacks, mistakes, and “aha!” moments.  
This project distills those lessons into a workflow you can **actually reproduce**, learn from, and extend.

<Admonition type="tip" title="Who should care?">
If you want to go beyond “just simulations”—if you want to make robots that do something _useful_ in real rooms, with real objects—LIMO Cobot is for you.
</Admonition>

---

## Next: [Key Features and Capabilities](./key-features.md)
