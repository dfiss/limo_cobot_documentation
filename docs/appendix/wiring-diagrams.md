---
sidebar_position: 21
---

# 🔌 Wiring Diagrams & Connection Guide

This section shows the complete wiring and connection layout for the **LIMO Pro + MyCobot + YOLO** system, including cable specifications and assembly instructions.

import Admonition from '@theme/Admonition';
import ThemedImage from '@theme/ThemedImage';

---

## 🖇 System Overview Diagram

<ThemedImage
  alt="LIMO Pro + MyCobot + YOLO Wiring Diagram"
  sources={{
    light: '/img/wiring_diagram_light.svg',
    dark: '/img/wiring_diagram_dark.svg',
  }}
/>

---

## 🔗 Connection Schema

| Component | Connection Type | Destination | Notes |
|-----------|-----------------|-------------|-------|
| **LIMO Pro Base** | Internal power & CAN bus | Onboard computer | Pre-wired by AgileX |
| **Orbbec DaBai Camera** | USB 3.0 | Onboard computer USB port | Use supplied high-speed cable |
| **MyCobot Arm** | Wi-Fi 2.4 GHz | Onboard computer network | Static SSID: `MyCobotWiFi2.4G` |
| **Gripper** | Internal MyCobot control line | MyCobot wrist port | Pre-installed |
| **Onboard Computer Power** | DC barrel jack | LIMO Pro 24V output | Requires step-down converter to 12V if not natively supported |
| **E-stop Switch** | LIMO safety circuit | LIMO motor drivers | Interrupts drive power immediately |

---

## 📏 Cable Specifications

| Cable Type | Spec | Recommended Length | Notes |
|------------|------|--------------------|-------|
| USB 3.0 Type-A to Type-C (Camera) | Shielded, 5Gbps | 1.5 m | Avoid hubs to reduce latency |
| DC Barrel Power Cable | 5.5 × 2.5 mm, 12–24V | 0.5 m | Use strain relief to prevent disconnection |
| Ethernet (Optional) | Cat6 | As required | For wired networking during debugging |

<Admonition type="tip" title="Tip">
Label both ends of every cable — it saves hours during reassembly.
</Admonition>

---

## 🛠 Assembly Instructions

1. **Mount the MyCobot**  
   - Secure the arm base to the LIMO mounting plate using M4 screws.  
   - Ensure no cable interference during arm motion.

2. **Connect the Orbbec Camera**  
   - Mount securely at the front of the LIMO base.  
   - Run the USB 3.0 cable along the chassis, avoiding wheel areas.

3. **Power the Onboard Computer**  
   - Connect DC barrel jack to power source.  
   - If using the LIMO’s 24V port, ensure proper voltage regulation.

4. **Setup MyCobot Wi-Fi**  
   - Power on the arm.  
   - Connect to hotspot `MyCobotWiFi2.4G` (pass: `mycobot123`).  
   - The `full_system.launch.py` script will auto-detect IP.

5. **Final Safety Check**  
   - Test E-stop function.  
   - Verify no loose or pinched cables.

---

<Admonition type="danger" title="Safety Warning">
Always power down and disconnect batteries before rewiring.  
Avoid routing cables near moving wheels, gears, or arm joints.
</Admonition>
