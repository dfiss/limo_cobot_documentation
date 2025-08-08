---
sidebar_position: 5
---

# 🌐 Advanced Usage — Web Interface

This section explains how to set up and use a **web-based interface** for your LIMO robot to enable **remote control**, **status monitoring**, **automation**, and **custom UI elements**.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

---

## ⚙️ Web UI Setup

You can expose your LIMO's controls and live video feed via a browser using **ROS 2 web tools**.

**Option 1 — Foxglove Studio (Recommended)**  
- Install:
```bash
sudo apt install ros-foxy-foxglove-bridge
```
- Run:
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
- Open Foxglove Studio, connect to your LIMO's IP & port.

**Option 2 — Rosbridge + Webviz**
- Install:
```bash
sudo apt install ros-foxy-rosbridge-server
```
- Run:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
- Use Webviz or a custom HTML/JS dashboard.

<Admonition type="tip" title="Pro Tip">
For external access over the internet, use a VPN (e.g., Tailscale, Zerotier) for security instead of exposing ports publicly.
</Admonition>

---

## 🎮 Remote Control Capabilities

With the web UI, you can:
- Send **2D Nav Goals** to `/navigate_to_pose`.
- **Teleoperate** using virtual joystick controls.
- View **live camera feed** (`/yolo/annotated`).
- Monitor `/amcl_pose` for current location.
- View and change **waypoints** remotely.

Example: Send a goal from JavaScript via rosbridge:

```javascript
var goal = new ROSLIB.Goal({
  actionClient: new ROSLIB.ActionClient({
    ros: ros,
    serverName: '/navigate_to_pose',
    actionName: 'nav2_msgs/NavigateToPoseAction'
  }),
  goalMessage: {
    pose: {
      header: { frame_id: 'map' },
      pose: { position: { x: 1.0, y: 0.5, z: 0.0 }, orientation: { w: 1.0 } }
    }
  }
});
goal.send();
```

---

## 🤖 Automation Features

- **Scheduled Missions**: Use a backend (Node.js / Python Flask) to trigger full launch at set times.
- **Trigger on Detection**: Subscribe to `/object_found` and run pick/drop remotely.
- **Remote Map Switching**: Update `map:=` parameter in Nav2 launch without SSH.

Example automation script:

```bash
ros2 service call /waypoint_update nav_handler/WaypointList "{waypoints: [...]}"
```

---

## 🎨 Interface Customization

You can customize the UI to show:
- **System health** (CPU, RAM, battery).
- **Mission timeline** (current state from Mission Manager).
- **Manual pick/drop triggers**.
- **Live performance graphs** (FPS, latency).

<Tabs>
<TabItem value="react" label="React.js Example">

```javascript
function StatusCard({ title, value }) {
  return (
    <div className="p-4 bg-gray-900 text-white rounded-lg shadow-lg">
      <h2>{title}</h2>
      <p>{value}</p>
    </div>
  );
}
```

</TabItem>
<TabItem value="css" label="Custom Styling">

```css
.status-card {
  background: #1a1a1a;
  border-radius: 12px;
  padding: 10px;
  color: white;
}
```

</TabItem>
</Tabs>

---

## ✅ Success Indicators

- Browser UI loads with real-time telemetry.
- Remote commands reflect on the robot without delay.
- Camera feed shows with < 1s latency on LAN.
- All UI buttons correctly trigger their respective ROS 2 actions/services.

---

**Next: 🛡 Troubleshooting & FAQ**
