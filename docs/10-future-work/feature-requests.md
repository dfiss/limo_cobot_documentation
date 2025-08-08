---
sidebar_position: 18
---

# 💭 Feature Requests

We want the **LIMO Pro + MyCobot + YOLO** system to evolve with the needs of its users and contributors.  
This page explains **how to submit feature requests**, how they’re evaluated, and how we decide on implementation priority.

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Link from '@docusaurus/Link';

---

## 📌 Current Community Requests

| Feature | Description | Status |
|---------|-------------|--------|
| Multi-object queue handling | Detect, store, and collect multiple targets in a single mission | Under Review |
| Web dashboard control | Browser-based UI for monitoring and manual override | Planned |
| Voice command support | Control robot states using voice inputs | Planned |
| Configurable YOLO classes | Switch detection targets without retraining | Accepted |
| Health diagnostics node | Monitors CPU, battery, motor status | Accepted |

---

## 📨 Request Submission Process

<Tabs>
<TabItem value="github" label="GitHub Issues">

1. Go to the [Project GitHub Repository](https://github.com/your-org/limo-cobot-documentation/issues).
2. Click **New Issue** → Select **Feature Request**.
3. Fill in:
   - **Feature name**
   - **Problem it solves**
   - **Proposed implementation**
   - **Priority level**

</TabItem>

<TabItem value="discord" label="Discord">

- Join the [Discord server](https://discord.gg/your-discord).  
- Post your request in the **#feature-requests** channel.
- A maintainer will tag it for tracking.

</TabItem>
</Tabs>

---

## ⚖️ Evaluation Criteria

When deciding whether to implement a feature, we consider:

- **Impact** — Will it significantly improve workflow or capabilities?
- **Feasibility** — Can it be implemented with available hardware and time?
- **Performance Cost** — Will it slow down detection, navigation, or arm control?
- **Safety** — Does it introduce any risk to the robot, objects, or users?
- **Maintainability** — Can it be supported long-term without major rework?

---

## 🏷 Implementation Priorities

| Priority | Definition | Example |
|----------|------------|---------|
| **High** | Critical for performance, reliability, or safety | Health diagnostics node |
| **Medium** | Improves usability or flexibility | Voice command support |
| **Low** | Nice-to-have but non-essential | Fun voice feedback variations |

---

<Admonition type="tip" title="Pro Tip">
The fastest way to get your feature accepted is to submit a **proof-of-concept** or **working prototype** along with your request.
</Admonition>

---

<Link
  className="button button--primary button--lg"
  to="/docs/developer/contributing"
>
📖 See How to Contribute Your Feature
</Link>
