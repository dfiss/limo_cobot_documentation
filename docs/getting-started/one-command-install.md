---
sidebar_position: 5
title: One-Command Install Script
---

import Admonition from '@theme/Admonition';

# 🚀 One-Command Install Script

If you want setup to be **lightning fast and repeatable**, use the included install script.  
It’ll set up all dependencies, build everything, and leave you ready to launch.

---

## 📝 Using the Script

1. Make the script executable:
    ```bash
    chmod +x scripts/setup_all.sh
    ```

2. Run the script from your workspace root:
    ```bash
    ./scripts/setup_all.sh
    ```

3. Follow any prompts for sudo password or confirmations.

---

<Admonition type="info" title="What does this script do?">
- Installs ROS2 and system dependencies (if missing)
- Installs Python requirements
- Builds the workspace with <code>colcon</code>
- Optionally runs <code>rosdep</code> and other setup tools
</Admonition>

---

## 🛠️ If you run into issues...

- Double-check you’re running the script from <code>~/limo_ws/</code> or your workspace root
- If the script fails, read the error log—most issues are missing system packages or internet connection
- You can always fall back to [manual installation](./installing-code.md) if needed

---

Once setup is complete, you’re ready for [First Run](./first-run.md)!
