---
sidebar_position: 5
title: Backup and Restore
---

import Admonition from '@theme/Admonition';
import Details from '@theme/Details';

# 💾 Backup and Restore

Mistakes happen—even pros can brick a robot or lose weeks of work!  
This section shows you how to **back up your workspace, restore your system, and avoid data disasters.**

---

## 🛡️ Why Backup Matters

<Admonition type="warning" title="True Story">
  A single accidental `colcon build` in the vendor workspace can corrupt your robot's OS.  
  Always back up before any risky operation or major change!
</Admonition>

---

## 🗂️ What Should I Back Up?

- Your entire ROS2 **workspace** (e.g., `~/limo_ws`)
- All **custom packages** and code (everything in `src/`)
- Any **custom launch/config files** you created
- **Trained models** (YOLO weights, etc)
- **Maps and calibration files**  
- **Vendor SD card image** (if possible)

<Details summary="Checklist of Important Files">
- `~/limo_ws/` (all)
- `~/limo_ws/src/` (all your packages)
- `~/limo_ws/maps/`
- `~/limo_ws/runs/detect/` (custom YOLO weights)
- Any files you edited or created
- SD card image from the robot vendor (ask for this!)
</Details>

---

## 🏠 How to Back Up

### 1. Simple Copy (USB/Cloud)

```bash
# Backup workspace to USB drive
cp -r ~/limo_ws /media/usb_drive/limo_ws_backup_$(date +%Y%m%d)

# Or use cloud storage:
rclone copy ~/limo_ws remote:robot-backups/limo_ws_$(date +%Y%m%d)
```

### 2. Git Version Control (Recommended for Code)

```bash
cd ~/limo_ws/src/
git init                # if not already
git add .
git commit -m "Backup before changes"
git remote add origin https://github.com/your-org/limo-robotics.git
git push origin main
```

**Do not push `build/install/log` directories—use a `.gitignore`.**

---

## 🆘 Restore Procedures

### 1. Restoring Your Workspace

```bash
# If disaster strikes, just copy your backup back to home:
cp -r /media/usb_drive/limo_ws_backup_YYYYMMDD ~/limo_ws
```

Re-source your workspace:

```bash
cd ~/limo_ws
source install/setup.bash
```

### 2. Restoring Vendor System (SD Card)

If you break the vendor workspace or OS:

1. Contact vendor for a fresh SD card image
2. Use tools like **balenaEtcher** or **Raspberry Pi Imager** to re-flash the SD card
3. Re-add your `~/limo_ws` and custom files

<Details summary="Need a full reset?">
1. Power off robot
2. Remove SD card
3. Flash image using your PC
4. Insert card, boot up, and re-copy your workspace
</Details>

---

## 🔄 Recovery Strategies

- **Always test your backups!** (Try restoring to a different directory)
- If unsure, **never overwrite**—copy to a new folder first
- **Ask for help** before deleting or overwriting anything
- Keep **multiple backups:**
  - Local (USB), cloud, and Git for code

---

## ❗ Pro Tips & Gotchas

- **Never** build or install packages in `/opt/ros` or vendor-provided folders
- If you get stuck in a boot loop or OS won't load, suspect a **corrupted SD card**
- Keep backup copies **off the robot**—if hardware fails, you still have your files
- **Document** any parameter changes or hardware tweaks

<Admonition type="info" title="Real-World Lesson">
I lost days waiting for a new SD card because I didn't have my workspace backed up. **Now I backup every time before major updates or new experiments.**
</Admonition>

---

## 🎯 Next Steps

- [Troubleshooting & FAQ](../troubleshooting/faq)
- [Step-by-Step System Usage](../usage-guide/full-workflow)
