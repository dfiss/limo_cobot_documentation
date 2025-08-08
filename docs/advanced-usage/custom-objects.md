---
sidebar_position: 2
---

# 🎯 Advanced Usage — Custom Objects

This section explains how to change the robot's object detection targets by creating **custom datasets**, training your own YOLO model, and optimizing detection performance for your LIMO system.

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Admonition from '@theme/Admonition';

---

## 🛠 Changing Object Detection Classes

Your YOLO detector (`object_detector.py`) currently loads a custom model:
```python
self.model = YOLO("/home/agilex/krish_ws/runs/detect/mycobot_final2/weights/best.pt")
```

To change classes:

1. Train a new YOLO model with your desired objects.
2. Replace the `.pt` file path in the detector node.
3. Ensure `self.model.names` contains the correct class labels.
4. Restart the system to load the new model.

💡 **Note**: If you only want to filter detections to certain classes, you can check `cls_name` inside the YOLO callback and ignore others.

---

## 📸 Custom Dataset Creation

### 1. Collect Images
Use the LIMO's camera:

```bash
ros2 run rqt_image_view rqt_image_view
```

Take screenshots or record video, then extract frames.

### 2. Annotate
Use LabelImg or Label Studio:

```bash
labelImg /path/to/images classes.txt
```

Export format: YOLO TXT format (one file per image).

### 3. Organize Dataset
```
dataset/
 ├── images/
 │    ├── train/
 │    ├── val/
 ├── labels/
 │    ├── train/
 │    ├── val/
 └── data.yaml
```

Example `data.yaml`:

```yaml
train: /absolute/path/to/dataset/images/train
val: /absolute/path/to/dataset/images/val
nc: 3
names: ["objectA", "objectB", "objectC"]
```

---

## 🧠 Model Training Procedures

<Tabs>
<TabItem value="python" label="Train in Python">

```python
from ultralytics import YOLO

model = YOLO("yolov8n.pt")  # or yolov8s.pt for more accuracy
model.train(
    data="/path/to/data.yaml",
    epochs=50,
    imgsz=640,
    batch=16,
    device=0  # GPU ID
)
```

</TabItem>
<TabItem value="cli" label="Train via CLI">

```bash
yolo detect train data=/path/to/data.yaml model=yolov8n.pt epochs=50 imgsz=640 batch=16 device=0
```

</TabItem>
</Tabs>

After training, your weights will be at:

```bash
runs/detect/trainX/weights/best.pt
```

Update the `object_detector.py` model path to this file.

---

## ⚡ Detection Optimization

### Confidence Threshold (`conf`)
Increase to reduce false positives.

```python
results = self.model.predict(source=color_img, conf=0.6, verbose=False)
```

### Image Size
Larger sizes improve accuracy, but increase inference time.

### Model Variant
- `yolov8n.pt` → fastest, least accurate.
- `yolov8s.pt` → balanced.
- `yolov8m.pt` → most accurate, slower.

### GPU Acceleration

```bash
export CUDA_VISIBLE_DEVICES=0
```

### Class Filtering

```python
if cls_name not in ["target_class"]:
    return
```

---

## 🧪 Testing Your Model

After swapping the model:

```bash
ros2 run object_detector yolo_detector
```

Look for:

```
✅ YOLO detector node initialized with synchronized inputs.
[📍 DETECTED] target_class 2D(320,240) 3D(0.42, 0.15, 0.80)
```

Verify:
- Bounding boxes appear correctly in `/yolo/annotated`.
- `/target_pose` publishes when your target is detected.

<Admonition type="tip" title="Pro Tip">
If detection is unstable, collect **more diverse training images** with different lighting, angles, and distances.
</Admonition>

---

**Next: 📍 Customizing Waypoints**
