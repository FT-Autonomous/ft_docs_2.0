# Steps to Train a YOLO model 

This document describes the process for training a YOLO object detection model to identify cones used in Formula Trinity's perception module. The model is trained on the **FSOCO dataset**, a standardized dataset for Formula Student driverless teams.

---

## 1. Dataset Setup

### Download Dataset

Access the **FSOCO training/validation dataset** used for YOLOv5 training from the shared Google Drive:

🔗 [FSOCO Dataset (TCD access required)](https://drive.google.com/drive/folders/1NtRSsSpDN0kVyCnilNPOLVW9iurcGkq-?usp=sharing)

The above dataset maintains a **80/20 training–validation split**. 

```
dataset/
├── train/
│   ├── images/
│   └── labels/
└── val/
    ├── images/
    └── labels/
```

---

## 2. Environment Setup

### Install Ultralytics

YOLOv12 is part of the unified **Ultralytics** framework.

```bash
pip install ultralytics
```

If the `yolo` executable is not found in your PATH, update it:

```bash
export PATH=$PATH:/home/$USER/.local/bin
```

Verify installation:

```bash
yolo
```

---

## 3. Create the Dataset Configuration File

Create a file named `dataset.yml` to describe your dataset layout and class structure.

```yaml
path: /home/$USER/fsoco  # Update path to the dataset root

train: train/images
val: val/images

nc: 6
names:
  0: yellow_cone
  1: orange_cone
  2: large_orange_cone
  3: blue_cone
  4: other_cone
  5: unknown_cone
```
- YOLO uses this YAML file to locate your training/validation images and to define the class labels for object detection.

---

## 4. Model Selection & Rationale

### Why use the **YOLO-*s* (small)** model variant?

* **Speed-critical environment:** The perception stack in Formula Trinity runs in real-time on a Jetson AGX Orin.
* **Trade-off balance:** The `s` model provides a strong balance between inference speed and accuracy.

---

## 5. Train the Model

- Create a bash script to automate training. E.g. `train_v12.sh`.
- Note: You may need to download the base model from the ultralytics repo and place in `/home/$USER/ft/`. [See here](https://docs.ultralytics.com/)

```bash
#!/usr/bin/env bash

yolo detect train \
  data=/home/$USER/ft/dataset.yml \
  model=/home/$USER/ft/yolo12s.pt \    
  epochs=50 \
  imgsz=640 \
  batch=10
```

Make the script executable:

```bash
chmod 700 train_v12.sh
```

Execute training:

```bash
./train_v12.sh
```

Training typically takes **~2 hours for 50 epochs** (on fta-pc).

---

## 6. Outputs

After training completes, the results are saved under:

```
runs/detect/train/
```

Key files:

* `weights/best.pt` → best-performing model weights
* `results.png` → training curve visualization 

---

## 7. Validate the Model

To validate the trained model on the validation set:

```bash
yolo detect val model=/home/$USER/ft/runs/detect/train/weights/best.pt data=/home/$USER/ft/dataset.yml
```

This will output accuracy metrics including **precision**, **recall**, and **mAP**, along with visualized validation samples.

---

## 8. Next Steps

Once validated, the model (`best.pt`) can be integrated into the **perception stack** for cone detection.

---




