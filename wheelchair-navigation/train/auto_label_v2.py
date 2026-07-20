"""
auto_label_v2.py — תיוג אוטומטי לדאטאסט החדש (dataset_v2/raw_frames בלבד)
מריץ את YOLOv8n על כל תמונה ושומר קבצי .txt בפורמט YOLO.

שימוש: python auto_label_v2.py
דרישה: pip install ultralytics
"""

import os
import shutil
import random
from pathlib import Path
from ultralytics import YOLO

# ── הגדרות ────────────────────────────────────────────────────
RAW_DIR   = "dataset_v2/raw_frames"
IMG_TRAIN = "dataset_v2/images/train"
IMG_VAL   = "dataset_v2/images/val"
LBL_TRAIN = "dataset_v2/labels/train"
LBL_VAL   = "dataset_v2/labels/val"
YAML_PATH = "dataset_v2/dataset.yaml"

CONF_THRESH = 0.10
SPLIT       = 0.80

COCO_TO_OURS = {
    0:  (0, "person"),
    2:  (1, "car"),
    5:  (2, "bus"),
    3:  (3, "motorcycle"),
    1:  (4, "bicycle"),
    9:  (5, "traffic light"),
    11: (6, "stop sign"),
}

CLASS_NAMES = ["person", "car", "bus", "motorcycle", "bicycle", "traffic light", "stop sign"]

for d in [IMG_TRAIN, IMG_VAL, LBL_TRAIN, LBL_VAL]:
    os.makedirs(d, exist_ok=True)

print("[AUTO-LABEL-V2] טוען YOLOv8n...")
model = YOLO("yolov8n.pt")

images = sorted(Path(RAW_DIR).glob("frame_*.jpg"))
if not images:
    print(f"[ERROR] לא נמצאו תמונות ב-{RAW_DIR}")
    exit(1)

print(f"[AUTO-LABEL-V2] נמצאו {len(images)} תמונות — מריץ זיהוי...")

labeled = 0
skipped = 0
label_data = {}

for img_path in images:
    results = model(str(img_path), conf=CONF_THRESH, verbose=False)[0]
    lines = []
    for box in results.boxes:
        coco_id = int(box.cls.item())
        if coco_id not in COCO_TO_OURS:
            continue
        our_id, name = COCO_TO_OURS[coco_id]
        x, y, w, h = box.xywhn[0].tolist()
        lines.append(f"{our_id} {x:.6f} {y:.6f} {w:.6f} {h:.6f}")

    label_data[img_path] = lines
    if lines:
        labeled += 1
    else:
        skipped += 1

print(f"[AUTO-LABEL-V2] תויגו: {labeled}  |  ריקות: {skipped}")

random.seed(42)
keys = list(label_data.keys())
random.shuffle(keys)
split_idx  = int(len(keys) * SPLIT)
train_imgs = keys[:split_idx]
val_imgs   = keys[split_idx:]

def save_split(img_list, img_dir, lbl_dir):
    for img_path in img_list:
        name = img_path.stem
        shutil.copy(str(img_path), os.path.join(img_dir, img_path.name))
        with open(os.path.join(lbl_dir, name + ".txt"), "w") as f:
            f.write("\n".join(label_data[img_path]))

save_split(train_imgs, IMG_TRAIN, LBL_TRAIN)
save_split(val_imgs,   IMG_VAL,   LBL_VAL)

print(f"[AUTO-LABEL-V2] Train: {len(train_imgs)}  Val: {len(val_imgs)}")

import yaml
cfg = {
    "path":  str(Path("dataset_v2").resolve()).replace("\\", "/"),
    "train": "images/train",
    "val":   "images/val",
    "nc":    7,
    "names": CLASS_NAMES,
}
with open(YAML_PATH, "w", encoding="utf-8") as f:
    yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)

print("[AUTO-LABEL-V2] dataset_v2/dataset.yaml נוצר.")
print("[DONE]")
