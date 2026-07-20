"""
auto_label.py — תיוג אוטומטי של raw_frames עם YOLOv8n (COCO)
מריץ את המודל על כל תמונה ושומר קבצי .txt בפורמט YOLO.

שימוש: python auto_label.py
דרישה: pip install ultralytics
"""

import os
import shutil
import random
from pathlib import Path
from ultralytics import YOLO

# ── הגדרות ────────────────────────────────────────────────────
RAW_DIR   = "dataset/raw_frames"
LABEL_DIR = "dataset/labels"
IMG_TRAIN = "dataset/images/train"
IMG_VAL   = "dataset/images/val"
LBL_TRAIN = "dataset/labels/train"
LBL_VAL   = "dataset/labels/val"
YAML_PATH = "dataset/dataset.yaml"

CONF_THRESH = 0.10   # סף נמוך — עדיף שיתייג יותר מדי מאשר מעט מדי
SPLIT       = 0.80   # 80% train

# מיפוי COCO class_id → class_id שלנו + שם
# (רק הקלאסות הרלוונטיות לסימולציה)
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

# ── הכנת תיקיות ───────────────────────────────────────────────
for d in [IMG_TRAIN, IMG_VAL, LBL_TRAIN, LBL_VAL]:
    os.makedirs(d, exist_ok=True)

# ── טעינת מודל ────────────────────────────────────────────────
print("[AUTO-LABEL] טוען YOLOv8n...")
model = YOLO("yolov8n.pt")

# ── רשימת תמונות — כל התיקיות בdataset (מלבד תיקיות פלט) ────
EXCLUDE = {"images", "labels"}
images = []
for folder in Path("dataset").iterdir():
    if folder.is_dir() and folder.name not in EXCLUDE:
        images.extend(folder.glob("*.jpg"))
        images.extend(folder.glob("*.png"))
images = sorted(set(images))
if not images:
    print("[ERROR] לא נמצאו תמונות בתיקיית dataset")
    exit(1)

print(f"[AUTO-LABEL] נמצאו {len(images)} תמונות — מריץ זיהוי...")

# ── זיהוי ושמירת תיוגים ───────────────────────────────────────
labeled = 0
skipped = 0

label_data = {}  # path → txt content

for img_path in images:
    results = model(str(img_path), conf=CONF_THRESH, verbose=False)[0]

    lines = []
    for box in results.boxes:
        coco_id = int(box.cls.item())
        if coco_id not in COCO_TO_OURS:
            continue
        our_id, name = COCO_TO_OURS[coco_id]
        conf = box.conf.item()

        # פורמט YOLO: class cx cy w h (נורמלי 0-1)
        x, y, w, h = box.xywhn[0].tolist()
        lines.append(f"{our_id} {x:.6f} {y:.6f} {w:.6f} {h:.6f}")

    label_data[img_path] = lines

    if lines:
        labeled += 1
    else:
        skipped += 1

print(f"[AUTO-LABEL] תויגו: {labeled}  |  ריקות (ללא זיהוי): {skipped}")
print()

# ── חלוקה train/val ───────────────────────────────────────────
random.seed(42)
keys = list(label_data.keys())
random.shuffle(keys)
split_idx   = int(len(keys) * SPLIT)
train_imgs  = keys[:split_idx]
val_imgs    = keys[split_idx:]

def save_split(img_list, img_dir, lbl_dir):
    for img_path in img_list:
        name = img_path.stem  # frame_00001
        # תמונה
        shutil.copy(str(img_path), os.path.join(img_dir, img_path.name))
        # תיוג
        with open(os.path.join(lbl_dir, name + ".txt"), "w") as f:
            f.write("\n".join(label_data[img_path]))

save_split(train_imgs, IMG_TRAIN, LBL_TRAIN)
save_split(val_imgs,   IMG_VAL,   LBL_VAL)

print(f"[AUTO-LABEL] Train: {len(train_imgs)} תמונות → {IMG_TRAIN} + {LBL_TRAIN}")
print(f"[AUTO-LABEL] Val:   {len(val_imgs)} תמונות → {IMG_VAL} + {LBL_VAL}")

# ── dataset.yaml ──────────────────────────────────────────────
import yaml
cfg = {
    "path":  str(Path("dataset").resolve()).replace("\\", "/"),
    "train": "images/train",
    "val":   "images/val",
    "nc":    7,
    "names": CLASS_NAMES,
}
with open(YAML_PATH, "w", encoding="utf-8") as f:
    yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)

print(f"[AUTO-LABEL] dataset.yaml נוצר.")
print()
print("השלבים הבאים:")
print("  1. בדקי כמה תמונות תויגו (labeled vs skipped)")
print("  2. אם הרבה ריקות — זה בסדר, זה אומר YOLO לא זיהה כלום")
print("  3. הרצי: python train.py")
print()
print("[DONE]")
