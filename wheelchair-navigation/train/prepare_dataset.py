"""
שלב 2: ארגון frames לאימון YOLOv8.
שימוש: python prepare_dataset.py
דרישה: הרץ את הסימולציה קודם — frames יישמרו ב-dataset/raw_frames/
"""
import os
import shutil
import random
import glob
import yaml

DATASET_DIR = "dataset"
RAW_DIR     = os.path.join(DATASET_DIR, "raw_frames")
TRAIN_DIR   = os.path.join(DATASET_DIR, "images", "train")
VAL_DIR     = os.path.join(DATASET_DIR, "images", "val")
YAML_PATH   = os.path.join(DATASET_DIR, "dataset.yaml")
SPLIT       = 0.8  # 80% train

# --- איסוף frames ---
frames = sorted(glob.glob(os.path.join(RAW_DIR, "frame_*.jpg")))
if not frames:
    print(f"[ERROR] לא נמצאו frames ב-{RAW_DIR}")
    print("הרץ את הסימולציה קודם ואחכה ל-200 frames.")
    exit(1)

print(f"[INFO] נמצאו {len(frames)} frames")

random.seed(42)
random.shuffle(frames)

split_idx     = int(len(frames) * SPLIT)
train_frames  = frames[:split_idx]
val_frames    = frames[split_idx:]

print(f"[INFO] Train: {len(train_frames)}  |  Val: {len(val_frames)}")

# --- יצירת תיקיות והעתקה ---
os.makedirs(TRAIN_DIR, exist_ok=True)
os.makedirs(VAL_DIR,   exist_ok=True)

for f in train_frames:
    shutil.copy(f, os.path.join(TRAIN_DIR, os.path.basename(f)))

for f in val_frames:
    shutil.copy(f, os.path.join(VAL_DIR, os.path.basename(f)))

print(f"[INFO] תמונות הועתקו.")

# --- dataset.yaml ---
cfg = {
    "path":  os.path.abspath(DATASET_DIR).replace("\\", "/"),
    "train": "images/train",
    "val":   "images/val",
    "nc":    7,
    "names": ["person", "car", "bus", "motorcycle", "bicycle", "traffic light", "stop sign"]
}
with open(YAML_PATH, "w", encoding="utf-8") as f:
    yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)

print(f"[DONE] dataset.yaml נוצר: {YAML_PATH}")
print()
print("השלבים הבאים:")
print(f"  1. העלה את {TRAIN_DIR}/ ו-{VAL_DIR}/ ל-Roboflow לתיוג")
print(f"  2. ב-Roboflow: סמן person/car/bus/motorcycle/bicycle/traffic light/stop sign")
print(f"  3. ייצא מ-Roboflow בפורמט YOLOv8 לתוך: {DATASET_DIR}/labels/")
print(f"  4. הרץ: python train.py")
