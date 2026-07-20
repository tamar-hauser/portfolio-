"""
שלב 4: אימון YOLOv8s על דאטאסט Webots.
דרישות מוקדמות:
  pip install ultralytics
  dataset/dataset.yaml קיים  (python prepare_dataset.py)
  dataset/labels/train/*.txt  ו-dataset/labels/val/*.txt קיימים (מ-Roboflow)
שימוש: python train.py
"""
import os
import shutil
from pathlib import Path

YAML_PATH   = "dataset/dataset.yaml"
BASE_MODEL  = "yolov8s.pt"
OUT_ONNX    = "models/yolov8s_webots.onnx"
EPOCHS      = 50
IMG_SIZE    = 640
DEVICE      = "cpu"   # שנה ל-"cuda" אם יש GPU

# --- בדיקות מוקדמות ---
if not os.path.exists(YAML_PATH):
    print(f"[ERROR] {YAML_PATH} לא נמצא.")
    print("הרץ: python prepare_dataset.py")
    exit(1)

labels_train = "dataset/labels/train"
if not os.path.exists(labels_train) or not list(Path(labels_train).glob("*.txt")):
    print(f"[ERROR] תיוגים לא נמצאו ב-{labels_train}")
    print("ייצא תיוגים מ-Roboflow בפורמט YOLOv8 ושמור ב-dataset/labels/")
    exit(1)

print("[INFO] מתחיל אימון YOLOv8s על Webots dataset")
print(f"       epochs={EPOCHS}  imgsz={IMG_SIZE}  device={DEVICE}")
print(f"       data={YAML_PATH}")
print()

os.makedirs("models", exist_ok=True)

from ultralytics import YOLO

model   = YOLO(BASE_MODEL)
results = model.train(
    data     = YAML_PATH,
    epochs   = EPOCHS,
    imgsz    = IMG_SIZE,
    device   = DEVICE,
    name     = "webots_detection",
    exist_ok = True,
    verbose  = True,
)

print()
print("[INFO] האימון הסתיים. מייצא ל-ONNX...")

onnx_path = model.export(format="onnx", imgsz=IMG_SIZE, simplify=True)
print(f"[INFO] ONNX נוצר: {onnx_path}")

shutil.copy(onnx_path, OUT_ONNX)
size = os.path.getsize(OUT_ONNX) // 1024
print()
print(f"[DONE] המודל נשמר: {OUT_ONNX}  ({size} KB)")
print("הרץ את הסימולציה — המודל החדש יזוהה אוטומטית.")
