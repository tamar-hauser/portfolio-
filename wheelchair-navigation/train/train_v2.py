"""
שלב 4 (v2): אימון YOLOv8s על דאטאסט_v2 (עדכני, נקי).
דרישות מוקדמות:
  pip install ultralytics
  python auto_label_v2.py   (יוצר dataset_v2/dataset.yaml + תיוגים)
שימוש: python train_v2.py
"""
import os
import shutil
from pathlib import Path

YAML_PATH   = "dataset_v2/dataset.yaml"
BASE_MODEL  = "yolov8s.pt"
OUT_ONNX    = "models/yolov8s_webots.onnx"
EPOCHS      = 50
IMG_SIZE    = 640
DEVICE      = "cpu"

if not os.path.exists(YAML_PATH):
    print(f"[ERROR] {YAML_PATH} לא נמצא.")
    print("הרץ: python auto_label_v2.py")
    exit(1)

labels_train = "dataset_v2/labels/train"
if not os.path.exists(labels_train) or not list(Path(labels_train).glob("*.txt")):
    print(f"[ERROR] תיוגים לא נמצאו ב-{labels_train}")
    exit(1)

print("[INFO] מתחיל אימון YOLOv8s על dataset_v2")
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
    name     = "webots_detection_v2",
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
