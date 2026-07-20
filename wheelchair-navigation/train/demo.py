"""
demo.py — הדגמת השוואה בין שני מודלי YOLO
  שמאל: סרטון אמיתי   → YOLOv8n (COCO)
  ימין: סימולציה Webots → YOLOv8s מאומן

שימוש: python demo.py
"""

import cv2
import time
from pathlib import Path
from ultralytics import YOLO

# ── הגדרות ────────────────────────────────────────────────────
REAL_VIDEO   = "MVI_6442.MP4"
CUSTOM_MODEL = "models/yolov8s_webots.onnx"
BASE_MODEL   = "yolov8n.pt"

DISPLAY_W    = 640
DISPLAY_H    = 480
SIM_FPS      = 4   # מהירות בדיקת פריים חדש מהסימולציה

CLASS_NAMES  = ["person", "car", "bus", "motorcycle", "bicycle", "traffic light", "stop sign"]

# ── טעינת מודלים ──────────────────────────────────────────────
print("[DEMO] טוען מודלים...")
model_real = YOLO(BASE_MODEL)

if Path(CUSTOM_MODEL).exists():
    model_sim = YOLO(CUSTOM_MODEL)
    sim_label = "Webots YOLOv8s (מאומן)"
    print(f"[DEMO] נמצא מודל מותאם: {CUSTOM_MODEL}")
else:
    model_sim = YOLO(BASE_MODEL)
    sim_label = "YOLOv8n (לא מאומן עדיין)"
    print("[DEMO] מודל מותאם לא נמצא — משתמש ב-YOLOv8n")

# ── פונקציה: הוספת תיבות זיהוי על פריים ─────────────────────
def draw_results(frame, results, title, title_color):
    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        conf  = box.conf.item()
        cls   = int(box.cls.item())
        names = results[0].names
        label = f"{names[cls]} {conf:.0%}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, max(y1-6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    # כותרת
    cv2.rectangle(frame, (0, 0), (DISPLAY_W, 36), (0, 0, 0), -1)
    cv2.putText(frame, title, (8, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, title_color, 2)
    return frame

# ── פתיחת סרטון אמיתי ────────────────────────────────────────
cap = cv2.VideoCapture(REAL_VIDEO)
if not cap.isOpened():
    print(f"[ERROR] לא ניתן לפתוח {REAL_VIDEO}")
    exit(1)

real_fps = cap.get(cv2.CAP_PROP_FPS) or 25
print(f"[DEMO] סרטון אמיתי: {REAL_VIDEO}  FPS={real_fps:.0f}")
print("[DEMO] סימולציה: קורא frames חיים מ-dataset_v2/raw_frames")
print("[DEMO] לחצי Q לסיום")

last_sim_t   = time.time()
sim_interval = 1.0 / SIM_FPS
last_sim_frame = None

def get_latest_sim_frame():
    """מחזיר את הפריים הכי חדש שנשמר מהסימולציה (dataset_v2 העדכני)."""
    frames = list(Path("dataset_v2/raw_frames").glob("frame_*.jpg"))
    if not frames:
        return None
    newest = max(frames, key=lambda f: f.stat().st_mtime)
    return cv2.imread(str(newest))

while True:
    # ── פריים אמיתי ──────────────────────────────────────────
    ret, real_frame = cap.read()
    if not ret:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        ret, real_frame = cap.read()

    real_frame = cv2.resize(real_frame, (DISPLAY_W, DISPLAY_H))
    res_real   = model_real(real_frame, conf=0.35, verbose=False)
    real_frame = draw_results(real_frame, res_real,
                              "מציאות אמיתית — YOLOv8n", (0, 220, 255))

    # ── פריים סימולציה — הכי חדש מ-raw_frames ────────────────
    now = time.time()
    if now - last_sim_t >= sim_interval:
        last_sim_t = now
        frame = get_latest_sim_frame()
        if frame is not None:
            last_sim_frame = frame

    if last_sim_frame is None:
        sim_frame = 255 * __import__('numpy').ones((DISPLAY_H, DISPLAY_W, 3), dtype='uint8')
        cv2.putText(sim_frame, "מחכה לסימולציה...", (120, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,0), 2)
        res_sim = None
    else:
        sim_frame = cv2.resize(last_sim_frame, (DISPLAY_W, DISPLAY_H))
        res_sim   = model_sim(sim_frame, conf=0.25, verbose=False)

    if res_sim is not None:
        sim_frame = draw_results(sim_frame, res_sim,
                                 f"סימולציה Webots — {sim_label}", (0, 255, 180))
    else:
        cv2.rectangle(sim_frame, (0,0), (DISPLAY_W, 36), (0,0,0), -1)
        cv2.putText(sim_frame, f"סימולציה Webots — {sim_label}", (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,180), 2)

    # ── שילוב זה-לצד-זה ──────────────────────────────────────
    combined = cv2.hconcat([real_frame, sim_frame])
    cv2.imshow("YOLO Demo — השוואת מודלים", combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("[DEMO] נסגר.")
