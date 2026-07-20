"""
יוצר סרטון הדגמה של YOLO על פריימי המצלמה מהסימולציה.
שימוש: python make_yolo_video.py
"""
import cv2
import os
from ultralytics import YOLO

FRAMES_DIR  = "yolo_frames"          # פריימי JPEG שנשמרו בזמן הסימולציה
INPUT_VIDEO = "yolo_demo_final.avi"  # fallback אם אין פריימים
OUTPUT_MP4  = "yolo_exam_demo.mp4"
CONF        = 0.01   # סף זיהוי — כל עצם מעל 1% נכנס למערכת
LABEL_CONF  = 0.02   # מתחת ל-2% confidence → "unknown", מעל → שם המחלקה

model = YOLO("build/Debug/yolov8n.onnx", task="detect")

# --- איסוף פריימים ---
import glob as _glob

frames = []

# קודם כל מנסה לטעון פריימי JPEG מהסימולציה
jpg_files = sorted(_glob.glob(os.path.join(FRAMES_DIR, "frame_*.jpg")))
if jpg_files:
    for f in jpg_files:
        img = cv2.imread(f)
        if img is not None:
            frames.append(img)
    print(f"[INFO] Loaded {len(frames)} JPEG frames from {FRAMES_DIR}/")

# fallback: AVI ישן
if not frames and os.path.exists(INPUT_VIDEO):
    cap = cv2.VideoCapture(INPUT_VIDEO)
    while True:
        ret, f = cap.read()
        if not ret:
            break
        frames.append(f)
    cap.release()
    print(f"[INFO] Loaded {len(frames)} frames from {INPUT_VIDEO}")

# fallback: debug PNG
if not frames:
    debug_png = "controllers/wheelchair_cpp_controller/debug_camera_frame.png"
    if os.path.exists(debug_png):
        img = cv2.imread(debug_png)
        frames = [img] * 30
        print(f"[INFO] Using debug PNG (repeated 30x)")

if not frames:
    print("[ERROR] No source frames found. Run the simulation first.")
    exit(1)

h, w = frames[0].shape[:2]

# --- הגדרת VideoWriter ---
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
out    = cv2.VideoWriter(OUTPUT_MP4, fourcc, 2.0, (w, h))

print(f"[INFO] Running YOLO (conf={CONF}) on {len(frames)} frames...")

for i, frame in enumerate(frames):
    results = model(frame, conf=CONF, verbose=False)[0]

    annotated = frame.copy()

    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf  = float(box.conf[0])
        cls   = int(box.cls[0])
        label = model.names[cls]

        # צבע לפי קלאס
        color = (0, 200, 0)
        if label == "person":
            color = (255, 80, 0)
        elif label in ("car", "truck", "bus", "motorcycle"):
            color = (0, 50, 255)
        elif label == "traffic light":
            color = (0, 220, 220)

        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

        txt = f"{label} {conf*100:.0f}%"
        (tw, th), bl = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(y1 - 5, th + 5)
        cv2.rectangle(annotated, (x1, ty - th - 2), (x1 + tw, ty + bl), color, cv2.FILLED)
        cv2.putText(annotated, txt, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # HUD — background rectangle + clean white text
    n_det = len(results.boxes)
    hud = f"Frame {i+1}/{len(frames)}  |  Objects: {n_det}"
    (tw, th), bl = cv2.getTextSize(hud, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
    cv2.rectangle(annotated, (0, 0), (tw + 16, th + bl + 10), (0, 0, 0), cv2.FILLED)
    cv2.putText(annotated, hud, (8, th + 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    out.write(annotated)
    print(f"  frame {i+1}: {n_det} detections")

out.release()

size = os.path.getsize(OUTPUT_MP4) if os.path.exists(OUTPUT_MP4) else 0
print(f"\n[DONE] Saved: {OUTPUT_MP4}  ({size} bytes)")
print("Open with VLC or any player that supports mp4v codec.")
print("If it won't open, run: python convert_to_wmv.py")
