import os
os.environ["HF_HUB_DISABLE_XET"] = "1"

import struct
import time
import serial
import numpy as np
from faster_whisper import WhisperModel

SERIAL_PORT = "COM14"
BAUD_RATE = 921600

AUDIO_MARKER = "SSTEP_AUDIO_BEGIN"
TRANSCRIPT_PREFIX = "SSTEP_TRANSCRIPT:"
SAMPLE_RATE = 16000

print("טוענת מודל Whisper (medium - יכול לקחת כמה דקות בפעם הראשונה, יש הורדה חד-פעמית)...")
model = WhisperModel("medium", device="cpu", compute_type="int8")
print("המודל נטען.")

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

print(f"מאזינה לפורט {SERIAL_PORT} במהירות {BAUD_RATE}...")

while True:
    raw_line = ser.readline()
    if not raw_line:
        continue

    line = raw_line.decode(errors="ignore").strip()

    if line == AUDIO_MARKER:
        print("זוהתה תחילת שידור אודיו...")
        length_bytes = ser.read(4)
        if len(length_bytes) < 4:
            print("שגיאה: לא התקבל אורך תקין מה-ESP32")
            continue
        length = struct.unpack("<I", length_bytes)[0]

        raw_pcm = ser.read(length)
        if len(raw_pcm) < length:
            print(f"אזהרה: התקבלו רק {len(raw_pcm)} מתוך {length} בתים - ייתכן חוסר סנכרון בפרוטוקול")
            continue

        audio_int16 = np.frombuffer(raw_pcm, dtype=np.int16)
        audio_float32 = audio_int16.astype(np.float32) / 32768.0
        segments, _ = model.transcribe(audio_float32, language="he")
        transcript = "".join(segment.text for segment in segments).strip()
        print(f"תמלול: {transcript!r}")

        ser.write(f"{TRANSCRIPT_PREFIX}{transcript}\n".encode("utf-8"))
    elif line:
        print(f"[מה-ESP32] {line}")
