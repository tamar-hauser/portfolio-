
import os
import struct
import time
import wave
import serial

SERIAL_PORT = "COM12"
BAUD_RATE = 460800

AUDIO_FOLDER = os.path.join(os.path.dirname(__file__), "speaker_audio")

REQUEST_PREFIX = "SSTEP_PLAY_REQUEST:"
AUDIO_MARKER = "SSTEP_AUDIO_BEGIN"
NOTFOUND_MARKER = "SSTEP_AUDIO_NOTFOUND"

def loadPcmFromWav(name):
    path = os.path.join(AUDIO_FOLDER, f"{name}.wav")
    if not os.path.isfile(path):
        return None

    with wave.open(path, "rb") as wf:
        if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getframerate() != 16000:
            print(f"אזהרה: '{name}.wav' לא בפורמט הנכון "
                  f"(צריך מונו, 16-bit, 16000Hz - יש: "
                  f"{wf.getnchannels()}ch, {wf.getsampwidth()*8}bit, {wf.getframerate()}Hz)")
        return wf.readframes(wf.getnframes())

if not os.path.isdir(AUDIO_FOLDER):
    os.makedirs(AUDIO_FOLDER)
    print(f">>> יצרתי את התיקייה {AUDIO_FOLDER} - שימי שם את קובצי ה-WAV שלך "
          f"(category_0.wav עד category_7.wav, ו-song.wav) <<<")

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

print(f"מאזינה לפורט {SERIAL_PORT} במהירות {BAUD_RATE}...")

while True:
    raw_line = ser.readline()
    if not raw_line:
        continue

    line = raw_line.decode(errors="ignore").strip()

    if line.startswith(REQUEST_PREFIX):
        name = line[len(REQUEST_PREFIX):]
        print(f"בקשה להשמעת: {name!r}")

        pcm = loadPcmFromWav(name)

        if pcm is None and name.startswith("combo_"):
            feedbackCategory = name.split("_")[-1]
            fallbackName = f"category_{feedbackCategory}"
            print(f"אין הקלטה ספציפית ל-{name!r}, נופלת חזרה ל-{fallbackName!r}")
            pcm = loadPcmFromWav(fallbackName)

        if pcm is None:
            print(f"לא נמצא קובץ מתאים בתיקייה {AUDIO_FOLDER}")
            ser.write(f"{NOTFOUND_MARKER}\n".encode("utf-8"))
            continue

        ser.write(f"{AUDIO_MARKER}\n".encode("utf-8"))
        ser.write(struct.pack("<I", len(pcm)))

        SEND_CHUNK_SIZE = 2048
        SEND_CHUNK_DELAY_SEC = 0.02
        for offset in range(0, len(pcm), SEND_CHUNK_SIZE):
            ser.write(pcm[offset:offset + SEND_CHUNK_SIZE])
            ser.flush()
            time.sleep(SEND_CHUNK_DELAY_SEC)

        print(f"נשלחו {len(pcm)} בתים")
    elif line:
        print(f"[מבקר הרמקול] {line}")
