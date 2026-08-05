
import os
os.environ["HF_HUB_DISABLE_XET"] = "1"

import winsound
import numpy as np
from flask import Flask, request, jsonify
from faster_whisper import WhisperModel

app = Flask(__name__)

AUDIO_FOLDER = os.path.join(os.path.dirname(__file__), "speaker_audio")

MODEL_DIR = os.path.join(os.path.dirname(__file__), "whisper_medium_model")
print("טוענת מודל Whisper מהתיקייה המקומית...")
model = WhisperModel(MODEL_DIR, device="cpu", compute_type="int8")
print("המודל נטען. השרת מוכן.")

SAMPLE_RATE = 16000

@app.route("/", methods=["GET"])
def health_check():
    return "השרת המקומי רץ ונגיש!"

@app.route("/stt", methods=["POST"])
def transcribe():
    raw_pcm = request.get_data()
    if not raw_pcm:
        return jsonify({"error": "no audio received"}), 400

    audio_int16 = np.frombuffer(raw_pcm, dtype=np.int16)
    audio_float32 = audio_int16.astype(np.float32) / 32768.0

    segments, _ = model.transcribe(audio_float32, language="he")
    transcript = "".join(segment.text for segment in segments).strip()

    print(f"תמלול: {transcript!r}")
    return jsonify({"transcript": transcript})

@app.route("/play", methods=["POST"])
def play_feedback():
    body = request.get_json(force=True, silent=True) or {}
    movement = body.get("movementProblem")
    feedback = body.get("feedbackCategory")
    if movement is None or feedback is None:
        return jsonify({"error": "missing movementProblem/feedbackCategory"}), 400

    name = f"combo_{movement}_{feedback}"
    path = os.path.join(AUDIO_FOLDER, f"{name}.wav")
    if not os.path.isfile(path):
        name = f"category_{feedback}"
        path = os.path.join(AUDIO_FOLDER, f"{name}.wav")

    if not os.path.isfile(path):
        print(f"לא נמצא קובץ מתאים ({name}.wav) בתיקייה {AUDIO_FOLDER}")
        return jsonify({"error": "not found", "tried": name}), 404

    print(f"משמיעה {name}.wav דרך רמקולי המחשב...")
    winsound.PlaySound(path, winsound.SND_FILENAME)
    return jsonify({"played": name})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=3001)
