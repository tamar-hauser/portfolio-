# SmartStep Shoes — Embedded IoT System

Wearable smart-shoe system for real-time workout analysis. A pair of instrumented
shoes streams pressure and motion data to a separate handheld "virtual coach"
device, which scores the movement against a reference exercise, counts
repetitions, and answers the trainee out loud.

> **Stack:** C++ / Arduino (ESP32, ESP32-C6), FreeRTOS, ESP-NOW, FSR pressure
> sensors, MPU-9250 IMU, I2S audio (INMP441 mic, MAX98357A amp), ST7789 LCD,
> Python / Flask, faster-whisper, React + Vite, Firebase.

---

## Architecture

```
   ┌──────────────┐        ┌──────────────┐
   │  Right shoe  │        │  Left shoe   │      FSR pressure + MPU-9250 IMU
   │   (Master)   │◄──────►│   (Slave)    │
   └──────┬───────┘ ESP-NOW└──────────────┘
          │
          │ ESP-NOW (both shoes' data, aggregated by Master)
          ▼
   ┌────────────────────────────────────────┐
   │      ESP32-C6 Virtual Coach            │   ST7789 172x320 LCD
   │                                        │
   │  taskCoach (high prio)                 │   scoring, rep counting, display
   │  taskVoice (low prio)                  │   I2S mic -> STT -> classification
   └───────┬──────────────────────┬─────────┘
           │ HTTP "play category" │ WiFi
           ▼                      ▼
   ┌────────────────┐    ┌─────────────────┐
   │ Speaker board  │    │  Firebase  ──►  │  React dashboard
   │ WAV + I2S amp  │    │                 │  (history, charts, trainer videos)
   └────────────────┘    └─────────────────┘

   Transcription: local Whisper server on the PC (server/), reached over WiFi
   or over the USB serial link. Spoken replies: WAV files pre-rendered by
   edge-tts, fetched from the same server.
```

**Two design decisions worth calling out:**

1. **The coach is a separate device, not a phone.** The shoes talk to it over
   ESP-NOW, so a workout runs with no phone, no router and no internet in the
   loop for the analysis path.
2. **Nothing in the voice path leaves the local network.** The C6 records audio
   and sends it to the Whisper server in `server/` — over WiFi or over the USB
   serial link — and gets plain text back. It then classifies that text into a
   feedback category with on-device keyword scoring: no LLM, no cloud STT, no
   server-side logic. Spoken replies are pre-rendered WAV files, so playback is
   just a file fetch. Audio output lives on a *third* board next to the speaker,
   so the C6 never has to drive an amplifier while it is scoring movement.

---

## Repository layout

| Path | What it is |
|---|---|
| `firmware/SmartStep_C6/` | **Virtual coach.** FreeRTOS tasks, ESP-NOW receive, movement scoring, reference data, LCD UI, voice pipeline. The main body of work. |
| `firmware/SmartStep_Master/` | Right shoe — reads its own sensors, receives the Slave's, forwards both. |
| `firmware/SmartStep_Slave/` | Left shoe — sensor sampling and ESP-NOW transmit. |
| `firmware/SmartStep_SpeakerReceiver/` | Speaker board — listens on ESP-NOW, fetches the matching WAV from the local server, plays it over I2S into a MAX98357A. |
| `firmware/SmartStep_C6_VoiceRecorder/` | Standalone recorder used to capture mic samples for tuning. |
| `server/` | The **offline** STT server: Flask + faster-whisper on the PC. Also renders the spoken-reply WAVs with edge-tts, and holds the serial bridges used during bring-up. |
| `app/` | React + Vite dashboard — session history, pressure maps, charts, trainer video upload, Firebase-backed. |
| `hardware_tests/` | Small single-purpose sketches written while bringing each peripheral up (mic, amp, WiFi, ESP-NOW, serial, wake-word). Kept because they document what was actually verified on hardware. |
| `tools/` | `convert_to_rgb565.py` — turns trainer video frames into the `demo_images.h` PROGMEM array the C6 displays. |

On-screen text is English because the Adafruit_GFX default font has no Hebrew
glyphs; the coach's spoken and serial output is Hebrew, which is the language the
trainee is actually addressed in.

---

## Running it

### Firmware (Arduino IDE)

Open the `.ino` file inside any `firmware/<name>/` folder. Each sketch folder is
named after its main file, as the Arduino IDE requires.

Boards: ESP32 dev boards for the shoes and the speaker; Waveshare
ESP32-C6-LCD-1.47 for the coach.
Libraries: `Adafruit_GFX`, `Adafruit_ST7789`, `ArduinoJson`.

WiFi credentials are masked as `"********"` in the sketches that need them. Fill
in your own before flashing, and do not commit them. `LOCAL_STT_SERVER` in
`SmartStep_C6.ino` also needs your PC's address on the same network.

### Local STT server

```bash
cd server
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python server.py
```

The Whisper `medium` weights (~1.5 GB) are **not** in this repository. Download
them into `server/whisper_medium_model/` — `server.py` loads from that local
folder rather than by model name, because the download was repeatedly cut off on
the network this was built on.

Voice-feedback clips in `server/speaker_audio/` are generated, not tracked:

```bash
python generate_voice_feedback.py
```

### Dashboard

```bash
cd app
npm install
cp .env.example .env    # fill in your own Firebase project keys
npm run dev
```

`app/.env` holds real credentials and is deliberately untracked — only
`.env.example` is committed.
