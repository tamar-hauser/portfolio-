import asyncio
import os
import subprocess
import edge_tts

VOICE = "he-IL-HilaNeural"
OUT_DIR = os.path.join(os.path.dirname(__file__), "speaker_audio")

MESSAGES = [
    "מעולה, את מבצעת את זה נהדר! תמשיכי כך.",                          # 0 STABLE
    "נראה שזה קל לך - בואי נעלה קצת את הקצב.",                         # 1 TOO_EASY
    "בואי נאט את הקצב, אין ללחוץ - זה בסדר.",                          # 2 TOO_HARD
    "שימי לב ליציבות - נסי לפזר את המשקל שווה בין הרגליים.",           # 3 UNSTABLE
    "אם זה כואב, בואי נעצור לרגע. הבריאות שלך קודם.",                  # 4 DISCOMFORT
    "את עושה עבודה נהדרת. קחי כמה נשימות ותמשיכי כשמוכנה.",            # 5 TIRED
    "בסדר גמור, עוצרים כאן. כל הכבוד על האימון.",                      # 6 STOP_REQUEST
    "אין בעיה, בואי נסביר שוב לאט - עמידה ישרה, ואז ירידה מבוקרת.",     # 7 HELP_NEEDED
]

async def generate_one(index, text):
    mp3_path = os.path.join(OUT_DIR, f"category_{index}.mp3")
    wav_path = os.path.join(OUT_DIR, f"category_{index}.wav")

    communicate = edge_tts.Communicate(text, VOICE)
    await communicate.save(mp3_path)

    subprocess.run(
        ["ffmpeg", "-y", "-i", mp3_path, wav_path],
        check=True, capture_output=True,
    )
    os.remove(mp3_path)
    print(f"category_{index}.wav נוצר: {text!r}")

async def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for i, text in enumerate(MESSAGES):
        await generate_one(i, text)
    print("סיימתי - כל 8 קבצי המשוב הקולי מוכנים ב-speaker_audio/")

if __name__ == "__main__":
    asyncio.run(main())
