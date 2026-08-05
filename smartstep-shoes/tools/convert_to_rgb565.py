
from PIL import Image

SCREEN_W = 172
SCREEN_H = round(SCREEN_W * 1080 / 1920)

FRAMES = [
    ("MVI_1998", "frame_001.png", "running_1"),
    ("MVI_1998", "frame_002.png", "running_2"),
    ("MVI_1998", "frame_003.png", "running_3"),
    ("MVI_1998", "frame_004.png", "running_4"),
    ("MVI_2000", "frame_100.png", "calf_raise_1"),
    ("MVI_2000", "frame_101.png", "calf_raise_2"),
    ("MVI_2000", "frame_102.png", "calf_raise_3"),
    ("MVI_2000", "frame_103.png", "calf_raise_4"),
]

BASE = r"C:\Users\User\Documents\SmartStep_Codes_\TrainerVideoFrames"

def to_rgb565_bytes(img: Image.Image) -> bytes:
    out = bytearray()
    for r, g, b in img.getdata():
        val = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        out.append((val >> 8) & 0xFF)
        out.append(val & 0xFF)
    return bytes(out)

header_lines = [
    "#pragma once",
    "#include <Arduino.h>",
    "",
    "// נוצר אוטומטית ע\"י convert_to_rgb565.py מתוך פריימי סרטון המאמן",
    f"#define DEMO_IMG_W {SCREEN_W}",
    f"#define DEMO_IMG_H {SCREEN_H}",
    "",
]

for folder, filename, var_name in FRAMES:
    path = f"{BASE}\\{folder}\\{filename}"
    img = Image.open(path).convert("RGB")
    img = img.resize((SCREEN_W, SCREEN_H), Image.LANCZOS)
    data = to_rgb565_bytes(img)

    header_lines.append(f"// {folder}/{filename}")
    header_lines.append(f"const uint16_t {var_name}[] PROGMEM = {{")
    hex_bytes = [f"0x{data[i]:02X}{data[i+1]:02X}" for i in range(0, len(data), 2)]
    for i in range(0, len(hex_bytes), 12):
        header_lines.append("  " + ", ".join(hex_bytes[i:i + 12]) + ",")
    header_lines.append("};")
    header_lines.append("")

out_path = f"{BASE}\\demo_images.h"
with open(out_path, "w", encoding="utf-8") as f:
    f.write("\n".join(header_lines))

print(f"נכתב: {out_path}")
import os
print(f"גודל קובץ: {os.path.getsize(out_path) / 1024:.0f} KB")
