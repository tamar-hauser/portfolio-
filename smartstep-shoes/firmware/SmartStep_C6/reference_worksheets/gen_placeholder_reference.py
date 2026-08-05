
import math

BASELINE = (2000, 1800, 1900, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)

def frame_line(vals):
    return "  {" + ", ".join(f"{v:.2f}" if isinstance(v, float) else str(v) for v in vals) + "},"

def lunge_frames(n):
    lines = []
    baseline_n = 4
    for i in range(baseline_n):
        lines.append(frame_line((*BASELINE, *BASELINE)))
    cycle_len = n - baseline_n
    for i in range(cycle_len):
        phase = (i % 10) / 10.0
        depth = math.sin(phase * math.pi)
        r_fsr = (int(2000 + 1000 * depth), int(1800 - 600 * depth), int(1900 + 1400 * depth))
        l_fsr = (int(2000 - 500 * depth), int(1800 + 1500 * depth), int(1900 - 600 * depth))
        r_acc = (round(0.3 * depth, 2), round(-0.7 * depth, 2), round(1.0 - 0.25 * depth, 2))
        l_acc = (round(-0.3 * depth, 2), round(-0.7 * depth, 2), round(1.0 - 0.25 * depth, 2))
        r_gyro = (round(10.0 * depth, 1), round(6.0 * depth, 1), round(2.0 * depth, 1))
        l_gyro = (round(-10.0 * depth, 1), round(6.0 * depth, 1), round(-2.0 * depth, 1))
        lines.append(frame_line((*r_fsr, *r_acc, *r_gyro, *l_fsr, *l_acc, *l_gyro)))
    return lines

def mountain_climber_frames(n):
    lines = []
    baseline_n = 4
    for i in range(baseline_n):
        lines.append(frame_line((*BASELINE, *BASELINE)))
    cycle_len = n - baseline_n
    for i in range(cycle_len):
        right_bears_weight = (i % 2 == 0)
        heavy = (2700, 3000, 2900, 0.15, -0.15, 0.75, 3.0, 2.0, 1.0)
        light = (300, 350, 320, 0.35, 0.25, 0.55, 18.0, 6.0, 3.0)
        if right_bears_weight:
            lines.append(frame_line((*heavy, *light)))
        else:
            lines.append(frame_line((*light, *heavy)))
    return lines

def situp_frames(n):
    lines = []
    baseline_n = 4
    for i in range(baseline_n):
        lines.append(frame_line((*BASELINE, *BASELINE)))
    cycle_len = n - baseline_n
    for i in range(cycle_len):
        phase = (i % 6) / 6.0
        crunch = math.sin(phase * math.pi)
        fsr = (int(2400 + 200 * crunch), int(2400 + 200 * crunch), int(2400 + 200 * crunch))
        acc = (0.0, round(-0.1 * crunch, 2), round(1.0 - 0.05 * crunch, 2))
        gyro = (0.0, 0.0, 0.0)
        lines.append(frame_line((*fsr, *acc, *gyro, *fsr, *acc, *gyro)))
    return lines

EXERCISES = [
    ("lungeReference", 34, lunge_frames),
    ("mountainClimberReference", 66, mountain_climber_frames),
    ("situpReference", 19, situp_frames),
]

out_lines = []
for var_name, count, fn in EXERCISES:
    out_lines.append(f"// --- {var_name} ({count} דגימות, PLACEHOLDER) ---")
    out_lines.append(f"const ReferenceFrame {var_name}[] = {{")
    out_lines.extend(fn(count))
    out_lines.append("};")
    out_lines.append(f"const int {var_name}Count = sizeof({var_name}) / sizeof(ReferenceFrame);")
    out_lines.append("")

with open("generated_output.txt", "w", encoding="utf-8") as f:
    f.write("\n".join(out_lines))
print("done, wrote generated_output.txt with explicit utf-8 encoding")
