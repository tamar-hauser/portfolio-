
EXERCISES = [
    ("lunge", "לאנג'", 16.6166, "MVI_2067"),
    ("mountainClimber", "מטפסי הרים", 32.9996, "MVI_2063"),
    ("situp", "סיט אפ", 9.1425, "MVI_2066"),
]

COLUMNS = [
    "r_fsr0", "r_fsr1", "r_fsr2", "r_ax", "r_ay", "r_az", "r_gx", "r_gy", "r_gz",
    "l_fsr0", "l_fsr1", "l_fsr2", "l_ax", "l_ay", "l_az", "l_gx", "l_gy", "l_gz",
]

for key, label, duration, video in EXERCISES:
    sample_count = int(duration // 0.5) + 1
    lines = [
        f"# דף עבודה - {label} ({video}.mp4, אורך {duration:.1f}s)",
        "",
        f"מספר דגימות נדרש: **{sample_count}** (כל 0.5 שנייה, כולל t=0.0)",
        "",
        "שתי השניות הראשונות (t=0.0 עד t=1.5) הן עמידה ישרה לפני תחילת התרגיל -",
        "ראו הסבר ב-README_reference_data.md.",
        "",
        "מלאי כל שורה מול ה-Serial Monitor באותה שנייה בדיוק (השתמשי בסרטון",
        "כמדריך קצב בזמן שאת לובשת את הנעליים ומבצעת את התרגיל בפועל):",
        "",
        "| t(s) | " + " | ".join(COLUMNS) + " |",
        "|---" * (len(COLUMNS) + 1) + "|",
    ]
    for i in range(sample_count):
        t = i * 0.5
        blanks = " | ".join(["___"] * len(COLUMNS))
        lines.append(f"| {t:.1f} | {blanks} |")

    lines += [
        "",
        "## אחרי המילוי",
        "",
        "יש שתי דרכים להכניס את הנתונים למערכת - בחרי אחת:",
        "",
        "1. **דרך האפליקציה (מומלץ):** הפכי את הטבלה ל-CSV נקי -"
        " שורה אחת לכל דגימה, 18 מספרים מופרדים בפסיק, **בלי** עמודת t(s)"
        " ו**בלי** שורת כותרת - ותעלי אותה דרך טאב \"אזור מאמן\" באפליקציה.",
        "2. **ידני בקוד:** הכניסי את הנתונים ל-`" + key + "Reference[]` ב-"
        "`Reference_Data.ino`, שורה אחת לכל דגימה, באותו סדר עמודות.",
    ]

    out_path = f"C:\\Users\\User\\Documents\\SmartStep_Codes_\\SmartStep_C6\\reference_worksheets\\{key}_worksheet.md"
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"נכתב: {out_path} ({sample_count} דגימות)")
