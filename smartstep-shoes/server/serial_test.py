import serial
import time

SERIAL_PORT = "COM19"
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

print(f"מאזינה לפורט {SERIAL_PORT} במהירות {BAUD_RATE}...")

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        print(f"התקבל מה-ESP32: {line}")
        if line == "PING":
            ser.write(b"PONG\n")
            print("שלחתי PONG בחזרה")
