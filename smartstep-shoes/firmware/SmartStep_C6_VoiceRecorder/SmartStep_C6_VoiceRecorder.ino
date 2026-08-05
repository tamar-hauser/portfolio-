#include <SPI.h>

#define ISD_PIN_SS    4
#define ISD_PIN_SCLK  2
#define ISD_PIN_MOSI  5
#define ISD_PIN_MISO  3

SPIClass isdSPI(HSPI);

struct VoiceMessage {
  const char* name;
  uint32_t addr;
  const char* text;
};

VoiceMessage MESSAGES[] = {
  {"STABLE (יציב/טוב)",       0x000000, "מעולה, את מבצעת את זה נהדר! תמשיכי כך."},
  {"TOO_EASY (קל מדי)",       0x002000, "נראה שזה קל לך - בואי נעלה קצת את הקצב."},
  {"TOO_HARD (קשה מדי)",      0x004000, "בואי נאט את הקצב, אין ללחוץ - זה בסדר."},
  {"UNSTABLE (לא יציב)",      0x006000, "שימי לב ליציבות - נסי לפזר את המשקל שווה בין הרגליים."},
  {"DISCOMFORT (כואב)",       0x008000, "אם זה כואב, בואי נעצור לרגע. הבריאות שלך קודם."},
  {"TIRED (עייף)",            0x00A000, "את עושה עבודה נהדרת. קחי כמה נשימות ותמשיכי כשמוכנה."},
  {"STOP_REQUEST (לעצור)",    0x00C000, "בסדר גמור, עוצרים כאן. כל הכבוד על האימון."},
  {"HELP_NEEDED (עזרה)",      0x00E000, "אין בעיה, בואי נסביר שוב לאט - עמידה ישרה, ואז ירידה מבוקרת."},
};
const int MESSAGES_COUNT = sizeof(MESSAGES) / sizeof(VoiceMessage);

void isdSelect()   { digitalWrite(ISD_PIN_SS, LOW); }
void isdDeselect() { digitalWrite(ISD_PIN_SS, HIGH); }

void isd4004Init() {
  pinMode(ISD_PIN_SS, OUTPUT);
  isdDeselect();
  isdSPI.begin(ISD_PIN_SCLK, ISD_PIN_MISO, ISD_PIN_MOSI, ISD_PIN_SS);
}

void isd4004PowerUp() {
  isdSPI.beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE0));
  isdSelect();
  isdSPI.transfer(0x20); // powerup
  isdDeselect();
  isdSPI.endTransaction();
  delay(50); // Tpud
}

void isd4004StartOp(uint8_t setOpcode, uint8_t goOpcode, uint32_t addr24) {
  isdSPI.beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE0));
  isdSelect();
  isdSPI.transfer(setOpcode | ((addr24 >> 16) & 0x07));
  isdSPI.transfer((addr24 >> 8) & 0xFF);
  isdSPI.transfer(addr24 & 0xFF);
  isdDeselect();
  delayMicroseconds(50);
  isdSelect();
  isdSPI.transfer(goOpcode);
  isdDeselect();
  isdSPI.endTransaction();
}

void isd4004Record(uint32_t addr24) {
  isd4004PowerUp();
  isd4004StartOp(0xA0, 0xB0, addr24); // setrecord / rec
}

void isd4004Play(uint32_t addr24) {
  isd4004PowerUp();
  isd4004StartOp(0xE0, 0xF0, addr24); // setplay / play
}

void isd4004Stop() {
  isdSPI.beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE0));
  isdSelect();
  isdSPI.transfer(0x00);
  isdDeselect();
  isdSPI.endTransaction();
}

enum RecorderState { WAIT_SELECT, WAIT_START, RECORDING, DONE_RECORDING };
RecorderState state = WAIT_SELECT;
int selectedIndex = -1;

void printMenu() {
  Serial.println();
  Serial.println("=== תפריט הקלטת הודעות ISD4004 ===");
  for (int i = 0; i < MESSAGES_COUNT; i++) {
    Serial.print(i); Serial.print(") "); Serial.println(MESSAGES[i].name);
  }
  Serial.println("הקלידי מספר (0-7) לבחירת הודעה להקלטה.");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  isd4004Init();
  Serial.println("ISD4004 Voice Recorder מוכן.");
  printMenu();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r') {
    if (state == WAIT_START) {
      Serial.println(">>> מקליטה... דברי עכשיו! לחצי Enter שוב כשסיימת. <<<");
      isd4004Record(MESSAGES[selectedIndex].addr);
      state = RECORDING;
    } else if (state == RECORDING) {
      isd4004Stop();
      Serial.println("--- ההקלטה נעצרה. ---");
      Serial.println("לחצי 'p' לניגון חזרה לבדיקה, או הקלידי מספר (0-7) להודעה הבאה.");
      state = DONE_RECORDING;
    }
    return;
  }

  if (state == WAIT_SELECT || state == DONE_RECORDING) {
    if (c >= '0' && c <= '9') {
      int idx = c - '0';
      if (idx < MESSAGES_COUNT) {
        selectedIndex = idx;
        Serial.println();
        Serial.print("נבחרה הודעה: "); Serial.println(MESSAGES[idx].name);
        Serial.print("טקסט לומר: \""); Serial.print(MESSAGES[idx].text); Serial.println("\"");
        Serial.println("היכוני, ולחצי Enter כשאת מוכנה להתחיל לדבר.");
        state = WAIT_START;
      }
    } else if (c == 'p' && selectedIndex >= 0) {
      Serial.print("מנגנת בחזרה: "); Serial.println(MESSAGES[selectedIndex].name);
      isd4004Play(MESSAGES[selectedIndex].addr);
      state = DONE_RECORDING;
    } else if (c == 'm') {
      printMenu();
      state = WAIT_SELECT;
    }
  }
}
