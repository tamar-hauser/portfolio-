#include <SPI.h>

#define PIN_SS   4
#define PIN_SCLK 2
#define PIN_MOSI 5
#define PIN_MISO 3
#define PIN_RAC  1
#define PIN_INT  0

SPIClass isdSPI(HSPI);
int lastIntState = -1;
int lastRacState = -1;

void isdSelect()   { digitalWrite(PIN_SS, LOW); }
void isdDeselect() { digitalWrite(PIN_SS, HIGH); }

void isdPowerUp() {
  isdSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  isdSelect();
  isdSPI.transfer(0b00100000); // POWERUP = 00100XXX
  isdDeselect();
  isdSPI.endTransaction();
  delay(50); // TPUD
}

void isdSetOp(uint8_t opcode, uint16_t addr) {
  isdSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  isdSelect();
  isdSPI.transfer(opcode);
  isdSPI.transfer((addr >> 8) & 0xFF);
  isdSPI.transfer(addr & 0xFF);
  isdDeselect();
  isdSPI.endTransaction();
}

void isdGoOp(uint8_t opcode) {
  isdSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  isdSelect();
  isdSPI.transfer(opcode);
  isdDeselect();
  isdSPI.endTransaction();
}

void doRecord(uint16_t addr) {
  Serial.println(">>> Record: PowerUp -> SetRec -> Rec");
  isdPowerUp();
  isdSetOp(0b10100000, addr); // SETREC = 10100XXX
  isdGoOp(0b10110000);        // REC    = 10110XXX
}

void doPlay(uint16_t addr) {
  Serial.println(">>> Play: PowerUp -> SetPlay -> Play");
  isdPowerUp();
  isdSetOp(0b11100000, addr); // SETPLAY = 11100XXX
  isdGoOp(0b11110000);        // PLAY    = 11110XXX
}

void doStop() {
  isdGoOp(0b00010000);
}

void printMenu() {
  Serial.println();
  Serial.println("=== ניסוי 3 (פרוטוקול רשמי) ===");
  Serial.println("1 = Record מכתובת 0 (3 שניות)");
  Serial.println("3 = Play מכתובת 0");
  Serial.println("m = תפריט שוב");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_SS, OUTPUT);
  pinMode(PIN_RAC, INPUT);
  pinMode(PIN_INT, INPUT_PULLUP);

  isdDeselect();
  isdSPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_SS);

  printMenu();
}

void loop() {
  int intState = digitalRead(PIN_INT);
  if (intState != lastIntState) {
    Serial.print("[INT -> ] "); Serial.println(intState);
    lastIntState = intState;
  }

  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 2000) {
    lastHeartbeat = millis();
    Serial.println("(ממתינה לפקודה - הקלידי 1 או 3 ולחצי Enter/Send)");
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      doRecord(0);
      Serial.println("מקליטה... דברי עכשיו!");
      delay(3000);
      doStop();
      Serial.println(">>> הסתיים.");
    } else if (c == '3') {
      doPlay(0);
      Serial.println("מנגנת... ממתינה עד 10 שניות ל-INT להשתנות לבד (EOM)...");
      unsigned long start = millis();
      int startInt = digitalRead(PIN_INT);
      while (millis() - start < 10000) {
        int now = digitalRead(PIN_INT);
        if (now != lastIntState) {
          Serial.print("[INT -> ] "); Serial.println(now);
          lastIntState = now;
        }
        if (now != startInt) {
          Serial.println(">>> INT השתנה לבד! כנראה הגיע ל-EOM - סימן טוב!");
          break;
        }
      }
      doStop();
      Serial.println(">>> הסתיים.");
    } else if (c == 'm') {
      printMenu();
    }
  }
}
