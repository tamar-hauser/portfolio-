unsigned long replyTimeoutMs = 3000;
unsigned long loopDelayMs    = 2000;

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  Serial.println("PING");

  unsigned long waitStart = millis();
  bool gotReply = false;

  while (millis() - waitStart < replyTimeoutMs) {
    if (Serial.available()) {
      String reply = Serial.readStringUntil('\n');
      reply.trim();
      if (reply.length() > 0) {
        Serial.print("קיבלתי תשובה מהמחשב: ");
        Serial.println(reply);
        gotReply = true;
      }
      break;
    }
  }

  if (!gotReply) {
    Serial.println("לא התקבלה תשובה מהמחשב - בדקי שהסקריפט הפייתון רץ ומאזין לפורט הנכון.");
  }

  delay(loopDelayMs);
}
