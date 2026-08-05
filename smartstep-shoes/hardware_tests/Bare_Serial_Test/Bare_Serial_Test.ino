int counter = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== בדיקה בסיסית התחילה! ===");
}

void loop() {
  Serial.print("סופר: ");
  Serial.println(counter);
  counter++;
  delay(500);
}
