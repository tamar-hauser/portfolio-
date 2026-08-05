
#define MIC_ADC_PIN 0

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("בדיקת מיקרופון אנלוגי מוכנה.");
  Serial.println("דברי / מחאי כפיים ליד המיקרופון וצפי במספרים למטה.");
}

void loop() {
  int minVal = 4095, maxVal = 0;
  for (int i = 0; i < 300; i++) {
    int v = analogRead(MIC_ADC_PIN);
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
    delayMicroseconds(50);
  }

  int peakToPeak = maxVal - minVal;
  Serial.print("ADC peak-to-peak: ");
  Serial.print(peakToPeak);
  Serial.print("   (min=" ); Serial.print(minVal);
  Serial.print(" max="); Serial.print(maxVal); Serial.println(")");

  delay(100);
}
