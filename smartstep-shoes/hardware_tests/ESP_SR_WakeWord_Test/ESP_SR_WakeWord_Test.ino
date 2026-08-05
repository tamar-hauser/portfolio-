#include <Arduino.h>
#include "ESP_I2S.h"
#include "ESP_SR.h"

#define I2S_PIN_BCK 18
#define I2S_PIN_WS  19
#define I2S_PIN_DIN 20

#define I2S_SAMPLE_RATE 16000

#define SR_INPUT_FORMAT     "M"
#define SR_INPUT_CHANNELS   SR_CHANNELS_MONO
#define I2S_OUTPUT_CHANNELS I2S_SLOT_MODE_MONO

I2SClass i2s;

void onSrEvent(sr_event_t event, int command_id, int phrase_id) {
  switch (event) {
    case SR_EVENT_WAKEWORD:
      Serial.println(">>> מילת ההפעלה זוהתה! (Hi, ESP) <<<");
      break;
    case SR_EVENT_WAKEWORD_CHANNEL:
      Serial.printf("WakeWord Channel %d Verified!\n", command_id);
      break;
    default:
      Serial.println("אירוע לא צפוי");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("מאתחלת I2S + ESP_SR (ניסוי מבודד)...");

  i2s.setTimeout(1000);
  i2s.setPins(I2S_PIN_BCK, I2S_PIN_WS, -1, I2S_PIN_DIN);
  i2s.begin(I2S_MODE_STD, I2S_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_OUTPUT_CHANNELS, I2S_STD_SLOT_LEFT);
  i2s.configureRX(I2S_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_OUTPUT_CHANNELS, I2S_RX_TRANSFORM_32_TO_16);

  ESP_SR.onEvent(onSrEvent);
  bool started = ESP_SR.begin(i2s, NULL, 0, SR_INPUT_CHANNELS, SR_MODE_WAKEWORD, SR_INPUT_FORMAT);

  if (started) {
    Serial.println("ESP_SR מוכן! תגידי 'Hi, ESP' ותראי מה קורה.");
  } else {
    Serial.println("ESP_SR.begin() נכשל - יש לבדוק הגדרות (partition scheme וכו').");
  }
}

void loop() {
  delay(100);
}
