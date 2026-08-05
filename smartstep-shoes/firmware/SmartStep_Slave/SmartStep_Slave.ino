#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

#define MPU9250_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

uint8_t masterMAC[] = {0x10, 0x00, 0x3B, 0xCD, 0xF8, 0x90};

int wifiChannel = 1;

struct ShoeData {
  bool isRight;
  int fsr0, fsr1, fsr2;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long timestamp;
};

ShoeData myData;
bool mpuFound = false;

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t readWord(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);
  return (Wire.read() << 8) | Wire.read();
}

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "שליחה הצליחה" : "שליחה נכשלה");
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Slave מתחיל...");
  analogReadResolution(12);

  Wire.begin(8, 9);
  Wire.setTimeOut(1000);

  Wire.beginTransmission(MPU9250_ADDR);
  int error = Wire.endTransmission();

  if (error == 0) {
    mpuFound = true;
    Serial.println("MPU9250 נמצא!");
    writeRegister(MPU9250_ADDR, PWR_MGMT_1, 0x80);
    delay(100);
    writeRegister(MPU9250_ADDR, PWR_MGMT_1, 0x00);
    delay(100);
  } else {
    mpuFound = false;
    Serial.print("MPU9250 לא נמצא! שגיאה: ");
    Serial.println(error);
    Serial.println("ממשיכה בלי MPU9250...");
  }

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  Serial.print("נעולה על ערוץ WiFi: "); Serial.println(wifiChannel);
  esp_now_init();
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, masterMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  myData.isRight = false;
  Serial.println("Slave מוכן!");
}

void loop() {
  myData.fsr0 = analogRead(0);
  myData.fsr1 = analogRead(1);
  myData.fsr2 = analogRead(2);

  if (mpuFound) {
    myData.ax = readWord(MPU9250_ADDR, ACCEL_XOUT_H) / 16384.0;
    myData.ay = readWord(MPU9250_ADDR, ACCEL_XOUT_H + 2) / 16384.0;
    myData.az = readWord(MPU9250_ADDR, ACCEL_XOUT_H + 4) / 16384.0;
    myData.gx = readWord(MPU9250_ADDR, GYRO_XOUT_H) / 131.0;
    myData.gy = readWord(MPU9250_ADDR, GYRO_XOUT_H + 2) / 131.0;
    myData.gz = readWord(MPU9250_ADDR, GYRO_XOUT_H + 4) / 131.0;
  }
  myData.timestamp = millis();

  esp_now_send(masterMAC, (uint8_t*)&myData, sizeof(myData));

  Serial.print("FSR: "); Serial.print(myData.fsr0);
  Serial.print(" | "); Serial.print(myData.fsr1);
  Serial.print(" | "); Serial.println(myData.fsr2);

  if (mpuFound) {
    Serial.print("תאוצה: X="); Serial.print(myData.ax);
    Serial.print(" Y="); Serial.print(myData.ay);
    Serial.print(" Z="); Serial.println(myData.az);

    Serial.print("ג'ירו: X="); Serial.print(myData.gx);
    Serial.print(" Y="); Serial.print(myData.gy);
    Serial.print(" Z="); Serial.println(myData.gz);
  }

  Serial.println("---");

  delay(500);
}
