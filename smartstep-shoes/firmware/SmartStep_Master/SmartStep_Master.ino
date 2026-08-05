#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

#define MPU9250_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

uint8_t c6MAC[] = {0xE4, 0xB3, 0x23, 0xB2, 0x2B, 0xE4};

int wifiChannel = 1;

struct ShoeData {
  bool isRight;
  int fsr0, fsr1, fsr2;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long timestamp;
};

struct CombinedData {
  ShoeData right;
  ShoeData left;
};

CombinedData combined;
bool leftReceived = false;
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
  if (Wire.available() < 2) return 0;
  return (Wire.read() << 8) | Wire.read();
}

void onReceiveESPNOW(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&combined.left, data, sizeof(ShoeData));
  leftReceived = true;
}

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "נשלח ל-C6!" : "שליחה ל-C6 נכשלה!");
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("מתחיל...");
  analogReadResolution(12);

  Wire.begin(8, 9);
  Wire.setTimeOut(1000);
  Serial.println("Wire אותחל!");

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
    Serial.println("ממשיך בלי MPU9250...");
  }

  WiFi.mode(WIFI_STA);
  Serial.print("MAC שלי (Master) - עדכני ב-masterMAC ב-SmartStep_Slave.ino: ");
  Serial.println(WiFi.macAddress());
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  Serial.print("נעולה על ערוץ WiFi: "); Serial.println(wifiChannel);
  esp_now_init();
  esp_now_register_recv_cb(onReceiveESPNOW);
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, c6MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  combined.right.isRight = true;
  Serial.println("Master מוכן!");
}

void loop() {
  combined.right.fsr0 = analogRead(0);
  combined.right.fsr1 = analogRead(1);
  combined.right.fsr2 = analogRead(2);

  if (mpuFound) {
    combined.right.ax = readWord(MPU9250_ADDR, ACCEL_XOUT_H) / 16384.0;
    combined.right.ay = readWord(MPU9250_ADDR, ACCEL_XOUT_H + 2) / 16384.0;
    combined.right.az = readWord(MPU9250_ADDR, ACCEL_XOUT_H + 4) / 16384.0;
    combined.right.gx = readWord(MPU9250_ADDR, GYRO_XOUT_H) / 131.0;
    combined.right.gy = readWord(MPU9250_ADDR, GYRO_XOUT_H + 2) / 131.0;
    combined.right.gz = readWord(MPU9250_ADDR, GYRO_XOUT_H + 4) / 131.0;
  }
  combined.right.timestamp = millis();

  esp_now_send(c6MAC, (uint8_t*)&combined, sizeof(combined));

  Serial.println("=== נתונים ===");
  Serial.print("ימין FSR: ");
  Serial.print(combined.right.fsr0); Serial.print(" | ");
  Serial.print(combined.right.fsr1); Serial.print(" | ");
  Serial.println(combined.right.fsr2);

  if (mpuFound) {
    Serial.print("ימין תאוצה: X="); Serial.print(combined.right.ax);
    Serial.print(" Y="); Serial.print(combined.right.ay);
    Serial.print(" Z="); Serial.println(combined.right.az);

    Serial.print("ימין סיבוב: X="); Serial.print(combined.right.gx);
    Serial.print(" Y="); Serial.print(combined.right.gy);
    Serial.print(" Z="); Serial.println(combined.right.gz);
  }

  if (leftReceived) {
    Serial.print("שמאל FSR: ");
    Serial.print(combined.left.fsr0); Serial.print(" | ");
    Serial.print(combined.left.fsr1); Serial.print(" | ");
    Serial.println(combined.left.fsr2);
  }

  Serial.println("---");
  delay(500);
}
