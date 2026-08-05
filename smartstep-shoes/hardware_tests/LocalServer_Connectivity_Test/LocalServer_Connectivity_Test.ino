#include <WiFi.h>
#include <HTTPClient.h>

const char* WIFI_SSID     = "********";
const char* WIFI_PASSWORD = "********";
const char* SERVER_TEST_URL = "http://192.168.137.1:3001/";

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.print("מתחברת לרשת WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("מחוברת! כתובת ה-IP שלי: ");
  Serial.println(WiFi.localIP());
  Serial.print("עוצמת אות (RSSI): "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  Serial.print("כתובת MAC של ה-ESP32: ");
  Serial.println(WiFi.macAddress());

  Serial.print("שולחת GET לשרת: ");
  Serial.println(SERVER_TEST_URL);

  HTTPClient http;
  http.begin(SERVER_TEST_URL);
  http.setTimeout(10000);
  int code = http.GET();

  Serial.print("קוד תשובה: ");
  Serial.println(code);

  if (code > 0) {
    Serial.print("תוכן התשובה: ");
    Serial.println(http.getString());
  } else {
    Serial.print("שגיאת HTTPClient (קוד שלילי): ");
    Serial.println(HTTPClient::errorToString(code));
  };
  http.end();
}

void loop() {
}
