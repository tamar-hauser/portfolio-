#include <WiFi.h>
#include <HTTPClient.h>

const char* WIFI_SSID     = "********";
const char* WIFI_PASSWORD = "********";
const char* FRIEND_PC_IP  = "192.168.0.132";
const int   FRIEND_PC_PORT = 8000;

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
  Serial.print("כתובת MAC של הבקר: ");
  Serial.println(WiFi.macAddress());

  String url = "http://" + String(FRIEND_PC_IP) + ":" + String(FRIEND_PC_PORT) + "/";
  Serial.print("שולחת GET לשרת: ");
  Serial.println(url);

  HTTPClient http;
  http.begin(url);
  http.setTimeout(10000);
  int code = http.GET();

  Serial.print("קוד תשובה: ");
  Serial.println(code);

  if (code > 0) {
    Serial.println(">>> הצליח! זה מוכיח שהבעיה היא סביבתית (הרשת/המחשב שלי) ולא בקוד או בבקר <<<");
  } else {
    Serial.print("שגיאת HTTPClient (קוד שלילי): ");
    Serial.println(HTTPClient::errorToString(code));
  }
  http.end();
}

void loop() {
}

