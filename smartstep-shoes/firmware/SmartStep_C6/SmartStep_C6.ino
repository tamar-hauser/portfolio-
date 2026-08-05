#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "driver/i2s_std.h"
#include <WebServer.h>
#include "demo_images.h"

enum FeedbackCategory {
  FEEDBACK_STABLE, FEEDBACK_TOO_EASY, FEEDBACK_TOO_HARD,
  FEEDBACK_UNSTABLE, FEEDBACK_DISCOMFORT,
  FEEDBACK_TIRED, FEEDBACK_STOP_REQUEST, FEEDBACK_HELP_NEEDED,
  FEEDBACK_UNCERTAIN
};
#define FEEDBACK_CATEGORY_COUNT 9
QueueHandle_t voiceFeedbackQueue;

bool voiceFeedbackBannerActive = false;
unsigned long voiceFeedbackBannerUntil = 0;
int voiceFeedbackBannerDurationMs = 4000;

volatile bool recordRequested = false;

volatile bool textInputRequested = false;
char manualTranscript[256] = "";

volatile bool serialTranscriptReceived = false;
char serialTranscriptText[256] = "";
const char* SERIAL_AUDIO_MARKER      = "SSTEP_AUDIO_BEGIN";
const char* SERIAL_TRANSCRIPT_PREFIX = "SSTEP_TRANSCRIPT:";
int serialTranscriptTimeoutMs = 30000;

int voiceActivityThreshold     = 1000000;
int voiceActivitySustainNeeded = 3;
int consecutiveVoiceDetections = 0;
bool vadEnabled = false;

int doublePressMaxGapMs = 900;

#define I2S_PORT          I2S_NUM_0
#define I2S_PIN_BCLK      18
#define I2S_PIN_WS        19
#define I2S_PIN_MIC_SD    20

uint8_t speakerReceiverMac[6] = {0xD4, 0xE9, 0xF4, 0xB3, 0x9A, 0x5C};

bool speakerReceiverConnected = true;

bool playThroughPcSpeakers = false;
const char* LOCAL_SPEAKER_SERVER = "http://192.168.100.173:3001/play";

#define AUDIO_SAMPLE_RATE_HZ   16000
#define AUDIO_RECORD_SECONDS   3
#define AUDIO_SAMPLES          (AUDIO_SAMPLE_RATE_HZ * AUDIO_RECORD_SECONDS)
#define AUDIO_BUFFER_BYTES     (AUDIO_SAMPLES * 2)
uint8_t audioBuffer[AUDIO_BUFFER_BYTES];

WebServer debugAudioServer(80);
bool hasRecording = false;

const char* FIRESTORE_PROJECT_ID = "smartstep-15b73";
String firestoreBaseUrl = String("https://firestore.googleapis.com/v1/projects/") + FIRESTORE_PROJECT_ID + "/databases/(default)/documents";
int lessonPollIntervalMs = 3000;
unsigned long lastLessonPollTime = 0;
String lastSeenLessonRequestedAt = "";

// TODO: fill in before flashing
#define WIFI_SSID          "********"
#define WIFI_PASSWORD      "********"
int sttHttpTimeoutMs = 20000;

const char* LOCAL_STT_SERVER = "http://192.168.100.173:3001/stt";

enum SttTransportMode { STT_TRANSPORT_SERIAL, STT_TRANSPORT_WIFI };
SttTransportMode sttTransportMode = STT_TRANSPORT_WIFI;

i2s_chan_handle_t micRxHandle = NULL;

int i2sReadTimeoutMs = 2000;

#define TFT_MOSI 6
#define TFT_SCLK 7
#define TFT_CS   14
#define TFT_DC   15
#define TFT_RST  21
#define TFT_BL   22

SPIClass tftSPI(FSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&tftSPI, TFT_CS, TFT_DC, TFT_RST);

#define BOOT_BUTTON_PIN 9
int bootHoldThresholdMs = 2000;

enum ScreenMode { MODE_SPLASH, MODE_STANDALONE_DEMO, MODE_APP_TRAINING, MODE_NO_CONNECTION };
ScreenMode screenMode = MODE_SPLASH;

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

struct SpeakerRequest {
  int movementProblem;
  int feedbackCategory;
};

struct ReferenceFrame {
  int r_fsr0, r_fsr1, r_fsr2;
  float r_ax, r_ay, r_az;
  float r_gx, r_gy, r_gz;
  int l_fsr0, l_fsr1, l_fsr2;
  float l_ax, l_ay, l_az;
  float l_gx, l_gy, l_gz;
};

enum ExerciseType { EX_MOUNTAIN_CLIMBER, EX_LUNGE, EX_SITUP, EX_RUNNING_IN_PLACE, EX_CALF_RAISE };

enum MovementProblem {
  MOVEMENT_OK,
  MOVEMENT_UNSTABLE,
  MOVEMENT_PRONATION,
  MOVEMENT_PRESSURE_ISSUE,
  MOVEMENT_ASYMMETRY,
  MOVEMENT_PROBLEM_COUNT
};

enum ProblemOrigin { ORIGIN_NONE, ORIGIN_CURRENT, ORIGIN_CONSISTENT, ORIGIN_PAST_EVENT };

enum VoiceState {
  VOICE_IDLE,
  VOICE_RECORDING,
  VOICE_SENDING_STT,
  VOICE_CLASSIFYING,
  VOICE_REQUESTING_PLAY
};

float calculateShoeDeviation(ShoeData &live, int r_fsr0, int r_fsr1, int r_fsr2,
                               float r_ax, float r_ay, float r_az,
                               float r_gx, float r_gy, float r_gz);
float calculateFrameScore(CombinedData &live, const ReferenceFrame &ref);
void computeDeviationComponents(ShoeData &live, int r_fsr0, int r_fsr1, int r_fsr2,
                                  float r_ax, float r_ay, float r_az,
                                  float r_gx, float r_gy, float r_gz,
                                  float* outFsr, float* outAccelXY, float* outAccelZ, float* outGyro);
MovementProblem diagnoseMovementProblem(CombinedData &live, const ReferenceFrame &ref);
void selectExercise(ExerciseType type);
void startTrainingIfConnected(ExerciseType type);
MovementProblem diagnoseProblemOrigin(MovementProblem currentProblem, ProblemOrigin* outOrigin);
const char* problemOriginPrefix(ProblemOrigin origin);
bool parseCsvReferenceLine(char* line, ReferenceFrame &out);

ExerciseType currentExercise = EX_MOUNTAIN_CLIMBER;
const int repsPerExercise = 3;

const ReferenceFrame* activeReference;
int activeReferenceCount;
const char* exerciseName = "מטפסי הרים";
const char* exerciseNameEn = "MOUNTAIN CLIMBER";
const char* exerciseFirestoreId = "mountainClimber";

CombinedData liveData;
bool dataReceived = false;

unsigned long lastShoeDataTime = 0;
int shoeConnectionTimeoutMs = 3000;

bool trainingActive = false;
unsigned long trainingStartTime = 0;
const unsigned long SAMPLE_INTERVAL_MS = 500;
unsigned long lastSampleTime = 0;

int currentFrameIndex = 0;
int currentRep = 1;
int framesPerRep = 0;

float lastScore = 0;
float repScores[10];
int repScoreIndex = 0;

void setup() {
  initDefaultActiveReference();

  Serial.begin(921600);
  delay(3000);
  WiFi.mode(WIFI_STA);
  Serial.print("MAC של C6: ");
  Serial.println(WiFi.macAddress());

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("מתחברת ל-WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.println(WiFi.status() == WL_CONNECTED ? "WiFi מחובר." : "WiFi לא התחבר (STT לא יעבוד, ESP-NOW כן).");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("עוצמת אות WiFi (RSSI): "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    Serial.print("ערוץ WiFi של ה-C6 (חייב להתאים ל-Master/Slave!): "); Serial.println(WiFi.channel());
  }
  WiFi.setSleep(false);

  if (WiFi.status() == WL_CONNECTED) {
    configTime(7200, 3600, "pool.ntp.org");
  }

  esp_now_init();
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t speakerPeer = {};
  memcpy(speakerPeer.peer_addr, speakerReceiverMac, 6);
  speakerPeer.channel = 0;
  speakerPeer.encrypt = false;
  esp_err_t peerResult = esp_now_add_peer(&speakerPeer);
  if (peerResult != ESP_OK) {
    Serial.print("אזהרה: הוספת peer לבקר הרמקול נכשלה - ");
    Serial.println(esp_err_to_name(peerResult));
  }

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tftSPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.init(172, 320);
  tft.setRotation(0);

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  drawLogoScreen();

  debugAudioServer.on("/recording.wav", handleServeRecording);
  debugAudioServer.begin();
  Serial.print(">>> להאזנה להקלטה האחרונה (אחרי שליחת v): http://");
  Serial.print(WiFi.localIP());
  Serial.println("/recording.wav");

  Serial.println("C6 מוכן! (מאמן וירטואלי)");
  Serial.println("שלח 'm' להתחלת אימון מטפסי הרים, 'l' ללאנג', 's' לסיט אפ");

  initCombinedRecommendationMatrix();

  voiceFeedbackQueue = xQueueCreate(4, sizeof(FeedbackCategory));

  xTaskCreate(taskCoach, "Coach", 8192, NULL, 3, NULL);
  xTaskCreate(taskVoice, "Voice", 8192, NULL, 1, NULL);
}

void loop() {
  debugAudioServer.handleClient();
}
