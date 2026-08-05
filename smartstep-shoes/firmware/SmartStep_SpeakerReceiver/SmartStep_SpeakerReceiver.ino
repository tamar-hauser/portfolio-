#include <esp_now.h>
#include <WiFi.h>
#include "driver/i2s_std.h"

#define I2S_PORT       I2S_NUM_0
#define I2S_PIN_BCLK   27
#define I2S_PIN_WS     25
#define I2S_PIN_DOUT   26

#define AUDIO_SAMPLE_RATE_HZ 16000

i2s_chan_handle_t txHandle = NULL;

float playbackVolumeScale = 3.0;

const char* REQUEST_PREFIX     = "SSTEP_PLAY_REQUEST:";
const char* AUDIO_MARKER       = "SSTEP_AUDIO_BEGIN";
const char* NOTFOUND_MARKER    = "SSTEP_AUDIO_NOTFOUND";
int serialAudioTimeoutMs = 15000;

struct SpeakerRequest {
  int movementProblem;
  int feedbackCategory;
};

#define CATEGORY_SONG 100

volatile bool requestPending = false;
volatile SpeakerRequest pendingRequest = { -1, -1 };

void onEspNowReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len == sizeof(SpeakerRequest)) {
    memcpy((void*)&pendingRequest, data, sizeof(SpeakerRequest));
    requestPending = true;
  }
}

bool i2sConfigForSpeaker() {
  if (txHandle != NULL) {
    i2s_channel_disable(txHandle);
    i2s_del_channel(txHandle);
    txHandle = NULL;
  }

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);

  chanCfg.dma_desc_num = 8;
  chanCfg.dma_frame_num = 2000;
  esp_err_t e1 = i2s_new_channel(&chanCfg, &txHandle, NULL);

  i2s_std_config_t stdCfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_PIN_BCLK,
      .ws   = (gpio_num_t)I2S_PIN_WS,
      .dout = (gpio_num_t)I2S_PIN_DOUT,
      .din  = I2S_GPIO_UNUSED,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };
  stdCfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  esp_err_t e2 = i2s_channel_init_std_mode(txHandle, &stdCfg);
  esp_err_t e3 = i2s_channel_enable(txHandle);

  if (e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK) {
    Serial.printf("i2sConfigForSpeaker: כשל! new_channel=%s init_std=%s enable=%s\n",
      esp_err_to_name(e1), esp_err_to_name(e2), esp_err_to_name(e3));
    return false;
  }
  return true;
}

void requestAndPlayAudio(const char* name) {
  Serial.print(REQUEST_PREFIX);
  Serial.println(name);

  unsigned long waitStart = millis();
  String line;
  while (millis() - waitStart < (unsigned long)serialAudioTimeoutMs) {
    if (Serial.available()) {
      line = Serial.readStringUntil('\n');
      line.trim();
      break;
    }
    delay(10);
  }

  if (line == NOTFOUND_MARKER) {
    Serial.println("requestAndPlayAudio: המחשב לא מצא את הקובץ המבוקש");
    return;
  }
  if (line != AUDIO_MARKER) {
    Serial.println("requestAndPlayAudio: לא התקבלה תשובה תקינה מהמחשב (בדקי ש-serial_speaker_bridge.py רץ)");
    return;
  }

  uint8_t lenBytes[4];
  if (Serial.readBytes(lenBytes, 4) != 4) {
    Serial.println("requestAndPlayAudio: לא התקבל אורך תקין");
    return;
  }
  uint32_t totalLen = lenBytes[0] | (lenBytes[1] << 8) | (lenBytes[2] << 16) | ((uint32_t)lenBytes[3] << 24);

  if (!i2sConfigForSpeaker()) return;

  Serial.printf("requestAndPlayAudio: מנגנת %u בתים...\n", totalLen);

  uint8_t chunk[4096];
  uint32_t received = 0;
  unsigned long playbackStartMs = millis();
  int chunkIndex = 0;

  bool hasPendingByte = false;
  uint8_t pendingByte = 0;

  while (received < totalLen) {
    int prefixLen = hasPendingByte ? 1 : 0;
    if (hasPendingByte) {
      chunk[0] = pendingByte;
      hasPendingByte = false;
    }

    int toRead = min((uint32_t)(sizeof(chunk) - prefixLen), totalLen - received);

    unsigned long chunkReadStart = millis();
    int got = Serial.readBytes(chunk + prefixLen, toRead);
    unsigned long chunkReadMs = millis() - chunkReadStart;

    Serial.printf("  קטע %d: התבקשו %d, התקבלו %d, לקח %lums (סה\"כ עד עכשיו: %lums)\n",
      chunkIndex++, toRead, got, chunkReadMs, millis() - playbackStartMs);

    if (got <= 0) break;
    received += got;

    int bytesInChunk = prefixLen + got;
    int usableBytes = bytesInChunk;
    if (usableBytes % 2 != 0) {
      usableBytes--;
      pendingByte = chunk[usableBytes];
      hasPendingByte = true;
    }

    int16_t* samples = (int16_t*)chunk;
    int sampleCount = usableBytes / 2;
    for (int i = 0; i < sampleCount; i++) {
      float scaled = samples[i] * playbackVolumeScale;
      if (scaled > 32767) scaled = 32767;
      if (scaled < -32768) scaled = -32768;
      samples[i] = (int16_t)scaled;
    }

    size_t written;
    esp_err_t writeResult = i2s_channel_write(txHandle, chunk, usableBytes, &written, pdMS_TO_TICKS(1000));
    if (writeResult != ESP_OK || written != (size_t)usableBytes) {
      Serial.printf("requestAndPlayAudio: i2s_channel_write בעיה! result=%s written=%u/%d\n",
        esp_err_to_name(writeResult), (unsigned)written, usableBytes);
    }
  }

  i2s_channel_disable(txHandle);
  i2s_del_channel(txHandle);
  txHandle = NULL;

  if (received != totalLen) {
    Serial.printf("requestAndPlayAudio: אזהרה - התקבלו רק %u מתוך %u בתים!\n", received, totalLen);
  }
  Serial.println("requestAndPlayAudio: הניגון הסתיים");
}

void setup() {

  Serial.setRxBufferSize(4096);

  Serial.begin(460800);

  Serial.setTimeout(5000);

  delay(1500);

  WiFi.mode(WIFI_STA);
  delay(100);
  Serial.print("MAC של בקר הרמקול (למלא ב-SmartStep_C6 כדי לשלוח אליו): ");
  Serial.println(WiFi.macAddress());

  esp_now_init();
  esp_now_register_recv_cb(onEspNowReceive);

  Serial.println("בקר הרמקול מוכן, ממתין לבקשות מה-C6 (ESP-NOW)...");

  Serial.println("בדיקה עצמית: מבקשת ומנגנת test.wav מהשרת...");
  requestAndPlayAudio("test");
}

void loop() {
  if (requestPending) {
    SpeakerRequest req;
    req.movementProblem = pendingRequest.movementProblem;
    req.feedbackCategory = pendingRequest.feedbackCategory;
    requestPending = false;

    if (req.feedbackCategory == CATEGORY_SONG) {
      requestAndPlayAudio("song");
    } else {
      char name[24];
      snprintf(name, sizeof(name), "combo_%d_%d", req.movementProblem, req.feedbackCategory);
      requestAndPlayAudio(name);
    }
  }
  delay(20);
}
