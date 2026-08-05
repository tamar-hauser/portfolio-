const char* VOICE_FEEDBACK_MESSAGES[] = {
  "מעולה, את מבצעת את זה נהדר! תמשיכי כך.",
  "נראה שזה קל לך - בואי נעלה קצת את הקצב.",
  "בואי נאט את הקצב, אין ללחוץ - זה בסדר.",
  "שימי לב ליציבות - נסי לפזר את המשקל שווה בין הרגליים.",
  "אם זה כואב, בואי נעצור לרגע. הבריאות שלך קודם.",
  "את עושה עבודה נהדרת. קחי כמה נשימות ותמשיכי כשמוכנה.",
  "בסדר גמור, עוצרים כאן. כל הכבוד על האימון.",
  "אין בעיה, בואי נסביר שוב לאט - עמידה ישרה, ואז ירידה מבוקרת.",
};

bool i2sConfigForMic() {
  if (micRxHandle != NULL) {
    i2s_channel_disable(micRxHandle);
    i2s_del_channel(micRxHandle);
    micRxHandle = NULL;
  }

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
  esp_err_t e1 = i2s_new_channel(&chanCfg, NULL, &micRxHandle);

  i2s_std_config_t stdCfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_PIN_BCLK,
      .ws   = (gpio_num_t)I2S_PIN_WS,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)I2S_PIN_MIC_SD,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv   = false,
      },
    },
  };
  stdCfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  esp_err_t e2 = i2s_channel_init_std_mode(micRxHandle, &stdCfg);
  esp_err_t e3 = i2s_channel_enable(micRxHandle);

  if (e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK) {
    Serial.printf("i2sConfigForMic: כשל! new_channel=%s init_std=%s enable=%s\n",
      esp_err_to_name(e1), esp_err_to_name(e2), esp_err_to_name(e3));
    return false;
  }
  Serial.println("i2sConfigForMic: הערוץ הוקם והופעל בהצלחה, מתחילים לקרוא...");
  return true;
}

bool recordFromI2SMic(uint8_t* buffer, int numSamples) {
  if (!i2sConfigForMic()) return false;

  int32_t raw32[64];
  size_t bytesRead;
  int samplesWritten = 0;
  bool success = true;

  Serial.println("recordFromI2SMic: לפני קריאת ה-chunk הראשון...");

  while (samplesWritten < numSamples) {
    int toRead = min(64, numSamples - samplesWritten);
    esp_err_t r = i2s_channel_read(micRxHandle, raw32, toRead * sizeof(int32_t),
                                    &bytesRead, pdMS_TO_TICKS(i2sReadTimeoutMs));
    if (r != ESP_OK || bytesRead == 0) {
      Serial.printf("recordFromI2SMic: אין דאטה מהמיקרופון (%s) - בדקי חיווט INMP441 לפינים %d/%d/%d\n",
        esp_err_to_name(r), I2S_PIN_BCLK, I2S_PIN_WS, I2S_PIN_MIC_SD);
      success = false;
      break;
    }
    int gotSamples = bytesRead / sizeof(int32_t);
    for (int i = 0; i < gotSamples && samplesWritten < numSamples; i++) {
      int16_t pcm16 = (int16_t)(raw32[i] >> 14);
      buffer[samplesWritten * 2]     = pcm16 & 0xFF;
      buffer[samplesWritten * 2 + 1] = (pcm16 >> 8) & 0xFF;
      samplesWritten++;
    }
  }

  i2s_channel_disable(micRxHandle);
  i2s_del_channel(micRxHandle);
  micRxHandle = NULL;
  return success;
}

bool checkVoiceActivity() {
  if (!i2sConfigForMic()) return false;

  const int vadSamples = 800;
  int32_t raw32[vadSamples];
  size_t bytesRead;
  esp_err_t r = i2s_channel_read(micRxHandle, raw32, vadSamples * sizeof(int32_t),
                                  &bytesRead, pdMS_TO_TICKS(i2sReadTimeoutMs));

  i2s_channel_disable(micRxHandle);
  i2s_del_channel(micRxHandle);
  micRxHandle = NULL;

  if (r != ESP_OK || bytesRead == 0) return false;

  int samples = bytesRead / sizeof(int32_t);
  int64_t sum = 0;
  for (int i = 0; i < samples; i++) sum += (raw32[i] >> 8);
  int32_t dcOffset = samples > 0 ? (sum / samples) : 0;

  int32_t minVal = INT32_MAX, maxVal = INT32_MIN;
  for (int i = 0; i < samples; i++) {
    int32_t sample = (raw32[i] >> 8) - dcOffset;
    if (sample < minVal) minVal = sample;
    if (sample > maxVal) maxVal = sample;
  }
  int32_t peakToPeak = (samples > 0) ? (maxVal - minVal) : 0;

  return peakToPeak > voiceActivityThreshold;
}

void requestSpeakerPlay(MovementProblem movementProblem, FeedbackCategory fb) {
  SpeakerRequest req = { (int)movementProblem, (int)fb };
  esp_err_t result = esp_now_send(speakerReceiverMac, (uint8_t*)&req, sizeof(req));
  if (result != ESP_OK) {
    Serial.print("requestSpeakerPlay: שליחת ESP-NOW נכשלה, קוד "); Serial.println(esp_err_to_name(result));
  }
}

void requestSpeakerPlayViaPc(MovementProblem movementProblem, FeedbackCategory fb) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("requestSpeakerPlayViaPc: אין WiFi - לא ניתן לבקש ניגון מהמחשב");
    return;
  }
  StaticJsonDocument<128> body;
  body["movementProblem"] = (int)movementProblem;
  body["feedbackCategory"] = (int)fb;
  String bodyStr;
  serializeJson(body, bodyStr);

  HTTPClient http;
  http.begin(LOCAL_SPEAKER_SERVER);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Content-Length", String(bodyStr.length()));
  int code = http.POST(bodyStr);
  if (code != 200) {
    Serial.print("requestSpeakerPlayViaPc: שרת המחשב החזיר שגיאה "); Serial.println(code);
  }
  http.end();
}

struct Keyword { const char* word; FeedbackCategory category; int weight; };
Keyword KEYWORDS[] = {
  {"יציב",         FEEDBACK_STABLE,     2},
  {"יציבה",        FEEDBACK_STABLE,     2},
  {"נוח",          FEEDBACK_STABLE,     2},
  {"טוב",          FEEDBACK_STABLE,     3},
  {"מעולה",        FEEDBACK_STABLE,     3},
  {"בטוחה",        FEEDBACK_STABLE,     2},

  {"קל מדי",       FEEDBACK_TOO_EASY,   2},
  {"פשוט",         FEEDBACK_TOO_EASY,   2},
  {"קל לי",        FEEDBACK_TOO_EASY,   2},
  {"משעמם",        FEEDBACK_TOO_EASY,   2},

  {"קשה",          FEEDBACK_TOO_HARD,   2},
  {"קשה מדי",      FEEDBACK_TOO_HARD,   3},
  {"כבד עליי",     FEEDBACK_TOO_HARD,   2},
  {"מתקשה",        FEEDBACK_TOO_HARD,   2},
  {"לא מסוגלת",    FEEDBACK_TOO_HARD,   3},

  {"לא יציב",      FEEDBACK_UNSTABLE,   3},
  {"מתנדנד",       FEEDBACK_UNSTABLE,   3},
  {"מאבדת שיווי משקל", FEEDBACK_UNSTABLE, 3},
  {"רועדת",        FEEDBACK_UNSTABLE,   2},

  {"כואב",         FEEDBACK_DISCOMFORT, 3},
  {"לא נוח",       FEEDBACK_DISCOMFORT, 2},
  {"כואב לי",      FEEDBACK_DISCOMFORT, 3},
  {"לוחץ לי",      FEEDBACK_DISCOMFORT, 2},

  {"עייף",         FEEDBACK_TIRED,        2},
  {"עייפה",        FEEDBACK_TIRED,        2},
  {"אין לי כוח",   FEEDBACK_TIRED,        3},
  {"מותשת",        FEEDBACK_TIRED,        3},
  {"חלשה",         FEEDBACK_TIRED,        2},

  {"לעצור",        FEEDBACK_STOP_REQUEST, 3},
  {"די",           FEEDBACK_STOP_REQUEST, 2},
  {"הפסקה",        FEEDBACK_STOP_REQUEST, 2},
  {"מספיק",        FEEDBACK_STOP_REQUEST, 2},
  {"רוצה לעצור",   FEEDBACK_STOP_REQUEST, 3},

  {"עזרה",         FEEDBACK_HELP_NEEDED,  3},
  {"לא מבינה",     FEEDBACK_HELP_NEEDED,  3},
  {"מה עושים",     FEEDBACK_HELP_NEEDED,  2},
  {"איך עושים את זה", FEEDBACK_HELP_NEEDED, 2},
  {"תסבירי שוב",   FEEDBACK_HELP_NEEDED,  2},
};
const int KEYWORDS_COUNT = sizeof(KEYWORDS) / sizeof(Keyword);

const int NEGATION_LOOKBACK_BYTES = 20;

bool isNegatedBefore(const char* fullText, const char* matchPos) {
  int lookback = matchPos - fullText;
  if (lookback > NEGATION_LOOKBACK_BYTES) lookback = NEGATION_LOOKBACK_BYTES;

  char window[NEGATION_LOOKBACK_BYTES + 1];
  memcpy(window, matchPos - lookback, lookback);
  window[lookback] = '\0';

  return (strstr(window, "לא") != nullptr) || (strstr(window, "אין") != nullptr);
}

bool sendAudioToLocalServer(uint8_t* buffer, int length, char* resultText, int maxLen) {
  if (WiFi.status() != WL_CONNECTED || length == 0) return false;

  HTTPClient http;
  http.begin(LOCAL_STT_SERVER);
  http.setTimeout(sttHttpTimeoutMs);
  http.addHeader("Content-Type", "application/octet-stream");
  int code = http.POST(buffer, length);
  if (code != 200) {
    Serial.print("שרת התמלול המקומי החזיר שגיאה: "); Serial.println(code);
    http.end();
    return false;
  }

  StaticJsonDocument<512> resp;
  deserializeJson(resp, http.getString());
  http.end();

  const char* t = resp["transcript"];
  if (!t || strlen(t) == 0) return false;
  strncpy(resultText, t, maxLen - 1);
  resultText[maxLen - 1] = '\0';
  return true;
}

bool sendAudioOverSerial(uint8_t* buffer, int length, char* resultText, int maxLen) {
  serialTranscriptReceived = false;
  Serial.println(SERIAL_AUDIO_MARKER);
  uint32_t len32 = (uint32_t)length;
  Serial.write((uint8_t*)&len32, sizeof(len32));
  Serial.write(buffer, length);
  Serial.flush();

  unsigned long waitStart = millis();
  while (millis() - waitStart < (unsigned long)serialTranscriptTimeoutMs) {
    if (serialTranscriptReceived) {
      strncpy(resultText, serialTranscriptText, maxLen - 1);
      resultText[maxLen - 1] = '\0';
      serialTranscriptReceived = false;
      return strlen(resultText) > 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  Serial.println("sendAudioOverSerial: לא התקבל תמלול מהמחשב בזמן - ודאי ש-serial_stt_bridge.py רץ ומאזין לפורט הנכון");
  return false;
}

FeedbackCategory classifySTTText(const char* text) {
  int scores[FEEDBACK_CATEGORY_COUNT] = {0};

  for (int i = 0; i < KEYWORDS_COUNT; i++) {
    const char* matchPos = strstr(text, KEYWORDS[i].word);
    if (matchPos != nullptr && !isNegatedBefore(text, matchPos)) {
      scores[(int)KEYWORDS[i].category] += KEYWORDS[i].weight;
    }
  }

  int bestScore = 0;
  FeedbackCategory best = FEEDBACK_UNCERTAIN;
  for (int cat = 0; cat < FEEDBACK_CATEGORY_COUNT - 1; cat++) {
    if (scores[cat] > bestScore) {
      bestScore = scores[cat];
      best = (FeedbackCategory)cat;
    }
  }
  return (bestScore == 0) ? FEEDBACK_UNCERTAIN : best;
}

VoiceState voiceState = VOICE_IDLE;

void taskVoice(void* pvParameters) {
  static char transcript[256];
  static FeedbackCategory currentFeedback;
  static bool sttSucceeded;
  static bool recordingSucceeded;

  for (;;) {
    switch (voiceState) {

      case VOICE_IDLE:
        if (recordRequested) {
          recordRequested = false;
          consecutiveVoiceDetections = 0;
          Serial.println("taskVoice: מקליטה...");
          voiceState = VOICE_RECORDING;
        } else if (textInputRequested) {
          textInputRequested = false;
          consecutiveVoiceDetections = 0;
          strncpy(transcript, manualTranscript, sizeof(transcript) - 1);
          transcript[sizeof(transcript) - 1] = '\0';
          Serial.println("taskVoice: מסווגת מלל ידני (בלי הקלטה/STT)...");
          voiceState = VOICE_CLASSIFYING;
        } else if (vadEnabled && checkVoiceActivity()) {
          consecutiveVoiceDetections++;
          if (consecutiveVoiceDetections >= voiceActivitySustainNeeded) {
            consecutiveVoiceDetections = 0;
            Serial.println("taskVoice: זוהה קול (VAD) - מתחילה הקלטה אוטומטית...");
            voiceState = VOICE_RECORDING;
          }
        } else {
          consecutiveVoiceDetections = 0;
        }
        break;

      case VOICE_RECORDING:
        recordingSucceeded = recordFromI2SMic(audioBuffer, AUDIO_SAMPLES);
        hasRecording = recordingSucceeded;
        if (recordingSucceeded) {
          Serial.println("taskVoice: הקלטה הסתיימה, שולחת לשרת התמלול...");
        }
        voiceState = recordingSucceeded ? VOICE_SENDING_STT : VOICE_IDLE;
        break;

      case VOICE_SENDING_STT:
        switch (sttTransportMode) {
          case STT_TRANSPORT_SERIAL:
            sttSucceeded = sendAudioOverSerial(audioBuffer, AUDIO_BUFFER_BYTES, transcript, sizeof(transcript));
            break;
          case STT_TRANSPORT_WIFI:
            sttSucceeded = sendAudioToLocalServer(audioBuffer, AUDIO_BUFFER_BYTES, transcript, sizeof(transcript));
            break;
        }
        if (sttSucceeded) {
          Serial.print("Transcript: "); Serial.println(transcript);
        } else {
          Serial.println("taskVoice: שליחה ל-STT נכשלה (בדקי WiFi/שרת).");
        }
        voiceState = sttSucceeded ? VOICE_CLASSIFYING : VOICE_IDLE;
        break;

      case VOICE_CLASSIFYING:
        currentFeedback = classifySTTText(transcript);
        xQueueSend(voiceFeedbackQueue, &currentFeedback, pdMS_TO_TICKS(50));
        voiceState = (currentFeedback != FEEDBACK_UNCERTAIN) ? VOICE_REQUESTING_PLAY : VOICE_IDLE;
        break;

      case VOICE_REQUESTING_PLAY: {
        MovementProblem currentMovementProblem = MOVEMENT_OK;
        if (trainingActive && currentFrameIndex < activeReferenceCount) {
          currentMovementProblem = diagnoseMovementProblem(liveData, activeReference[currentFrameIndex]);
        }

        ProblemOrigin origin;
        MovementProblem movementProblem = diagnoseProblemOrigin(currentMovementProblem, &origin);

        Serial.print("תגובת המאמן הווירטואלי (הצלבת תנועה+קול): ");
        Serial.print(problemOriginPrefix(origin));
        Serial.println(COMBINED_RECOMMENDATION_MATRIX[movementProblem][currentFeedback].message);

        if (playThroughPcSpeakers) {
          Serial.print("taskVoice: מבקשת מהמחשב לנגן שילוב תנועה="); Serial.print((int)movementProblem);
          Serial.print(" קול="); Serial.println((int)currentFeedback);
          requestSpeakerPlayViaPc(movementProblem, currentFeedback);
        } else if (speakerReceiverConnected) {
          Serial.print("taskVoice: מבקשת מבקר הרמקול לנגן שילוב תנועה="); Serial.print((int)movementProblem);
          Serial.print(" קול="); Serial.println((int)currentFeedback);
          requestSpeakerPlay(movementProblem, currentFeedback);
        }
        voiceState = VOICE_IDLE;
        break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
