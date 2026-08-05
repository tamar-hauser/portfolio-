void taskCoach(void* pvParameters) {
  static unsigned long bootPressStartTime = 0;
  static bool bootButtonWasPressed = false;
  static unsigned long lastButtonPressEdgeTime = 0;

  for (;;) {
    bool bootPressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);

    if (screenMode == MODE_SPLASH) {
      if (bootPressed && !bootButtonWasPressed) {
        bootPressStartTime = millis();
      } else if (bootPressed && (millis() - bootPressStartTime >= (unsigned long)bootHoldThresholdMs)) {
        screenMode = MODE_STANDALONE_DEMO;
        beginStandaloneDemo();
      }
    } else if (screenMode == MODE_STANDALONE_DEMO) {
      stepStandaloneDemo();
    }

    if (bootPressed && !bootButtonWasPressed) {
      unsigned long now = millis();
      unsigned long gapFromLastPress = now - lastButtonPressEdgeTime;
      Serial.print("לחיצת BOOT נתפסה, פער מהלחיצה הקודמת: "); Serial.print(gapFromLastPress); Serial.println("ms");
      if (gapFromLastPress <= (unsigned long)doublePressMaxGapMs) {
        Serial.println("=== לחיצה כפולה על BOOT - בקשת הקלטת קול ===");
        recordRequested = true;
        lastButtonPressEdgeTime = 0;
      } else {
        lastButtonPressEdgeTime = now;
      }
    }
    bootButtonWasPressed = bootPressed;

    FeedbackCategory fb;
    if (xQueueReceive(voiceFeedbackQueue, &fb, 0) == pdTRUE) {
      Serial.print("משוב קולי התקבל, קטגוריה: ");
      Serial.println((int)fb);
      showVoiceFeedbackBanner(fb);
    }

    updateVoiceFeedbackBanner();

    updateConnectionGate();

    checkForAppLessonRequest();

    postCoachStatusToFirestore();
    printConnectionTransitions();

    static char lineBuffer[256];
    static int lineIndex = 0;
    static bool collectingManualText = false;
    static bool collectingSerialTranscript = false;

    if (Serial.available()) {
      char c = Serial.read();

      if (collectingManualText) {
        if (c == '\n' || c == '\r') {
          if (lineIndex > 0) {
            lineBuffer[lineIndex] = '\0';
            strncpy(manualTranscript, lineBuffer, sizeof(manualTranscript) - 1);
            manualTranscript[sizeof(manualTranscript) - 1] = '\0';
            Serial.print("התקבל מלל ידני: "); Serial.println(manualTranscript);
            textInputRequested = true;
            lineIndex = 0;
            collectingManualText = false;
          }
        } else if (lineIndex < (int)sizeof(lineBuffer) - 1) {
          lineBuffer[lineIndex++] = c;
        }
      } else if (collectingSerialTranscript) {
        if (c == '\n' || c == '\r') {
          if (lineIndex > 0) {
            lineBuffer[lineIndex] = '\0';
            if (strncmp(lineBuffer, SERIAL_TRANSCRIPT_PREFIX, strlen(SERIAL_TRANSCRIPT_PREFIX)) == 0) {
              strncpy(serialTranscriptText, lineBuffer + strlen(SERIAL_TRANSCRIPT_PREFIX), sizeof(serialTranscriptText) - 1);
              serialTranscriptText[sizeof(serialTranscriptText) - 1] = '\0';
              serialTranscriptReceived = true;
            }
            lineIndex = 0;
          }
          collectingSerialTranscript = false;
        } else if (lineIndex < (int)sizeof(lineBuffer) - 1) {
          lineBuffer[lineIndex++] = c;
        }
      } else if (c == 'S') {
        lineBuffer[0] = c;
        lineIndex = 1;
        collectingSerialTranscript = true;
      } else if (c == 'm') { startTrainingIfConnected(EX_MOUNTAIN_CLIMBER); }
      else if (c == 'l') { startTrainingIfConnected(EX_LUNGE); }
      else if (c == 's') { startTrainingIfConnected(EX_SITUP); }
      else if (c == 'v') {
        Serial.println("=== בקשת הקלטת קול (taskVoice) ===");
        recordRequested = true;
      }
      else if (c == 't') {
        Serial.println("הקלידי משפט ולחצי Enter (סיווג ידני, זמני עד שהרשת תתוקן):");
        collectingManualText = true;
      }
    }

    if (trainingActive) {
      unsigned long now = millis();
      if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        lastSampleTime = now;
        processTrainingStep();

        Serial.print("תרגיל: "); Serial.print(exerciseName);
        Serial.print(" | חזרה "); Serial.print(currentRep);
        Serial.print("/"); Serial.print(repsPerExercise);
        Serial.print(" | ציון: "); Serial.println(lastScore);

        if (trainingActive) updateTrainingScreen();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
