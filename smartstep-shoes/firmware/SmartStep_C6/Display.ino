void writeWavHeader(WiFiClient &client, uint32_t dataLen) {
  uint32_t sampleRate = AUDIO_SAMPLE_RATE_HZ;
  uint16_t bitsPerSample = 16;
  uint16_t numChannels = 1;
  uint32_t byteRate = sampleRate * numChannels * bitsPerSample / 8;
  uint16_t blockAlign = numChannels * bitsPerSample / 8;
  uint32_t chunkSize = 36 + dataLen;
  uint32_t subchunk1Size = 16;
  uint16_t audioFormat = 1; // PCM

  client.write((const uint8_t*)"RIFF", 4);
  client.write((const uint8_t*)&chunkSize, 4);
  client.write((const uint8_t*)"WAVEfmt ", 8);
  client.write((const uint8_t*)&subchunk1Size, 4);
  client.write((const uint8_t*)&audioFormat, 2);
  client.write((const uint8_t*)&numChannels, 2);
  client.write((const uint8_t*)&sampleRate, 4);
  client.write((const uint8_t*)&byteRate, 4);
  client.write((const uint8_t*)&blockAlign, 2);
  client.write((const uint8_t*)&bitsPerSample, 2);
  client.write((const uint8_t*)"data", 4);
  client.write((const uint8_t*)&dataLen, 4);
}

void handleServeRecording() {
  if (!hasRecording) {
    debugAudioServer.send(404, "text/plain; charset=utf-8", "עוד אין הקלטה - שלחי v ב-Serial קודם");
    return;
  }
  debugAudioServer.setContentLength(44 + AUDIO_BUFFER_BYTES);
  debugAudioServer.send(200, "audio/wav", "");
  WiFiClient client = debugAudioServer.client();
  writeWavHeader(client, AUDIO_BUFFER_BYTES);
  client.write(audioBuffer, AUDIO_BUFFER_BYTES);
}

uint16_t scoreColor(float score) {
  if (score >= 80) return ST77XX_GREEN;
  if (score >= 50) return ST77XX_YELLOW;
  return ST77XX_RED;
}

uint16_t feedbackCategoryColor(FeedbackCategory fb) {
  switch (fb) {
    case FEEDBACK_STABLE:       return ST77XX_GREEN;
    case FEEDBACK_TOO_EASY:     return 0x07FF;
    case FEEDBACK_TOO_HARD:     return ST77XX_YELLOW;
    case FEEDBACK_UNSTABLE:     return 0xFD20;
    case FEEDBACK_DISCOMFORT:   return ST77XX_RED;
    case FEEDBACK_TIRED:        return 0xFD20;
    case FEEDBACK_STOP_REQUEST: return ST77XX_RED;
    case FEEDBACK_HELP_NEEDED:  return ST77XX_YELLOW;
    default:                    return 0x8410;
  }
}

const char* feedbackCategoryLabel(FeedbackCategory fb) {
  switch (fb) {
    case FEEDBACK_STABLE:       return "GREAT JOB!";
    case FEEDBACK_TOO_EASY:     return "TOO EASY";
    case FEEDBACK_TOO_HARD:     return "TOO HARD";
    case FEEDBACK_UNSTABLE:     return "UNSTABLE";
    case FEEDBACK_DISCOMFORT:   return "DISCOMFORT";
    case FEEDBACK_TIRED:        return "TIRED";
    case FEEDBACK_STOP_REQUEST: return "STOPPING";
    case FEEDBACK_HELP_NEEDED:  return "HELP NEEDED";
    default:                    return "";
  }
}

void drawCelebrationCheck(int cx, int cy) {
  tft.drawLine(cx - 8, cy,     cx - 3, cy + 6, ST77XX_WHITE);
  tft.drawLine(cx - 3, cy + 6, cx + 9, cy - 8, ST77XX_WHITE);
  tft.drawLine(cx - 8, cy + 1, cx - 3, cy + 7, ST77XX_WHITE);
  tft.drawLine(cx - 3, cy + 7, cx + 9, cy - 7, ST77XX_WHITE);
}

void showVoiceFeedbackBanner(FeedbackCategory fb) {
  if (fb == FEEDBACK_UNCERTAIN) return;

  voiceFeedbackBannerActive = true;
  voiceFeedbackBannerUntil = millis() + (unsigned long)voiceFeedbackBannerDurationMs;

  uint16_t bgColor = feedbackCategoryColor(fb);
  tft.fillRect(0, 0, 172, 40, bgColor);
  printCentered(feedbackCategoryLabel(fb), 14, 2, ST77XX_BLACK);

  if (fb == FEEDBACK_STABLE) {
    drawCelebrationCheck(150, 20);
  }
}

void restoreTopStripAfterBanner() {
  switch (screenMode) {
    case MODE_SPLASH:
    case MODE_STANDALONE_DEMO:
      tft.fillRect(0, 0, 172, 40, ST77XX_BLACK);
      break;
    case MODE_APP_TRAINING:
      tft.fillRect(0, 0, 172, 36, 0x0861);
      if (trainingActive) {
        tft.setTextSize(1);
        tft.setTextColor(0x7DEA);
        tft.setCursor(12, 13);
        tft.print(exerciseNameEn);
      } else {
        printCentered("SmartStep", 11, 1, ST77XX_WHITE);
      }
      break;
  }
}

void updateVoiceFeedbackBanner() {
  if (voiceFeedbackBannerActive && millis() >= voiceFeedbackBannerUntil) {
    voiceFeedbackBannerActive = false;
    restoreTopStripAfterBanner();
  }
}

void printCentered(const char* text, int y, uint8_t textSize, uint16_t color) {
  tft.setTextSize(textSize);
  tft.setTextColor(color);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  int x = (172 - w) / 2;
  tft.setCursor(x, y);
  tft.print(text);
}

void drawIdleScreen() {
  tft.fillScreen(ST77XX_BLACK);

  tft.fillRect(0, 0, 172, 36, 0x0861);
  printCentered("SmartStep", 11, 1, ST77XX_WHITE);

  int cx = 86, cy = 115;
  tft.drawCircle(cx, cy, 50, 0x07FF);
  tft.drawCircle(cx, cy, 49, 0x07FF);
  printCentered("EXERCISE", cy - 25, 1, 0x5DCA);
  printCentered(exerciseNameEn, cy - 5, 2, ST77XX_WHITE);
  char repsLabel[10];
  snprintf(repsLabel, sizeof(repsLabel), "%d reps", repsPerExercise);
  printCentered(repsLabel, cy + 25, 1, 0x8410);

  drawButton(36, 200, 100, 34, ST77XX_GREEN, "START", 0x04342C);

  printCentered("Sync starts on press", 270, 1, 0x8410);
}

void drawNoConnectionScreen() {
  tft.fillScreen(ST77XX_BLACK);

  int cx = 86, cy = 110;
  tft.drawLine(cx, cy - 40, cx - 40, cy + 30, ST77XX_RED);
  tft.drawLine(cx, cy - 40, cx + 40, cy + 30, ST77XX_RED);
  tft.drawLine(cx - 40, cy + 30, cx + 40, cy + 30, ST77XX_RED);
  tft.fillRect(cx - 3, cy - 15, 6, 25, ST77XX_RED);
  tft.fillRect(cx - 3, cy + 15, 6, 6, ST77XX_RED);

  printCentered("NO CONNECTION", cy + 70, 2, ST77XX_RED);
  printCentered("Check that the shoes", cy + 100, 1, 0x8410);
  printCentered("are powered on", cy + 115, 1, 0x8410);
}

void drawLogoScreen() {
  tft.fillScreen(ST77XX_BLACK);
  int cx = 86, cy = 130;

  tft.fillCircle(cx, cy, 52, 0x0421);
  tft.drawCircle(cx, cy, 52, 0x07FF);

  tft.fillCircle(cx, cy - 28, 7, ST77XX_GREEN);
  tft.fillCircle(cx - 18, cy + 18, 7, ST77XX_GREEN);
  tft.fillCircle(cx + 18, cy + 18, 7, ST77XX_GREEN);
  tft.drawLine(cx, cy - 28, cx - 18, cy + 18, 0x5DCA);
  tft.drawLine(cx, cy - 28, cx + 18, cy + 18, 0x5DCA);
  tft.drawLine(cx - 18, cy + 18, cx + 18, cy + 18, 0x5DCA);

  printCentered("SmartStep", 215, 2, ST77XX_WHITE);
  printCentered("VIRTUAL COACH", 240, 1, 0x8410);
}

void drawButton(int x, int y, int w, int h, uint16_t bgColor, const char* label, uint16_t textColor) {
  tft.fillRoundRect(x, y, w, h, h / 2, bgColor);
  tft.setTextSize(2);
  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
  tft.setCursor(x + (w - tw) / 2, y + (h - th) / 2);
  tft.setTextColor(textColor);
  tft.print(label);
}

void clearArea(int x, int y, int w, int h) {
  tft.fillRect(x, y, w, h, ST77XX_BLACK);
}

void drawProgressRing(int cx, int cy, int radius, int thickness, uint16_t bgColor) {
  for (int r = radius - thickness; r <= radius; r++) {
    tft.drawCircle(cx, cy, r, 0x1A86);
  }
}

void drawRingArc(int cx, int cy, int radius, int thickness, float pct, uint16_t color) {
  float startAngle = -90;
  float endAngle = startAngle + (360.0 * pct / 100.0);
  for (float a = startAngle; a <= endAngle; a += 2) {
    float rad = a * 3.14159 / 180.0;
    for (int r = radius - thickness; r <= radius; r++) {
      int x = cx + r * cos(rad);
      int y = cy + r * sin(rad);
      tft.drawPixel(x, y, color);
      tft.drawPixel(x + 1, y, color);
      tft.drawPixel(x, y + 1, color);
    }
  }
}

void drawInfoCard(int x, int y, int w, int h, const char* label, const char* value, uint16_t valueColor, bool alignRight) {
  tft.fillRoundRect(x, y, w, h, 6, 0x10A2);
  tft.setTextSize(1);
  tft.setTextColor(0x8410);
  int16_t x1, y1; uint16_t tw, th;
  tft.getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
  int lx = alignRight ? (x + w - tw - 8) : (x + 8);
  tft.setCursor(lx, y + 8);
  tft.print(label);

  tft.setTextSize(2);
  tft.setTextColor(valueColor);
  tft.getTextBounds(value, 0, 0, &x1, &y1, &tw, &th);
  int vx = alignRight ? (x + w - tw - 8) : (x + 8);
  tft.setCursor(vx, y + 22);
  tft.print(value);
}

void drawTrainingFrame() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0, 0, 172, 36, 0x0861);
  tft.setTextSize(1);
  tft.setTextColor(0x7DEA);
  tft.setCursor(12, 13);
  tft.print(exerciseNameEn);

  drawProgressRing(86, 115, 58, 9, 0x1A86);
  printCentered("MATCH SCORE", 192, 1, 0x8410);
}

void updateTrainingScreen() {
  drawProgressRing(86, 115, 58, 9, 0x1A86);
  uint16_t color = scoreColor(lastScore);
  drawRingArc(86, 115, 58, 9, lastScore, color);

  clearArea(56, 95, 60, 40);
  char scoreStr[5];
  snprintf(scoreStr, sizeof(scoreStr), "%d", (int)lastScore);
  printCentered(scoreStr, 100, 4, color);

  clearArea(130, 8, 35, 20);
  char repStr[8];
  snprintf(repStr, sizeof(repStr), "%d/%d", currentRep, repsPerExercise);
  tft.setTextSize(1);
  tft.setTextColor(0x8410);
  tft.setCursor(135, 13);
  tft.print(repStr);

  int barY = 215;
  int barWidth = 40;
  int gap = 10;
  int totalWidth = repsPerExercise * barWidth + (repsPerExercise - 1) * gap;
  int startX = (172 - totalWidth) / 2;
  for (int i = 0; i < repsPerExercise; i++) {
    uint16_t barColor;
    if (i < repScoreIndex) {
      barColor = scoreColor(repScores[i]);
    } else if (i == currentRep - 1) {
      barColor = 0x39E7;
    } else {
      barColor = 0x2104;
    }
    tft.fillRoundRect(startX + i * (barWidth + gap), barY, barWidth, 8, 4, barColor);
  }

  clearArea(10, 250, 152, 50);
  float deviationPct = 100.0 - lastScore;
  if (deviationPct < 0) deviationPct = 0;
  char devStr[8];
  snprintf(devStr, sizeof(devStr), "%d%%", (int)deviationPct);
  const char* stabilityLabel = (deviationPct < 20) ? "Good" : (deviationPct < 40) ? "Fair" : "Off";
  uint16_t stabilityColor = (deviationPct < 20) ? ST77XX_GREEN : (deviationPct < 40) ? ST77XX_YELLOW : ST77XX_RED;

  drawInfoCard(10, 250, 72, 46, "STABILITY", stabilityLabel, stabilityColor, false);
  drawInfoCard(90, 250, 72, 46, "DEVIATION", devStr, scoreColor(lastScore), true);
}

void drawSummaryScreen() {
  tft.fillScreen(ST77XX_BLACK);
  printCentered("SUMMARY", 15, 2, 0x07FF);
  tft.drawFastHLine(10, 45, 152, 0x39C7);

  float totalScore = 0;
  int yPos = 65;
  for (int i = 0; i < repScoreIndex; i++) {
    char line[20];
    snprintf(line, sizeof(line), "Rep %d:  %d", i + 1, (int)repScores[i]);
    printCentered(line, yPos, 2, scoreColor(repScores[i]));
    yPos += 35;
    totalScore += repScores[i];
  }

  if (repScoreIndex > 0) {
    float avg = totalScore / repScoreIndex;
    tft.drawFastHLine(10, yPos + 5, 152, 0x39C7);
    char avgStr[10];
    snprintf(avgStr, sizeof(avgStr), "%d", (int)avg);
    printCentered("AVERAGE", yPos + 25, 1, 0x8410);
    printCentered(avgStr, yPos + 45, 4, scoreColor(avg));
  }
}
