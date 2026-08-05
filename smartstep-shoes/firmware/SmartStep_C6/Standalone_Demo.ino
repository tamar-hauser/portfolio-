struct DemoExerciseInfo {
  ExerciseType exercise;
  const char* name;
  const uint16_t* images[4];
};

DemoExerciseInfo DEMO_SEQUENCE[] = {
  { EX_RUNNING_IN_PLACE, "ריצה במקום", { running_1, running_2, running_3, running_4 } },
  { EX_CALF_RAISE,       "עליות עקב",   { calf_raise_1, calf_raise_2, calf_raise_3, calf_raise_4 } },
};
const int DEMO_SEQUENCE_COUNT = sizeof(DEMO_SEQUENCE) / sizeof(DemoExerciseInfo);

enum DemoStage { DEMO_SHOWING_IMAGES, DEMO_COUNTDOWN, DEMO_TRAINING, DEMO_FINISHED };
DemoStage demoStage = DEMO_SHOWING_IMAGES;
int demoExerciseIndex = 0;
int demoImageIndex = 0;
int demoCountdownValue = 3;
unsigned long demoStageStartTime = 0;
int demoImageDisplayMs = 400;
int demoCountdownStepMs = 1000;

void drawDemoImage(const uint16_t* img) {
  int x = (172 - DEMO_IMG_W) / 2;
  int y = (320 - DEMO_IMG_H) / 2 - 20;
  tft.drawRGBBitmap(x, y, img, DEMO_IMG_W, DEMO_IMG_H);
  printCentered(DEMO_SEQUENCE[demoExerciseIndex].name, y + DEMO_IMG_H + 24, 2, ST77XX_WHITE);
}

void showDemoCountdown(int value) {
  tft.fillScreen(ST77XX_BLACK);
  char countStr[2];
  snprintf(countStr, sizeof(countStr), "%d", value);
  printCentered(countStr, 140, 6, ST77XX_GREEN);
}

void beginStandaloneDemo() {
  demoExerciseIndex = 0;
  demoImageIndex = 0;
  demoStage = DEMO_SHOWING_IMAGES;
  demoStageStartTime = millis();
  tft.fillScreen(ST77XX_BLACK);
  drawDemoImage(DEMO_SEQUENCE[demoExerciseIndex].images[demoImageIndex]);
  Serial.print("=== מצב עצמאי: מדגימה "); Serial.println(DEMO_SEQUENCE[demoExerciseIndex].name);
}

void stepStandaloneDemo() {
  unsigned long now = millis();

  switch (demoStage) {

    case DEMO_SHOWING_IMAGES:
      if (now - demoStageStartTime >= (unsigned long)demoImageDisplayMs) {
        demoImageIndex++;
        if (demoImageIndex < 4) {
          drawDemoImage(DEMO_SEQUENCE[demoExerciseIndex].images[demoImageIndex]);
        } else {
          demoCountdownValue = 3;
          showDemoCountdown(demoCountdownValue);
          demoStage = DEMO_COUNTDOWN;
        }
        demoStageStartTime = now;
      }
      break;

    case DEMO_COUNTDOWN:
      if (now - demoStageStartTime >= (unsigned long)demoCountdownStepMs) {
        demoCountdownValue--;
        if (demoCountdownValue >= 1) {
          showDemoCountdown(demoCountdownValue);
        } else {
          selectExercise(DEMO_SEQUENCE[demoExerciseIndex].exercise);
          startTraining();
          demoStage = DEMO_TRAINING;
        }
        demoStageStartTime = now;
      }
      break;

    case DEMO_TRAINING:
      if (!trainingActive) {
        demoExerciseIndex++;
        if (demoExerciseIndex < DEMO_SEQUENCE_COUNT) {
          demoImageIndex = 0;
          tft.fillScreen(ST77XX_BLACK);
          drawDemoImage(DEMO_SEQUENCE[demoExerciseIndex].images[demoImageIndex]);
          Serial.print("=== מצב עצמאי: מדגימה "); Serial.println(DEMO_SEQUENCE[demoExerciseIndex].name);
          demoStage = DEMO_SHOWING_IMAGES;
        } else {
          Serial.println("=== מצב עצמאי הסתיים - חוזרת למסך פתיחה ===");
          screenMode = MODE_SPLASH;
          drawLogoScreen();
          demoStage = DEMO_FINISHED;
        }
        demoStageStartTime = now;
      }
      break;

    case DEMO_FINISHED:
      break;
  }
}
