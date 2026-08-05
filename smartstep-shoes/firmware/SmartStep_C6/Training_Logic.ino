//  ESP-NOW callback
void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len == sizeof(CombinedData)) {
    memcpy(&liveData, data, sizeof(CombinedData));
    dataReceived = true;
    lastShoeDataTime = millis();
  }
}

bool shoesConnected() {
  return (millis() - lastShoeDataTime) < (unsigned long)shoeConnectionTimeoutMs;
}

void printConnectionTransitions() {
  static bool wasConnected = false;
  bool isConnected = shoesConnected();
  if (isConnected != wasConnected) {
    Serial.println(isConnected ? ">>> הנעליים מחוברות! מתקבל דאטה." : ">>> הנעליים התנתקו - אין דאטה.");
    wasConnected = isConnected;
  }
}

void updateConnectionGate() {
  switch (screenMode) {
    case MODE_APP_TRAINING:
      if (!shoesConnected()) {
        trainingActive = false;
        screenMode = MODE_NO_CONNECTION;
        drawNoConnectionScreen();
      }
      break;
    case MODE_NO_CONNECTION:
      if (shoesConnected()) {
        screenMode = MODE_SPLASH;
        drawLogoScreen();
      }
      break;
    case MODE_SPLASH:
    case MODE_STANDALONE_DEMO:
      break;
  }
}

struct ExerciseNameMap { const char* firestoreId; ExerciseType type; };
ExerciseNameMap EXERCISE_NAME_MAP[] = {
  { "mountainClimber", EX_MOUNTAIN_CLIMBER },
  { "lunge",           EX_LUNGE            },
  { "situp",           EX_SITUP            },
};
const int EXERCISE_NAME_MAP_COUNT = sizeof(EXERCISE_NAME_MAP) / sizeof(ExerciseNameMap);

void checkForAppLessonRequest() {
  if (screenMode != MODE_SPLASH) return;
  if (millis() - lastLessonPollTime < (unsigned long)lessonPollIntervalMs) return;
  lastLessonPollTime = millis();

  HTTPClient http;
  http.begin(firestoreBaseUrl + "/state/currentLesson");
  int code = http.GET();
  if (code != 200) {
    http.end();
    return;
  }

  StaticJsonDocument<512> doc;
  deserializeJson(doc, http.getString());
  http.end();

  const char* exerciseId = doc["fields"]["exercise"]["stringValue"];
  const char* requestedAt = doc["fields"]["requestedAt"]["integerValue"];
  if (!exerciseId || !requestedAt) return;
  if (lastSeenLessonRequestedAt == requestedAt) return;

  lastSeenLessonRequestedAt = requestedAt;

  for (int i = 0; i < EXERCISE_NAME_MAP_COUNT; i++) {
    if (strcmp(EXERCISE_NAME_MAP[i].firestoreId, exerciseId) == 0) {
      Serial.print("checkForAppLessonRequest: בקשת שיעור חדשה מהאפליקציה - "); Serial.println(exerciseId);
      startTrainingIfConnected(EXERCISE_NAME_MAP[i].type);
      return;
    }
  }
}

bool getEpochMillis(double* outMs) {
  time_t nowSeconds = time(nullptr);
  if (nowSeconds < 1700000000) return false;
  *outMs = (double)nowSeconds * 1000.0;
  return true;
}

void postSessionToFirestore(float avgScore) {
  if (WiFi.status() != WL_CONNECTED) return;

  double timestampMs;
  if (!getEpochMillis(&timestampMs)) {
    Serial.println("postSessionToFirestore: השעון עדיין לא הסתנכרן (NTP) - מדלגת על כתיבת הסשן");
    return;
  }

  StaticJsonDocument<768> body;
  JsonObject fields = body["fields"].to<JsonObject>();
  fields["exercise"]["stringValue"] = exerciseFirestoreId;
  fields["timestamp"]["doubleValue"] = timestampMs;
  fields["reps"]["integerValue"] = String(repScoreIndex);
  fields["avgScore"]["doubleValue"] = avgScore;
  JsonArray scoresArrayValues = fields["scores"]["arrayValue"]["values"].to<JsonArray>();
  for (int i = 0; i < repScoreIndex; i++) {
    scoresArrayValues.createNestedObject()["doubleValue"] = repScores[i];
  }

  String bodyStr;
  serializeJson(body, bodyStr);

  HTTPClient http;
  http.begin(firestoreBaseUrl + "/sessions");
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(bodyStr);
  if (code != 200) {
    Serial.print("postSessionToFirestore: כתיבה ל-Firestore נכשלה, קוד "); Serial.println(code);
  } else {
    Serial.println("postSessionToFirestore: הסשן נשמר באפליקציה בהצלחה");
  }
  http.end();
}

int coachStatusPostIntervalMs = 2000;
unsigned long lastCoachStatusPostTime = 0;

void postCoachStatusToFirestore() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastCoachStatusPostTime < (unsigned long)coachStatusPostIntervalMs) return;
  lastCoachStatusPostTime = millis();

  double timestampMs;
  if (!getEpochMillis(&timestampMs)) return;

  StaticJsonDocument<256> body;
  JsonObject fields = body["fields"].to<JsonObject>();
  fields["state"]["stringValue"] = trainingActive ? "training" : "idle";
  fields["shoesConnected"]["booleanValue"] = shoesConnected();
  fields["timestamp"]["doubleValue"] = timestampMs;

  String bodyStr;
  serializeJson(body, bodyStr);

  String url = firestoreBaseUrl + "/state/coachStatus?updateMask.fieldPaths=state&updateMask.fieldPaths=shoesConnected&updateMask.fieldPaths=timestamp";
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.PATCH(bodyStr);
  http.end();
}

void initDefaultActiveReference() {
  activeReference = mountainClimberReference;
  activeReferenceCount = mountainClimberReferenceCount;
}

void selectExercise(ExerciseType type) {
  currentExercise = type;
  switch (type) {
    case EX_MOUNTAIN_CLIMBER:
      activeReference = mountainClimberReference;
      activeReferenceCount = mountainClimberReferenceCount;
      exerciseName = "מטפסי הרים";
      exerciseNameEn = "MOUNTAIN CLIMBER";
      exerciseFirestoreId = "mountainClimber";
      break;
    case EX_LUNGE:
      activeReference = lungeReference;
      activeReferenceCount = lungeReferenceCount;
      exerciseName = "לאנג'";
      exerciseNameEn = "LUNGE";
      exerciseFirestoreId = "lunge";
      break;
    case EX_SITUP:
      activeReference = situpReference;
      activeReferenceCount = situpReferenceCount;
      exerciseName = "סיט אפ";
      exerciseNameEn = "SIT UP";
      exerciseFirestoreId = "situp";
      break;
    case EX_RUNNING_IN_PLACE:
      activeReference = runningInPlaceReference;
      activeReferenceCount = runningInPlaceReferenceCount;
      exerciseName = "ריצה במקום";
      exerciseNameEn = "RUNNING";
      exerciseFirestoreId = "running_in_place";
      break;
    case EX_CALF_RAISE:
      activeReference = calfRaiseReference;
      activeReferenceCount = calfRaiseReferenceCount;
      exerciseName = "עליות עקב";
      exerciseNameEn = "CALF RAISE";
      exerciseFirestoreId = "calf_raise";
      break;
  }
  framesPerRep = activeReferenceCount / repsPerExercise;
  if (framesPerRep < 1) framesPerRep = 1;

  int appFrameCount;
  if (fetchReferenceFromApp(exerciseFirestoreId, &appFrameCount)) {
    activeReference = appReferenceBuffer;
    activeReferenceCount = appFrameCount;
    framesPerRep = activeReferenceCount / repsPerExercise;
    if (framesPerRep < 1) framesPerRep = 1;
    Serial.print("selectExercise: נטען רפרנס מהאפליקציה - "); Serial.print(appFrameCount); Serial.println(" פריימים");
  } else {
    Serial.println("selectExercise: אין קובץ רפרנס מהאפליקציה לתרגיל הזה - משתמשת ברפרנס המקומי הקבוע");
  }

  drawIdleScreen();
}

void startTrainingIfConnected(ExerciseType type) {
  if (!shoesConnected()) {
    screenMode = MODE_NO_CONNECTION;
    drawNoConnectionScreen();
    return;
  }
  screenMode = MODE_APP_TRAINING;
  selectExercise(type);
  startTraining();
}

void startTraining() {
  trainingActive = true;
  trainingStartTime = millis();
  lastSampleTime = trainingStartTime;
  currentFrameIndex = 0;
  currentRep = 1;
  repScoreIndex = 0;
  Serial.println("=== אימון התחיל! ===");
  drawTrainingFrame();
  updateTrainingScreen();
}

void stopTraining() {
  trainingActive = false;
  Serial.println("=== אימון הסתיים! ===");
  float totalScore = 0;
  for (int i = 0; i < repScoreIndex; i++) {
    Serial.print("חזרה "); Serial.print(i + 1);
    Serial.print(": "); Serial.println(repScores[i]);
    totalScore += repScores[i];
  }
  if (repScoreIndex > 0) {
    float avgScore = totalScore / repScoreIndex;
    Serial.print("ציון כולל ממוצע: ");
    Serial.println(avgScore);
    postSessionToFirestore(avgScore);
  }
  drawSummaryScreen();
}

void processTrainingStep() {
  if (!dataReceived) return;
  if (currentFrameIndex >= activeReferenceCount) {
    stopTraining();
    return;
  }

  const ReferenceFrame &ref = activeReference[currentFrameIndex];
  lastScore = calculateFrameScore(liveData, ref);

  recordSensorHistory(diagnoseMovementProblem(liveData, ref), 100.0 - lastScore);

  int newRep = (currentFrameIndex / framesPerRep) + 1;
  if (newRep != currentRep && newRep <= repsPerExercise) {
    if (repScoreIndex < 10) {
      repScores[repScoreIndex++] = lastScore;
    }
    currentRep = newRep;
  }

  currentFrameIndex++;
}
