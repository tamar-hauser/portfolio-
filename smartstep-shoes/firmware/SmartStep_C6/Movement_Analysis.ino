const float FSR_RANGE = 2900.0;
const float ACCEL_RANGE = 2.0;
const float GYRO_RANGE = 30.0;

const float W_FSR = 1.0;
const float W_ACCEL_XY = 1.0;
const float W_ACCEL_Z = 2.0;
const float W_GYRO = 2.0;

float calculateShoeDeviation(ShoeData &live, int r_fsr0, int r_fsr1, int r_fsr2,
                               float r_ax, float r_ay, float r_az,
                               float r_gx, float r_gy, float r_gz) {
  float totalWeight = 0;
  float weightedError = 0;

  float dFsr0 = abs(live.fsr0 - r_fsr0) / FSR_RANGE;
  float dFsr1 = abs(live.fsr1 - r_fsr1) / FSR_RANGE;
  float dFsr2 = abs(live.fsr2 - r_fsr2) / FSR_RANGE;
  weightedError += (dFsr0 + dFsr1 + dFsr2) * W_FSR;
  totalWeight += 3 * W_FSR;

  float dAx = abs(live.ax - r_ax) / ACCEL_RANGE;
  float dAy = abs(live.ay - r_ay) / ACCEL_RANGE;
  weightedError += (dAx + dAy) * W_ACCEL_XY;
  totalWeight += 2 * W_ACCEL_XY;

  float dAz = abs(live.az - r_az) / ACCEL_RANGE;
  weightedError += dAz * W_ACCEL_Z;
  totalWeight += W_ACCEL_Z;

  float dGx = abs(live.gx - r_gx) / GYRO_RANGE;
  float dGy = abs(live.gy - r_gy) / GYRO_RANGE;
  float dGz = abs(live.gz - r_gz) / GYRO_RANGE;
  weightedError += (dGx + dGy + dGz) * W_GYRO;
  totalWeight += 3 * W_GYRO;

  return weightedError / totalWeight;
}

void computeDeviationComponents(ShoeData &live, int r_fsr0, int r_fsr1, int r_fsr2,
                                  float r_ax, float r_ay, float r_az,
                                  float r_gx, float r_gy, float r_gz,
                                  float* outFsr, float* outAccelXY, float* outAccelZ, float* outGyro) {
  float dFsr0 = abs(live.fsr0 - r_fsr0) / FSR_RANGE;
  float dFsr1 = abs(live.fsr1 - r_fsr1) / FSR_RANGE;
  float dFsr2 = abs(live.fsr2 - r_fsr2) / FSR_RANGE;
  *outFsr = (dFsr0 + dFsr1 + dFsr2) / 3.0f;

  float dAx = abs(live.ax - r_ax) / ACCEL_RANGE;
  float dAy = abs(live.ay - r_ay) / ACCEL_RANGE;
  *outAccelXY = (dAx + dAy) / 2.0f;

  *outAccelZ = abs(live.az - r_az) / ACCEL_RANGE;

  float dGx = abs(live.gx - r_gx) / GYRO_RANGE;
  float dGy = abs(live.gy - r_gy) / GYRO_RANGE;
  float dGz = abs(live.gz - r_gz) / GYRO_RANGE;
  *outGyro = (dGx + dGy + dGz) / 3.0f;
}

float asymmetryDiagnosisThreshold = 0.15;
float stabilityDiagnosisThreshold = 0.20;
float pronationDiagnosisThreshold = 0.20;
float pressureDiagnosisThreshold  = 0.25;

struct MovementCheck { float value; float threshold; MovementProblem problem; };

MovementProblem diagnoseMovementProblem(CombinedData &live, const ReferenceFrame &ref) {
  float rFsr, rAccelXY, rAccelZ, rGyro;
  float lFsr, lAccelXY, lAccelZ, lGyro;

  computeDeviationComponents(live.right, ref.r_fsr0, ref.r_fsr1, ref.r_fsr2,
    ref.r_ax, ref.r_ay, ref.r_az, ref.r_gx, ref.r_gy, ref.r_gz,
    &rFsr, &rAccelXY, &rAccelZ, &rGyro);
  computeDeviationComponents(live.left, ref.l_fsr0, ref.l_fsr1, ref.l_fsr2,
    ref.l_ax, ref.l_ay, ref.l_az, ref.l_gx, ref.l_gy, ref.l_gz,
    &lFsr, &lAccelXY, &lAccelZ, &lGyro);

  float avgFsr    = (rFsr + lFsr) / 2.0f;
  float avgAccelZ = (rAccelZ + lAccelZ) / 2.0f;
  float avgGyro   = (rGyro + lGyro) / 2.0f;
  float asymmetry = abs(rFsr - lFsr) + abs(rGyro - lGyro);

  MovementCheck checks[] = {
    { asymmetry, asymmetryDiagnosisThreshold, MOVEMENT_ASYMMETRY },
    { avgAccelZ, stabilityDiagnosisThreshold, MOVEMENT_UNSTABLE },
    { avgGyro,   pronationDiagnosisThreshold, MOVEMENT_PRONATION },
    { avgFsr,    pressureDiagnosisThreshold,  MOVEMENT_PRESSURE_ISSUE },
  };
  const int CHECKS_COUNT = sizeof(checks) / sizeof(MovementCheck);

  MovementProblem best = MOVEMENT_OK;
  float bestRatio = 1.0f;
  for (int i = 0; i < CHECKS_COUNT; i++) {
    float ratio = checks[i].value / checks[i].threshold;
    if (ratio > bestRatio) {
      bestRatio = ratio;
      best = checks[i].problem;
    }
  }
  return best;
}

struct CombinedFeedback { const char* message; };
CombinedFeedback COMBINED_RECOMMENDATION_MATRIX[MOVEMENT_PROBLEM_COUNT][FEEDBACK_CATEGORY_COUNT];

struct CombinedFeedbackEntry { MovementProblem movement; FeedbackCategory feedback; const char* message; };
CombinedFeedbackEntry COMBINED_FEEDBACK_TABLE[] = {
  { MOVEMENT_OK, FEEDBACK_STABLE,       "מעולה, גם התנועה וגם התחושה שלך מראות שהכל תקין!" },
  { MOVEMENT_OK, FEEDBACK_TOO_EASY,     "התנועה תקינה וגם קלה לך - בואי נעלה קצת את הקצב" },
  { MOVEMENT_OK, FEEDBACK_TOO_HARD,     "התנועה שלך תקינה, גם אם מרגיש קשה - זה בסדר להתאמץ, את מבצעת נכון" },
  { MOVEMENT_OK, FEEDBACK_UNSTABLE,     "מעניין - החיישנים לא מזהים חוסר יציבות, אולי זו תחושה זמנית. תני לי לדעת אם זה נמשך" },
  { MOVEMENT_OK, FEEDBACK_DISCOMFORT,   "התנועה תקינה לפי החיישנים, אבל אם כואב לך - תמיד עדיף לעצור, הבריאות קודם" },
  { MOVEMENT_OK, FEEDBACK_TIRED,        "העייפות טבעית, והתנועה שלך עדיין תקינה - קחי נשימה ותמשיכי כשמוכנה" },
  { MOVEMENT_OK, FEEDBACK_STOP_REQUEST, "בסדר גמור, עוצרים - ביצעת את התרגיל נכון עד עכשיו" },
  { MOVEMENT_OK, FEEDBACK_HELP_NEEDED,  "התנועה שלך תקינה עד עכשיו - תגידי לי מה בדיוק לא ברור" },

  { MOVEMENT_UNSTABLE, FEEDBACK_STABLE,       "שמתי לב לחוסר יציבות בחיישנים, גם אם את מרגישה יציבה - נסי להתמקד בשיווי המשקל" },
  { MOVEMENT_UNSTABLE, FEEDBACK_TOO_EASY,     "החיישנים מזהים חוסר יציבות בתנועה - לפני שמעלים קצב, בואי נתקן קודם את היציבות" },
  { MOVEMENT_UNSTABLE, FEEDBACK_TOO_HARD,     "חוסר היציבות קשור כנראה לעומס - האטי את הקצב" },
  { MOVEMENT_UNSTABLE, FEEDBACK_UNSTABLE,     "כן, גם אני מזהה חוסר יציבות - נסי לפזר את המשקל שווה יותר בין הרגליים" },
  { MOVEMENT_UNSTABLE, FEEDBACK_DISCOMFORT,   "זיהיתי חוסר יציבות - עצרי ובדקי את התנוחה שלך" },
  { MOVEMENT_UNSTABLE, FEEDBACK_TIRED,        "העייפות עלולה לגרום לחוסר היציבות שזיהיתי - שקלי לנוח רגע" },
  { MOVEMENT_UNSTABLE, FEEDBACK_STOP_REQUEST, "בסדר, עוצרים - שימי לב ליציבות בפעם הבאה" },
  { MOVEMENT_UNSTABLE, FEEDBACK_HELP_NEEDED,  "זיהיתי חוסר יציבות - נסי לפזר משקל שווה בין שתי הרגליים ולכופף מעט את הברכיים" },

  { MOVEMENT_PRONATION, FEEDBACK_STABLE,       "יש קריסת קרסול פנימה שהחיישנים מזהים, גם אם מרגיש יציב - כדאי לשים לב" },
  { MOVEMENT_PRONATION, FEEDBACK_TOO_EASY,     "לפני שמעלים קצב - זיהיתי קריסת קרסול פנימה, בואי נתקן את זה קודם" },
  { MOVEMENT_PRONATION, FEEDBACK_TOO_HARD,     "הקושי עשוי לנבוע מקריסת הקרסול שזיהיתי - נסי לפזר משקל שווה יותר" },
  { MOVEMENT_PRONATION, FEEDBACK_UNSTABLE,     "זיהיתי קריסת קרסול פנימה - נסי לפזר את המשקל שווה יותר בין הרגליים" },
  { MOVEMENT_PRONATION, FEEDBACK_DISCOMFORT,   "הפרונציה שזיהיתי עלולה לגרום לכאב - עצרי לרגע ותקני את היציבה" },
  { MOVEMENT_PRONATION, FEEDBACK_TIRED,        "העייפות עלולה להחמיר את קריסת הקרסול שזיהיתי - שקלי לנוח" },
  { MOVEMENT_PRONATION, FEEDBACK_STOP_REQUEST, "בסדר, עוצרים - שימי לב שהקרסול לא יקרוס פנימה בפעם הבאה" },
  { MOVEMENT_PRONATION, FEEDBACK_HELP_NEEDED,  "זיהיתי שהקרסול נוטה פנימה - נסי לדמיין שאת דורכת על הקצה החיצוני של כף הרגל" },

  { MOVEMENT_PRESSURE_ISSUE, FEEDBACK_STABLE,       "פיזור הלחץ לא אחיד לפי החיישנים, גם אם מרגיש טוב - כדאי לשים לב" },
  { MOVEMENT_PRESSURE_ISSUE, FEEDBACK_TOO_EASY,     "לפני שמעלים קצב - הלחץ על כף הרגל לא מפוזר שווה, נתקן קודם" },
  { MOVEMENT_PRESSURE_ISSUE, FEEDBACK_TOO_HARD,     "הקושי עשוי לנבוע מפיזור לחץ לא נכון על כף הרגל" },
  { MOVEMENT_PRESSURE_ISSUE, FEEDBACK_UNSTABLE,     "פיזור הלחץ הלא אחיד עלול לגרום לתחושת חוסר היציבות שלך" },
  { MOVEMENT_PRESSURE_ISSUE, FEEDBACK_DISCOMFORT,   "פיזור הלחץ על כף הרגל לא אחיד - זה עשוי להסביר את אי הנוחות" },
  { MOVEMENT_PRESSURE_ISSUE, FEEDBACK_TIRED,        "העייפות עלולה להשפיע על פיזור הלחץ - שקלי לנוח" },
  { MOVEMENT_PRESSURE_ISSUE, FEEDBACK_STOP_REQUEST, "בסדר, עוצרים - שימי לב לפזר משקל שווה על כל כף הרגל בפעם הבאה" },
  { MOVEMENT_PRESSURE_ISSUE, FEEDBACK_HELP_NEEDED,  "הלחץ לא מתפזר שווה על כף הרגל - נסי לוודא שהעקב, האגודל והזרת נוגעים כולם ברצפה" },

  { MOVEMENT_ASYMMETRY, FEEDBACK_STABLE,       "יש הבדל בין הרגליים לפי החיישנים, גם אם מרגיש טוב - כדאי לשים לב" },
  { MOVEMENT_ASYMMETRY, FEEDBACK_TOO_EASY,     "לפני שמעלים קצב - יש הבדל בין הרגל הימנית לשמאלית, נתקן קודם" },
  { MOVEMENT_ASYMMETRY, FEEDBACK_TOO_HARD,     "הקושי עשוי לנבוע מהבדל בין הרגליים שזיהיתי" },
  { MOVEMENT_ASYMMETRY, FEEDBACK_UNSTABLE,     "ההבדל בין הרגליים עלול להסביר את תחושת חוסר היציבות שלך" },
  { MOVEMENT_ASYMMETRY, FEEDBACK_DISCOMFORT,   "יש הבדל משמעותי בין הרגליים - זה עלול להסביר את אי הנוחות" },
  { MOVEMENT_ASYMMETRY, FEEDBACK_TIRED,        "העייפות עלולה להחמיר את ההבדל שזיהיתי בין הרגליים" },
  { MOVEMENT_ASYMMETRY, FEEDBACK_STOP_REQUEST, "בסדר, עוצרים - שימי לב לפזר משקל שווה בין שתי הרגליים בפעם הבאה" },
  { MOVEMENT_ASYMMETRY, FEEDBACK_HELP_NEEDED,  "יש הבדל בין הרגל הימנית לשמאלית - נסי להתמקד בפיזור משקל שווה" },
};
const int COMBINED_FEEDBACK_TABLE_COUNT = sizeof(COMBINED_FEEDBACK_TABLE) / sizeof(CombinedFeedbackEntry);

void initCombinedRecommendationMatrix() {
  for (int m = 0; m < MOVEMENT_PROBLEM_COUNT; m++) {
    for (int f = 0; f < FEEDBACK_CATEGORY_COUNT; f++) {
      COMBINED_RECOMMENDATION_MATRIX[m][f].message = "התנועה תקינה, המשיכי כך!";
    }
  }

  for (int i = 0; i < COMBINED_FEEDBACK_TABLE_COUNT; i++) {
    CombinedFeedbackEntry &entry = COMBINED_FEEDBACK_TABLE[i];
    COMBINED_RECOMMENDATION_MATRIX[entry.movement][entry.feedback].message = entry.message;
  }
}

struct SensorHistoryEntry { MovementProblem problem; float deviationScore; unsigned long timestamp; };
const int SENSOR_HISTORY_SIZE = 360;
SensorHistoryEntry sensorHistory[SENSOR_HISTORY_SIZE];
int historyWriteIndex = 0;
int historyFilledCount = 0;

void recordSensorHistory(MovementProblem problem, float deviationScore) {
  sensorHistory[historyWriteIndex] = { problem, deviationScore, millis() };
  historyWriteIndex = (historyWriteIndex + 1) % SENSOR_HISTORY_SIZE;
  if (historyFilledCount < SENSOR_HISTORY_SIZE) historyFilledCount++;
}

int consistentProblemPercentThreshold = 60;
unsigned long pastEventLookbackMinMs = 30000;
unsigned long pastEventLookbackMaxMs = 90000;
float pastEventDeviationThreshold = 40;

MovementProblem findMostConsistentProblem(int* outPercent) {
  int counts[MOVEMENT_PROBLEM_COUNT] = {0};
  for (int i = 0; i < historyFilledCount; i++) {
    counts[sensorHistory[i].problem]++;
  }
  MovementProblem best = MOVEMENT_OK;
  int bestCount = 0;
  for (int p = 1; p < MOVEMENT_PROBLEM_COUNT; p++) {
    if (counts[p] > bestCount) {
      bestCount = counts[p];
      best = (MovementProblem)p;
    }
  }
  *outPercent = (historyFilledCount > 0) ? (bestCount * 100 / historyFilledCount) : 0;
  return best;
}

bool findPastEventSpike(MovementProblem* outProblem) {
  unsigned long now = millis();
  for (int i = 0; i < historyFilledCount; i++) {
    unsigned long age = now - sensorHistory[i].timestamp;
    if (age < pastEventLookbackMinMs || age > pastEventLookbackMaxMs) continue;
    if (sensorHistory[i].deviationScore >= pastEventDeviationThreshold) {
      *outProblem = sensorHistory[i].problem;
      return true;
    }
  }
  return false;
}

MovementProblem diagnoseProblemOrigin(MovementProblem currentProblem, ProblemOrigin* outOrigin) {
  if (currentProblem != MOVEMENT_OK) {
    *outOrigin = ORIGIN_CURRENT;
    return currentProblem;
  }

  int consistentPercent;
  MovementProblem consistentProblem = findMostConsistentProblem(&consistentPercent);
  if (consistentProblem != MOVEMENT_OK && consistentPercent >= consistentProblemPercentThreshold) {
    *outOrigin = ORIGIN_CONSISTENT;
    return consistentProblem;
  }

  MovementProblem pastProblem;
  if (findPastEventSpike(&pastProblem)) {
    *outOrigin = ORIGIN_PAST_EVENT;
    return pastProblem;
  }

  *outOrigin = ORIGIN_NONE;
  return MOVEMENT_OK;
}

const char* problemOriginPrefix(ProblemOrigin origin) {
  switch (origin) {
    case ORIGIN_CONSISTENT: return "שמתי לב שזו לא פעם ראשונה באימון הזה - ";
    case ORIGIN_PAST_EVENT: return "יכול להיות שזה מרגע קודם באימון, לא ממש עכשיו - ";
    case ORIGIN_CURRENT:
    case ORIGIN_NONE:
    default: return "";
  }
}

float calculateFrameScore(CombinedData &live, const ReferenceFrame &ref) {
  float devRight = calculateShoeDeviation(live.right,
    ref.r_fsr0, ref.r_fsr1, ref.r_fsr2,
    ref.r_ax, ref.r_ay, ref.r_az,
    ref.r_gx, ref.r_gy, ref.r_gz);

  float devLeft = calculateShoeDeviation(live.left,
    ref.l_fsr0, ref.l_fsr1, ref.l_fsr2,
    ref.l_ax, ref.l_ay, ref.l_az,
    ref.l_gx, ref.l_gy, ref.l_gz);

  float avgDeviation = (devRight + devLeft) / 2.0;

  float closeness = 1.0 - avgDeviation;
  if (closeness < 0) closeness = 0;
  float score = 100.0 * closeness * closeness;

  if (score < 0) score = 0;
  if (score > 100) score = 100;
  return score;
}
