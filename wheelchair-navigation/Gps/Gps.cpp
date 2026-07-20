#include "GPSLocation.hpp"
#include <sstream>
#include <iomanip>
#include <string_view>
#include <string>
#include <vector>
#include <cmath>     // נוסף עבור std::exp
#include <algorithm> // נוסף עבור std::clamp

namespace {
double nmeaDdmmToDecimal(double value)
{
    if (value == 0.0) return 0.0;

    double absValue = std::abs(value);
    int degrees = static_cast<int>(absValue / 100.0);
    double minutes = absValue - degrees * 100.0;

    double decimal = degrees + minutes / 60.0;
    return value < 0.0 ? -decimal : decimal;
}
}


// שחרור מחסימת הערה ותיקון המרה ל-double
double GPSLocation::toDouble(const std::string_view& s) {                  
    if (s.empty()) return 0.0;
    try {
        return std::stod(std::string(s)); 
    } catch (...) { 
        return 0.0; 
    }
}

// שחרור מחסימת הערה ותיקון המרה ל-int (ללא ציון ערך דיפולט במימוש)
int GPSLocation::toInt(const std::string_view& s, int defaultValue) {    
    if (s.empty()) return defaultValue;
    try {
        return std::stoi(std::string(s)); 
    } catch (...) {
        return defaultValue;
    }
}
// int GPSLocation::toInt(const std::string_view& s, int defaultValue = 0) {   צריך לחזור לתקינות
//     if (s.empty()) return defaultValue;
//     try {
//         return std::stoi(std::string(s));
//     } catch (...) {
//         return defaultValue;
//     }
// } 

//פונקציה שמפרידה את הקלט על פי תו מפריד
void GPSLocation::split(const std::string_view& s, char delimiter) {
    tokens.clear(); 
    size_t start = 0;
    size_t end = s.find(delimiter);

    while (end != std::string_view::npos) {  
        std::string_view sub = s.substr(start, end - start);
        
        // התיקון: דוחפים לווקטור רק אם התא לא ריק!
        if (!sub.empty()) {
            tokens.emplace_back(sub);
        }
        
        start = end + 1; 
        end = s.find(delimiter, start);
    }
    
    // גם עבור האיבר האחרון (אחרי הפסיק האחרון)
    std::string_view lastSub = s.substr(start);
    if (!lastSub.empty()) {
        tokens.emplace_back(lastSub);
    }
}

bool GPSLocation::validateChecksum(const std::string_view& sentence) {
    // 1. חיפוש הכוכבית המסמנת את תחילת ה-Checksum
    size_t starPos = sentence.find('*');
    
    // בדיקה שיש כוכבית ושנותרו אחריה לפחות 2 תווים (הקוד עצמו)
    if (starPos == std::string_view::npos || starPos + 2 >= sentence.length()) {
        return false;
    }

    // 2. חילוץ ה-Checksum הצפוי (המרת 2 התווים שאחרי הכוכבית מהקסדצימלי למספר)
    // הערה: stoi דורש string, לכן ניצור אובייקט זמני קטן מאוד
    int expectedCheck = 0;
    try {
        expectedCheck = std::stoi(std::string(sentence.substr(starPos + 1, 2)), nullptr, 16);
    } catch (...) {
        return false; 
    }
    // 3. חישוב ה-Checksum בפועל (XOR על כל התווים בין ה-'$' לכוכבית)
    int calculatedCheck = 0;
    // מתחילים ב-1 כדי לדלג על ה-'$'
    for (size_t i = 1; i < starPos; ++i) {
        calculatedCheck ^= sentence[i];
    }
    // 4. השוואה
    return calculatedCheck == expectedCheck;
}

// --- לוגיקת הזרמה ועיבוד ---

void GPSLocation::updateFromStream(std::istream& dataStream) {
    char c;
    while (dataStream.get(c)) {
        if (c == '$') {
            this->buffer.clear();
            this->buffer += c;
        } else if (c == '\n' || c == '\r') {
            if (!this->buffer.empty() && this->buffer[0] == '$') {
                fillGPSLocation(this->buffer);
            }
            this->buffer.clear();
        } else if (!buffer.empty()) {
            // הגנה מפני הצפה - שימוש בקבוע שהגדרנו
            if (buffer.size() < MAX_SENTENCE_SIZE) {
                buffer += c;
            } else {
                buffer.clear(); 
            }
        }
    }
}

void GPSLocation::fillGPSLocation(const std::string& nmeaSentence) {
    split(nmeaSentence, ',');
    if (tokens.empty() || tokens[0].length() < 0) return;
    size_t starPos = tokens.back().find('*');
    if (starPos != std::string_view::npos) {
        tokens.back() = tokens.back().substr(0, starPos);
    }

    std::string header = tokens[0].substr(tokens[0].length() - 3, 3);
    if      (header == "GGA") parseGGA(tokens);
    else if (header == "RMC") parseRMC(tokens);
    else if (header == "GSA") parseGSA(tokens);
    else if (header == "VTG") parseVTG(tokens);
    else if (header == "HDT") parseHDT(tokens);
    else if (header == "APB") parseAPB(tokens);
    else if (header == "AAM") parseAAM(tokens);
    else if (header == "GSV") parseGSV(tokens);
}

// --- מימוש פונקציות ה-Parse הפרטיות ---


// עזר קטן: בודק אם השדה ריק לפני שמעדכנים, כדי לא לדרוס מידע קיים באפסים
void GPSLocation::updateIfNotEmpty(double& target, const std::string& field) {
    if (!field.empty()) {
        target = toDouble(field);
    }
}

void GPSLocation::parseGGA(const std::vector<std::string>& f) {
    if (f.size() < 10) return;
    
   
    if (!f[1].empty()) dateTime.time = f[1];
    if (!f[2].empty()) {
        double rawLat = toDouble(f[2]);
        position.latitude = nmeaDdmmToDecimal(rawLat);
        std::cout << "[GPS][DEBUG] GGA raw lat=" << rawLat << " decimal lat=" << position.latitude << std::endl;
    }
    if (!f[4].empty()) {
        double rawLon = toDouble(f[4]);
        position.longitude = nmeaDdmmToDecimal(rawLon);
        std::cout << "[GPS][DEBUG] GGA raw lon=" << rawLon << " decimal lon=" << position.longitude << std::endl;
    }
    updateIfNotEmpty(position.altitudeMSL, f[9]);

    if (!f[6].empty()) quality.lockQuality = toInt(f[6], 0);
    if (!f[7].empty()) quality.satellitesInUse = toInt(f[7], 0);
    if (!f[8].empty()) quality.HDOP = toDouble(f[8]);

    if (quality.lockQuality > 0) {
        quality.statusActive = true;
        if (quality.lockType < 2) {
            quality.lockType = 3; // נניח נעילת 3D בהתאם ל-GGA כדי שלא יתאפס ל-0
        }
    }
}

void GPSLocation::parseRMC(const std::vector<std::string>& f) {
    if (f.size() < 10) return;

    if (!f[1].empty()) dateTime.time = f[1];
    if (!f[9].empty()) dateTime.date = f[9];

    if (!f[2].empty()) {
        quality.statusActive = (f[2][0] == 'A' || f[2][0] == 'a');   
    }

    // מעדכנים מיקום ומהירות רק אם הסטטוס הוא 'A' (Active)
    if (quality.statusActive) {
        if (!f[3].empty()) {
            double rawLat = toDouble(f[3]);
            position.latitude = nmeaDdmmToDecimal(rawLat);
            std::cout << "[GPS][DEBUG] RMC raw lat=" << rawLat << " decimal lat=" << position.latitude << std::endl;
        }
        if (!f[5].empty()) {
            double rawLon = toDouble(f[5]);
            position.longitude = nmeaDdmmToDecimal(rawLon);
            std::cout << "[GPS][DEBUG] RMC raw lon=" << rawLon << " decimal lon=" << position.longitude << std::endl;
        }
        updateIfNotEmpty(navigation.speedOverGround, f[7]);
        updateIfNotEmpty(navigation.courseDegrees, f[8]);
        if (quality.lockType < 2) {
            quality.lockType = 2;
    }
  }
}
void GPSLocation::parseGSA(const std::vector<std::string>& f) {
    if (f.size() < 18) return;
    if (!f[2].empty()) quality.lockType = toInt(f[2], 1);    
    updateIfNotEmpty(quality.PDOP, f[15]);
    updateIfNotEmpty(quality.HDOP, f[16]);
    updateIfNotEmpty(quality.VDOP, f[17]);
}

void GPSLocation::parseVTG(const std::vector<std::string>& f) {
    if (f.size() < 9) return;
    // VTG נותן מהירות וקורס בדיוק גבוה, הוא ידרוס את מה שהגיע מה-RMC
    updateIfNotEmpty(navigation.courseDegrees, f[1]);
    updateIfNotEmpty(navigation.speedOverGround, f[7]); 
}

void GPSLocation::parseGSV(const std::vector<std::string>& f) {
    if (f.size() < 4) return;
    if (!f[3].empty()) quality.satellitesInView = toInt(f[3], 0);
}

void GPSLocation::parseHDT(const std::vector<std::string>& f) {
    if (f.size() < 2) return;
    updateIfNotEmpty(navigation.heading, f[1]);
}

void GPSLocation::parseAPB(const std::vector<std::string>& f) {
    if (f.size() < 14) return;
    updateIfNotEmpty(navigation.XTE, f[2]);
    updateIfNotEmpty(navigation.bearingToWaypoint, f[13]);
}

void GPSLocation::parseAAM(const std::vector<std::string>& f) {
    if (f.size() < 4) return;
    if (!f[3].empty()) navigation.arrivalStatus = (f[3] == "A");
}
//לשנות אותה לדינאמית!
//להבין אותה היטב
void GPSLocation::calculateConfidence() {
    // 1. תנאי סף בסיסי: אם המכשיר מדווח על סטטוס לא פעיל, הביטחון הוא אפס מוחלט
    if (!quality.statusActive || quality.lockType < static_cast<int>(LockType::Fix2D)) {
    quality.confidenceLevel = 0.0;        
    return;
    }

    // 2. שקלול מבוסס HDOP (Horizontal Dilution of Precision)
    // נשתמש בפונקציית דעיכה כדי שהביטחון ירד בצורה חלקה ככל שה-HDOP עולה.
    // ערך HDOP אידיאלי הוא 1.0. מעל 5.0 הדיוק נחשב בינוני ומעלה 20 הוא חסר ערך.
    double hdopWeight = std::exp(-(quality.HDOP - 1.0) / 4.0);
    hdopWeight = std::clamp(hdopWeight, 0.0, 1.0);

    // 3. שקלול מבוסס מספר לוויינים
    // המינימום לנעילה הוא 4. מעל 12 לוויינים השיפור הוא שולי.
    // נשתמש בפונקציית סיגמואיד פשוטה או נרמול ליניארי.
    double satWeight = (quality.satellitesInUse >= 4) ? 
                       (1.0 - std::exp(-(quality.satellitesInUse - 4) / 5.0)) : 0.0;

    // 4. בונוס על סוג הנעילה (Lock Type)
    // נעילה תלת-ממדית (3D) מקבלת עדיפות על פני דו-ממדית (2D)
    double lockWeight = (quality.lockType == static_cast<int>(LockType::Fix3D)) ? 1.0 : 0.6;

    // 5. שילוב המדדים לערך סופי (ממוצע משוקלל או מכפלה)
    // מכפלה מבטיחה שאם אחד המדדים גרוע מאוד, כל רמת הביטחון תרד בהתאם.
    double finalConfidence = hdopWeight * satWeight * lockWeight;

    // נרמול הערך הסופי לטווח שבין 0 ל-1
    quality.confidenceLevel = std::clamp(finalConfidence, 0.0, 1.0);
}

