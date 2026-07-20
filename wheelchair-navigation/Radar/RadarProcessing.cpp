#include "RadarProcessing.hpp"
#include <cmath>
#include <algorithm>
#include "RadarData.hpp"

#include <vector>

std::vector<RadarObject> RadarProcessing::process(std::vector<RadarMeasurement>& input, double ts) {
    this->ts = ts; // תוקן מ-tss ל-ts
    
    // אם פונקציית Clean בהערה, אנו נשתמש ישירות ב-input.
    // אם בעתיד תרצה להשתמש ב-Clean, פתח את ההערה ותגדיר משתנה cleaned_data.
    
    std::vector<RadarObject> myRadar;
    constexpr double kMinRadarRange = 0.2; // meters: ignore very close returns (Webots virtual radar uses LiDAR points, typical range 0.2-5m)
    for (size_t i = 0; i < input.size(); i++) { // רץ על input במקום cleaned_data
        const auto& it = input[i];
        double r = std::sqrt(it.pos_x * it.pos_x + it.pos_y * it.pos_y + it.pos_z * it.pos_z);
        if (r < kMinRadarRange) {
            std::cout << "[RadarProcessing] Skipping input[" << i << "] at range " << r << " m (below " << kMinRadarRange << ")" << std::endl;
            continue;
        }
        myRadar.push_back(createObject(input[i])); // תוקן: הוסר ה-'&' השגוי
    }
       
    return myRadar;
}

/*
// אם תרצה להשתמש בזה בעתיד, שים לב שגם פה צריך להיות RadarMeasurement
std::vector<RadarMeasurement> RadarProcessing::Clean(std::vector<RadarMeasurement> r) {
    std::vector<RadarMeasurement> filtered;
    for (const auto& obj : r) {
        // סינון בסיסי: מתעלם ממדידות ריקות שנמצאות בראשית הצירים (0,0)
        if (std::abs(obj.pos_x) > 0.001 || std::abs(obj.pos_y) > 0.001) {
            filtered.push_back(obj);
        }
    }
    return filtered;
}
*/

// תוקן: RadarMeasurement במקום RadarMeasurement
RadarObject RadarProcessing::createObject(RadarMeasurement& raw_item) {
    RadarObject RO;
    RO.timestamp = this->ts;
    
    // השמת הנתונים התלת-ממדיים המלאים שהגיעו מהחיישן
    RO.position.x() = raw_item.pos_x;
    RO.position.y() = raw_item.pos_y;
    RO.position.z() = raw_item.pos_z; 
    
    RO.velocity.x() = raw_item.vel_x;
    RO.velocity.y() = raw_item.vel_y;
    RO.velocity.z() = raw_item.vel_z;
    RO.range = std::sqrt(raw_item.pos_x * raw_item.pos_x + 
                         raw_item.pos_y * raw_item.pos_y + 
                         raw_item.pos_z * raw_item.pos_z);

    double R = RO.range;

    // 2. מודל RCS דינמי דועך לפי המרחק (R)
    // נניח ערך מקסימלי קרוב של 20 (כמו רכב), שחוסם ודועך ככל שמתרחקים
    if (R < 1.0) R = 1.0; // הגנה מחלוקה באפס או ערכים קרובים מדי
    RO.rcs = 20.0 / (R * 0.1); 
            
    // חסימת ערכים כדי שה-RCS יישאר הגיוני (למשל בין 0.5 ל-30.0)
    if (RO.rcs < 0.5)  RO.rcs = 0.5;
    if (RO.rcs > 30.0) RO.rcs = 30.0;
    
    double conf_rcs = std::min(1.0, RO.rcs / 40.0);
    double total_speed = std::sqrt(RO.velocity.x() * RO.velocity.x() + 
                                   RO.velocity.y() * RO.velocity.y() + 
                                   RO.velocity.z() * RO.velocity.z());

    double conf_vel = 1.0;
    if (total_speed > 0.1) {
        conf_vel = std::max(0.2, 1.0 - (total_speed / 50.0)); 
    }
    
    RO.confidence = conf_vel * conf_rcs;
    RO.confidence = std::clamp(RO.confidence, 0.1, 1.0);
    
    RadarData RD;
    RD.process(RO);
    
    return RO;
}