#include "RadarData.hpp"
#include <Eigen/Dense>
#include <cmath>
#include "SensorConfig.hpp"



void RadarData::process(RadarObject& RO) {
    auto radar_cfg = ConfigManager::getRadar("front_radar");
    Eigen::Vector3f pos_sensor(RO.position.x(), RO.position.y(), RO.position.z());
    Eigen::Vector3f pos_base = radar_cfg.transformPoint(pos_sensor);
    RO.position = pos_base.cast<double>();
     
    buildZ(RO); // בונה את וקטור המדידה
    buildH(RO); // מחשבת את מטריצת המעבר
    buildR(RO); // קובעת את הרעש הדינמי

}
// בניית וקטור המדידה Z (מבוסס על קואורדינטות קרטזיות של הראדאר)[cite: 10]
void RadarData::buildZ( RadarObject& RO) {
// במידה וקבענו בבנאי ש-Z הוא בגודל 3 (DimTrackedObject)
// בתוך פונקציית העדכון של הראדאר
    RO.Z(0) = RO.position.x();
    RO.Z(1) = RO.position.y();
    RO.Z(2) = RO.position.z();
    RO.Z(3) = RO.velocity.x();
    RO.Z(4) = RO.velocity.y();
    RO.Z(5) = RO.velocity.z();
}


void RadarData::buildH(RadarObject& RO) {
    RO.H.setZero(); // חשוב מאוד לאפס לפני שכותבים!

    double x = RO.position.x(); 
    double y = RO.position.y();
    double rho2 = x*x + y*y;
    double rho = std::sqrt(rho2);

    // הגנה מפני חלוקה באפס
    if (rho < 0.001) {
        // ערך ברירת מחדל בטוח
        RO.H(0, static_cast<int>(Config::StateMembersRobot::StateX)) = 1.0;
        RO.H(1, static_cast<int>(Config::StateMembersRobot::StateY)) = 1.0;
    } else {
        // הנגזרות של הקואורדינטות הקוטביות
        // H[0,:] היא הנגזרת של rho לפי x ו-y
        RO.H(0, static_cast<int>(Config::StateMembersRobot::StateX)) = x / rho;
        RO.H(0, static_cast<int>(Config::StateMembersRobot::StateY)) = y / rho;
        
        // H[1,:] היא הנגזרת של theta לפי x ו-y
        RO.H(1, static_cast<int>(Config::StateMembersRobot::StateX)) = -y / rho2;
        RO.H(1, static_cast<int>(Config::StateMembersRobot::StateY)) = x / rho2;
    }
    
    // אם הראדאר מודד גם מהירות (Doppler), הנגזרות לפי Vx ו-Vy הם 1
    RO.H(3, static_cast<int>(Config::StateMembersRobot::StateVx)) = 1.0;
    // וכן הלאה לכל איבר שאת מודדת...
}

// void RadarData::buildH(RadarObject& RO) {
//     //לשנות שהמטריצות יכילו את  ווקטור המצב ויהוהי יעקביין
//     double x = RO.position.x(); 
//     double y = RO.position.y();
//     double z = RO.position.z();
//     double rho2 = x*x + y*y;
//     double rho = std::sqrt(rho2);
//     if (rho < 0.0001) {
//         RO.H(0, 0) = 1.0;
//         RO.H(1, 1) = 1.0;
//         RO.H(2,2) = 1.0;
//     } else {

//         RO.H(0, 0) = x / rho;
//         RO.H(0, 1) = y / rho;
//         RO.H(1, 0) = -y / rho2;
//         RO.H(1, 1) = x / rho2;
//     }
//     RO.H(2, 2) = 1.0;
//     RO.H(3, 3) = 1.0;
//     RO.H(4, 4) = 1.0;
// }

// בניית מטריצת הרעש R המושפעת מה-Confidence הדינמי
void RadarData::buildR( RadarObject& RO) {
    //שגיאה בסיסית שצריך בעיקרון לשמור בקובץ נפרד
    double base_sigma = 0.05;
    // התאמה דינמית: ככל שה-RCS קטן, השגיאה גדלה
    double rcs_factor =10.0;
    if(RO.rcs > 0)
     { rcs_factor=1.0 / std::sqrt(RO.rcs);}
            // התאמה למרחק: ככל שהאובייקט רחוק, השגיאה גדלה
    double range_factor = 1.0 + (RO.range * 0.1); 
    double sigma = base_sigma * rcs_factor * range_factor;
    double final_variance = sigma * sigma;
   // מילוי המטריצה בערך הדינמי שחושב
    RO.R.setIdentity();
    RO.R *= final_variance;
   
}

