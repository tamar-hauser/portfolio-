#include "GpsData.hpp"
#include "Constants.hpp"
#include "GpsObject.hpp"
#include <cmath>
#include <algorithm>
#include "GPSLocation.hpp"
#include "SensorConfig.hpp"
#include "Thread/NavigationManager.hpp"

// הפונקציה המרכזית המסנכרנת את ה-Pipeline
void GpsData::process(GpsObject& GO) {
    // שלב 1: המרת lat/lon (דרגות) למטרים מקומיים ביחס ל-origin
    // GpsProcessing::createObject שמר: x_local=latitude, y_local=longitude
    // NavigationManager: StateX=East (lon-based), StateY=North (lat-based)
    auto map = NavigationManager::getInstance().getMapData();
    if (map && map->origin_lat != 0.0 && map->origin_lon != 0.0) {
        const double raw_lat = GO.x_local;
        const double raw_lon = GO.y_local;
        const double MPD = 111320.0;
        const double lat_rad = map->origin_lat * M_PI / 180.0;
        GO.x_local = (raw_lon - map->origin_lon) * MPD * std::cos(lat_rad); // East → StateX
        GO.y_local = (raw_lat - map->origin_lat) * MPD;                     // North → StateY
    }

    // שלב 2: הזזת נקודת ה-GPS לנקודת המוצא של המסגרת (extrinsic transform)
    auto gps_cfg = ConfigManager::getGps("gps_main");
    Eigen::Vector3f pos_sensor(static_cast<float>(GO.x_local),
                               static_cast<float>(GO.y_local),
                               static_cast<float>(GO.z_local));
    Eigen::Vector3f pos_base = gps_cfg.transformPoint(pos_sensor);
    GO.x_local = pos_base.x();
    GO.y_local = pos_base.y();
    GO.z_local = pos_base.z();

    buildZ(GO);
    buildH(GO);
}


void GpsData::buildZ(GpsObject& GO) {
    GO.Z(0) = GO.x_local; // StateX
    GO.Z(1) = GO.y_local; // StateY
    GO.Z(2) = GO.z_local; // StateZ
    GO.Z(3) = M_PI / 2.0 - GO.heading * M_PI / 180.0; // compass [CW from North] → ENU yaw [CCW from East]
    GO.Z(4) = GO.speed;   // StateVx
}
 
void GpsData::buildH(GpsObject& GO) {
    GO.H.setZero(); 
    // תיקון 1: שינוי השם ל-RobotLocalization בהתאם להגדרות ה-Constants שלך
    GO.H(0,0) = 1.0; 
    GO.H(1,1) = 1.0;
    GO.H(2,2) = 1.0;
    GO.H(3,4) = 1.0;  // heading → StateYaw (index 4)
    GO.H(4,5) = 1.0;  // speed   → StateVx  (index 5)
}

void GpsData::buildR(GpsObject& GO) {}

// void GpsData::buildR(GpsObject& GO)
// {
//     GO.R.setZero();
//     double base_pos_noise   = 0.25; // X,Y variance
//     double base_alt_noise   = 1.0;  // Z variance
//     double base_speed_noise = 0.04; // Vx variance
//     double base_yaw_noise   = 0.01; // Yaw variance
//     double confidence =std::clamp(GO.confidence, 0.0, 1.0);
//     double dynamic_scale;
//     if (confidence > 0.01)
//     {
//         dynamic_scale =
//             1.0 / (confidence * confidence);
//     }
//     else
//     {
//         dynamic_scale = 10000.0;
//     }
//     GO.R(0,0) =
//         base_pos_noise * dynamic_scale;
//     GO.R(1,1) =
//         base_pos_noise * dynamic_scale;
//     GO.R(2,2) =
//         base_alt_noise * dynamic_scale;
//     GO.R(3,3) =
//         base_speed_noise * dynamic_scale;
//     GO.R(4,4) =
//         base_yaw_noise * dynamic_scale;
// }

// Eigen::MatrixXd GpsData::buildF(const GpsObject& GO) {
//     double dt = 0.01; // קבוע זמן המערכת
//     Eigen::MatrixXd F_mat = Eigen::MatrixXd::Identity(8, 8);
    
//     // קינמטיקה של הכיסא: עדכון מיקום לפי מהירות קווית
//     F_mat(RobotLocalization::StateX, RobotLocalization::StateVx) = dt; // X = X + Vx*dt
    
//     // עדכון כיוון לפי מהירות זוויתית
//     F_mat(RobotLocalization::StateYaw, RobotLocalization::StateVyaw) = dt; // Yaw = Yaw + Vyaw*dt
    
//     // עדכון מהירות לפי תאוצה
//     F_mat(RobotLocalization::StateVx, RobotLocalization::StateAx) = dt; // Vx = Vx + Ax*dt
    
//     return F_mat;
// }