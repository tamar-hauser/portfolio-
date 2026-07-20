#include "ImuData.hpp"
#include "SensorConfig.hpp"
#include "Constants.hpp"
#include <algorithm> // חובה בשביל std::max
#include <cmath>     // חובה בשביל std::cos
#include <Eigen/Dense>
#include <iostream>
// הפונקציה המרכזית שמתקשרת עם ה-Processing
void ImuData::process(ImuObject& IO) {
    auto imu_cfg = ConfigManager::getImu("imu_main");

    // שנה ל-Vector3f כדי להתאים ל-Matrix3f של ה-Config
    Eigen::Vector3f accel_sensor(static_cast<float>(IO.Ax), 0.0f, 0.0f);

    // כעת הטיפוסים תואמים
    Eigen::Vector3f accel_base = imu_cfg.transformVector(accel_sensor);
    IO.Ax = static_cast<double>(accel_base.x());

    buildZ(IO);
    buildH(IO);
    buildR(IO);

    std::cout << "[IMU_EKF_DEBUG] Ax=" << IO.Ax << " Vyaw=" << IO.Vyaw
              << " Pitch_blocked=1 Yaw_blocked=1 H_nonzero=" << (IO.H.array() != 0.0).count() << std::endl;
}

// עדכון EKF ישיר מה-IMU מוגבל כרגע ל-Ax ו-Vyaw בלבד.
// Pitch/Yaw לא מתעדכנים: raw_mag_y/raw_mag_z הם ערכי מגנטומטר גולמיים ולא זוויות מכוילות,
// והזרמתם ל-EKF תשחית את StatePitch/StateYaw. תיקון Yaw אמיתי ייכנס בעתיד דרך
// heading של GPS (yaw = M_PI/2 - heading_rad) או מגנטומטר מכוייל (atan2 + tilt compensation).
void ImuData::buildZ(ImuObject& IO) {
    IO.Z(0) = IO.Ax;
    IO.Z(1) = IO.Vyaw;
}

void ImuData::buildH(ImuObject& IO) {
    IO.H.setZero();
    IO.H(0, static_cast<int>(Config::StateMembersRobot::StateAx))   = 1.0; // Ax   → StateAx
    IO.H(1, static_cast<int>(Config::StateMembersRobot::StateVyaw)) = 1.0; // Vyaw → StateVyaw
}

void ImuData::buildR(ImuObject& IO) {
    float base_ax_noise   = 0.1f;
    float base_vyaw_noise = 0.03f;

    double confidence_scale = 1.0 / std::max(IO.confidence, 0.01);

    IO.R(0, 0) = base_ax_noise   * confidence_scale;
    IO.R(1, 1) = base_vyaw_noise * confidence_scale;
}