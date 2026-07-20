#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>
#include "SensorObject.hpp"
#include "LidarObject.hpp"
#include "SensorFusionLidar.hpp"
#include "SensorFusionObject.hpp"
#include "Constants.hpp" 
#include "SensorFusionM.hpp"

SensorFusionObject SensorFusionLidar::createNewObject(LidarObject& LO) {
    SensorFusionObject sf;
    
    sf.filter.x.setZero();
    
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateX)) = static_cast<double>(LO.Z(0));
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateY)) = static_cast<double>(LO.Z(1));
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateZ)) = static_cast<double>(LO.Z(2));
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateYaw)) = static_cast<double>(LO.Z(3));
    
    sf.filter.P.setIdentity();
    
    sf.filter.P.block<Config::MeasurementSizeLidar, Config::MeasurementSizeLidar>(0, 0) = 
    LO.R.topLeftCorner<Config::MeasurementSizeLidar, Config::MeasurementSizeLidar>().cast<double>();

    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateVx), static_cast<int>(Config::StateIndicesObject::StateVx)) = 15.0; 
    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateVy), static_cast<int>(Config::StateIndicesObject::StateVy)) = 15.0;
    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateVz), static_cast<int>(Config::StateIndicesObject::StateVz)) = 15.0;
    
    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateYawRate), static_cast<int>(Config::StateIndicesObject::StateYawRate)) = 1.0;

    sf.length = LO.length;
    sf.width  = LO.width;
    sf.height = LO.height;
    sf.yaw = LO.yaw;
    
    sf.timestamp  = LO.timestamp;
    sf.confidence = LO.confidence;
    sf.has_camera = false;
    sf.has_lidar  = true;
    sf.has_radar  = false;
    sf.cloud = LO.cloud;
    sf.img_points.clear();

    return sf;
}

void SensorFusionLidar::updateObject(LidarObject& lo, SensorFusionObject& sf) {
    float alpha = this->calculateAlpha(lo, sf);

    sf.length = (alpha * sf.length) + ((1.0f - alpha) * lo.length);
    sf.width  = (alpha * sf.width)  + ((1.0f - alpha) * lo.width);
    sf.height = (alpha * sf.height) + ((1.0f - alpha) * lo.height);
    sf.yaw = (alpha * sf.yaw) + ((1.0f - alpha) * lo.yaw);

    // תוקנה הכפילות בשם של Config
    double current_yaw = sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateYaw));
    double measured_yaw = static_cast<double>(lo.yaw);
    
    double yaw_diff = measured_yaw - current_yaw;
    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateYaw)) = current_yaw + (1.0 - alpha) * yaw_diff;

    Eigen::Matrix<double, Config::MeasurementSizeLidar, 1> z = 
        lo.Z.head<Config::MeasurementSizeLidar>().cast<double>();

    Eigen::Matrix<double, Config::MeasurementSizeLidar, Config::StateSizeObject> H = 
        lo.H.block<Config::MeasurementSizeLidar, Config::StateSizeObject>(0, 0).cast<double>();

    Eigen::Matrix<double, Config::MeasurementSizeLidar, Config::MeasurementSizeLidar> R = 
        lo.R.block<Config::MeasurementSizeLidar, Config::MeasurementSizeLidar>(0, 0).cast<double>();

    Eigen::Matrix<double, Config::MeasurementSizeLidar, 1> z_pred = H * sf.filter.x;

    sf.filter.update<Config::MeasurementSizeLidar>(z, H, R, z_pred);

    sf.timestamp = lo.timestamp;
    sf.has_lidar = true;
    sf.confidence = (alpha * sf.confidence) + ((1.0f - alpha) * lo.confidence);
}

// float SensorFusionLidar::calculate_cost(
//     LidarObject& ro,
//     SensorFusionObject& sensor_object)
// {
//     return SensorFusionM<
//         LidarObject,
//         SensorFusionObject>::calculate_cost(ro, sensor_object);
// }