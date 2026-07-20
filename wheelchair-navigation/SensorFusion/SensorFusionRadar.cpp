#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>
#include "SensorObject.hpp"
#include "SensorFusionObject.hpp"
#include "Radar/RadarObject.hpp"
#include "SensorFusionRadar.hpp"
#include "Constants.hpp" 
#include "SensorFusionM.hpp"

SensorFusionObject SensorFusionRadar::createNewObject(RadarObject& RO) {
    SensorFusionObject sf;
    
    sf.filter.x.setZero();
    
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateX)) = static_cast<double>(RO.Z(0));
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateY)) = static_cast<double>(RO.Z(1));
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateZ)) = static_cast<double>(RO.Z(2));
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateVx)) = static_cast<double>(RO.Z(3));
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateVy)) = static_cast<double>(RO.Z(4));
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateVz)) = static_cast<double>(RO.Z(5));

    sf.filter.P.setIdentity();
    
    sf.filter.P.block<Config::MeasurementSizeLidar, Config::MeasurementSizeLidar>(0, 0) = 
    RO.R.topLeftCorner<Config::MeasurementSizeLidar, Config::MeasurementSizeLidar>().cast<double>();
    
    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateYaw), static_cast<int>(Config::StateIndicesObject::StateYaw)) = 1.0; 
    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateYawRate), static_cast<int>(Config::StateIndicesObject::StateYawRate)) = 1.0;
    
    sf.timestamp  = RO.timestamp;
    sf.confidence = RO.confidence;
    sf.has_camera = false;
    sf.has_lidar  = false;
    sf.has_radar  = true;
    sf.img_points.clear();

    return sf;
}

void SensorFusionRadar::updateObject(RadarObject& ro, SensorFusionObject& sf) {
    float alpha = calculateAlpha(ro, sf);
    
    Eigen::Matrix<double, Config::MeasurementSizeRadar, 1> z = 
        ro.Z.head<Config::MeasurementSizeRadar>().cast<double>();

    Eigen::Matrix<double, Config::MeasurementSizeRadar, Config::StateSizeObject> H = 
        ro.H.block<Config::MeasurementSizeRadar, Config::StateSizeObject>(0, 0).cast<double>();

    Eigen::Matrix<double, Config::MeasurementSizeRadar, Config::MeasurementSizeRadar> R = 
        ro.R.block<Config::MeasurementSizeRadar, Config::MeasurementSizeRadar>(0, 0).cast<double>();

    Eigen::Matrix<double, Config::MeasurementSizeRadar, 1> z_pred = H * sf.filter.x;

    sf.filter.update<Config::MeasurementSizeRadar>(z, H, R, z_pred);

    sf.timestamp = ro.timestamp;
    sf.has_radar = true;
    sf.confidence = (alpha * sf.confidence) + ((1.0f - alpha) * ro.confidence);
}