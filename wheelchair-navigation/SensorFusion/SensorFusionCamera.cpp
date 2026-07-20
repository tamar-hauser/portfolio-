#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>
#include "SensorObject.hpp"
#include "Camera/CameraObject.hpp"
#include "SensorFusionCamera.hpp"
#include "Constants.hpp" 
#include "SensorFusionObject.hpp" 
#include "SensorFusionM.hpp"

SensorFusionObject SensorFusionCamera::createNewObject(CameraObject& CO) 
{
    SensorFusionObject sf;
    
    // 1. אתחול וקטור המצב x (גודל 8)
    sf.filter.x.setZero();
    
    // העתקת מיקום (X, Y, Z) מהמצלמה לפי האינדקסים
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateX)) = static_cast<double>(CO.Z(0));
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateY)) = static_cast<double>(CO.Z(1));
    sf.filter.x(static_cast<int>(Config::StateIndicesObject::StateZ)) = static_cast<double>(CO.Z(2));
    
    // 2. אתחול מטריצת האי-וודאות P
    sf.filter.P.setIdentity();
    
    sf.filter.P.block<Config::MeasurementSizeCamera, Config::MeasurementSizeCamera>(0, 0) = 
    CO.R.topLeftCorner<Config::MeasurementSizeCamera, Config::MeasurementSizeCamera>().cast<double>();

    // הגדרת אי-וודאות גבוהה לרכיבים שלא נמדדו ישירות
    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateVx), static_cast<int>(Config::StateIndicesObject::StateVx)) = 15.0; 
    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateVy), static_cast<int>(Config::StateIndicesObject::StateVy)) = 15.0;
    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateVz), static_cast<int>(Config::StateIndicesObject::StateVz)) = 15.0;
    
    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateYaw), static_cast<int>(Config::StateIndicesObject::StateYaw)) = 1.0; 
    sf.filter.P(static_cast<int>(Config::StateIndicesObject::StateYawRate), static_cast<int>(Config::StateIndicesObject::StateYawRate)) = 1.0;

    // 3. מילוי מימדים גאומטריים
    sf.length = CO.length;
    sf.width  = CO.width;
    sf.height = CO.height;

    // 4. מטא-דאטה ודגלים
    sf.type_label = CO.type_label;
    sf.timestamp  = CO.timestamp;
    sf.confidence = CO.confidence;
    sf.has_camera = true;
    sf.has_lidar  = false;
    sf.has_radar  = false;
    sf.traffic_light_color=CO.traffic_light_color;
    sf.img_points.clear();
    sf.img_points.push_back(CO.position_2d);
    sf.bounding_box=CO.bounding_box;
    return sf;
}

void SensorFusionCamera::updateObject(CameraObject& co, SensorFusionObject& sf) {
    float alpha = this->calculateAlpha(co, sf);

    sf.length = (alpha * sf.length) + ((1.0f - alpha) * co.length);
    sf.width  = (alpha * sf.width)  + ((1.0f - alpha) * co.width);
    sf.height = (alpha * sf.height) + ((1.0f - alpha) * co.height);

    Eigen::Matrix<double, Config::MeasurementSizeCamera, 1> z = 
        co.Z.head<Config::MeasurementSizeCamera>().cast<double>();

    Eigen::Matrix<double, Config::MeasurementSizeCamera, Config::StateSizeObject> H = 
        co.H.block<Config::MeasurementSizeCamera, Config::StateSizeObject>(0, 0).cast<double>();

    Eigen::Matrix<double, Config::MeasurementSizeCamera, Config::MeasurementSizeCamera> R = 
        co.R.block<Config::MeasurementSizeCamera, Config::MeasurementSizeCamera>(0, 0).cast<double>();

    Eigen::Matrix<double, Config::MeasurementSizeCamera, 1> z_pred = H * sf.filter.x;

    sf.filter.update<Config::MeasurementSizeCamera>(z, H, R, z_pred);
    sf.bounding_box=co.bounding_box;
    sf.traffic_light_color=co.traffic_light_color;

    sf.timestamp = co.timestamp;
    sf.has_camera = true;
    sf.confidence = (alpha * sf.confidence) + ((1.0f - alpha) * co.confidence);
}
// float SensorFusionCamera::calculate_cost(
//     CameraObject& co,
//     SensorFusionObject& sf)
// {
//     return SensorFusionM<
//         CameraObject,
//         SensorFusionObject>::calculate_cost(co, sf);
// }