#pragma once
#include "Constants.hpp" // וודאי ששם הקובץ תואם (אותיות גדולות/קטנות)
#include "SensorObject.hpp" // וודאי ששם הקובץ תואם (אותיות גדולות/קטנות)
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

struct CameraObject : public SensorObject {
    std::string type_label;
    float length, width, height;
    cv::Point2f position_2d;
    Eigen::Vector3f position_3d;
    cv::Rect bounding_box;
    std::string traffic_light_color;
    CameraObject() : SensorObject(Config::StateSizeObject, Config::MeasurementSizeCamera, SensorType::Camera,false,true,false),
                     type_label("unknown"), 
                     length(0.0f), width(0.0f), height(0.0f),
                     position_2d(0.0f, 0.0f),
                     bounding_box(0, 0, 0, 0),
                     traffic_light_color("unknown")
    {
        position_3d.setZero();
    }
};

