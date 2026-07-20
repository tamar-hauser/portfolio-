#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "SensorObject.hpp"
#include "Constants.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "KalmanFilter/ekf.hpp"

struct SensorFusionObject:public SensorObject {
    std::string type_label;
    std::string traffic_light_color;
    double timestamp;

    // [ px, py, pz, vx, vy, vz, yaw, yaw_rate ]
    // Eigen::Matrix<float, 8, 1> state; 
    // Eigen::Matrix<float, 8, 8> P; 
    EKF<Config::StateSizeObject> filter;
    Eigen::MatrixXd F;
    float length, width, height;
    float yaw; 
    
    // 4. נתונים גולמיים
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    // רשימת נקודות מהתמונה (אם משתמשים ב-OpenCV cv::Point2f)
    std::vector<cv::Point2f> img_points; 
    cv::Rect bounding_box;

    Eigen::Vector3f position() const { 
        return filter.x.head<3>().cast<float>(); 
    }  

    float get_speed_kph() const {
        Eigen::Vector3f vel = filter.x.segment<3>(3).cast<float>();
        return vel.norm() * 3.6f;
    }
    Eigen::Vector3f velocity() const { 
        return filter.x.template segment<3>(3).cast<float>(); 
    }
    // אתחול האובייקט לאחר האיחוד הראשוני של החיישנים
    SensorFusionObject()
    :SensorObject(Config::StateSizeObject, Config::MeasurementSizeSensorObgect, SensorType::SensorObject,false,false,false),
     type_label("unknown"),
     traffic_light_color("unknown"),
      timestamp(0.0),

      length(0.0f),
      width(0.0f),
      height(0.0f),
      yaw(0.0f),
      bounding_box(0, 0, 0, 0),
      cloud(new pcl::PointCloud<pcl::PointXYZI>())
      {        
        filter.x.setZero();
        filter.P.setIdentity();        
        F.setZero();
        confidence = 0.0f;
        img_points.clear();
     } 
};