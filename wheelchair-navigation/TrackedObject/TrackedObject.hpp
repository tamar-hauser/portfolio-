#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp> // הוספת אינקלוד עבור cv::Point2f
#include <string>
#include <vector>
#include "KalmanFilter/ekf.hpp"
#include "SensorObject.hpp"
#include "Constants.hpp"


struct TrackedObject : public SensorObject {
    std::string type_label;
    std::string traffic_light_color;
    int frames_active;
    int missed_frames_counter;
    bool updated_this_frame;
    EKF<Config::StateSizeObject> filter;
    Eigen::MatrixXd F;
    float length, width, height;
    float curvature;        
    Eigen::Vector4f normal;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud; 
    std::vector<cv::Point2f> image_points;    
    cv::Rect bounding_box;
    std::vector<Eigen::Vector3f> path;         
    Eigen::Matrix3f point_cloud_covariance;   
    TrackedObject()
        : SensorObject( Config::StateSizeObject, Config::MeasurementSizeSensorObgect,SensorType::TrackedObject,false,false,false),
          traffic_light_color("unknown"),
          frames_active(0),
          missed_frames_counter(0),
          updated_this_frame(false),
          length(0.0f),
          width(0.0f),
          height(0.0f),
          curvature(0.0f),
          bounding_box(0, 0, 0, 0),
          cloud(new pcl::PointCloud<pcl::PointXYZI>()) 
    {
        filter.x.setZero();
        filter.P.setIdentity();
        F.setZero();
        normal.setZero();
        point_cloud_covariance.setZero();        
        confidence = 0.0f;        
        image_points.clear();
        path.clear();
    }

 
    Eigen::Vector3f position() const { 
        return filter.x.template head<3>().cast<float>(); 
    }
    
    Eigen::Vector3f velocity() const override{ 
        return filter.x.template segment<3>(3).cast<float>(); 
    }
    float get_speed_kph() const {
        Eigen::Vector3f vel = velocity();
        return vel.norm() * 3.6f;
    }

    float get_acceleration_norm() const {
        if (Config::StateSizeObject >= 9) {
            Eigen::Vector3f acc = filter.x.template tail<3>().cast<float>();
            return acc.norm();
        }
        return 0.0f;
    }
    float get_travel_yaw() const {
        Eigen::Vector3f vel = velocity();
        return std::atan2(vel.y(), vel.x());
    }
    void init(const Eigen::Vector3f& pos, float initial_confidence) {
        filter.x.setZero();
        filter.x.template head<3>() = pos.cast<double>();       
        filter.P = Eigen::Matrix<double, Config::StateSizeObject, Config::StateSizeObject>::Identity() * 1.0;        
        if (Config::StateSizeObject > 3) {
            filter.P.template block<Config::StateSizeObject - 3, Config::StateSizeObject - 3>(3, 3) *= 10.0;
        }
        
        confidence = initial_confidence;
        path.push_back(pos);
    }
};