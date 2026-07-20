#pragma once
#include "Constants.hpp"   
#include "SensorObject.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <memory>
#include <boost/make_shared.hpp>

struct LidarObject : public SensorObject {
    Eigen::Vector3f position;
    float yaw;
    float length, width, height;
    Eigen::Vector4f normal;
    float curvature;
    Eigen::Vector4f center;
    Eigen::Matrix3f covariance_matrix;
    std::pair<int, int> matrix_position;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

    LidarObject() : 
        // SensorObject expects (stateSize, measurementSize)
        SensorObject(Config::StateSizeObject, Config::SensorFusionLidar, SensorType::Lidar ,true,false,false), 
        position(0, 0, 0), 
        yaw(0.0f), 
        length(0.0f), width(0.0f), height(0.0f), 
        curvature(0.0f),
        center(0, 0, 0, 1),
        matrix_position({0, 0})
    {
        this->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>()); 
        this->covariance_matrix.setZero(); 
        this->normal.setZero();
    } 
};