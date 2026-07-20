#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "SensorList.hpp"
#include "LidarObject.hpp" 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct LidarList : public SensorList<LidarObject> {
    pcl::PointCloud<pcl::PointXYZI>::Ptr Icp_Cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud;
    Eigen::Matrix3d delta_T_lidar;

    LidarList()
        : Icp_Cloud(new pcl::PointCloud<pcl::PointXYZI>()),
          global_cloud(new pcl::PointCloud<pcl::PointXYZI>()),
          delta_T_lidar(Eigen::Matrix3d::Zero())
    {
    }
};