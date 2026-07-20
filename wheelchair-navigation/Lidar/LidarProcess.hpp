#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>

class LidarProcess {
public:
    LidarProcess(const Eigen::Affine3f& calibration_matrix, float voxel_size);
    pcl::PointCloud<pcl::PointXYZI>::Ptr process(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_in);
private:
    Eigen::Affine3f calibration_matrix;
    float voxel_size;
    pcl::PointCloud<pcl::PointXYZI>::Ptr applyVoxelGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const;
    pcl::PointCloud<pcl::PointXYZI>::Ptr applyTransform(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const;
};
