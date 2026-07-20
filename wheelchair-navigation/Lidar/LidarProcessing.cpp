// 1. C++ Standard Libraries
#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <memory>

// 2. Eigen
#include <Eigen/Dense>

// 3. PCL - Base, Types & IO
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

// 4. PCL - Copy & Implementation
#include <pcl/common/io.h>

// 5. PCL - Search & Features
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

// 6. PCL - Search & Features Implementation
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/impl/normal_3d.hpp>

// 7. Project Headers
#include "LidarProcessing.hpp"
#include "LidarObject.hpp" 
#include "LidarCloud.hpp"
#include "LidarProcess.hpp"
#include "SensorConfig.hpp"
#include "LidarData.hpp"

std::vector<LidarObject> LidarProcessing::process(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& raw_clouds,double tss) 
{
    this->ts=tss;
    processLidar(raw_clouds);
    std::vector<LidarObject> mylidar;

    if (!this->globalCloud || this->globalCloud->empty()) {
        return mylidar;
    }

    for (size_t i = 0; i < this->output.size(); i++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr objectCloud = createcluod(this->output[i]);
        if (objectCloud && !objectCloud->empty()) {
            mylidar.push_back(createObject(objectCloud));
        }
    }

    return mylidar;
}

void LidarProcessing::processLidar(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& raw_clouds) {
    if (raw_clouds.size() < 3) {
        std::cerr << "Error: Not enough raw clouds provided!" << std::endl;
        return;
    }

    auto config = ConfigManager::getLidar("front_lidar");
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(config.translation);
    transform.rotate(config.rotation);
    LidarProcess lidarfront(transform, ConfigManager::getROI().voxelSize);
    if (raw_clouds[0]) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr processed = lidarfront.process(raw_clouds[0]);
        raw_clouds[0] = processed;
    }

    config = ConfigManager::getLidar("left_lidar");
    transform = Eigen::Affine3f::Identity();
    transform.translate(config.translation);
    transform.rotate(config.rotation);
    LidarProcess lidar_left(transform, ConfigManager::getROI().voxelSize);
    if (raw_clouds[1]) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr processed = lidar_left.process(raw_clouds[1]);
        raw_clouds[1] = processed;
    }

    config = ConfigManager::getLidar("right_lidar");
    transform = Eigen::Affine3f::Identity();
    transform.translate(config.translation);
    transform.rotate(config.rotation);
    LidarProcess lidar_right(transform, ConfigManager::getROI().voxelSize);
    if (raw_clouds[2]) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr processed = lidar_right.process(raw_clouds[2]);
        raw_clouds[2] = processed;
    }

    LidarCloud globalProcessor(this->roi);
    globalProcessor.processFrame(raw_clouds);
    this->globalCloud = globalProcessor.getFinalCloud();
    this->output = globalProcessor.getObjects(); 
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcessing::createcluod(pcl::PointIndices indices) {
    auto cloud_out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();    
    if (!this->globalCloud || this->globalCloud->empty()) {
        return cloud_out;
    }
    pcl::copyPointCloud(*this->globalCloud, indices, *cloud_out);
    return cloud_out;
}

void LidarProcessing::extractGeometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, LidarObject& LO) {
    if (!cloud || cloud->empty()) return;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    LO.center = centroid;
    LO.position = centroid.head<3>(); 
    pcl::computeCovarianceMatrix(*cloud, centroid, LO.covariance_matrix); 
    pcl::PointXYZI min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    LO.length = max_pt.x - min_pt.x;
    LO.width  = max_pt.y - min_pt.y;
    LO.height = max_pt.z - min_pt.z;

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);

    if (!cloud_normals->empty()) {
        LO.normal = Eigen::Vector4f(cloud_normals->points[0].normal_x, 
                                   cloud_normals->points[0].normal_y, 
                                   cloud_normals->points[0].normal_z, 0.0f);
        LO.curvature = cloud_normals->points[0].curvature; 
    }

    // Ensure measurement vector has correct size and assign safely (avoid comma-initializer mismatches)
    if (LO.Z.size() != Config::SensorFusionLidar) {
        LO.Z.resize(Config::SensorFusionLidar);
    }
    LO.Z(0) = static_cast<double>(LO.position.x());
    LO.Z(1) = static_cast<double>(LO.position.y());
    LO.Z(2) = static_cast<double>(LO.position.z());
    LO.Z(3) = static_cast<double>(LO.yaw);
}

float LidarProcessing::calculateConfidence(const LidarObject& LO, const ROI_box& params) {
    if (!LO.cloud) return 0.1f;

    double point_count = static_cast<double>(LO.cloud->size());
    double conf_points = std::clamp((point_count - params.minPointsInCloud) / 50.0, 0.1, 1.0);

    double conf_surface = std::max(0.1, 1.0 - (static_cast<double>(LO.curvature) * 5.0));

    double distance = LO.position.norm();
    double conf_dist = std::max(0.1, 1.0 - (distance / 40.0)); 

    float final_conf = static_cast<float>(conf_points * conf_surface * conf_dist);

    return std::clamp(final_conf, 0.1f, 1.0f); 
}

LidarObject LidarProcessing::createObject(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    LidarObject LO; 
    LO.timestamp = this->ts;
    LO.cloud = cloud; 
    extractGeometry(LO.cloud, LO);
    LO.confidence = calculateConfidence(LO, this->roi);
    LidarData LD;
    LD.process(LO);
    return LO;
}
