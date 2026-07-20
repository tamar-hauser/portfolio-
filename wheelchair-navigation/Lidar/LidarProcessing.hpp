#pragma once
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

#include <yaml-cpp/yaml.h>
#include "SensorProcessing.hpp"
#include "LidarObject.hpp"
#include "SensorConfig.hpp"
#include "ROI_box.hpp"

class LidarProcessing : public SensorProcessing<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, std::vector<LidarObject>> {
public:
    LidarProcessing() = default; 
    std::vector<LidarObject> process(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& raw_clouds,double tss) override;
    pcl::PointCloud<pcl::PointXYZI>::Ptr getGlobalCloud() const { return globalCloud; }

private:
    void processLidar(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& raw_clouds);

    pcl::PointCloud<pcl::PointXYZI>::Ptr createcluod(pcl::PointIndices indices);
    
    void extractGeometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, LidarObject& LO);
    float calculateConfidence(const LidarObject& LO, const ROI_box& params);

    LidarObject createObject(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void LoadingConfiglidar(const YAML::Node& node);
    
    std::vector<pcl::PointIndices> output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalCloud;
    ROI_box roi;
    SensorLidarConfig Config;
    double ts;
};