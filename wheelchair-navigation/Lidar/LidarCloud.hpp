#pragma once
#include <vector>
#include <string>
#include <atomic>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ROI_box.hpp"
#include "Lidar/LidarProcess.hpp"
#include <pcl/PointIndices.h>
#include "SensorConfig.hpp"

class LidarCloud {
public:
    LidarCloud(const ROI_box& params);
    void addLidar(const LidarProcess& lidarprocess);

    void processFrame(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& raw_clouds);

    pcl::PointCloud<pcl::PointXYZI>::Ptr getFinalCloud() const { return LidarCloudGlobal; }
    std::vector<pcl::PointIndices> getObjects() const { return LidarCloudOutput; }

    static void enablePipelineCapture(const std::string& output_dir);

private:
    ROI_box roiParms;
    std::vector<LidarProcess> sensors;
    pcl::PointCloud<pcl::PointXYZI>::Ptr LidarCloudGlobal;
    std::vector<pcl::PointIndices> LidarCloudOutput;

    void mergeClouds(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& processed_clouds);
    void downSampleGlobalCloud();
    void applyCropBox();
    void removeOutliers();
    void ClusteringCloudToObjects();

    static void saveStepPCD(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                            const std::string& filename);
    static void appendParams(const std::string& line);

    static std::atomic<bool> s_capture_requested;
    static std::atomic<bool> s_capture_done;
    static std::string       s_capture_dir;
};


