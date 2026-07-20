
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "LidarCloud.hpp"

// ─── statics ────────────────────────────────────────────────────────────────
std::atomic<bool> LidarCloud::s_capture_requested{false};
std::atomic<bool> LidarCloud::s_capture_done{false};
std::string       LidarCloud::s_capture_dir;

void LidarCloud::enablePipelineCapture(const std::string& output_dir) {
    s_capture_dir      = output_dir;
    s_capture_done     = false;
    s_capture_requested= true;
    std::filesystem::create_directories(output_dir);
    std::ofstream f(output_dir + "/pipeline_params.txt", std::ios::trunc);
    f.close();
}

void LidarCloud::saveStepPCD(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                              const std::string& filename) {
    if (!cloud || cloud->empty()) return;
    std::ofstream f(filename);
    if (!f.is_open()) return;
    const size_t n = cloud->size();
    f << "# .PCD v0.7 - Point Cloud Data file format\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z intensity\n"
      << "SIZE 4 4 4 4\n"
      << "TYPE F F F F\n"
      << "COUNT 1 1 1 1\n"
      << "WIDTH "  << n << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << n << "\n"
      << "DATA ascii\n";
    f << std::fixed;
    for (const auto& p : cloud->points)
        f << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << '\n';
    std::cout << "[CAPTURE] saved " << n << " pts -> " << filename << "\n";
}

void LidarCloud::appendParams(const std::string& line) {
    std::ofstream f(s_capture_dir + "/pipeline_params.txt", std::ios::app);
    f << line << "\n";
}


LidarCloud::LidarCloud(const ROI_box& params) : roiParms(params) {
    LidarCloudGlobal = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
}
void LidarCloud::processFrame(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& raw_clouds) {

    if (raw_clouds.empty()) return;
    const bool capture = s_capture_requested && !s_capture_done;

    std::cout << "\n--- Starting Global Update Pipeline ---" << std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> processed_clouds;
    for (size_t i = 0; i < raw_clouds.size(); ++i) {
        std::cout << "Processing Sensor [" << i << "]..." << std::endl;
        processed_clouds.push_back(raw_clouds[i]);
    }

    // STEP 1: MERGE
    mergeClouds(processed_clouds);
    size_t pts_after_merge = LidarCloudGlobal->size();
    std::cout << "[Merge] Total merged points: " << pts_after_merge << std::endl;
    if (capture) {
        saveStepPCD(LidarCloudGlobal, s_capture_dir + "/step1_merged.pcd");
        appendParams("step1_merged_pts=" + std::to_string(pts_after_merge));
        for (size_t i = 0; i < processed_clouds.size(); ++i)
            appendParams("step1_sensor" + std::to_string(i) + "_pts=" +
                         std::to_string(processed_clouds[i] ? processed_clouds[i]->size() : 0));
    }

    // STEP 2: CROP BOX
    size_t before = LidarCloudGlobal->size();
    applyCropBox();
    size_t pts_after_crop = LidarCloudGlobal->size();
    std::cout << "[CropBox] Points: " << before << " -> " << pts_after_crop << std::endl;
    if (capture) {
        saveStepPCD(LidarCloudGlobal, s_capture_dir + "/step2_cropbox.pcd");
        appendParams("step2_cropbox_pts=" + std::to_string(pts_after_crop));
        appendParams("step2_roi_min=" + std::to_string(roiParms.minPt.x()) + "," +
                     std::to_string(roiParms.minPt.y()) + "," + std::to_string(roiParms.minPt.z()));
        appendParams("step2_roi_max=" + std::to_string(roiParms.maxPt.x()) + "," +
                     std::to_string(roiParms.maxPt.y()) + "," + std::to_string(roiParms.maxPt.z()));
        appendParams("step2_min_intensity=" + std::to_string(roiParms.minIntensity));
    }

    // STEP 3: VOXEL GRID DOWNSAMPLE
    before = LidarCloudGlobal->size();
    downSampleGlobalCloud();
    size_t pts_after_voxel = LidarCloudGlobal->size();
    std::cout << "[Global Downsample] Points: " << before << " -> " << pts_after_voxel << std::endl;
    if (capture) {
        saveStepPCD(LidarCloudGlobal, s_capture_dir + "/step3_voxel.pcd");
        appendParams("step3_voxel_pts=" + std::to_string(pts_after_voxel));
        appendParams("step3_voxel_size=" + std::to_string(roiParms.voxelSize));
    }

    // STEP 4: OUTLIER REMOVAL
    before = LidarCloudGlobal->size();
    removeOutliers();
    size_t pts_after_outlier = LidarCloudGlobal->size();
    std::cout << "[Outlier Removal] Points: " << before << " -> " << pts_after_outlier << std::endl;
    if (capture) {
        saveStepPCD(LidarCloudGlobal, s_capture_dir + "/step4_outliers.pcd");
        appendParams("step4_outlier_pts=" + std::to_string(pts_after_outlier));
        appendParams("step4_min_neighbors=" + std::to_string(roiParms.minNeighbors));
        appendParams("step4_std_thresh=" + std::to_string(roiParms.searchRadius));
    }

    // STEP 5: EUCLIDEAN CLUSTERING
    ClusteringCloudToObjects();
    std::cout << "--- Pipeline Finished ---\n" << std::endl;
    if (capture) {
        appendParams("step5_num_clusters=" + std::to_string(LidarCloudOutput.size()));
        appendParams("step5_cluster_tolerance=" + std::to_string(roiParms.maxDistanceBetweenPoints));
        appendParams("step5_min_cluster=" + std::to_string(roiParms.minPointsInCloud));
        appendParams("step5_max_cluster=" + std::to_string(roiParms.maxPointsInCloud));

        for (size_t c = 0; c < LidarCloudOutput.size(); ++c) {
            auto cluster_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            for (int idx : LidarCloudOutput[c].indices)
                cluster_cloud->push_back(LidarCloudGlobal->points[idx]);
            saveStepPCD(cluster_cloud,
                        s_capture_dir + "/step5_cluster_" + std::to_string(c) + ".pcd");
            appendParams("step5_cluster_" + std::to_string(c) + "_pts=" +
                         std::to_string(cluster_cloud->size()));
        }
        s_capture_done = true;
        std::cout << "[CAPTURE] Pipeline capture complete -> " << s_capture_dir << "\n";
    }
}

void LidarCloud::addLidar(const LidarProcess& lidarprocess) {
    sensors.push_back(lidarprocess);
}

void LidarCloud::mergeClouds(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& processed_clouds) {
    LidarCloudGlobal->clear();
    for (const auto& cloud : processed_clouds) {
        if (cloud && !cloud->empty()) {
            *LidarCloudGlobal += *cloud;
        }
    }
    std::cout << "[Merge] Total merged points: " << LidarCloudGlobal->size() << std::endl;
}

void LidarCloud::downSampleGlobalCloud() {
    if (LidarCloudGlobal->empty()) return;

    pcl::ApproximateVoxelGrid <pcl::PointXYZI> sor;
    sor.setInputCloud(LidarCloudGlobal);
    sor.setLeafSize(roiParms.voxelSize, roiParms.voxelSize, roiParms.voxelSize);
    sor.filter(*LidarCloudGlobal);
}

void LidarCloud::applyCropBox() {
    if (!LidarCloudGlobal || LidarCloudGlobal->empty()) return;
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(LidarCloudGlobal);
    crop.setMin(roiParms.minPt);
    crop.setMax(roiParms.maxPt);
    crop.filter(*LidarCloudGlobal);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(LidarCloudGlobal);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(roiParms.minIntensity, 1000.0);
    pass.filter(*LidarCloudGlobal);
}
void LidarCloud::removeOutliers() {
    if (LidarCloudGlobal->empty()) return;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(LidarCloudGlobal);
    sor.setMeanK(roiParms.minNeighbors);
    sor.setStddevMulThresh(roiParms.searchRadius);
    sor.filter(*LidarCloudGlobal);
}
void LidarCloud::ClusteringCloudToObjects()
{
    if (LidarCloudGlobal->empty())
        return;
    LidarCloudOutput.clear();
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(LidarCloudGlobal);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(LidarCloudGlobal);
    ec.setClusterTolerance(roiParms.maxDistanceBetweenPoints);
    ec.setMinClusterSize(roiParms.minPointsInCloud);
    ec.setMaxClusterSize(roiParms.maxPointsInCloud);
    ec.setSearchMethod(tree);
    ec.extract(LidarCloudOutput);
    std::cout << "[Clustering] Found " << LidarCloudOutput.size() << " objects." << std::endl;
}