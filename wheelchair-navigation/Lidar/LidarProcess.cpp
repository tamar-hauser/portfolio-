#include "LidarProcess.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>              // בשביל pcl::PointXYZI
#include <pcl/filters/approximate_voxel_grid.h> // בשביל אלגוריתם הדילול המקורב
#include <pcl/point_cloud.h>              // בשביל מחלקת הענן עצמה
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

// דוגמה לאיך זה אמור להיראות בתוך פונקציה במחלקה
//LidarProcess::LidarProcess(const Eigen::Affine3f& calibration_matrix, float voxel_size)
//    : calibration_matrix(calibration_matrix), voxel_size(voxel_size) {} 
//
//pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcess::process(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_in) {
//    // 1. הגנה: בדיקה שהענן אינו ריק
//    if (!cloud_in || cloud_in->empty()) {
//        PCL_ERROR("Input cloud is empty or null!\n");
//        return nullptr;
//    }
//    auto filtered_cloud = applyVoxelGrid(cloud_in);
//    auto transformed_cloud = applyTransform(filtered_cloud);
//
//    return transformed_cloud;
//}

LidarProcess::LidarProcess(const Eigen::Affine3f& calibration_matrix, float voxel_size)
    : calibration_matrix(calibration_matrix), voxel_size(voxel_size) {
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcess::process(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_in) {

    if (!cloud_in || cloud_in->empty()) {
        std::cout << "[ERROR] Input cloud is empty!" << std::endl;
        return nullptr;
    }
    // Transform points from the sensor frame to the common/world frame first,
    // then apply voxel grid downsampling in the common frame. Doing voxelization
    // before transform can collapse distinct sensor returns into the same voxel
    // when sensors have different extrinsics.
    auto transformed_input = applyTransform(cloud_in);

    auto filtered_cloud = applyVoxelGrid(transformed_input);
    return filtered_cloud;
}



// שים לב ל-const בסוף חתימת הפונקציה
pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcess::applyVoxelGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const
{
    // בדיקת תקינות - חובה!
    if (!cloud) return nullptr;

    // Temporary debug: log cloud stats before voxel grid
    // print first 10 points
    const size_t printN = std::min<size_t>(10, cloud->size());
    for (size_t i = 0; i < printN; ++i) {
        const auto &p = cloud->points[i];
    }
    // compute min/max bounds
    pcl::PointXYZI min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    // simple spread check
    const float spread_x = max_pt.x - min_pt.x;
    const float spread_y = max_pt.y - min_pt.y;
    const float spread_z = max_pt.z - min_pt.z;
    if (spread_x < 1e-6f && spread_y < 1e-6f && spread_z < 1e-6f) {
        std::cerr << "[VoxelGrid][WARN] input cloud has near-zero spatial spread; possible duplicate coordinates or invalid conversion." << std::endl;
    }

    // שימוש ב-make_shared (יעיל ובטוח)
    auto cloud_filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    pcl::ApproximateVoxelGrid <pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud_filtered);
    // Defensive: if voxel grid collapsed the cloud to almost nothing,
    // likely due to bad input ranges or an excessively large leaf size.
    if (cloud_filtered->size() < 2) {
        std::cerr << "[WARN] VoxelGrid collapsed cloud: " << cloud->size()
                  << " -> " << cloud_filtered->size() << std::endl;

        // If the input cloud has near-zero spatial spread and appears to
        // consist of repeated (0,0,0) hits (common when lidar returns are
        // invalid), do NOT return the original copy — return an empty cloud
        // so downstream consumers skip processing.
        pcl::PointXYZI in_min, in_max;
        pcl::getMinMax3D(*cloud, in_min, in_max);
        const float spread_x_in = in_max.x - in_min.x;
        const float spread_y_in = in_max.y - in_min.y;
        const float spread_z_in = in_max.z - in_min.z;
        const float EPS_SPREAD = 1e-4f;

        if (std::abs(in_min.x) <= EPS_SPREAD && std::abs(in_min.y) <= EPS_SPREAD && std::abs(in_min.z) <= EPS_SPREAD
            && spread_x_in < EPS_SPREAD && spread_y_in < EPS_SPREAD && spread_z_in < EPS_SPREAD) {
            std::cerr << "[VoxelGrid][WARN] input appears to be all-zero/invalid; dropping cloud." << std::endl;
            return pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        }

        // Otherwise return original cloud copy as a last-resort fallback.
        auto copy_back = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*cloud);
        return copy_back;
    }

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcess::applyTransform(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const
{
    if (!cloud) return nullptr;

    auto cloud_transformed = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    //make_shared מונע הקצאת זיכרון פעמיים
    // ביצוע ההתמרה
    pcl::transformPointCloud(*cloud, *cloud_transformed, calibration_matrix);

    return cloud_transformed;
}

