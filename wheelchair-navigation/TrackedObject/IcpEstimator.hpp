#pragma once
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <memory>
#include "TrackedObject.hpp"
#include "SensorFrame.hpp"


class IcpEstimator {
public:

    struct IcpResult {
        Eigen::Matrix4f transformation;
        double fitness_score;
        bool is_valid;
    };

    IcpEstimator() = default;   
    IcpResult process(std::shared_ptr<FramePointers> fram);

    private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr createNewCloud(const std::vector<std::shared_ptr<TrackedObject>>& best_objs);    
    IcpResult applyICP(pcl::PointCloud<pcl::PointXYZI>::Ptr source, 
                             pcl::PointCloud<pcl::PointXYZI>::Ptr target);
};