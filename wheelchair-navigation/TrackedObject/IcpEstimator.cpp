#include "IcpEstimator.hpp"
#include "SensorFrame.hpp"


IcpEstimator::IcpResult IcpEstimator::process(std::shared_ptr<FramePointers> fram)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudnew = createNewCloud(fram->tracked);
    
    auto prev_frame = fram->previousFrame.lock();
    fram->lidar->Icp_Cloud=cloudnew;
    if (prev_frame && prev_frame->lidar) {
        auto& prev_cloud = prev_frame->lidar->Icp_Cloud;
        if (cloudnew && !cloudnew->empty() && prev_cloud && !prev_cloud->empty()) {
            return applyICP(cloudnew, prev_cloud);
        }
    }    
    IcpResult invalid_result;
    invalid_result.is_valid = false;
    invalid_result.fitness_score = 0.0f;
    invalid_result.transformation = Eigen::Matrix4f::Identity();
    return invalid_result;
}



pcl::PointCloud<pcl::PointXYZI>::Ptr IcpEstimator::createNewCloud(const std::vector<std::shared_ptr<TrackedObject>>& best_objs) {    pcl::PointCloud<pcl::PointXYZI>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (const auto& obj : best_objs) {
        if (obj && obj->cloud && !obj->cloud->empty()) {
            *combined_cloud += *(obj->cloud);
        }
    }

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(combined_cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    sor.filter(*filtered_cloud);

    return filtered_cloud;
}

IcpEstimator::IcpResult IcpEstimator::applyICP(pcl::PointCloud<pcl::PointXYZI>::Ptr source, 
                                               pcl::PointCloud<pcl::PointXYZI>::Ptr target) {
    
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    
    icp.setMaxCorrespondenceDistance(1.0); 
    icp.setMaximumIterations(50);          
    icp.setTransformationEpsilon(1e-8);    
    icp.setEuclideanFitnessEpsilon(0.01);  

    pcl::PointCloud<pcl::PointXYZI> final_cloud;
    icp.align(final_cloud);

    IcpResult result;
    result.is_valid = icp.hasConverged();
    
    if (result.is_valid) {
        result.transformation = icp.getFinalTransformation();
        result.fitness_score = icp.getFitnessScore();
    } else {
        result.transformation = Eigen::Matrix4f::Identity();
        result.fitness_score = 1000.0;
        std::cerr << "ICP did not converge!" << std::endl;
    }
    
    return result;
}

