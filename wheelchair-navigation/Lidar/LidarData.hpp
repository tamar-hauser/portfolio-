#pragma once
#include "SensorData.hpp"
#include "LidarObject.hpp"
#include <vector>
#include <memory> 
#include <Eigen/Dense>

class LidarData: public SensorData<LidarObject>  {
public:
    LidarData() = default;
    void process(LidarObject& LO) override;
private:
   void buildZ(LidarObject& myObject)override; 
   void buildR(LidarObject& myObject)override;
   void buildH(LidarObject& myObject)override;
//    void fillParamForCompute(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,std::LidarObject myObject) override;
};