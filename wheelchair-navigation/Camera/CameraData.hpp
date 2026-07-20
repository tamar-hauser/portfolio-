#pragma once
#include "CameraObject.hpp"
#include "SensorObject.hpp"
#include "SensorData.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class CameraData : public SensorData<CameraObject> {
public:
    CameraData() = default;    
    void process(CameraObject& CO) override;


private:
   cv::Mat currentFrame;
   void buildZ(CameraObject& CO) override; 
   void buildR(CameraObject& CO) override;
   void buildH(CameraObject& CO) override;
};