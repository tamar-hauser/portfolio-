#include <iostream>
#include <cmath>
#include "LidarObject.hpp"
#include "LidarData.hpp"
#include "Constants.hpp"
#include <Eigen/Dense>

void LidarData::process(LidarObject& LO) {
  
    buildZ(LO);
    buildH(LO);
    buildR(LO);
   
}

void LidarData::buildZ(LidarObject& LO) {
    LO.Z(0) = LO.position.x();
    LO.Z(1) = LO.position.y();
    LO.Z(2) = LO.position.z();
    LO.Z(3) = LO.yaw; 

}

void LidarData::buildH(LidarObject& LO) {
    LO.H.setZero();
    float vx = 1.0f; 
    float vy =1.0f;
    float v2 = vx * vx + vy * vy;

    LO.H(0, 0) = 1.0f; 
    LO.H(1, 1) = 1.0f; 
    LO.H(2, 2) = 1.0f; 
    LO.H(3, 3) = 1.0f; 
    
}

void LidarData::buildR(LidarObject& LO) {  
    float confidence_factor = 1.0f / std::max<float>(LO.confidence, 0.1f);
    LO.R(0, 0) = (LO.covariance_matrix(0, 0) + 0.01f) * confidence_factor; // X
    LO.R(1, 1) = (LO.covariance_matrix(1, 1) + 0.01f) * confidence_factor; // Y
    LO.R(2, 2) = (LO.covariance_matrix(2, 2) + 0.02f) * confidence_factor; // Z
    float curvature_impact = 1.0f + LO.curvature; 
    float base_yaw_noise = 0.02f;
    LO.R(3, 3) = base_yaw_noise * curvature_impact * confidence_factor;

}