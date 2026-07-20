#pragma once
#include <vector>
#include <memory>
#include "SensorObject.hpp"
#include "SensorFusionM.hpp"
#include "Camera/CameraObject.hpp"
#include "SensorFusionObject.hpp"
#include "HungarianAlgorithm.hpp"
#include "KalmanFilter/ekf.hpp"

class SensorFusionCamera : public SensorFusionM<CameraObject, SensorFusionObject>
{
public:
    virtual ~SensorFusionCamera() = default;

protected:
     HungarianAlgorithm HA;
     SensorFusionObject createNewObject(CameraObject& CO) override;
     void updateObject(CameraObject& co, SensorFusionObject& sensor_object)override;
     Eigen::Matrix3f calculateDynamicCameraSigma(const CameraObject& obj);

};
