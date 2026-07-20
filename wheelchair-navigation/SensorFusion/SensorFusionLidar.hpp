#include <iostream>
#include <vector>
#include "SensorFrame.hpp" // בהנחה שזה הקובץ המגדיר את הטיפוס
#include <Eigen/Dense>
#include "SensorObject.hpp"
#include "SensorFusionM.hpp"
#include "LidarObject.hpp"
#include "SensorFusionObject.hpp"
#include "HungarianAlgorithm.hpp"
#include "KalmanFilter/ekf.hpp"
#include "Constants.hpp"


class SensorFusionLidar : public SensorFusionM<LidarObject, SensorFusionObject>
{
public:
     virtual ~SensorFusionLidar() = default;

protected:
     void updateObject(LidarObject& ro, SensorFusionObject& sensor_object)override;
     SensorFusionObject createNewObject(LidarObject& LO) override;
     // float calculate_cost(LidarObject& ro, SensorFusionObject& sensor_object) override;
};