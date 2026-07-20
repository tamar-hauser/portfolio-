#pragma once
#include <vector>
#include <string>
#include "SensorFusion/SensorFusionObject.hpp"
#include "Sensor/SensorList.hpp"

struct SensorFusionList: public SensorList<SensorFusionObject> {
};
