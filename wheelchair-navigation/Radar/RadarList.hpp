#pragma once
#include <vector>
#include <string>
#include "SensorList.hpp"
#include "RadarObject.hpp"

struct RadarList: public SensorList<RadarObject> {
};
