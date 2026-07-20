#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "TrackedObject.hpp"
#include "SensorFusion/SensorFusionObject.hpp"

struct TrackeList: public SensorList<TrackedObject> {

};
