#pragma once
#include <iostream>
#include <string>
#include "ImuObject.hpp"
#include "SensorList.hpp"

struct ImuList: public SensorList<ImuObject> {

};
