// EncoderList.hpp
#pragma once
#include <vector>
#include <memory>
#include "EncoderObject.hpp"
#include "SensorList.hpp"

struct EncoderList : public SensorList<EncoderObject> {
};