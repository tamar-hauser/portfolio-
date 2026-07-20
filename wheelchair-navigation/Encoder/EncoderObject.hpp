#pragma once
#include "SensorObject.hpp"
#include "Constants.hpp"
#include <Eigen/Dense>

struct EncoderObject : public SensorObject {
    double left_velocity;
    double right_velocity;
    double v_linear;
    double v_angular;
    EncoderObject() : SensorObject(Config::DimWheelchairStateVector, Config::MeasurementSizeEncoder, SensorType::Encoder ,false,false,false),
                      left_velocity(0.0), 
                      right_velocity(0.0), 
                      v_linear(0.0), 
                      v_angular(0.0)
                  {}
};