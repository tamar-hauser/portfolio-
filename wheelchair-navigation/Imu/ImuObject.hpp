#pragma once
#include "SensorObject.hpp"
#include "Constants.hpp"
#include <Eigen/Dense>

struct ImuObject : public SensorObject {
    double Pitch;
    double Yaw;
    double Vyaw;    // מהירות זוויתית (סביב ציר Z)
    double Ax;      // תאוצה קווית בציר X
    ImuObject() :SensorObject(Config::DimWheelchairStateVector,Config::MeasurementSizeImu,SensorType::Imu,false,false,false),
                 Pitch(0.0),
                 Yaw(0.0),
                 Vyaw(0.0),
                 Ax(0.0)
{}
};