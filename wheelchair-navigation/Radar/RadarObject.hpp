#pragma once
#include "SensorObject.hpp"
#include <Eigen/Dense>
#include <memory>
#include "Constants.hpp"   

struct RadarObject : public SensorObject {
    double range;                // מרחק במטרים
    Eigen::Vector3d position;    // מיקום X, Y, Z
    Eigen::Vector3d velocity;    // מהירות Vx, Vy, Vz
    double rcs;                  // עוצמת החזר     

    RadarObject() : SensorObject(Config::StateSizeObject, Config::MeasurementSizeRadar, SensorType::Radar ,false,false,true),
                    range(0.0), 
                    rcs(0.0) 
    {
        position.setZero();
        velocity.setZero();
    }
};