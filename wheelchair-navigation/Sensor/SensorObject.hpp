#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <string>
enum class SensorType {
    Lidar,
    Camera,
    Radar,
    Gps,
    Imu,
    Encoder,
    SensorObject,
    TrackedObject
};

struct SensorObject {
    int id;
    SensorType type; 
    double timestamp;
    bool isValid;           
    double confidence;      
    Eigen::VectorXd Z;
    Eigen::MatrixXd R;
    Eigen::MatrixXd H;
    bool has_lidar;
    bool has_camera;
    bool has_radar;
    virtual Eigen::Vector3f velocity() const { return Eigen::Vector3f::Zero(); }
        // Note: constructor arguments are (stateSize, measurementSize)
        SensorObject(int stateSize, int measurementSize, SensorType type, bool has_lidar, bool has_cameera, bool has_radar)
                : id(-1), type(type), timestamp(0.0), isValid(false), confidence(0.0f),
                    Z(measurementSize), R(measurementSize, measurementSize), H(measurementSize, stateSize), has_lidar(has_lidar),
                    has_camera(has_cameera), has_radar(has_radar)
    {
        Z.setZero();
        R.setIdentity(); 
        H.setZero();
    }
};