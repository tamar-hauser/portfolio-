#pragma once
#include <map>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <iostream>

struct ROI_box {
    Eigen::Vector4f minPt;
    Eigen::Vector4f maxPt;
    float voxelSize;
    int minNeighbors;
    float searchRadius;
    float minIntensity;
    float maxDistanceBetweenPoints;
    int minPointsInCloud;
    int maxPointsInCloud;

    ROI_box();
};

struct BaseSensorConfig {
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    Eigen::Vector3f translation = Eigen::Vector3f::Zero();

    Eigen::Vector3f transformPoint(const Eigen::Vector3f& p) const {
        return rotation * p + translation;
    }
    
    Eigen::Vector3f transformVector(const Eigen::Vector3f& v) const {
        return rotation * v;
    }
};

// יצירת שמות ברורים לסוגי הקונפיגורציות
using SensorLidarConfig   = BaseSensorConfig;
using SensorCameraConfig  = BaseSensorConfig;
using SensorRadarConfig   = BaseSensorConfig;
using SensorImuConfig     = BaseSensorConfig;
using SensorGpsConfig     = BaseSensorConfig;
using SensorEncoderConfig = BaseSensorConfig;

class ConfigManager {
public:
    static bool load(const std::string& filename);

    // Getters - שליפה לפי השם המדויק מה-YAML (למשל "lidar_front")
    static SensorLidarConfig   getLidar(const std::string& name)   { return lidar_map[name]; }
    static SensorCameraConfig  getCamera(const std::string& name)  { return camera_map[name]; }
    static SensorRadarConfig   getRadar(const std::string& name)   { return radar_map[name]; }
    static SensorImuConfig     getImu(const std::string& name)     { return imu_map[name]; }
    static SensorGpsConfig     getGps(const std::string& name)     { return gps_map[name]; }
    static SensorEncoderConfig getEncoder(const std::string& name) { return encoder_map[name]; }
    static ROI_box             getROI()                            { return global_roi; }

private:
    static std::map<std::string, SensorLidarConfig>   lidar_map;
    static std::map<std::string, SensorCameraConfig>  camera_map;
    static std::map<std::string, SensorRadarConfig>   radar_map;
    static std::map<std::string, SensorImuConfig>     imu_map;
    static std::map<std::string, SensorGpsConfig>     gps_map;
    static std::map<std::string, SensorEncoderConfig> encoder_map;
    static ROI_box global_roi;

    template <typename T>
    static void fillBaseConfig(const YAML::Node& node, T& config);
    static ROI_box parseROI(const YAML::Node& node);
};