#include "SensorConfig.hpp"
#include <cmath>

// אתחול משתנים סטטיים
std::map<std::string, SensorLidarConfig>   ConfigManager::lidar_map;
std::map<std::string, SensorCameraConfig>  ConfigManager::camera_map;
std::map<std::string, SensorRadarConfig>   ConfigManager::radar_map;
std::map<std::string, SensorImuConfig>     ConfigManager::imu_map;
std::map<std::string, SensorGpsConfig>     ConfigManager::gps_map;
std::map<std::string, SensorEncoderConfig> ConfigManager::encoder_map;
ROI_box ConfigManager::global_roi;

ROI_box::ROI_box() {
    minPt << -50.0f, -20.0f, 0.15f, 1.0f;  // z=0.15: ground removal
    maxPt << 50.0f, 20.0f, 2.2f, 1.0f;
    voxelSize = 0.1f;
    minNeighbors = 5;
    searchRadius = 0.2f;
    minIntensity = 10.0f;
    maxDistanceBetweenPoints = 0.5f;  // was 0.3: keeps pedestrian as one cluster
    minPointsInCloud = 30;            // was 10: filters noise/small structures
    maxPointsInCloud = 5000;
}

template <typename T>
void ConfigManager::fillBaseConfig(const YAML::Node& node, T& config) {
    // translation: {x: ..., y: ..., z: ...}
    if (node["translation"]) {
        if (node["translation"].IsMap()) {
            config.translation.x() = node["translation"]["x"].as<float>(0.0f);
            config.translation.y() = node["translation"]["y"].as<float>(0.0f);
            config.translation.z() = node["translation"]["z"].as<float>(0.0f);
        } else if (node["translation"].IsSequence()) {
            config.translation << node["translation"][0].as<float>(),
                                  node["translation"][1].as<float>(),
                                  node["translation"][2].as<float>();
        }
    }
    // rotation_rpy: {roll: ..., pitch: ..., yaw: ...}  →  מטריצת סיבוב ZYX
    if (node["rotation_rpy"] && node["rotation_rpy"].IsMap()) {
        float roll  = node["rotation_rpy"]["roll"].as<float>(0.0f);
        float pitch = node["rotation_rpy"]["pitch"].as<float>(0.0f);
        float yaw   = node["rotation_rpy"]["yaw"].as<float>(0.0f);
        Eigen::AngleAxisf rollAA (roll,  Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAA(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAA  (yaw,   Eigen::Vector3f::UnitZ());
        config.rotation = (yawAA * pitchAA * rollAA).toRotationMatrix();
    } else if (node["rotation"] && node["rotation"].IsSequence()) {
        // פורמט ישן: מטריצה 3x3 כ-sequence
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                config.rotation(i, j) = node["rotation"][i][j].as<float>();
    }
}

ROI_box ConfigManager::parseROI(const YAML::Node& node) {
    ROI_box r;
    if (node["min_pt"]) {
        auto p = node["min_pt"].as<std::vector<float>>();
        r.minPt << p[0], p[1], p[2], 1.0f;
    }
    if (node["max_pt"]) {
        auto p = node["max_pt"].as<std::vector<float>>();
        r.maxPt << p[0], p[1], p[2], 1.0f;
    }
    r.voxelSize = node["voxel_size"].as<float>(0.1f);
    r.minNeighbors = node["min_neighbors"].as<int>(5);
    r.searchRadius = node["search_radius"].as<float>(0.2f);
    r.minIntensity = node["min_intensity"].as<float>(10.0f);
    r.maxDistanceBetweenPoints = node["max_dist_pts"].as<float>(0.3f);
    r.minPointsInCloud = node["min_points"].as<int>(10);
    r.maxPointsInCloud = node["max_points"].as<int>(5000);
    return r;
}

bool ConfigManager::load(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);

        if (config["default_lidar_config"] && config["default_lidar_config"]["roi"]) {
            global_roi = parseROI(config["default_lidar_config"]["roi"]);
        }

        if (config["sensors"]) {
            for (auto it = config["sensors"].begin(); it != config["sensors"].end(); ++it) {
                std::string name = it->first.as<std::string>();
                YAML::Node node = it->second;

                // מיון לפי סוג חיישן על בסיס השם ב-YAML
                if (name.find("lidar") != std::string::npos) {
                    fillBaseConfig(node, lidar_map[name]);
                } 
                else if (name.find("camera") != std::string::npos) {
                    fillBaseConfig(node, camera_map[name]);
                } 
                else if (name.find("radar") != std::string::npos) {
                    fillBaseConfig(node, radar_map[name]);
                }
                else if (name.find("imu") != std::string::npos) {
                    fillBaseConfig(node, imu_map[name]);
                }
                else if (name.find("gps") != std::string::npos) {
                    fillBaseConfig(node, gps_map[name]);
                }
                else if (name.find("encoder") != std::string::npos) {
                    fillBaseConfig(node, encoder_map[name]);
                }
            }
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "ConfigManager Error: " << e.what() << std::endl;
        return false;
    }
}
