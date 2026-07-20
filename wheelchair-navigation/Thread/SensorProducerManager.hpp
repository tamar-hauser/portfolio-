#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp> 
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <mutex>
#include <chrono>
#include "SensorRing.hpp"
#include "SensorMeasurement.hpp" 
#include "SensorFrame.hpp"

struct EncoderMeasurement { int left_ticks; int right_ticks; };
struct ImuMeasurement {
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
    int16_t raw_temperature; 

    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
    
    int16_t raw_mag_x;
    int16_t raw_mag_y;
    int16_t raw_mag_z;
};
struct RadarMeasurement {int id;float pos_x;float pos_y;float pos_z;float vel_x;float vel_y;float vel_z;}; // תוקן שגיאת כתיב

class SensorProducerManager {
private:
    std::chrono::time_point<std::chrono::steady_clock> startTime_; 

    SensorProducerManager() {
        startTime_ = std::chrono::steady_clock::now(); 
    }
    
public:
    double getTimeSinceStart() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - startTime_).count();
    }
    
    SensorProducerManager(const SensorProducerManager&) = delete;
    SensorProducerManager& operator=(const SensorProducerManager&) = delete;
    
    static SensorProducerManager& getInstance() {
        static SensorProducerManager instance;  
        return instance;
    }
    
    SensorRing<std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>, 30> lidarRing;
    SensorRing<cv::Mat, 10>                                          cameraRing;
    SensorRing<std::vector<RadarMeasurement>, 15>                    radarRing;
    SensorRing<std::string, 15>                                      gpsRing;
    SensorRing<EncoderMeasurement, 100>                              encoderRing;
    SensorRing<ImuMeasurement, 100>                                  imuRing;

    void addLidarUpdate(double ts, std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud) {
        lidarRing.addData({ts, cloud});
    }

    void addRadarUpdate(double ts, const std::vector<RadarMeasurement>& data) {
        radarRing.addData({ts, data});
    }

    void addCameraUpdate(double ts, const cv::Mat& frame) {
        cameraRing.addData({ts, frame.clone()}); 
    }

    void addGpsUpdate(double ts, const std::string& sentence) {
        gpsRing.addData({ts, sentence});
    }

    void addEncoderUpdate(double ts, const EncoderMeasurement& data) {
        encoderRing.addData({ts, data});
    }

    void addImuUpdate(double ts, const ImuMeasurement& data) {
        imuRing.addData({ts, data});
    }

};