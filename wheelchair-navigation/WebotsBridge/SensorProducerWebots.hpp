#pragma once

#include <webots/types.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

class SensorProducerWebots {
public:
    struct Devices {
        WbDeviceTag gps = 0;
        WbDeviceTag imu = 0;
        WbDeviceTag accelerometer = 0;
        WbDeviceTag gyro = 0;

        WbDeviceTag frontLidar = 0;
        WbDeviceTag leftLidar = 0;
        WbDeviceTag rightLidar = 0;

        WbDeviceTag camera = 0;

        WbDeviceTag radar = 0;

        WbDeviceTag leftEncoder = 0;
        WbDeviceTag rightEncoder = 0;
    };

    SensorProducerWebots(Devices devices, int basicTimeStepMs);

    void enableDevices();
    void update();

private:
    double nowSec() const;

    void pushFastSensors(double t);
    void pushSlowSensors(double t);

    void pushImu(double t);
    void pushEncoder(double t);
    void pushGps(double t);
    void pushLidar(double t);
    void pushCamera(double t);
    void pushRadar(double t);

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>
    lidarToCloud(WbDeviceTag lidar);

    cv::Mat cameraToMat(WbDeviceTag camera);

    std::string makeNmea(double t) const;

    static int16_t toRaw(double value, double scale);
    static std::string decimalToNmea(double value, bool isLat, char& hemi);
    static std::string nmeaChecksumLine(const std::string& body);

private:
    Devices d_{};
    int stepMs_ = 10;

    double lastGpsPush_ = -1.0;
    double lastSlowPush_ = -1.0;
    double lastCameraProcessed_ = -1.0;
    bool savedDebugCameraFrame_ = false;

    double virtualLeftAngle_ = 0.0;
    double virtualRightAngle_ = 0.0;

    int lastLeftTicks_ = 0;
    int lastRightTicks_ = 0;


    static constexpr double BASE_LAT = 32.75530185851649;  
    static constexpr double BASE_LON = 35.1039595103117;
    static constexpr double METERS_PER_DEG_LAT = 111320.0;
};