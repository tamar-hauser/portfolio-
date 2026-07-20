#include "SensorProducerWebots.hpp"
#include "FrontLidarState.hpp"
#include "Thread/SensorProducerManager.hpp"

#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/lidar.h>
#include <webots/camera.h>
#include <webots/position_sensor.h>
#include <webots/radar.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <map>
#include <string>

namespace {
constexpr double GPS_PERIOD_SEC  = 0.10;  
constexpr double SLOW_PERIOD_SEC = 0.50; 
constexpr double TICKS_PER_REV   = 2048.0;
constexpr double PI              = 3.14159265358979323846;

bool finite3(const double* v)
{
    return v &&
           std::isfinite(v[0]) &&
           std::isfinite(v[1]) &&
           std::isfinite(v[2]);
}

bool printEvery(const std::string& name, int every)
{
    static std::map<std::string, int> counters;

    int& c = counters[name];
    ++c;

    return c % every == 0;
}
}

SensorProducerWebots::SensorProducerWebots(Devices devices, int basicTimeStepMs)
    : d_(devices),
      stepMs_(basicTimeStepMs)
{
}

void SensorProducerWebots::enableDevices()
{
    std::cout << "[WebotsProducer] enableDevices started" << std::endl;

    if (d_.gps) {
        wb_gps_enable(d_.gps, stepMs_);
        std::cout << "[WebotsProducer] GPS enabled at " << stepMs_ << " ms" << std::endl;
    } else {
        std::cout << "[WebotsProducer][WARN] GPS device missing" << std::endl;
    }

    if (d_.imu) {
        wb_inertial_unit_enable(d_.imu, stepMs_);
        std::cout << "[WebotsProducer] IMU/InertialUnit enabled at " << stepMs_ << " ms" << std::endl;
    } else {
        std::cout << "[WebotsProducer][WARN] IMU/InertialUnit device missing" << std::endl;
    }

    if (d_.accelerometer) {
        wb_accelerometer_enable(d_.accelerometer, stepMs_);
        std::cout << "[WebotsProducer] Accelerometer enabled at " << stepMs_ << " ms" << std::endl;
    } else {
        std::cout << "[WebotsProducer][WARN] Accelerometer device missing" << std::endl;
    }

    if (d_.gyro) {
        wb_gyro_enable(d_.gyro, stepMs_);
        std::cout << "[WebotsProducer] Gyro enabled at " << stepMs_ << " ms" << std::endl;
    } else {
        std::cout << "[WebotsProducer][WARN] Gyro device missing" << std::endl;
    }

    if (d_.leftEncoder) {
        wb_position_sensor_enable(d_.leftEncoder, stepMs_);
        std::cout << "[WebotsProducer] Left encoder enabled at " << stepMs_ << " ms" << std::endl;
    } else {
        std::cout << "[WebotsProducer][WARN] Left encoder missing" << std::endl;
    }

    if (d_.rightEncoder) {
        wb_position_sensor_enable(d_.rightEncoder, stepMs_);
        std::cout << "[WebotsProducer] Right encoder enabled at " << stepMs_ << " ms" << std::endl;
    } else {
        std::cout << "[WebotsProducer][WARN] Right encoder missing" << std::endl;
    }

    const int slowStepMs = static_cast<int>(SLOW_PERIOD_SEC * 1000.0);

    if (d_.frontLidar) {
        wb_lidar_enable(d_.frontLidar, slowStepMs);
        wb_lidar_enable_point_cloud(d_.frontLidar);
        std::cout << "[WebotsProducer] Front LiDAR enabled at " << slowStepMs << " ms" << std::endl;
    } else {
        std::cout << "[WebotsProducer][WARN] Front LiDAR missing" << std::endl;
    }

    if (d_.leftLidar) {
        wb_lidar_enable(d_.leftLidar, slowStepMs);
        wb_lidar_enable_point_cloud(d_.leftLidar);
        std::cout << "[WebotsProducer] Left LiDAR enabled at " << slowStepMs << " ms" << std::endl;
    } else {
        std::cout << "[WebotsProducer][WARN] Left LiDAR missing" << std::endl;
    }

    if (d_.rightLidar) {
        wb_lidar_enable(d_.rightLidar, slowStepMs);
        wb_lidar_enable_point_cloud(d_.rightLidar);
        std::cout << "[WebotsProducer] Right LiDAR enabled at " << slowStepMs << " ms" << std::endl;
    } else {
        std::cout << "[WebotsProducer][WARN] Right LiDAR missing" << std::endl;
    }

    if (d_.camera) {
        wb_camera_enable(d_.camera, slowStepMs);
        std::cout << "[WebotsProducer] Camera enabled at " << slowStepMs << " ms" << std::endl;
    } else {
        std::cout << "[WebotsProducer][WARN] Camera missing" << std::endl;
    }

    if (d_.radar) {
        std::cout << "[DIAG] Before wb_radar_enable" << std::endl; std::cout.flush();
        wb_radar_enable(d_.radar, slowStepMs);
        std::cout << "[DIAG] After wb_radar_enable OK" << std::endl; std::cout.flush();
    } else {
        std::cout << "[WebotsProducer][WARN] Radar missing" << std::endl;
    }
    lastSlowPush_ = nowSec();

    std::cout << "[WebotsProducer] enableDevices finished" << std::endl;
}

double SensorProducerWebots::nowSec() const
{
    return wb_robot_get_time();
}

void SensorProducerWebots::update()
{
    const double t = nowSec();

    if (printEvery("update", 100)) {
        std::cout << "[WebotsProducer][UPDATE] t=" << t << std::endl;
    }
    pushFastSensors(t);
    if (lastGpsPush_ < 0.0 || (t - lastGpsPush_) >= GPS_PERIOD_SEC) {
        pushGps(t);
        lastGpsPush_ = t;
    }
    if (lastSlowPush_ < 0.0 || (t - lastSlowPush_) >= SLOW_PERIOD_SEC) {
        pushSlowSensors(t);
        lastSlowPush_ = t;
    }

    
}

void SensorProducerWebots::pushFastSensors(double t)
{
    pushImu(t);
    pushEncoder(t);
}

void SensorProducerWebots::pushSlowSensors(double t)
{
    pushLidar(t);
    pushCamera(t);
    pushRadar(t);
}

void SensorProducerWebots::pushGps(double t)
{
    std::string nmea = makeNmea(t);

    auto& manager = SensorProducerManager::getInstance();
    manager.addGpsUpdate(t, nmea);

    if (printEvery("gps", 10)) {
        std::cout << "\n[PUSH][GPS] t=" << t
                  << " nmea_size=" << nmea.size()
                  << "\n"
                  << nmea
                  << std::endl;
    }
}

int16_t SensorProducerWebots::toRaw(double value, double scale)
{
    if (!std::isfinite(value))
        return 0;

    const double v = std::clamp(value * scale, -32768.0, 32767.0);
    return static_cast<int16_t>(std::lround(v));
}

void SensorProducerWebots::pushImu(double t)
{
    ImuMeasurement imu{};

    const double* a =
        d_.accelerometer ? wb_accelerometer_get_values(d_.accelerometer) : nullptr;

    const double* g =
        d_.gyro ? wb_gyro_get_values(d_.gyro) : nullptr;

    const double* rpy =
        d_.imu ? wb_inertial_unit_get_roll_pitch_yaw(d_.imu) : nullptr;

    if (finite3(a)) {
        imu.raw_accel_x = toRaw(a[0], 1000.0);
        imu.raw_accel_y = toRaw(a[1], 1000.0);
        imu.raw_accel_z = toRaw(a[2], 1000.0);
    }

    if (finite3(g)) {
        imu.raw_gyro_x = toRaw(g[0], 1000.0);
        imu.raw_gyro_y = toRaw(g[1], 1000.0);
        imu.raw_gyro_z = toRaw(g[2], 1000.0);
    }

    if (finite3(rpy)) {
        imu.raw_mag_x = toRaw(rpy[0], 1000.0);
        imu.raw_mag_y = toRaw(rpy[1], 1000.0);
        imu.raw_mag_z = toRaw(rpy[2], 1000.0);
    }

    imu.raw_temperature = 0;

    SensorProducerManager::getInstance().addImuUpdate(t, imu);

    if (printEvery("imu", 100)) {
        std::cout << "[PUSH][IMU] t=" << t
                  << " acc_raw=(" << imu.raw_accel_x << ", "
                                   << imu.raw_accel_y << ", "
                                   << imu.raw_accel_z << ")"
                  << " gyro_raw=(" << imu.raw_gyro_x << ", "
                                    << imu.raw_gyro_y << ", "
                                    << imu.raw_gyro_z << ")"
                  << " rpy_raw=(" << imu.raw_mag_x << ", "
                                  << imu.raw_mag_y << ", "
                                  << imu.raw_mag_z << ")"
                  << std::endl;
    }
}

void SensorProducerWebots::pushEncoder(double t)
{
    EncoderMeasurement enc{};

    if (d_.leftEncoder && d_.rightEncoder) {
        const double leftAngle  = wb_position_sensor_get_value(d_.leftEncoder);
        const double rightAngle = wb_position_sensor_get_value(d_.rightEncoder);

        const int leftCumulative  = static_cast<int>(std::lround(leftAngle  / (2.0 * PI) * TICKS_PER_REV));
        const int rightCumulative = static_cast<int>(std::lround(rightAngle / (2.0 * PI) * TICKS_PER_REV));

        enc.left_ticks  = leftCumulative  - lastLeftTicks_;
        enc.right_ticks = rightCumulative - lastRightTicks_;

        lastLeftTicks_  = leftCumulative;
        lastRightTicks_ = rightCumulative;
    } else {
        enc.left_ticks  = 0;
        enc.right_ticks = 0;
    }

    SensorProducerManager::getInstance().addEncoderUpdate(t, enc);

    if (printEvery("encoder", 100)) {
        std::cout << "[PUSH][ENCODER] t=" << t
                  << " left_ticks=" << enc.left_ticks
                  << " right_ticks=" << enc.right_ticks
                  << std::endl;
    }
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>
SensorProducerWebots::lidarToCloud(WbDeviceTag lidar)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    if (lidar == 0)
        return cloud;

    const int n = wb_lidar_get_number_of_points(lidar);
    const WbLidarPoint* pts = wb_lidar_get_point_cloud(lidar);

    if (!pts || n <= 0)
        return cloud;

    cloud->reserve(static_cast<std::size_t>(n));

    double device_max_range = 0.0;
    try {
        device_max_range = wb_lidar_get_max_range(lidar);
    } catch (...) {
        device_max_range = 0.0;
    }

    int valid_count = 0;
    int skipped_count = 0;
    int nan_count = 0;
    int inf_count = 0;
    int zero_count = 0;
    int maxrange_count = 0;

    const double MIN_VALID_RANGE = 0.01;
    const double EPS = 1e-6;

    for (int i = 0; i < n; ++i) {
        const WbLidarPoint& wp = pts[i];

        if (!std::isfinite(wp.x) || !std::isfinite(wp.y) || !std::isfinite(wp.z)) {
            if (std::isnan(wp.x) || std::isnan(wp.y) || std::isnan(wp.z)) ++nan_count;
            if (!std::isfinite(wp.x) || !std::isfinite(wp.y) || !std::isfinite(wp.z)) ++inf_count;
            ++skipped_count;
            continue;
        }

        double rx = wp.x;
        double ry = wp.y;
        double rz = wp.z;
        double r = std::sqrt(rx*rx + ry*ry + rz*rz);

        if (r <= MIN_VALID_RANGE) {
            ++zero_count; ++skipped_count; continue;
        }

        if (device_max_range > 0.0 && r >= device_max_range - EPS) {
            ++maxrange_count; ++skipped_count; continue;
        }

        pcl::PointXYZI p;
        p.x = static_cast<float>(wp.x);
        p.y = static_cast<float>(wp.y);
        p.z = static_cast<float>(wp.z);
        p.intensity = 50.0f;

        cloud->push_back(p);
        ++valid_count;
    }

    std::cout << "[LIDAR->CLOUD][DEBUG] lidar_points_total=" << n
              << " valid=" << valid_count
              << " skipped=" << skipped_count
              << " nan=" << nan_count
              << " inf=" << inf_count
              << " zero_or_too_close=" << zero_count
              << " maxrange_misses=" << maxrange_count
              << " device_max_range=" << device_max_range
              << std::endl;

    if (valid_count == 0) {
        std::cerr << "[LIDAR][WARN] Skipping LiDAR frame because all ranges are zero/invalid" << std::endl;
        return std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }

    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}

void SensorProducerWebots::pushLidar(double t)
{
    auto front = lidarToCloud(d_.frontLidar);
    auto left  = lidarToCloud(d_.leftLidar);
    auto right = lidarToCloud(d_.rightLidar);
    {
        constexpr float TAN30 = 0.5774f;
        float min_r = std::numeric_limits<float>::infinity();
        for (const auto& p : front->points) {
            if (p.x <= 0.05f) continue;
            if (std::fabs(p.y) > p.x * TAN30) continue;
            float r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (r < min_r) min_r = r;
        }
        FrontLidar::g_min_range_m.store(min_r,  std::memory_order_relaxed);
        FrontLidar::g_min_range_ts.store(t,      std::memory_order_relaxed);
    }

    auto& manager = SensorProducerManager::getInstance();

    manager.addLidarUpdate(t, front);
    manager.addLidarUpdate(t, right);
    manager.addLidarUpdate(t, left);

    if (printEvery("lidar", 1)) {
        std::cout << "[PUSH][LIDAR] t=" << t
                  << " front_points=" << front->size()
                  << " right_points=" << right->size()
                  << " left_points=" << left->size()
                  << std::endl;

        auto minRange = [](const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud) -> double {
            double min_r = std::numeric_limits<double>::infinity();
            for (const auto& p : cloud->points) {
                double r = std::sqrt(static_cast<double>(p.x) * p.x +
                                      static_cast<double>(p.y) * p.y +
                                      static_cast<double>(p.z) * p.z);
                if (r < min_r) min_r = r;
            }
            return min_r;
        };

        std::cout << "[OBSTACLE_SENSOR_DEBUG]"
                  << " front_lidar_valid_points=" << front->size()
                  << " left_lidar_valid_points=" << left->size()
                  << " right_lidar_valid_points=" << right->size()
                  << " min_front_range=" << minRange(front)
                  << " min_left_range=" << minRange(left)
                  << " min_right_range=" << minRange(right)
                  << std::endl;
    }
}

cv::Mat SensorProducerWebots::cameraToMat(WbDeviceTag camera)
{
    if (camera == 0)
        return {};

    const int width  = wb_camera_get_width(camera);
    const int height = wb_camera_get_height(camera);

    const unsigned char* img = wb_camera_get_image(camera);

    std::cout << "[CAMERA][DEBUG] cameraToMat width=" << width << " height=" << height
              << " img_ptr=" << (img ? "non-null" : "null") << std::endl;

    if (!img || width <= 0 || height <= 0)
        return {};

    cv::Mat bgr(height, width, CV_8UC3);

    double sum_brightness = 0.0;
    double min_brightness = 1e9;
    double max_brightness = -1e9;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const unsigned char r = static_cast<unsigned char>(wb_camera_image_get_red(img, width, x, y));
            const unsigned char g = static_cast<unsigned char>(wb_camera_image_get_green(img, width, x, y));
            const unsigned char b = static_cast<unsigned char>(wb_camera_image_get_blue(img, width, x, y));

            bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);

            double lum = 0.299 * static_cast<double>(r)
                       + 0.587 * static_cast<double>(g)
                       + 0.114 * static_cast<double>(b);
            sum_brightness += lum;
            if (lum < min_brightness) min_brightness = lum;
            if (lum > max_brightness) max_brightness = lum;
        }
    }

    double mean_brightness = sum_brightness / (static_cast<double>(width) * static_cast<double>(height));
    std::cout << "[CAMERA][DEBUG] min_brightness=" << min_brightness
              << " max_brightness=" << max_brightness
              << " mean_brightness=" << mean_brightness << std::endl;

    if (mean_brightness < 5.0) {
        std::cerr << "[CAMERA][WARN] captured frame is black or nearly black (mean=" << mean_brightness << ")" << std::endl;
    }

    return bgr;
}

void SensorProducerWebots::pushCamera(double t)
{
    // Avoid processing camera frames before the device sampling period
    if (lastCameraProcessed_ >= 0.0 && (t - lastCameraProcessed_) < SLOW_PERIOD_SEC) {
        if (printEvery("camera_skip", 1)) {
            std::cout << "[PUSH][CAMERA][SKIP] t=" << t << " before sampling period" << std::endl;
        }
        return;
    }

    cv::Mat frame = cameraToMat(d_.camera);

    if (frame.empty()) {
        if (printEvery("camera_empty", 1)) {
            std::cout << "[PUSH][CAMERA] t=" << t
                      << " EMPTY frame - not pushed"
                      << std::endl;
        }
        return;
    }

    lastCameraProcessed_ = t;

    SensorProducerManager::getInstance().addCameraUpdate(t, frame);

    if (printEvery("camera", 1)) {
        std::cout << "[PUSH][CAMERA] t=" << t
                  << " frame=" << frame.cols << "x" << frame.rows
                  << " channels=" << frame.channels()
                  << std::endl;
    }

    // Save one-time debug frame only after the system has warmed up (>=1.0s)
    if (!savedDebugCameraFrame_ && t >= 1.0) {
        try {
            cv::imwrite("debug_camera_frame.png", frame);
            std::cout << "[PUSH][CAMERA][DEBUG] saved debug_camera_frame.png" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[PUSH][CAMERA][DEBUG] failed to save debug frame: " << e.what() << std::endl;
        }
        savedDebugCameraFrame_ = true;
    }
}

void SensorProducerWebots::pushRadar(double t)
{
    std::vector<RadarMeasurement> targets;

    if (d_.radar != 0) {
        std::cout << "[DIAG] Before wb_radar_get_number_of_targets" << std::endl; std::cout.flush();
        int n = wb_radar_get_number_of_targets(d_.radar);
        std::cout << "[DIAG] n=" << n << " before wb_radar_get_targets" << std::endl; std::cout.flush();
        const WbRadarTarget* wbt = wb_radar_get_targets(d_.radar);
        std::cout << "[DIAG] After wb_radar_get_targets OK" << std::endl; std::cout.flush();

        for (int i = 0; i < n; ++i) {
            const WbRadarTarget& tgt = wbt[i];

            RadarMeasurement r{};
            r.id = i + 1;

            r.pos_x = static_cast<float>(tgt.distance * std::cos(tgt.azimuth));
            r.pos_y = static_cast<float>(tgt.distance * std::sin(tgt.azimuth));
            r.pos_z = 0.0f;

            r.vel_x = static_cast<float>(tgt.speed * std::cos(tgt.azimuth));
            r.vel_y = static_cast<float>(tgt.speed * std::sin(tgt.azimuth));
            r.vel_z = 0.0f;

            targets.push_back(r);
        }
    }

    SensorProducerManager::getInstance().addRadarUpdate(t, targets);

    if (printEvery("radar", 1)) {
        std::cout << "[PUSH][RADAR] t=" << t
                  << " targets=" << targets.size();
        for (const auto& r : targets) {
            std::cout << " | id=" << r.id
                      << " pos=(" << r.pos_x << ", " << r.pos_y << ", " << r.pos_z << ")"
                      << " vel=(" << r.vel_x << ", " << r.vel_y << ", " << r.vel_z << ")";
        }
        std::cout << std::endl;
    }
}

std::string SensorProducerWebots::decimalToNmea(double value, bool isLat, char& hemi)
{
    hemi = 'N';

    if (isLat)
        hemi = value >= 0.0 ? 'N' : 'S';
    else
        hemi = value >= 0.0 ? 'E' : 'W';

    value = std::abs(value);

    int deg = static_cast<int>(value);
    double minutes = (value - deg) * 60.0;

    std::ostringstream out;

    if (isLat)
        out << std::setw(2) << std::setfill('0') << deg;
    else
        out << std::setw(3) << std::setfill('0') << deg;

    out << std::fixed
    << std::setprecision(8)
    << std::setw(11)
    << std::setfill('0')
    << minutes;

    return out.str();
}

std::string SensorProducerWebots::nmeaChecksumLine(const std::string& body)
{
    unsigned char checksum = 0;

    for (char c : body)
        checksum ^= static_cast<unsigned char>(c);

    std::ostringstream out;

    out << "$"
        << body
        << "*"
        << std::uppercase
        << std::hex
        << std::setw(2)
        << std::setfill('0')
        << static_cast<int>(checksum);

    return out.str();
}

std::string SensorProducerWebots::makeNmea(double t) const
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    if (d_.gps) {
        const double* v = wb_gps_get_values(d_.gps);

        if (finite3(v)) {
            x = v[0];
            y = v[1];
            z = v[2];
        }
    }

    const double east_m  = x;  
    const double north_m = y;  
    const double alt_m   = z;  
    std::cout << "[GPS][RAW_WEBOTS] "
          << "x=" << x
          << " y=" << y
          << " z=" << z
          << " east=" << east_m
          << " north=" << north_m
          << " alt=" << alt_m
          << std::endl;
    std::cout << "[GPS_AXIS_DEBUG] raw_x=" << x << " raw_y=" << y << " raw_z=" << z
              << " east=" << east_m << " north=" << north_m << " alt=" << alt_m << std::endl;
    const double lat = BASE_LAT + north_m / METERS_PER_DEG_LAT;
    const double lon = BASE_LON + east_m / (METERS_PER_DEG_LAT * std::cos(BASE_LAT * PI / 180.0));

    char ns = 'N';
    char ew = 'E';

    const std::string latNmea = decimalToNmea(lat, true, ns);
    const std::string lonNmea = decimalToNmea(lon, false, ew);

    double yawDeg = 0.0;

    if (d_.imu) {
        const double* rpy = wb_inertial_unit_get_roll_pitch_yaw(d_.imu);

        if (finite3(rpy))
            yawDeg = rpy[2] * 180.0 / PI;
    }

    if (yawDeg < 0.0)
        yawDeg += 360.0;

    const int hhmmss = static_cast<int>(std::fmod(t, 86400.0));

    const int hh = hhmmss / 3600;
    const int mm = (hhmmss % 3600) / 60;
    const int ss = hhmmss % 60;

    std::ostringstream time;

    time << std::setw(2) << std::setfill('0') << hh
         << std::setw(2) << std::setfill('0') << mm
         << std::setw(2) << std::setfill('0') << ss
         << ".00";

    std::ostringstream gga;

    gga << "GPGGA,"
        << time.str() << ","
        << latNmea << "," << ns << ","
        << lonNmea << "," << ew
        << ",1,08,0.9,"
        << std::fixed << std::setprecision(2)
        << alt_m
        << ",M,0.0,M,,";

    std::ostringstream rmc;

    rmc << "GPRMC,"
        << time.str()
        << ",A,"
        << latNmea << "," << ns << ","
        << lonNmea << "," << ew
        << ",0.0,"
        << std::fixed << std::setprecision(1)
        << yawDeg
        << ",180526,,,A";

    std::ostringstream gsa;
    gsa << "GPGSA,A,3,01,02,03,04,05,06,07,08,,,,1.5,0.9,1.2";

    std::ostringstream gsv;
    gsv << "GPGSV,3,1,11,01,40,090,40,02,20,180,35,03,15,225,30,04,75,300,42";

    std::ostringstream hdt;
    hdt << "GPHDT,"
        << std::fixed << std::setprecision(1)
        << yawDeg
        << ",T";

    std::ostringstream vtg;
    vtg << "GPVTG,"
        << std::fixed << std::setprecision(1)
        << yawDeg
        << ",T,,M,0.0,N,0.0,K,A";

    std::ostringstream apb;
    apb << "GPAPB,A,A,0.02,L,N,V,V,"
        << std::fixed << std::setprecision(1)
        << yawDeg
        << ",T,WP001,"
        << yawDeg
        << ",T,"
        << yawDeg
        << ",T";

    std::ostringstream aam;
    aam << "GPAAM,A,A,0.1,N,WP001";

    return nmeaChecksumLine(gga.str()) + "\n" +
           nmeaChecksumLine(rmc.str()) + "\n" +
           nmeaChecksumLine(gsa.str()) + "\n" +
           nmeaChecksumLine(gsv.str()) + "\n" +
           nmeaChecksumLine(hdt.str()) + "\n" +
           nmeaChecksumLine(vtg.str()) + "\n" +
           nmeaChecksumLine(apb.str()) + "\n" +
           nmeaChecksumLine(aam.str()) + "\n";
}