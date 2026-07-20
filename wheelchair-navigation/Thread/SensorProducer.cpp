#include "SensorProducer.hpp"
#include "SensorProducerManager.hpp" // ה-Singleton של הנתונים
#include "Sensor/SensorConfig.hpp"
#include "SensorRing.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "Sensor\SensorMeasurement.hpp"
#include "Thread\SensorProducerManager.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void SensorProducer::init() {
    worker_manager.startWorker([this]() { this->ProducerImu(); });
    worker_manager.startWorker([this]() { this->ProducerLidar(); });
    worker_manager.startWorker([this]() { this->ProducerCamera(); });
    worker_manager.startWorker([this]() { this->ProducerRadar(); });
    worker_manager.startWorker([this]() { this->ProducerGps(); });
    worker_manager.startWorker([this]() { this->ProducerEncoder(); });

}

void SensorProducer::stop() {
    worker_manager.stop();
}

void SensorProducer::ProducerImu() {
    std::cout << "[Producer] IMU entered" << std::endl;
    ImuMeasurement dummy_imu{0, 9,0,0,0,0,0,0,0};    
    auto& manager = SensorProducerManager::getInstance();
    manager.addImuUpdate(manager.getTimeSinceStart(), dummy_imu);
    std::cout << "[Producer] IMU pushed" << std::endl;   
}

void SensorProducer::ProducerLidar() {
    static bool lidar_mock_mode = false;
    auto cloud_front = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto cloud_right = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto cloud_left = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    auto generate_clouds = [&](bool print) {
        const int N = 100;
        cloud_front->clear(); cloud_right->clear(); cloud_left->clear();
        for (int i = 0; i < N; ++i) {
            pcl::PointXYZI p;
            p.x = static_cast<float>(i) * 0.05f;
            p.y = 0.0f + static_cast<float>(i % 5) * 0.02f;
            p.z = 0.1f * (i % 3);
            p.intensity = 50.0f; 
            cloud_front->push_back(p);
        }
        for (int i = 0; i < N; ++i) {
            pcl::PointXYZI p;
            p.x = static_cast<float>(i) * 0.05f;
            p.y = 1.0f + static_cast<float>(i % 7) * 0.02f;
            p.z = 0.05f * (i % 4);
            p.intensity = 50.0f; 
            cloud_right->push_back(p);
        }
        for (int i = 0; i < N; ++i) {
            pcl::PointXYZI p;
            p.x = static_cast<float>(i) * 0.05f;
            p.y = -1.0f - static_cast<float>(i % 6) * 0.02f;
            p.z = 0.02f * (i % 5);
            p.intensity = 50.0f; 
            cloud_left->push_back(p);
        }
        if (print) std::cout << "[Producer] Lidar mock cloud generated" << std::endl;
    };

    if (!lidar_mock_mode) {
        bool front_ok = (pcl::io::loadPCDFile("lidar_front_test.pcd", *cloud_front) != -1);
        bool right_ok = (pcl::io::loadPCDFile("lidar_right_test.pcd", *cloud_right) != -1);
        bool left_ok = (pcl::io::loadPCDFile("lidar_left_test.pcd", *cloud_left) != -1);
        if (!front_ok || !right_ok || !left_ok) {
            lidar_mock_mode = true;
            generate_clouds(true);
        }
    }

    if (lidar_mock_mode) {
        generate_clouds(false);
    }

    auto& manager = SensorProducerManager::getInstance();
    manager.addLidarUpdate(manager.getTimeSinceStart(), cloud_front);
    manager.addLidarUpdate(manager.getTimeSinceStart(), cloud_right);
    manager.addLidarUpdate(manager.getTimeSinceStart(), cloud_left);
    std::cout << "[Producer] Lidar pushed" << std::endl;
}

void SensorProducer::ProducerEncoder() {
    static int left_ticks = 0;
    static int right_ticks = 0;
    
    int speed = 10;
    int turn_factor = 2; 
    
    left_ticks += (speed - turn_factor);
    right_ticks += (speed + turn_factor);

    EncoderMeasurement measurement;
    measurement.left_ticks = left_ticks;
    measurement.right_ticks = right_ticks;    
    auto& manager = SensorProducerManager::getInstance();
    manager.addEncoderUpdate(manager.getTimeSinceStart(), measurement);
    std::cout << "[Producer] Encoder pushed" << std::endl;

}

void SensorProducer::ProducerGps() {
    std::stringstream gpsStream;
    gpsStream << "$GPGGA,123456.00,3201.2345,N,03445.6789,E,1,08,0.9,105.2,M,12.3,M,,*64\n";
    gpsStream << "$GPRMC,123456.00,A,3201.2345,N,03445.6789,E,22.4,180.5,180526,,,A*7F\n";
    gpsStream << "$GPGSA,A,3,01,02,03,04,05,06,07,08,,,,1.5,0.9,1.2*36\n";
    gpsStream << "$GPGSV,3,1,11,01,40,090,40,02,20,180,35,03,15,225,30,04,75,300,42*7B\n";
    gpsStream << "$GPHDT,180.5,T*37\n";
    gpsStream << "$GPVTG,180.5,T,,M,22.4,N,41.5,K,A*31\n";
    gpsStream << "$GPAPB,A,A,0.02,L,N,V,V,179.5,T,WP001,178.0,T,178.0,T*23\n";
    gpsStream << "$GPAAM,A,A,0.1,N,WP001*3F\n";    
    auto& manager = SensorProducerManager::getInstance();
    manager.addGpsUpdate(manager.getTimeSinceStart(), gpsStream.str());
}

void SensorProducer::ProducerRadar() {
    std::cout << "[Producer] Radar entered" << std::endl;
    std::vector<RadarMeasurement> frame_data;
    RadarMeasurement target;
    target.id = 1;
    target.pos_x = 15.0f;
    target.pos_y = 2.5f;
    target.pos_z = 1.0f;  
    target.vel_x = 1.2f;
    target.vel_y = -0.3f;
    target.vel_z = 0.0f;
    RadarMeasurement clutter;
    clutter.id = 2;
    clutter.pos_x = 0.0f;
    clutter.pos_y = 0.0f;
    clutter.pos_z = 0.0f;
    clutter.vel_x = 0.0f;
    clutter.vel_y = 0.0f;
    clutter.vel_z = 0.0f;
    frame_data.push_back(target);
    frame_data.push_back(clutter);
    auto& manager = SensorProducerManager::getInstance();
    manager.addRadarUpdate(manager.getTimeSinceStart(), frame_data);
    std::cout << "[Producer] Radar pushed" << std::endl;
}
void SensorProducer::ProducerCamera() {
    static bool camera_blank_mode = false;
    cv::Mat frame;
    if (!camera_blank_mode) {
        frame = cv::imread("IMG_7257.jpg");
        if (frame.empty()) {
            camera_blank_mode = true;
            frame = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
            std::cout << "[Producer] Camera blank frame generated" << std::endl;
        }
    } else {
        frame = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    }
    auto& manager = SensorProducerManager::getInstance();
    manager.addCameraUpdate(manager.getTimeSinceStart(), frame);
    std::cout << "[Producer] Camera pushed" << std::endl;
}