// SystemOrchestrator.hpp
#pragma once
#include "SensorWorkerManager.hpp"
#include <opencv2/opencv.hpp>
#include <mutex>

class SensorConsumer {
public:
    void init();
    void stop();
private:
    SensorWorkerManager worker_manager;
    void ConsumerImu();
    void ConsumerLidar();
    void ConsumerRadar();
    void ConsumerCamera();
    void ConsumerGps();
    void ConsumerEncoder();

    std::mutex      videoMutex;
    int             frameIndex = 0;
};