// SystemOrchestrator.hpp
#pragma once
#include "SensorWorkerManager.hpp"

class SensorProducer {
public:
    void init();   
    void stop();  
private:
    SensorWorkerManager worker_manager;
    void ProducerImu(); 
    void ProducerLidar();
    void ProducerRadar();
    void ProducerCamera();
    void ProducerGps();
    void ProducerEncoder();

};