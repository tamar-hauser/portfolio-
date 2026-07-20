// SystemOrchestrator.hpp
#pragma once
#include <Eigen/Dense>
#include "SensorWorkerManager.hpp"

class ThreadGrafFector {
public:
    void init();
    void stop();
private:
    SensorWorkerManager worker_manager;
    void ConsumerProcessingFrame();
    void SecurityWall();
    void Optimization();
    void createNode();
    void removeNodeAndFector();
    Eigen::VectorXd m_last_kf_ekf_state;
    bool            m_last_kf_ekf_set = false;
};