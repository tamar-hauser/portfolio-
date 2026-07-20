#pragma once
#include <iostream>
#include <unordered_map>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <cstddef>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include "KalmanFilter\ekf.hpp" 
#include "SensorObject.hpp"
#include "Constants.hpp"
#include "Imu\ImuObject.hpp"     // *** הוספתי עבור ImuObject
#include "Encoder\EncoderObject.hpp" // *** הוספתי עבור EncoderObject
#include "Gps\GpsObject.hpp"     // *** הוספתי עבור GpsObject
#include "Encoder\EncoderList.hpp" // *** הוספתי עבור EncoderObject
#include "TrackedObject/IcpEstimator.hpp"
struct NodeFactor {
    EKF<Config::DimWheelchairStateVector> filter;
};

struct Factor {
    long long node_from;
    long long node_to;
    Eigen::VectorXd measurement;
    Eigen::MatrixXd H;
    double weight;
    SensorType type;
};

struct GpsBufferEntry {
    GpsObject gps;
    double    timestamp;
};

class FactorGraphManager {
private:
    std::unordered_map<long long, NodeFactor> poses;
    std::map<double, long long> time_to_id;      
    std::vector<Factor> factors;
    
    Eigen::SparseMatrix<double> A; 
    Eigen::VectorXd b;             
    Eigen::VectorXd delta_x;       
    
    long long current_node_id;
    int dim;
    bool new_factor;
    const size_t MAX_WINDOW_SIZE = 100;
    static constexpr double GPS_TIME_THRESHOLD = 0.5;
    mutable std::recursive_mutex m_mutex;

    std::vector<ImuObject>     m_imu_buffer;
    std::vector<EncoderObject> m_enc_buffer;
    Eigen::Matrix4f m_icp_accumulated = Eigen::Matrix4f::Identity();
    float  m_icp_fitness_sum = 0.0f;
    int    m_icp_count       = 0;
    std::vector<GpsBufferEntry> m_gps_buffer;
    std::chrono::steady_clock::time_point m_last_optimize_rejection{};
    void flushImuBuffer(long long from_id, long long to_id);
    void flushEncoderBuffer(long long from_id, long long to_id);
    void flushIcpBuffer(long long from_id, long long to_id);
    void flushGpsBuffer(long long to_id, double node_timestamp);

    FactorGraphManager();
    double normalizeAngle(double angle);

public:
    FactorGraphManager(const FactorGraphManager&) = delete;
    FactorGraphManager& operator=(const FactorGraphManager&) = delete;

    static FactorGraphManager& getInstance() {
        static FactorGraphManager instance;  
        return instance;
    }
    std::size_t getNodeCount();
    std::size_t getFactorCount();
    bool hasEnoughDataForOptimization();
    NodeFactor& getLastNode();
    long long addKeyframe(double timestamp, const NodeFactor& initial_guess);
    long long getClosestNodeId(double target_time);
    double getLastNodeTimestamp();
    Eigen::VectorXd computeError(const NodeFactor* from_node, const NodeFactor& to_node, const Factor& factor);
    bool optimizeGraph();
    void removeOldestNode();
    // פונקציות ליצירת פקטורים
    void createErrorIcp(const IcpEstimator::IcpResult& icp, double timenew, double timelast);
    void createErrorImu(const std::vector<ImuObject>& imu_list, double timenew, double timelast);
    void createErrorEncoder(const std::vector<EncoderObject>& encoder_list, double timenew, double timelast);
    void createErrorGps(const GpsObject& gps, double timenew);

    Eigen::VectorXd getMeasurementVector25D(const Eigen::Matrix4f& T);
};