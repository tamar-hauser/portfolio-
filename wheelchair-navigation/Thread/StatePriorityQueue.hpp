#pragma once

#include <mutex>
#include <memory>
#include <cmath>

#include "QueuePriority.hpp"
#include "Constants.hpp"
#include "KalmanFilter/ekf.hpp"
#include "Location.hpp"

class StatePriorityQueue {
private:
    EKF<Config::DimWheelchairStateVector> StateVector;
    ThreadSafePriorityQueue<StateUpdateTask> Queue;
    double TimeLastUpdate = -1.0;
    mutable std::mutex m_ekf_mutex;

    StatePriorityQueue() = default;

public:
    StatePriorityQueue(const StatePriorityQueue&) = delete;
    StatePriorityQueue& operator=(const StatePriorityQueue&) = delete;

    static StatePriorityQueue& getInstance() {
        static StatePriorityQueue instance;
        return instance;
    }

    StateUpdateTask popTask() {
        return Queue.pop();
    }

    void push(const StateUpdateTask& task) {
        Queue.push(task);
    }

    void pushTask(const StateUpdateTask& task) {
        Queue.push(task);
    }
    Eigen::VectorXd getStateVectorSafe() const {
        std::lock_guard<std::mutex> lock(m_ekf_mutex);
        return StateVector.x;
    }
    Eigen::VectorXd getStateVector() const {
        return getStateVectorSafe();
    }
    EKF<Config::DimWheelchairStateVector>& getEKF() {
        return StateVector;
    }

    const EKF<Config::DimWheelchairStateVector>& getEKF() const {
        return StateVector;
    }

    std::mutex& getEKFMutex() const {
        return m_ekf_mutex;
    }

    double getTimeLastUpdate() const {
        std::lock_guard<std::mutex> lock(m_ekf_mutex);
        return TimeLastUpdate;
    }

    void setTimeLastUpdate(double t) {
        std::lock_guard<std::mutex> lock(m_ekf_mutex);
        TimeLastUpdate = t;
    }

    Location myPlceInWorld() const {
        std::lock_guard<std::mutex> lock(m_ekf_mutex);

        constexpr double METERS_PER_DEG_LAT = 111320.0;

        const int ix = static_cast<int>(Config::StateMembersRobot::StateX);
        const int iy = static_cast<int>(Config::StateMembersRobot::StateY);

        const double x_meters = StateVector.x(ix);
        const double y_meters = StateVector.x(iy);

        const double lat = y_meters / METERS_PER_DEG_LAT;
        const double lat_rad = lat * (M_PI / 180.0);
        const double cos_lat = std::cos(lat_rad);

        const double lon = (std::abs(cos_lat) > 1e-9)
                               ? (x_meters / (METERS_PER_DEG_LAT * cos_lat))
                               : 0.0;

        return Location{lat, lon};
    }
};