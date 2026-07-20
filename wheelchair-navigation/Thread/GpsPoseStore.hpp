#pragma once
#include <mutex>
#include <atomic>

// Thread-safe store for the latest valid GPS pose.
// Written by ThreadGrafFector, read by the navigation loop in main.cpp.
struct GpsPoseStore {
    struct Pose {
        double x           = 0.0;  // East [m]
        double y           = 0.0;  // North [m]
        double heading_deg = 0.0;  // compass: CW from North [deg]
        double speed_mps   = 0.0;  // GPS-reported ground speed [m/s]
    };

    static GpsPoseStore& getInstance() {
        static GpsPoseStore instance;
        return instance;
    }

    void update(double x, double y, double heading_deg, double speed_mps) {
        std::lock_guard<std::mutex> lock(mtx);
        pose.x           = x;
        pose.y           = y;
        pose.heading_deg = heading_deg;
        pose.speed_mps   = speed_mps;
        valid_.store(true);
    }

    bool hasValid() const { return valid_.load(); }

    Pose get() const {
        std::lock_guard<std::mutex> lock(mtx);
        return pose;
    }

private:
    GpsPoseStore() = default;
    mutable std::mutex  mtx;
    std::atomic<bool>   valid_{false};
    Pose                pose;
};
