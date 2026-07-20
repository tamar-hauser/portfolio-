#pragma once
#include "MapData.hpp"
#include <chrono>
#include <cmath>
#include <utility>
#ifdef WEBOTS_CONTROLLER
#include "WebotsBridge/WebotsDrive.hpp"
#endif
constexpr double METERS_PER_DEG_LAT  = 111320.0;
constexpr float  CELL_REACH_THRESHOLD = 0.50f;  // lookahead [m] — Pure Pursuit
constexpr auto   NAV_LOOP_PERIOD      = std::chrono::milliseconds(200);

static std::pair<float,float> latLonToMeters(double lat, double lon)
{
    const double lat_rad = lat * (M_PI / 180.0);
    return {
        static_cast<float>(lon * METERS_PER_DEG_LAT * std::cos(lat_rad)),
        static_cast<float>(lat * METERS_PER_DEG_LAT)
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
static float distToNode(const Eigen::VectorXd& state, const Node& node)
{
    auto [nx, ny] = latLonToMeters(node.lat, node.lon);
    const float dx = static_cast<float>(
        state(static_cast<int>(Config::StateMembersRobot::StateX))) - nx;
    const float dy = static_cast<float>(
        state(static_cast<int>(Config::StateMembersRobot::StateY))) - ny;
    return std::sqrt(dx*dx + dy*dy);
}

// ─────────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
static float distToCell(const Eigen::VectorXd& state,
                         std::pair<int,int>     cell,
                         const MapData&         mapData)
{
    const float cx = mapData.originX + (cell.first  + 0.5f) * RESOLUTION;
    const float cy = mapData.originY + (cell.second + 0.5f) * RESOLUTION;
    const float dx = static_cast<float>(
        state(static_cast<int>(Config::StateMembersRobot::StateX))) - cx;
    const float dy = static_cast<float>(
        state(static_cast<int>(Config::StateMembersRobot::StateY))) - cy;
    return std::sqrt(dx*dx + dy*dy);
}

// ─────────────────────────────────────────────────────────────────────────────
//  sendDriveCommand()
// ─────────────────────────────────────────────────────────────────────────────
static void sendDriveCommand(float linear_speed, float angular_speed)
{
#ifdef WEBOTS_CONTROLLER
    // Webots controller mode: send directly to Webots wheel motors.
    WebotsDrive::getInstance().send(linear_speed, angular_speed);
#else
    // Real hardware mode / non-Webots build.
    // WheelchairBinding::getInstance().send(linear_speed, angular_speed);
    (void)linear_speed;
    (void)angular_speed;
#endif
}
static std::pair<float,float> computeCommand(const Eigen::VectorXd& state,
                                              std::pair<int,int>     next_cell,
                                            MapData&         mapData)
{

    const float target_x = mapData.originX + (next_cell.first  + 0.5f) * RESOLUTION;
    const float target_y = mapData.originY + (next_cell.second + 0.5f) * RESOLUTION;
    
    const float robot_x   = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateX)));
    const float robot_y   = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateY)));
    const float robot_yaw = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateYaw)));

    float target_heading = std::atan2(target_y - robot_y, target_x - robot_x);
    float heading_error  = target_heading - robot_yaw;
    while (heading_error >  M_PI) heading_error -= 2.0f * static_cast<float>(M_PI);
    while (heading_error < -M_PI) heading_error += 2.0f * static_cast<float>(M_PI);

    constexpr float ANGULAR_GAIN = 0.6f;
    constexpr float WB_OVER_2R  = 1.4118f;
    constexpr float INV_R       = 2.941f;   // 1/wheelRadius, R≈0.34

    float absErr = std::fabs(heading_error);

    float linear;
    float angular_max;
    if      (absErr > 1.4f) { linear = 0.10f; angular_max = 0.15f; }  // arc: safe=0.10*2.941/1.4118*0.95=0.198
    else if (absErr > 0.8f) { linear = 0.16f; angular_max = 0.22f; }  // safe: 0.16*2.941/1.4118*0.95=0.317
    else if (absErr > 0.5f) { linear = 0.26f; angular_max = 0.18f; }  // safe: 0.26*2.941/1.4118*0.95=0.515
    else                    { linear = 0.45f; angular_max = 0.15f; }  // safe: 0.45*2.941/1.4118*0.95=0.891

    float angular = ANGULAR_GAIN * heading_error;
    angular = std::clamp(angular, -angular_max, angular_max);

    if (linear > 0.0f) {
        float max_safe_angular = linear * INV_R / WB_OVER_2R * 0.95f;
        if (std::fabs(angular) > max_safe_angular) {
            float old_angular = angular;
            angular = std::copysign(max_safe_angular, angular);
            std::cout << "[ANGULAR_SAFETY_CLAMP] clamped from " << old_angular
                      << " to " << angular
                      << " max_safe=" << max_safe_angular
                      << std::endl;
        }
    }

    std::cout << "[MOVE_TARGET_DEBUG]"
              << " robot_x=" << robot_x
              << " robot_y=" << robot_y
              << " robot_yaw=" << robot_yaw
              << " target_x=" << target_x
              << " target_y=" << target_y
              << " target_heading=" << target_heading
              << " heading_error=" << heading_error
              << std::endl;

    std::cout << "[MOVE_GATED] heading_error=" << heading_error
              << " linear=" << linear
              << " angular=" << angular
              << std::endl;

    if (absErr > 1.4f) {
        std::cout << "[NO_SPIN_STARTUP] heading_error=" << heading_error
                  << " linear=" << linear << " angular=" << angular << std::endl;
    }

    float leftVel_approx  = linear * INV_R - angular * WB_OVER_2R;
    float rightVel_approx = linear * INV_R + angular * WB_OVER_2R;

    std::cout << "[WHEEL_SIGN_DEBUG] leftVel=" << leftVel_approx
              << " rightVel=" << rightVel_approx
              << std::endl;

    if (linear > 0.0f && leftVel_approx * rightVel_approx < 0.0f) {
        std::cout << "[TURN_IN_PLACE_WARNING] linear=" << linear
                  << " angular=" << angular
                  << " leftVel=" << leftVel_approx
                  << " rightVel=" << rightVel_approx
                  << std::endl;
    }

    return { linear, angular };
}

// ─────────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
static void stopEkf(StatePriorityQueue& stateQ)
{
    StateUpdateTask t;
    t.priority  = UpdatePriority::HIGH;
    // Avoid pushing a task with the sentinel -1.0 timestamp.
    // Use the EKF last update if available, otherwise use 0.0 (initial valid time).
    double last = stateQ.getTimeLastUpdate();
    t.timestamp = (last >= 0.0) ? last : 0.0;
    t.execute   = [](EKF<Config::DimWheelchairStateVector>& ekf) {
        ekf.x(static_cast<int>(Config::StateMembersRobot::StateVx))   = 0.0;
        ekf.x(static_cast<int>(Config::StateMembersRobot::StateVyaw)) = 0.0;
        ekf.x(static_cast<int>(Config::StateMembersRobot::StateAx))   = 0.0;
    };
    stateQ.push(t);
}