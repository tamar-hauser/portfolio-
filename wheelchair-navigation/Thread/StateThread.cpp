#include "StateThread.hpp"
#include "StatePriorityQueue.hpp"
#include "Constants.hpp"

#include <iostream>
#include <exception>
#include <cmath>
#include <mutex>
#include <algorithm>

void StateThread::init() {
    is_running_ = true;
    main_thread_ = std::thread(&StateThread::stateMainLoop, this);
}

void StateThread::stop() {
    is_running_ = false;    
    StateUpdateTask dummy_task;
    dummy_task.priority = UpdatePriority::CRITICAL;
    dummy_task.timestamp = 0.0;
    dummy_task.execute = nullptr;
    StatePriorityQueue::getInstance().pushTask(dummy_task);

    if (main_thread_.joinable()) {
        main_thread_.join();
    }
}

void StateThread::stateMainLoop() {
    auto& state_queue = StatePriorityQueue::getInstance();
    static int ekf_exec_count = 0;

    std::cout << "[StateThread] EKF Main Loop started." << std::endl;

    while (is_running_) {

        StateUpdateTask task = state_queue.popTask();
        if (!is_running_ || !task.execute) {
            break;
        }

        if (task.timestamp < 0.0) {
            std::cerr << "[StateThread] Skipping task with invalid timestamp: " << task.timestamp << std::endl;
            continue;
        }
        const double current_timestamp = task.timestamp;
        double last_timestamp = state_queue.getTimeLastUpdate();

        constexpr double MIN_VALID_DT = 0.0005;
        constexpr double MAX_VALID_DT = 0.5;   
        constexpr double MAX_ABS_VX           = 2.0; 
        constexpr double VX_DECAY_PER_PREDICT = 0.999; 
        const bool dt_known = (last_timestamp >= 0.0);
        double dt = 0.0;
        if (dt_known) {
            dt = current_timestamp - last_timestamp;
        }
        const bool dt_valid = dt_known && (dt >= MIN_VALID_DT) && (dt <= MAX_VALID_DT);
        {
            std::unique_lock<std::mutex> lock(state_queue.getEKFMutex());

            auto& ekf = state_queue.getEKF();
            if (ekf.getState().rows() != Config::DimWheelchairStateVector) {
                std::cerr << "[StateThread] EKF state invalid size: " << ekf.getState().rows() << std::endl;
                continue;
            }

            const int IDX_X     = static_cast<int>(Config::StateMembersRobot::StateX);
            const int IDX_Y     = static_cast<int>(Config::StateMembersRobot::StateY);
            const int IDX_YAW   = static_cast<int>(Config::StateMembersRobot::StateYaw);
            const int IDX_VX    = static_cast<int>(Config::StateMembersRobot::StateVx);
            const int IDX_VYAW  = static_cast<int>(Config::StateMembersRobot::StateVyaw);
            const int IDX_AX    = static_cast<int>(Config::StateMembersRobot::StateAx);

            const Eigen::VectorXd current_state = ekf.getState();
            const double vx_before   = current_state(IDX_VX);
            const double a_x_for_log = current_state(IDX_AX);
            double vx_after = vx_before;

            if (dt_valid) {

                const double yaw   = current_state(IDX_YAW);
                const double v_x   = current_state(IDX_VX);
                const double v_yaw = current_state(IDX_VYAW);
                const double a_x   = current_state(IDX_AX);

                const double dt2 = dt * dt;

                Eigen::VectorXd x_next = current_state;

                x_next(IDX_X) += (v_x * std::cos(yaw) * dt)
                               + (0.5 * a_x * std::cos(yaw) * dt2);

                x_next(IDX_Y) += (v_x * std::sin(yaw) * dt)
                               + (0.5 * a_x * std::sin(yaw) * dt2);

                x_next(IDX_YAW) += v_yaw * dt;
                x_next(IDX_VX) = v_x * VX_DECAY_PER_PREDICT + a_x * dt;
                x_next(IDX_VX) = std::clamp(x_next(IDX_VX), -MAX_ABS_VX, MAX_ABS_VX);

                // Jacobian F
                Eigen::MatrixXd F = Eigen::MatrixXd::Identity(Config::DimWheelchairStateVector, Config::DimWheelchairStateVector);

                F(IDX_X, IDX_VX) = std::cos(yaw) * dt;
                F(IDX_X, IDX_YAW) = -v_x * std::sin(yaw) * dt - 0.5 * a_x * std::sin(yaw) * dt2;
                F(IDX_X, IDX_AX) = 0.5 * std::cos(yaw) * dt2;
                F(IDX_Y, IDX_VX) = std::sin(yaw) * dt;
                F(IDX_Y, IDX_YAW) = v_x * std::cos(yaw) * dt + 0.5 * a_x * std::cos(yaw) * dt2;
                F(IDX_Y, IDX_AX) = 0.5 * std::sin(yaw) * dt2;
                F(IDX_YAW, IDX_VYAW) = dt;
                F(IDX_VX, IDX_AX) = dt;

                // Process noise Q
                Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(Config::DimWheelchairStateVector, Config::DimWheelchairStateVector);
                Q(IDX_X, IDX_X) = 0.05;
                Q(IDX_Y, IDX_Y) = 0.05;
                Q(IDX_YAW, IDX_YAW) = 0.01;
                Q(IDX_VX, IDX_VX) = 0.1;
                Q(IDX_VYAW, IDX_VYAW) = 0.1;
                Q(IDX_AX, IDX_AX) = 0.5;

                ekf.predict(x_next, F, Q);
                vx_after = x_next(IDX_VX);
            } else if (dt_known) {
                std::cerr << "[StateThread][WARN] dt out of range, skipping robot predict: dt=" << dt
                          << " task_source=" << task.source
                          << " task_ts=" << current_timestamp
                          << " last_ts=" << last_timestamp << std::endl;
            }

            static int dt_log_count = 0;
            const bool should_log_dt = (++dt_log_count % 100 == 1);
            if ((dt_known && !dt_valid) || should_log_dt) {
                std::cout << "[EKF_DT_DEBUG]"
                          << " task_type=" << task.source
                          << " task_ts=" << current_timestamp
                          << " last_ts=" << last_timestamp
                          << " dt=" << dt
                          << " dt_valid=" << dt_valid
                          << " Ax=" << a_x_for_log
                          << " Vx_before=" << vx_before
                          << " Vx_after=" << vx_after
                          << std::endl;
            }

            lock.unlock();
        }

        Eigen::VectorXd logged_state;
        bool should_log = (++ekf_exec_count % 100 == 0);
        try {
            {
                std::lock_guard<std::mutex> exec_lock(state_queue.getEKFMutex());
                auto& ekf_for_task = state_queue.getEKF();
                task.execute(ekf_for_task);
                if (should_log)
                    logged_state = ekf_for_task.getState();
            }
            state_queue.setTimeLastUpdate(current_timestamp);

            if (should_log) {
                const int IDX_X   = static_cast<int>(Config::StateMembersRobot::StateX);
                const int IDX_Y   = static_cast<int>(Config::StateMembersRobot::StateY);
                const int IDX_YAW = static_cast<int>(Config::StateMembersRobot::StateYaw);
                const int IDX_VX  = static_cast<int>(Config::StateMembersRobot::StateVx);
                std::cout << "[EKF] t=" << current_timestamp
                          << " x=" << logged_state(IDX_X)
                          << " y=" << logged_state(IDX_Y)
                          << " yaw=" << logged_state(IDX_YAW)
                          << " vx=" << logged_state(IDX_VX)
                          << " (exec#" << ekf_exec_count << ")" << std::endl;
            }

        } catch (const std::exception& e) {
            std::cerr << "[EKF] task failed: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[EKF] unknown task failure" << std::endl;
        }
    }

    std::cout << "[StateThread] EKF Main Loop stopped." << std::endl;
}