#include "ThreadGrafFector.hpp"
#include "FactorGraphManager.hpp"
#include "GpsPoseStore.hpp"
#include "Thread/StatePriorityQueue.hpp"
#include <cmath>
#include <limits>
#include "Constants.hpp"
#include "AStar.hpp"
#include "NavigationManager.hpp"
#include "Thread/SensorProducerManager.hpp"
#include "Thread/SensorFrameManager.hpp"
#include "SensorFusion/SensorFusionRadar.hpp"
#include "SensorFusion/SensorFusionManager.hpp"
#include "SensorFusion/SensorFusionLidar.hpp"
#include "SensorFusion/SensorFusionCamera.hpp"
#include "TrackedObject/IcpEstimator.hpp"
#include <iostream>
#include <chrono>
#include <exception>

void ThreadGrafFector::init() {
    std::cout << "[INIT] ThreadGrafFector::init() starting 5 workers" << std::endl;
    worker_manager.startWorker([this]() { this->ConsumerProcessingFrame(); });
    worker_manager.startWorker([this]() { this->Optimization(); });
    worker_manager.startWorker([this]() { this->createNode(); });
    worker_manager.startWorker([this]() { this->removeNodeAndFector(); });
    worker_manager.startWorker([this]() { this->SecurityWall(); });
    std::cout << "[INIT] ThreadGrafFector::init() all workers started" << std::endl;
}

void ThreadGrafFector::stop() {
    std::cout << "[THREAD] ThreadGrafFector::stop() called" << std::endl;
    worker_manager.stop();
    std::cout << "[THREAD] ThreadGrafFector::stop() complete" << std::endl;
}

void ThreadGrafFector::SecurityWall()
{
    static int sw_count = 0;
    if (++sw_count % 50 == 0)
        std::cout << "[THREAD] SecurityWall checking safety (iter=" << sw_count << ")" << std::endl;

    auto& state_queue  = StatePriorityQueue::getInstance();
    auto& managerframe = SensorFrameManager::getInstance();
    auto& navigation   = NavigationManager::getInstance();

    const Eigen::VectorXd state = state_queue.getStateVector();

    const float robot_x = static_cast<float>(
        state(static_cast<int>(Config::StateMembersRobot::StateX)));

    const float robot_y = static_cast<float>(
        state(static_cast<int>(Config::StateMembersRobot::StateY)));

    Eigen::Vector3f ego_pos(robot_x, robot_y, 0.0f);

    const auto& tracked_map = managerframe.getTrackedObject();

    Node* currentNode = navigation.getCurrentNode();

    bool atCrossing = false;

    if (currentNode)
    {
        constexpr double METERS_PER_DEG_LAT  = 111320.0;
        constexpr float  CROSSING_REACH_THRESHOLD = 1.0f;

        const double ref_lat = currentNode->lat * DEG_TO_RAD;

        const auto md = navigation.getMapData();
        const double origin_lon = md ? md->origin_lon : 0.0;
        const double origin_lat = md ? md->origin_lat : 0.0;

        const float node_x = static_cast<float>(
            (currentNode->lon - origin_lon) * METERS_PER_DEG_LAT * std::cos(ref_lat));

        const float node_y = static_cast<float>(
            (currentNode->lat - origin_lat) * METERS_PER_DEG_LAT);

        const float dx = robot_x - node_x;
        const float dy = robot_y - node_y;
        const float dist_to_node = std::sqrt(dx * dx + dy * dy);

        atCrossing = dist_to_node < CROSSING_REACH_THRESHOLD
                  && currentNode->is_crossing;
    }

    bool dynamicDangerExists = false;
    bool trafficLightSeen = false;
    bool trafficBlocked = false;
    float nearest_dynamic_distance = std::numeric_limits<float>::infinity();

    for (const auto& [id, track_ptr] : tracked_map)
    {
        if (!track_ptr) continue;

        const Eigen::Vector3f obj_pos = track_ptr->position();
        const Eigen::Vector3f obj_vel = track_ptr->velocity();

        const Eigen::Vector3f to_robot = ego_pos - obj_pos;

        const float distance = to_robot.norm();
        const float speed = obj_vel.norm();

        if (speed >= DYNAMIC_SPEED_THRESHOLD && distance < nearest_dynamic_distance) {
            nearest_dynamic_distance = distance;
        }

        bool movingTowardRobot = false;
        if (speed > 0.01f && distance > 0.01f) {
            movingTowardRobot = obj_vel.normalized().dot(to_robot.normalized()) > 0.3f;
        }

        if (speed >= DYNAMIC_SPEED_THRESHOLD &&
            distance < 1.5f &&
            movingTowardRobot)
        {
            dynamicDangerExists = true;
        }

        if (atCrossing &&
            currentNode &&
            currentNode->has_traffic_light &&
            track_ptr->type_label == "traffic light")
        {
            trafficLightSeen = true;

            const std::string& color = track_ptr->traffic_light_color;

            if (color != "green") {
                trafficBlocked = true;
            }
        }
    }
    if (atCrossing && currentNode && currentNode->has_traffic_light && !trafficLightSeen) {
        trafficBlocked = true;
    }

    bool crossingBlocked = false;
    if (atCrossing && currentNode && !currentNode->has_traffic_light) {
        crossingBlocked = dynamicDangerExists;
    }

    if (sw_count % 50 == 0) {
        std::cout << "[THREAD] SecurityWall result: dynamic=" << dynamicDangerExists
                  << " traffic=" << trafficBlocked
                  << " crossing=" << crossingBlocked << std::endl;
        std::cout << "[SECURITY_WALL_DEBUG]"
                  << " dynamicBlocked=" << dynamicDangerExists
                  << " trafficBlocked=" << trafficBlocked
                  << " crossingBlocked=" << crossingBlocked
                  << " nearest_dynamic_distance=" << nearest_dynamic_distance
                  << std::endl;
    }

    navigation.setDynamicBlocked(dynamicDangerExists);
    navigation.setTrafficBlocked(trafficBlocked);
    navigation.setCrossingBlocked(crossingBlocked);
}




///פונקציה ראשית
void ThreadGrafFector::ConsumerProcessingFrame() {
    static int cpf_count = 0;
    ++cpf_count;
    if (cpf_count % 50 == 0)
        std::cout << "[THREAD] ConsumerProcessingFrame entered (iter=" << cpf_count << ")" << std::endl;

    auto& managerframe = SensorFrameManager::getInstance();
    auto& graf_fector = FactorGraphManager::getInstance();
    auto& Navigation = NavigationManager::getInstance();
    auto& stateQ=StatePriorityQueue::getInstance();
    SensorFusionManager FusionManger;
    SensorFusionRadar FusionRadar;
    SensorFusionCamera FusionCamera;
    SensorFusionLidar FusionLidar;

    IcpEstimator icp_estimator;
    try {
        std::shared_ptr<FramePointers> matched_frame = managerframe.getProcessFrame();

        if (matched_frame) {
            bool has_lidar_data = false;  
            bool has_gps_data   = false;  
            {
            std::lock_guard<std::mutex> lock(matched_frame->frame_mutex);

            const bool has_imu    = matched_frame->imu     && !matched_frame->imu->list.empty();
            const bool has_enc    = matched_frame->encoder  && !matched_frame->encoder->list.empty();
            has_lidar_data        = matched_frame->lidar    && !matched_frame->lidar->list.empty();
            const bool has_camera = matched_frame->camera   && !matched_frame->camera->list.empty();
            const bool has_radar  = matched_frame->radar    && !matched_frame->radar->list.empty();
            const bool has_gps    = matched_frame->gps      && matched_frame->gps->isValid;
            has_gps_data          = has_gps;

            std::cout << "[FRAME][PROCESS] selected bucket ts=" << matched_frame->timestamp << std::endl;
            std::cout << "[FRAME][PROCESS] has imu=" << has_imu
                      << " encoder=" << has_enc
                      << " gps=" << has_gps
                      << " lidar=" << has_lidar_data
                      << " camera=" << has_camera
                      << " radar=" << has_radar << std::endl;

            if (!matched_frame->sensorfusion) {
                std::cerr << "[WARN] ConsumerProcessingFrame: sensorfusion is nullptr" << std::endl;
                matched_frame->is_processed = true;
                return;
            }

            if (has_lidar_data || has_camera || has_radar) {
                int sf_before = static_cast<int>(matched_frame->sensorfusion->list.size());
                if (has_radar)      FusionRadar.process(matched_frame->radar->list,  matched_frame->sensorfusion->list);
                int after_radar = static_cast<int>(matched_frame->sensorfusion->list.size());
                if (has_lidar_data) FusionLidar.process(matched_frame->lidar->list,  matched_frame->sensorfusion->list);
                int after_lidar = static_cast<int>(matched_frame->sensorfusion->list.size());
                if (has_camera)     FusionCamera.process(matched_frame->camera->list, matched_frame->sensorfusion->list);
                int after_camera = static_cast<int>(matched_frame->sensorfusion->list.size());
                std::cout << "[FUSION] from sensors: radar=" << (after_radar - sf_before)
                          << " lidar=" << (after_lidar - after_radar)
                          << " camera=" << (after_camera - after_lidar) << std::endl;
                std::vector<TrackedObject> TrackedList = FusionManger.getTrackedListAsVector(managerframe.getTrackedObject());
                FusionManger.clearFrameLists();
                FusionManger.process(matched_frame->sensorfusion->list, TrackedList);
                FusionManger.initTrackedMap(TrackedList);
                managerframe.setTrackedObjects(TrackedList);
                matched_frame->idObjectUpdate = FusionManger.getObjectUpdate();
                matched_frame->idObjectRemove = FusionManger.getObjectRemove();
                auto best_objects = FusionManger.getBestObjectsForICP();
                matched_frame->tracked.clear();
                matched_frame->tracked = best_objects;
                std::cout << "[FUSION] tracked objects count=" << matched_frame->tracked.size() << std::endl;
                std::cout << "[OBSTACLE_PROCESSING_DEBUG]"
                          << " lidar_objects=" << (after_lidar - after_radar)
                          << " camera_objects=" << (after_camera - after_lidar)
                          << " radar_objects=" << (after_radar - sf_before)
                          << " tracked_objects_count=" << matched_frame->tracked.size()
                          << std::endl;
            } else {
                std::cout << "[FRAME][PROCESS] skipping object fusion: no detection sensor data" << std::endl;
            }

            double prev_time = matched_frame->timestamp;
            bool can_icp = false;
            if (auto prev_frame = matched_frame->previousFrame.lock()) {
                prev_time = prev_frame->timestamp;
                can_icp = has_lidar_data;
            }
            if (can_icp) {
                graf_fector.createErrorIcp(icp_estimator.process(matched_frame), matched_frame->timestamp, prev_time);
            } else {
                std::cout << "[FRAME][PROCESS] skipping ICP: no lidar or no previous frame" << std::endl;
            }

            if (has_imu)
                graf_fector.createErrorImu(matched_frame->imu->list, matched_frame->timestamp, prev_time);
            else
                std::cout << "[FRAME][PROCESS] skipping IMU factor: no IMU data" << std::endl;

            if (has_enc)
                graf_fector.createErrorEncoder(matched_frame->encoder->list, matched_frame->timestamp, prev_time);
            else
                std::cout << "[FRAME][PROCESS] skipping Encoder factor: no encoder data" << std::endl;

            if (has_gps)
                graf_fector.createErrorGps(*matched_frame->gps, matched_frame->timestamp);
            else
                std::cout << "[FRAME][PROCESS] skipping GPS factor: no valid GPS in frame" << std::endl;

            matched_frame->is_processed = true;
            std::cout << "[FRAME][PROCESS] marking processed ts=" << matched_frame->timestamp << std::endl;
            }  
            Eigen::VectorXd state = stateQ.getStateVector();
            auto trackedObjects = managerframe.getTrackedObject();

            if (has_gps_data && matched_frame->gps) {
                const int IDX_X   = static_cast<int>(Config::StateMembersRobot::StateX);
                const int IDX_Y   = static_cast<int>(Config::StateMembersRobot::StateY);
                const int IDX_YAW = static_cast<int>(Config::StateMembersRobot::StateYaw);
                float ekf_x   = static_cast<float>(state(IDX_X));
                float ekf_y   = static_cast<float>(state(IDX_Y));
                float ekf_yaw = static_cast<float>(state(IDX_YAW));
                float gps_x   = static_cast<float>(matched_frame->gps->x_local);
                float gps_y   = static_cast<float>(matched_frame->gps->y_local);
                float gps_hdg   = static_cast<float>(matched_frame->gps->heading);
                float gps_speed = static_cast<float>(matched_frame->gps->speed);
                std::cout << "[POSE_SYNC_DEBUG] gps_x=" << gps_x << " gps_y=" << gps_y
                          << " ekf_x=" << ekf_x << " ekf_y=" << ekf_y
                          << " dx=" << (ekf_x - gps_x) << " dy=" << (ekf_y - gps_y)
                          << " ekf_yaw=" << ekf_yaw
                          << " gps_heading_deg=" << gps_hdg << std::endl;
                GpsPoseStore::getInstance().update(gps_x, gps_y, gps_hdg, gps_speed);
            }

            if (has_lidar_data) {
                double current_time_nav = stateQ.getTimeLastUpdate();
                double frame_age = current_time_nav - matched_frame->timestamp;
                if (frame_age > 3.5) {
                    std::cout << "[NAV_STALE_FRAME_SKIP] frame ts=" << matched_frame->timestamp
                              << " current_time=" << current_time_nav
                              << " age=" << frame_age << "s > 3.5s — skipping runDLite" << std::endl;
                } else {
                    std::cout << "[NAV] Navigation.runDLite starting ts=" << matched_frame->timestamp << std::endl;
                    Eigen::VectorXd planning_state = state;
                    if (GpsPoseStore::getInstance().hasValid()) {
                        auto gp = GpsPoseStore::getInstance().get();
                        planning_state(static_cast<int>(Config::StateMembersRobot::StateX)) = gp.x;
                        planning_state(static_cast<int>(Config::StateMembersRobot::StateY)) = gp.y;
                        std::cout << "[DSTAR_PLAN_POS] GPS x=" << gp.x << " y=" << gp.y
                                  << " (ekf_x=" << state(static_cast<int>(Config::StateMembersRobot::StateX))
                                  << " ekf_y=" << state(static_cast<int>(Config::StateMembersRobot::StateY)) << ")" << std::endl;
                    }
                    Navigation.runDLite(matched_frame, trackedObjects, planning_state);
                    std::cout << "[NAV] Navigation.runDLite ended ts=" << matched_frame->timestamp << std::endl;
                }
            }

            managerframe.cleanupOldFrames(matched_frame->timestamp);

        } else {
            if (cpf_count % 50 == 0)
                std::cerr << "[WARN] ConsumerProcessingFrame skipped: frame is nullptr (iter=" << cpf_count << ")" << std::endl;
        }

    } catch (const std::exception& e) { 
        std::cerr << "[ERROR] ConsumerProcessingFrame crashed: " << e.what() << std::endl;
    }
}

void ThreadGrafFector::Optimization() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    static int opt_count = 0;
    ++opt_count;
    if (opt_count % 10 == 0)
        std::cout << "[THREAD] Optimization entered (iter=" << opt_count << ")" << std::endl;

    auto& graf_fector = FactorGraphManager::getInstance();
    if (!graf_fector.hasEnoughDataForOptimization()) {
        if (opt_count % 50 == 0)
            std::cout << "[FACTOR] Optimization skipped in ThreadGrafFector: not enough graph data"
                      << " nodes=" << graf_fector.getNodeCount()
                      << " factors=" << graf_fector.getFactorCount()
                      << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return;
    }
    Eigen::VectorXd pre_opt_state = graf_fector.getLastNode().filter.x;

    bool opt_ok = graf_fector.optimizeGraph();
    if (!opt_ok) {
        std::cout << "[FACTOR_OPTIMIZATION_SKIPPED_EKF_CORRECTION]" << std::endl;
        return;
    }

    const auto& best_node      = graf_fector.getLastNode();
    double      node_timestamp = graf_fector.getLastNodeTimestamp();

    Eigen::VectorXd graph_delta  = best_node.filter.x - pre_opt_state;
    Eigen::MatrixXd post_opt_P   = best_node.filter.P;

    const int idx_x    = static_cast<int>(Config::StateMembersRobot::StateX);
    const int idx_y    = static_cast<int>(Config::StateMembersRobot::StateY);
    const int idx_yaw  = static_cast<int>(Config::StateMembersRobot::StateYaw);
    const int idx_vx   = static_cast<int>(Config::StateMembersRobot::StateVx);
    const int idx_vyaw = static_cast<int>(Config::StateMembersRobot::StateVyaw);
    const int idx_ax   = static_cast<int>(Config::StateMembersRobot::StateAx);

    double pre_x  = pre_opt_state(idx_x);
    double pre_y  = pre_opt_state(idx_y);
    double post_x = best_node.filter.x(idx_x);
    double post_y = best_node.filter.x(idx_y);

    double dx_corr = graph_delta(idx_x);
    double dy_corr = graph_delta(idx_y);
    double delta_xy_norm = std::hypot(dx_corr, dy_corr);

    constexpr double MAX_SANE_POSITION = 500.0;
    if (!std::isfinite(pre_x) || !std::isfinite(pre_y) ||
        !std::isfinite(post_x) || !std::isfinite(post_y) ||
        std::abs(pre_x) > MAX_SANE_POSITION || std::abs(pre_y) > MAX_SANE_POSITION ||
        std::abs(post_x) > MAX_SANE_POSITION || std::abs(post_y) > MAX_SANE_POSITION) {
        std::cerr << "[GRAPH_CORRECTION_REJECTED] position diverged"
                  << " pre=(" << pre_x << "," << pre_y << ")"
                  << " post=(" << post_x << "," << post_y << ")" << std::endl;
        return;
    }

    constexpr double MAX_GRAPH_DELTA_XY = 0.5;
    if (!std::isfinite(delta_xy_norm) || delta_xy_norm > MAX_GRAPH_DELTA_XY) {
        std::cerr << "[GRAPH_CORRECTION_REJECTED]"
                  << " delta_xy_norm=" << delta_xy_norm
                  << " threshold=" << MAX_GRAPH_DELTA_XY
                  << " pre_opt=(" << pre_x << "," << pre_y << ")"
                  << " post_opt=(" << post_x << "," << post_y << ")"
                  << std::endl;
        return;
    }

    std::cout << "[GRAPH_CORRECTION_DEBUG]"
              << " pre_opt=(" << pre_x << "," << pre_y << ")"
              << " post_opt=(" << post_x << "," << post_y << ")"
              << " dx=" << dx_corr << " dy=" << dy_corr
              << " delta_xy_norm=" << delta_xy_norm
              << " status=accepted"
              << std::endl;

    StateUpdateTask task;
    task.priority  = UpdatePriority::CRITICAL;
    task.timestamp = node_timestamp;
    task.source    = "GRAPH";
    if (!std::isfinite(task.timestamp) || task.timestamp < 0.0) {
        std::cerr << "[ERROR] Refusing to push task with invalid timestamp: "
                  << task.timestamp << "\n";
        return;
    }

    task.execute = [graph_delta, post_opt_P,
                    idx_x, idx_y, idx_yaw, idx_vx, idx_vyaw, idx_ax]
                   (EKF<Config::DimWheelchairStateVector>& ekf) {
        Eigen::VectorXd delta = graph_delta;

        double raw_yaw_delta = delta(idx_yaw);
        std::cout << "[GRAPH_YAW_CORRECTION_BLOCKED]"
                  << " pre_yaw=" << ekf.x(idx_yaw)
                  << " raw_yaw_delta=" << raw_yaw_delta
                  << " applied_yaw_delta=0"
                  << std::endl;

        delta(idx_yaw)  = 0.0;
        delta(idx_vx)   = 0.0;
        delta(idx_vyaw) = 0.0;
        delta(idx_ax)   = 0.0;

        ekf.x += Config::alpha * delta;
        ekf.P = (1.0 - Config::alpha) * ekf.P + Config::alpha * post_opt_P;
        ekf.P = 0.5 * (ekf.P + ekf.P.transpose());

        std::cout << "[EKF_CORRECTION_APPLIED]"
                  << " delta=(" << delta(idx_x) << "," << delta(idx_y) << ")"
                  << " norm=" << std::hypot(delta(idx_x), delta(idx_y))
                  << std::endl;
    };

    std::cout << "[GRAPH_CORRECTION_DEBUG] pushing EKF correction task ts=" << task.timestamp << std::endl;
    StatePriorityQueue::getInstance().push(task);
}


void ThreadGrafFector::createNode() {
    auto& graf_fector = FactorGraphManager::getInstance();
    auto& state_queue = StatePriorityQueue::getInstance();
    Eigen::VectorXd current_state = state_queue.getStateVector();
    double current_time = state_queue.getTimeLastUpdate();

    NodeFactor new_node;
    new_node.filter.x = current_state;

    if (!m_last_kf_ekf_set) {
        std::cout << "[FACTOR] createNode: first keyframe ts=" << current_time << std::endl;
        graf_fector.addKeyframe(current_time, new_node);
        m_last_kf_ekf_state = current_state;
        m_last_kf_ekf_set   = true;
        return;
    }

    int idx_x   = static_cast<int>(Config::StateMembersRobot::StateX);
    int idx_y   = static_cast<int>(Config::StateMembersRobot::StateY);
    int idx_yaw = static_cast<int>(Config::StateMembersRobot::StateYaw);

    double dx = current_state[idx_x]   - m_last_kf_ekf_state[idx_x];
    double dy = current_state[idx_y]   - m_last_kf_ekf_state[idx_y];
    double distance_moved = std::sqrt(dx * dx + dy * dy);

    double dyaw = std::abs(current_state[idx_yaw] - m_last_kf_ekf_state[idx_yaw]);
    if (dyaw > M_PI) dyaw = 2 * M_PI - dyaw;

    if (distance_moved >= Config::DISTANCE_THRESHOLD || dyaw >= Config::ANGLE_THRESHOLD) {
        std::cout << "[FACTOR] createNode: new keyframe ts=" << current_time
                  << " dist=" << distance_moved << " dyaw=" << dyaw << std::endl;
        graf_fector.addKeyframe(current_time, new_node);
        m_last_kf_ekf_state = current_state;
    }
}

void ThreadGrafFector::removeNodeAndFector() {
    static int rm_count = 0;
    if (++rm_count % 50 == 0)
        std::cout << "[FACTOR] removeNodeAndFector called (iter=" << rm_count << ")" << std::endl;
    auto& graf_fector = FactorGraphManager::getInstance();
    graf_fector.removeOldestNode();
}
