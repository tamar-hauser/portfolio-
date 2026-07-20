#include "FactorGraphManager.hpp"
#include "Constants.hpp"
#include "Imu/ImuObject.hpp"        // חסר כדי לזהות ImuObject
#include "Encoder/EncoderObject.hpp" // חסר כדי לזהות EncoderObject (הנחת נתיב)
#include "Encoder/EncoderList.hpp" // חסר כדי לזהות EncoderObject (הנחת נתיב)
#include "TrackedObject/IcpEstimator.hpp"
#include "Gps/GpsObject.hpp"         // חסר כדי לזהות GpsObject (הנחת נתיב)
#include <pcl/io/pcd_io.h>
#include "TrackedObject\IcpEstimator.hpp"
#include "KalmanFilter\ekf.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <iostream>
#include <string>
#include <algorithm>

namespace {
bool sparseMatrixAllFinite(const Eigen::SparseMatrix<double>& M) {
    for (int k = 0; k < M.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(M, k); it; ++it) {
            if (!std::isfinite(it.value())) return false;
        }
    }
    return true;
}
}

FactorGraphManager::FactorGraphManager() : current_node_id(0), new_factor(false), dim(Config::DimWheelchairStateVector) {
    poses.reserve(MAX_WINDOW_SIZE);
}

double FactorGraphManager::normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

NodeFactor& FactorGraphManager::getLastNode() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    auto it = poses.find(current_node_id);
    if (it == poses.end()) {
        static NodeFactor s_empty;
        return s_empty;
    }
    return it->second;
}

long long FactorGraphManager::addKeyframe(double timestamp, const NodeFactor& initial_guess) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

    auto it = time_to_id.find(timestamp);
    if (it != time_to_id.end()) {
        poses[it->second] = initial_guess;
        return it->second;
    }

    long long prev_id = current_node_id;
    current_node_id++;
    poses[current_node_id] = initial_guess;
    time_to_id[timestamp] = current_node_id;

    if (prev_id > 0) {
        flushImuBuffer(prev_id, current_node_id);
        flushEncoderBuffer(prev_id, current_node_id);
        flushIcpBuffer(prev_id, current_node_id);
    }
    flushGpsBuffer(current_node_id, timestamp);
    m_imu_buffer.clear();
    m_enc_buffer.clear();

    std::cout << "[FACTOR] addKeyframe id=" << current_node_id
              << " ts=" << timestamp
              << " nodes=" << poses.size()
              << " factors=" << factors.size() << std::endl;

    if (poses.size() > MAX_WINDOW_SIZE) {
        removeOldestNode();
    }

    return current_node_id;
}

// void FactorGraphManager::addFactor(long long from_id, long long to_id, const Eigen::VectorXd& meas, double weight) {
//     factors.push_back({from_id, to_id, meas, weight, SENSOR_UNKNOWN}); // יש לעדכן סוג בהתאם למבנה שלך
// }

long long FactorGraphManager::getClosestNodeId(double target_time) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (time_to_id.empty()) return -1;

    auto it = time_to_id.lower_bound(target_time);

    if (it == time_to_id.end()) return std::prev(it)->second;
    if (it == time_to_id.begin()) return it->second;

    auto prev_it = std::prev(it);
    if ((target_time - prev_it->first) < (it->first - target_time)) {
        return prev_it->second;
    } else {
        return it->second;
    }
}



Eigen::VectorXd FactorGraphManager::computeError(const NodeFactor* from_node, const NodeFactor& to_node, const Factor& factor) {

    // משיכת המצב הנוכחי של הצומת שאליו אנו מתקדמים
    Eigen::VectorXd current_state = to_node.filter.getState();
    Eigen::VectorXd error = Eigen::VectorXd::Zero(factor.measurement.size());

    switch (factor.type) {

        // ==========================================
        // 1. חיישן אבסולוטי: GPS
        // ==========================================
        case SensorType::Gps: {
            // ה-GPS שלך מוגדר בגודל 5: X, Y, Z, Yaw, Vx
            Eigen::VectorXd expected = Eigen::VectorXd::Zero(factor.measurement.size());

            expected(0) = current_state(static_cast<int>(Config::StateMembersRobot::StateX));
            expected(1) = current_state(static_cast<int>(Config::StateMembersRobot::StateY));
            expected(2) = current_state(static_cast<int>(Config::StateMembersRobot::StateZ));
            expected(3) = current_state(static_cast<int>(Config::StateMembersRobot::StateYaw));
            expected(4) = current_state(static_cast<int>(Config::StateMembersRobot::StateVx));

            error = expected - factor.measurement;
            error(3) = normalizeAngle(error(3));
            break;
        }

        // ==========================================
        // 2. אודומטריה יחסית: ICP / IMU (Pre-integrated) / Encoder (Pre-integrated)
        // ==========================================
        case SensorType::Lidar:
        case SensorType::Encoder:
        case SensorType::Imu: {

            if (from_node == nullptr) {
                throw std::runtime_error("Relative odometry factor requires a from_node!");
            }

            Eigen::VectorXd past_state = from_node->filter.getState();

            // א. שליפת מיקומים וזווית של צומת המקור (i) וצומת המטרה (j)
            double x_i = past_state(static_cast<int>(Config::StateMembersRobot::StateX));
            double y_i = past_state(static_cast<int>(Config::StateMembersRobot::StateY));
            double yaw_i = past_state(static_cast<int>(Config::StateMembersRobot::StateYaw));

            double x_j = current_state(static_cast<int>(Config::StateMembersRobot::StateX));
            double y_j = current_state(static_cast<int>(Config::StateMembersRobot::StateY));
            double yaw_j = current_state(static_cast<int>(Config::StateMembersRobot::StateYaw));

            // ב. חישוב ההפרש הגלובלי במרחב
            double dx_global = x_j - x_i;
            double dy_global = y_j - y_i;

            // ג. סיבוב ההפרש למערכת הצירים המקומית של הרובוט בתחילת התנועה (לפי yaw_i)
            double expected_dx_local = dx_global * std::cos(yaw_i) + dy_global * std::sin(yaw_i);
            double expected_dy_local = -dx_global * std::sin(yaw_i) + dy_global * std::cos(yaw_i);
            double expected_dyaw = yaw_j - yaw_i;

            // ד. הרכבת וקטור התצפית הצפוי לפי סוג החיישן והגדלים מ-Constants.hpp
            Eigen::VectorXd expected = Eigen::VectorXd::Zero(factor.measurement.size());

            if (factor.type == SensorType::Lidar) {
                // לידאר מוגדר בגודל 5 (getMeasurementVector25D): x, y, z, pitch, yaw
                expected(0) = expected_dx_local;
                expected(1) = expected_dy_local;
                expected(2) = current_state(static_cast<int>(Config::StateMembersRobot::StateZ)) - past_state(static_cast<int>(Config::StateMembersRobot::StateZ));
                expected(3) = normalizeAngle(current_state(static_cast<int>(Config::StateMembersRobot::StatePitch)) - past_state(static_cast<int>(Config::StateMembersRobot::StatePitch)));
                expected(4) = normalizeAngle(expected_dyaw);

                error = expected - factor.measurement;
                error(3) = normalizeAngle(error(3));
                error(4) = normalizeAngle(error(4));
            }
            else if (factor.type == SensorType::Imu) {
                // לפי הפונקציה שלך, IMU מייצר וקטור בגודל 6
                // אינדקס 0: התקדמות קדימה (dx)
                // אינדקס 4: שינוי pitch
                // אינדקס 5: שינוי yaw
                expected(0) = expected_dx_local;

                // ההחלקה הצידה (dy_local) בכיסא גלגלים צריכה להיות אפס (Non-holonomic constraint).
                // נכניס אותה לאינדקס 1 למרות שהמדידה שם היא 0, כדי שהגרף "יעניש" החלקה צידית!
                expected(1) = expected_dy_local;

                expected(4) = normalizeAngle(current_state(static_cast<int>(Config::StateMembersRobot::StatePitch)) - past_state(static_cast<int>(Config::StateMembersRobot::StatePitch)));
                expected(5) = normalizeAngle(expected_dyaw);

                error = expected - factor.measurement;
                error(4) = normalizeAngle(error(4)); // נרמול שגיאת ה-Pitch
                error(5) = normalizeAngle(error(5)); // נרמול שגיאת ה-Yaw
            }
            else if (factor.type == SensorType::Encoder) {
                // מוגדר בגודל 2: התקדמות קדימה (dx) ושינוי זווית (dyaw)
                expected(0) = expected_dx_local;
                expected(1) = normalizeAngle(expected_dyaw);

                error = expected - factor.measurement;
                error(1) = normalizeAngle(error(1)); // נרמול שגיאת הזווית
            }

            break;
        }

        default:
            throw std::runtime_error("Unknown sensor type in computeError!");
    }

    return error;
}

std::size_t FactorGraphManager::getNodeCount()
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    return poses.size();
}

std::size_t FactorGraphManager::getFactorCount()
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    return factors.size();
}

bool FactorGraphManager::hasEnoughDataForOptimization()
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    return poses.size() >= 2 && !factors.empty();
}

bool FactorGraphManager::optimizeGraph() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

    // Cooldown: after a sanity-check rejection, skip optimization for 2 seconds
    // to avoid CPU spinning while a bad ICP factor remains in the graph.
    {
        auto now = std::chrono::steady_clock::now();
        if (m_last_optimize_rejection != std::chrono::steady_clock::time_point{}) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - m_last_optimize_rejection).count();
            if (elapsed_ms < 2000) {
                return false;
            }
        }
    }

    // Early safety checks: nothing to optimize yet
    const int N = Config::DimWheelchairStateVector;

    int num_nodes = static_cast<int>(poses.size());
    if (num_nodes == 0) {
        std::cerr << "[WARN] optimizeGraph: no nodes to optimize" << std::endl;
        return false;
    }

    std::cout << "[FACTOR] optimizeGraph start nodes=" << num_nodes
              << " factors=" << factors.size() << std::endl;

    int total_states = num_nodes * N;
    A.resize(total_states, total_states); // נדרש אתחול גדלים למטריצות ספארס
    b.resize(total_states);

    A.setZero();
    b.setZero();

    // Build mapping from node id -> contiguous offset (0..num_nodes-1)
    std::unordered_map<long long,int> id_to_offset;
    id_to_offset.reserve(poses.size());
    int idx = 0;
    for (const auto &p : poses) {
        id_to_offset[p.first] = idx++;
    }

    std::unordered_map<long long,double> id_to_time;
    for (const auto& kv : time_to_id) id_to_time[kv.second] = kv.first;

    double anchor_weight = 1000.0;
    if (!time_to_id.empty()) {
        long long oldest_id = time_to_id.begin()->second;
        auto it_anc = id_to_offset.find(oldest_id);
        if (it_anc != id_to_offset.end()) {
            int anchor_start = it_anc->second * N;
            for (int i = 0; i < N; ++i)
                A.coeffRef(anchor_start + i, anchor_start + i) += anchor_weight;
        }
    }

    // --- diagnostics: residual stats per factor type (item 2) ---
    struct TypeStats {
        int count = 0;
        double sum_residual = 0.0;
        double max_residual = 0.0;
        double max_weighted_residual = 0.0;
        long long worst_from = -1, worst_to = -1;
    };
    std::unordered_map<int, TypeStats> stats_by_type;

    struct WorstFactor {
        SensorType type;
        long long node_from, node_to;
        double residual_norm, weighted_residual_norm;
    };
    std::vector<WorstFactor> worst_list;

    for (const auto& f : factors) {
        // Ensure node_to exists in mapping
        auto it_to = id_to_offset.find(f.node_to);
        if (it_to == id_to_offset.end()) {
            std::cerr << "[FACTOR] optimizeGraph: skipping factor - node_to=" << f.node_to << " not found" << std::endl;
            continue;
        }

        int off_to = it_to->second;
        int idx_to = off_to * N;

        int idx_from = -1;
        const NodeFactor* from_node_ptr = nullptr;
        if (f.node_from != -1) {
            auto it_from = id_to_offset.find(f.node_from);
            if (it_from == id_to_offset.end()) {
                std::cerr << "[FACTOR] optimizeGraph: skipping factor - node_from=" << f.node_from << " not found" << std::endl;
                continue;
            }
            int off_from = it_from->second;
            idx_from = off_from * N;
            // safe to take pointer to poses element
            auto pit = poses.find(f.node_from);
            if (pit != poses.end()) from_node_ptr = &pit->second;
        }

        // H matrix dimension check
        if (f.H.cols() != N) {
            std::cerr << "[WARN] optimizeGraph: factor H cols=" << f.H.cols()
                      << " != state_dim=" << N
                      << " from=" << f.node_from << " to=" << f.node_to
                      << " - skipping" << std::endl;
            continue;
        }

        const NodeFactor& to_node = poses.at(f.node_to);

        Eigen::MatrixXd H_local = f.H;
        Eigen::VectorXd e_local = computeError(from_node_ptr, to_node, f);

        double weight = f.weight;

        // --- diagnostics: residual stats per factor type (item 2) ---
        {
            double residual_norm = e_local.norm();
            double weighted_residual_norm = std::sqrt(std::max(0.0, e_local.dot(e_local) * weight));
            int key = static_cast<int>(f.type);
            auto& st = stats_by_type[key];
            st.count++;
            st.sum_residual += residual_norm;
            if (residual_norm > st.max_residual) {
                st.max_residual = residual_norm;
                st.worst_from = f.node_from;
                st.worst_to   = f.node_to;
            }
            if (weighted_residual_norm > st.max_weighted_residual) {
                st.max_weighted_residual = weighted_residual_norm;
            }
            worst_list.push_back({f.type, f.node_from, f.node_to, residual_norm, weighted_residual_norm});
        }

        Eigen::MatrixXd HtWH = H_local.transpose() * weight * H_local;
        Eigen::VectorXd HtWe = H_local.transpose() * weight * e_local;

        // node_to: Jacobian = +H
        for(int i = 0; i < N; ++i) b(idx_to + i) -= HtWe(i);
        for(int i = 0; i < N; ++i)
            for(int j = 0; j < N; ++j)
                A.coeffRef(idx_to + i, idx_to + j) += HtWH(i, j);

        if (f.node_from != -1) {
            // node_from: Jacobian = -H, סימן הפוך ב-b, cross-terms שליליים
            for(int i = 0; i < N; ++i) b(idx_from + i) += HtWe(i);
            for(int i = 0; i < N; ++i)
                for(int j = 0; j < N; ++j) {
                    A.coeffRef(idx_from + i, idx_from + j) += HtWH(i, j);
                    A.coeffRef(idx_from + i, idx_to + j)   -= HtWH(i, j);
                    A.coeffRef(idx_to + i, idx_from + j)   -= HtWH(i, j);
                }
        }
    }

    // --- print residual summary per type (item 2) ---
    auto typeName = [](int t) -> std::string {
        switch (static_cast<SensorType>(t)) {
            case SensorType::Gps:     return "GPS";
            case SensorType::Imu:     return "IMU";
            case SensorType::Encoder: return "ENCODER";
            case SensorType::Lidar:   return "ICP";
            default: return "OTHER";
        }
    };
    for (const auto& kv : stats_by_type) {
        const auto& st = kv.second;
        double avg_residual = st.count > 0 ? st.sum_residual / st.count : 0.0;
        double worst_ts_from = id_to_time.count(st.worst_from) ? id_to_time[st.worst_from] : -1.0;
        double worst_ts_to   = id_to_time.count(st.worst_to)   ? id_to_time[st.worst_to]   : -1.0;
        std::cout << "[FACTOR_RESIDUAL_SUMMARY] type=" << typeName(kv.first)
                  << " count=" << st.count
                  << " max_residual_norm=" << st.max_residual
                  << " max_weighted_residual_norm=" << st.max_weighted_residual
                  << " avg_residual_norm=" << avg_residual
                  << " worst_node_from=" << st.worst_from
                  << " worst_node_to=" << st.worst_to
                  << " worst_ts_from=" << worst_ts_from
                  << " worst_ts_to=" << worst_ts_to
                  << std::endl;
    }
    std::sort(worst_list.begin(), worst_list.end(),
              [](const WorstFactor& a, const WorstFactor& b) {
                  return a.weighted_residual_norm > b.weighted_residual_norm;
              });
    for (std::size_t i = 0; i < worst_list.size() && i < 5; ++i) {
        const auto& w = worst_list[i];
        std::cout << "[FACTOR_WORST] type=" << typeName(static_cast<int>(w.type))
                  << " node_from=" << w.node_from << " node_to=" << w.node_to
                  << " residual_norm=" << w.residual_norm
                  << " weighted_residual_norm=" << w.weighted_residual_norm
                  << std::endl;
    }

    constexpr double kLambda = 1e-4;
    for (int i = 0; i < total_states; ++i)
        A.coeffRef(i, i) += kLambda;

    // --- item 3: matrix/b finiteness check before solving ---
    bool matrix_finite = sparseMatrixAllFinite(A);
    bool b_finite      = b.allFinite();

    if (!matrix_finite || !b_finite) {
        std::cerr << "[FACTOR_SOLVER_DEBUG] rows=" << total_states << " cols=" << total_states
                  << " factors_count=" << factors.size() << " nodes_count=" << num_nodes
                  << " matrix_finite=" << matrix_finite << " b_finite=" << b_finite
                  << " solver_status=not_attempted" << std::endl;
        std::cerr << "[FACTOR_OPTIMIZE_REJECTED_BEFORE_APPLY] reason=matrix_or_b_not_finite"
                  << " delta_norm=nan max_xy_delta=nan max_yaw_delta=nan node_id=-1" << std::endl;
        return false;
    }

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    bool solver_ok = (solver.info() == Eigen::Success);

    std::cout << "[FACTOR_SOLVER_DEBUG] rows=" << total_states << " cols=" << total_states
              << " factors_count=" << factors.size() << " nodes_count=" << num_nodes
              << " matrix_finite=" << matrix_finite << " b_finite=" << b_finite
              << " solver_status=" << (solver_ok ? "success" : "failed") << std::endl;

    if (!solver_ok) {
        std::cerr << "[WARN] optimizeGraph: LDLT solver failed (Eigen::Success not reached)" << std::endl;
        return false;
    }

    delta_x = solver.solve(b);

    // --- item 1: sanity check BEFORE any write to poses ---
    const int idx_x_c   = static_cast<int>(Config::StateMembersRobot::StateX);
    const int idx_y_c   = static_cast<int>(Config::StateMembersRobot::StateY);
    const int idx_yaw_c = static_cast<int>(Config::StateMembersRobot::StateYaw);

    constexpr double MAX_XY_DELTA_PER_NODE  = 1.0;
    constexpr double MAX_YAW_DELTA_PER_NODE = 0.3;

    bool delta_finite = delta_x.allFinite();
    double delta_norm = delta_finite ? delta_x.norm() : std::numeric_limits<double>::quiet_NaN();

    double max_xy_delta_seen  = 0.0;
    double max_yaw_delta_seen = 0.0;
    long long offending_node_id = -1;
    bool reject = !delta_finite;
    std::string reject_reason = delta_finite ? "" : "delta_x_not_finite";

    if (delta_finite) {
        for (const auto& kv : id_to_offset) {
            long long node_id  = kv.first;
            int       start_idx = kv.second * N;
            Eigen::VectorXd node_delta = delta_x.segment(start_idx, N);

            double xy_delta  = std::hypot(node_delta(idx_x_c), node_delta(idx_y_c));
            double yaw_delta = std::abs(node_delta(idx_yaw_c));

            if (xy_delta > max_xy_delta_seen)   max_xy_delta_seen  = xy_delta;
            if (yaw_delta > max_yaw_delta_seen) max_yaw_delta_seen = yaw_delta;

            if (!reject && xy_delta > MAX_XY_DELTA_PER_NODE) {
                reject = true;
                reject_reason = "xy_delta_too_large";
                offending_node_id = node_id;
            }
            if (!reject && yaw_delta > MAX_YAW_DELTA_PER_NODE) {
                reject = true;
                reject_reason = "yaw_delta_too_large";
                offending_node_id = node_id;
            }
        }
    }

    if (reject) {
        m_last_optimize_rejection = std::chrono::steady_clock::now();
        std::cerr << "[FACTOR_OPTIMIZE_REJECTED_BEFORE_APPLY]"
                  << " reason=" << reject_reason
                  << " delta_norm=" << delta_norm
                  << " max_xy_delta=" << max_xy_delta_seen
                  << " max_yaw_delta=" << max_yaw_delta_seen
                  << " node_id=" << offending_node_id
                  << std::endl;
        std::cout << "[FACTOR_OPTIMIZATION_COOLDOWN] optimization blocked for 2s after rejection"
                  << " reason=" << reject_reason << std::endl;
        return false;
    }

    // apply delta_x using id_to_offset mapping
    for (auto& p : poses) {
        long long id = p.first;
        auto it = id_to_offset.find(id);
        if (it == id_to_offset.end()) continue; // shouldn't happen
        int start_idx = it->second * N;
        p.second.filter.x += delta_x.segment(start_idx, N);

        int yaw_idx = static_cast<int>(Config::StateMembersRobot::StateYaw);
        p.second.filter.x(yaw_idx) = normalizeAngle(p.second.filter.x(yaw_idx));
    }

    std::cout << "[FACTOR] optimizeGraph done" << std::endl;
    return true;
}

void FactorGraphManager::removeOldestNode() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (poses.size() <= MAX_WINDOW_SIZE) return;
    if (time_to_id.empty()) return;

    auto oldest_it = time_to_id.begin();
    long long oldest_id = oldest_it->second;

    poses.erase(oldest_id);

    factors.erase(
        std::remove_if(factors.begin(), factors.end(),
            [oldest_id](const Factor& f) {
                return f.node_from == oldest_id || f.node_to == oldest_id;
            }),
        factors.end()
    );

    time_to_id.erase(oldest_it);
}


Eigen::VectorXd FactorGraphManager::getMeasurementVector25D(const Eigen::Matrix4f& T) {
    // אנחנו מניחים וקטור תצפית Z בגודל 5 (מיקומים וזוויות)
    Eigen::VectorXd Z(5);

    // 1. חילוץ התזוזה (Translation) - העמודה הרביעית
    Z(0) = T(0, 3); // תזוזה בציר X
    Z(1) = T(1, 3); // תזוזה בציר Y
    Z(2) = T(2, 3); // תזוזה בציר Z

    // 2. חילוץ זוויות מתוך מטריצת הסיבוב 3x3 (Rotation)
    // חילוץ Pitch (עלרוד) - סביב ציר Y
    Z(3) = std::atan2(-T(2, 0), std::sqrt(T(2, 1) * T(2, 1) + T(2, 2) * T(2, 2)));

    // חילוץ Yaw (סבסוב) - סביב ציר Z
    Z(4) = std::atan2(T(1, 0), T(0, 0));

    return Z;
}

void FactorGraphManager::createErrorIcp(const IcpEstimator::IcpResult& icp, double timenew, double timelast) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!icp.is_valid) {
        std::cerr << "[WARN] createErrorIcp: invalid ICP, skipping" << std::endl;
        return;
    }
    m_icp_accumulated  = icp.transformation * m_icp_accumulated;
    m_icp_fitness_sum += icp.fitness_score;
    m_icp_count++;
    std::cout << "[ICP] accumulated frame " << m_icp_count << std::endl;
}

void FactorGraphManager::flushIcpBuffer(long long from_id, long long to_id) {
    if (m_icp_count == 0) return;

    if (poses.find(from_id) == poses.end() || poses.find(to_id) == poses.end()) {
        std::cerr << "[WARN] flushIcpBuffer: missing node from=" << from_id
                  << " to=" << to_id << std::endl;
        m_icp_accumulated = Eigen::Matrix4f::Identity();
        m_icp_fitness_sum = 0.0f;
        m_icp_count       = 0;
        return;
    }

    float avg_fitness = m_icp_fitness_sum / static_cast<float>(m_icp_count);

    if (!std::isfinite(avg_fitness) || avg_fitness > 2.0f) {
        std::cerr << "[ICP][SKIP] bad avg_fitness=" << avg_fitness
                  << " frames=" << m_icp_count
                  << " from=" << from_id
                  << " to=" << to_id
                  << std::endl;

        m_icp_accumulated = Eigen::Matrix4f::Identity();
        m_icp_fitness_sum = 0.0f;
        m_icp_count       = 0;
        return;
    }

    // Gate 2: compare ICP-predicted position for to_node against its GPS-initialized position.
    // In a featureless environment, ICP fitness can look good but the transform is wrong.
    // GPS is authoritative — if ICP predicts a position far from GPS, discard the factor.
    {
        constexpr int IX   = static_cast<int>(Config::StateMembersRobot::StateX);
        constexpr int IY   = static_cast<int>(Config::StateMembersRobot::StateY);
        constexpr int IYAW = static_cast<int>(Config::StateMembersRobot::StateYaw);
        constexpr double ICP_GPS_GATE_METERS = 1.0;

        const auto& from_pose = poses.at(from_id);
        const auto& to_pose   = poses.at(to_id);

        double from_x   = from_pose.filter.x(IX);
        double from_y   = from_pose.filter.x(IY);
        double from_yaw = from_pose.filter.x(IYAW);

        double icp_dx_body = static_cast<double>(m_icp_accumulated(0, 3));
        double icp_dy_body = static_cast<double>(m_icp_accumulated(1, 3));

        double icp_dx_world = std::cos(from_yaw) * icp_dx_body - std::sin(from_yaw) * icp_dy_body;
        double icp_dy_world = std::sin(from_yaw) * icp_dx_body + std::cos(from_yaw) * icp_dy_body;

        double pred_x = from_x + icp_dx_world;
        double pred_y = from_y + icp_dy_world;

        double actual_x = to_pose.filter.x(IX);
        double actual_y = to_pose.filter.x(IY);

        double err = std::hypot(pred_x - actual_x, pred_y - actual_y);
        bool   accepted = (err <= ICP_GPS_GATE_METERS);

        std::cout << "[ICP_GPS_GATE_DEBUG]"
                  << " from_id=" << from_id << " to_id=" << to_id
                  << " from_x=" << from_x << " from_y=" << from_y << " from_yaw=" << from_yaw
                  << " to_actual=(" << actual_x << "," << actual_y << ")"
                  << " icp_body=(" << icp_dx_body << "," << icp_dy_body << ")"
                  << " icp_world=(" << icp_dx_world << "," << icp_dy_world << ")"
                  << " pred=(" << pred_x << "," << pred_y << ")"
                  << " err=" << err
                  << " threshold=" << ICP_GPS_GATE_METERS
                  << (accepted ? " -> ACCEPTED" : " -> REJECTED")
                  << std::endl;

        if (!accepted) {
            m_icp_accumulated = Eigen::Matrix4f::Identity();
            m_icp_fitness_sum = 0.0f;
            m_icp_count       = 0;
            return;
        }
    }

    Factor f;
    f.node_from  = from_id;
    f.node_to    = to_id;
    f.type       = SensorType::Lidar;
    f.measurement = getMeasurementVector25D(m_icp_accumulated);
    f.weight     = 1.0 / (avg_fitness + 1e-6);

    f.H = Eigen::MatrixXd::Zero(5, Config::DimWheelchairStateVector);
    f.H(0, static_cast<int>(Config::StateMembersRobot::StateX))     = 1.0;
    f.H(1, static_cast<int>(Config::StateMembersRobot::StateY))     = 1.0;
    f.H(2, static_cast<int>(Config::StateMembersRobot::StateZ))     = 1.0;
    f.H(3, static_cast<int>(Config::StateMembersRobot::StatePitch)) = 1.0;
    f.H(4, static_cast<int>(Config::StateMembersRobot::StateYaw))   = 1.0;

    factors.push_back(f);
    std::cout << "[FACTOR] flushIcpBuffer from=" << from_id << " to=" << to_id
              << " frames=" << m_icp_count << " avg_fitness=" << avg_fitness << std::endl;

    m_icp_accumulated = Eigen::Matrix4f::Identity();
    m_icp_fitness_sum = 0.0f;
    m_icp_count       = 0;
}

void FactorGraphManager::createErrorImu(const std::vector<ImuObject>& imu_list, double timenew, double timelast) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (imu_list.empty()) return;
    m_imu_buffer.insert(m_imu_buffer.end(), imu_list.begin(), imu_list.end());
    std::cout << "[IMU] buffered " << imu_list.size()
              << " samples, total=" << m_imu_buffer.size() << std::endl;
}

void FactorGraphManager::flushImuBuffer(long long from_id, long long to_id) {
    if (m_imu_buffer.empty()) return;
    if (poses.find(from_id) == poses.end() || poses.find(to_id) == poses.end()) {
        std::cerr << "[WARN] flushImuBuffer: missing node from=" << from_id
                  << " to=" << to_id << std::endl;
        return;
    }

    double total_time  = static_cast<double>(m_imu_buffer.size()) * Config::ImuDt;
    double delta_pitch = normalizeAngle(m_imu_buffer.back().Pitch - m_imu_buffer.front().Pitch);
    double delta_yaw   = normalizeAngle(m_imu_buffer.back().Yaw   - m_imu_buffer.front().Yaw);

    Factor imu_factor;
    imu_factor.node_from = from_id;
    imu_factor.node_to   = to_id;
    imu_factor.type      = SensorType::Imu;

    imu_factor.measurement = Eigen::VectorXd::Zero(6);
    imu_factor.measurement(4) = delta_pitch;
    imu_factor.measurement(5) = delta_yaw;

    imu_factor.weight = 1.0 / (0.01 * total_time + 1e-6);

    imu_factor.H = Eigen::MatrixXd::Zero(6, Config::DimWheelchairStateVector);
    imu_factor.H(4, static_cast<int>(Config::StateMembersRobot::StatePitch)) = 1.0;
    imu_factor.H(5, static_cast<int>(Config::StateMembersRobot::StateYaw))   = 1.0;

    factors.push_back(imu_factor);
    std::cout << "[FACTOR] flushImuBuffer from=" << from_id << " to=" << to_id
              << " samples=" << m_imu_buffer.size()
              << " dyaw=" << delta_yaw << " dpitch=" << delta_pitch << std::endl;
}

void FactorGraphManager::createErrorEncoder(const std::vector<EncoderObject>& encoder_list, double timenew, double timelast) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (encoder_list.empty()) return;
    m_enc_buffer.insert(m_enc_buffer.end(), encoder_list.begin(), encoder_list.end());
    std::cout << "[ENC] buffered " << encoder_list.size()
              << " samples, total=" << m_enc_buffer.size() << std::endl;
}

void FactorGraphManager::flushEncoderBuffer(long long from_id, long long to_id) {
    if (m_enc_buffer.empty()) return;
    if (poses.find(from_id) == poses.end() || poses.find(to_id) == poses.end()) {
        std::cerr << "[WARN] flushEncoderBuffer: missing node from=" << from_id
                  << " to=" << to_id << std::endl;
        return;
    }

    double delta_x     = 0.0;
    double current_yaw = 0.0;
    double total_dist  = 0.0;
    double total_time  = 0.0;

    for (const auto& enc : m_enc_buffer) {
        double dt        = Config::EncoderDt;
        double d_yaw     = enc.v_angular * dt;
        double mid_yaw   = current_yaw + d_yaw / 2.0;
        double step_dist = enc.v_linear * dt;
        delta_x         += step_dist * std::cos(mid_yaw);
        current_yaw     += d_yaw;
        total_dist      += std::abs(step_dist);
        total_time      += dt;
    }

    double delta_yaw = normalizeAngle(current_yaw);

    Factor enc_factor;
    enc_factor.node_from = from_id;
    enc_factor.node_to   = to_id;
    enc_factor.type      = SensorType::Encoder;

    enc_factor.measurement = Eigen::VectorXd::Zero(2);
    enc_factor.measurement(0) = delta_x;
    enc_factor.measurement(1) = delta_yaw;

    enc_factor.weight = 1.0 / (0.05 * total_dist + 0.001 * total_time + 1e-6);

    enc_factor.H = Eigen::MatrixXd::Zero(2, Config::DimWheelchairStateVector);
    {
        double yaw_i = poses[from_id].filter.x(static_cast<int>(Config::StateMembersRobot::StateYaw));
        enc_factor.H(0, static_cast<int>(Config::StateMembersRobot::StateX)) = std::cos(yaw_i);
        enc_factor.H(0, static_cast<int>(Config::StateMembersRobot::StateY)) = std::sin(yaw_i);
    }
    enc_factor.H(1, static_cast<int>(Config::StateMembersRobot::StateYaw)) = 1.0;

    factors.push_back(enc_factor);
    std::cout << "[FACTOR] flushEncoderBuffer from=" << from_id << " to=" << to_id
              << " samples=" << m_enc_buffer.size()
              << " dx=" << delta_x << " dyaw=" << delta_yaw << std::endl;
}

void FactorGraphManager::createErrorGps(const GpsObject& gps, double timenew) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

    if (gps.Z.size() >= 2) {
        double z0 = std::abs(gps.Z(0));
        double z1 = std::abs(gps.Z(1));
        if (z0 > 100000.0 || z1 > 100000.0) {
            std::cerr << "[WARN] createErrorGps: GPS Z values suspiciously large"
                      << " Z(0)=" << gps.Z(0) << " Z(1)=" << gps.Z(1) << std::endl;
            return;
        }
    }

    double effective_confidence = (gps.confidence > 0.01) ? gps.confidence
                                : (gps.isValid ? 0.5 : 0.0);
    if (effective_confidence < 0.2) {
        std::cerr << "[WARN] createErrorGps: low confidence=" << effective_confidence << ", skipping" << std::endl;
        return;
    }

    m_gps_buffer.push_back({gps, timenew});
    std::cout << "[GPS_BUFFER_DEBUG] GPS pushed ts=" << timenew
              << " confidence=" << gps.confidence
              << " buffer_size=" << m_gps_buffer.size() << std::endl;
}

void FactorGraphManager::flushGpsBuffer(long long to_id, double node_timestamp) {
    if (m_gps_buffer.empty()) return;

    if (poses.find(to_id) == poses.end()) {
        std::cerr << "[GPS_FACTOR_DEBUG] node=" << to_id << " not in poses, clearing GPS buffer" << std::endl;
        m_gps_buffer.clear();
        return;
    }

    // find the GPS entry with timestamp closest to this node's timestamp
    auto best_it = m_gps_buffer.end();
    double best_dt = std::numeric_limits<double>::max();
    for (auto it = m_gps_buffer.begin(); it != m_gps_buffer.end(); ++it) {
        double dt = std::abs(it->timestamp - node_timestamp);
        if (dt < best_dt) {
            best_dt = dt;
            best_it = it;
        }
    }

    if (best_it == m_gps_buffer.end() || best_dt > GPS_TIME_THRESHOLD) {
        std::cerr << "[GPS_FACTOR_DEBUG] no GPS match for node=" << to_id
                  << " node_ts=" << node_timestamp
                  << " closest_dt=" << (best_it != m_gps_buffer.end() ? best_dt : -1.0)
                  << " threshold=" << GPS_TIME_THRESHOLD
                  << " — skipping GPS factor for this node" << std::endl;
        m_gps_buffer.clear();
        return;
    }

    const GpsBufferEntry& entry = *best_it;
    const Eigen::VectorXd node_state = poses[to_id].filter.getState();

    std::cout << "[GPS_FACTOR_DEBUG] attaching GPS to node=" << to_id
              << " node_ts=" << node_timestamp
              << " gps_ts=" << entry.timestamp
              << " dt=" << best_dt
              << " gps=(" << entry.gps.Z(0) << "," << entry.gps.Z(1) << ")"
              << " node=(" << node_state(static_cast<int>(Config::StateMembersRobot::StateX))
              << "," << node_state(static_cast<int>(Config::StateMembersRobot::StateY)) << ")"
              << std::endl;

    double effective_confidence = (entry.gps.confidence > 0.01) ? entry.gps.confidence
                                : (entry.gps.isValid ? 0.5 : 0.0);

    Factor gps_factor;
    gps_factor.node_from   = -1;
    gps_factor.node_to     = to_id;
    gps_factor.type        = SensorType::Gps;
    gps_factor.measurement = entry.gps.Z;
    gps_factor.weight      = 50.0 * effective_confidence;
    gps_factor.H           = entry.gps.H;

    // אבחון חד-פעמי: מיפוי ה-H של ה-GPS factor קבוע (לא משתנה בין קריאות) —
    // מדפיסים פעם אחת בלבד לוודא שאין swap/sign flip באינדקסים.
    {
        static bool logged_gps_h_mapping = false;
        if (!logged_gps_h_mapping) {
            logged_gps_h_mapping = true;
            std::cout << "[GPS_FACTOR_H_MAPPING_DEBUG] H_nonzero=[";
            bool first = true;
            for (int r = 0; r < gps_factor.H.rows(); ++r) {
                for (int c = 0; c < gps_factor.H.cols(); ++c) {
                    if (gps_factor.H(r, c) != 0.0) {
                        if (!first) std::cout << ",";
                        std::cout << "(row" << r << "->idx" << c << "=" << gps_factor.H(r, c) << ")";
                        first = false;
                    }
                }
            }
            std::cout << "]" << std::endl;
        }
    }

    factors.push_back(gps_factor);
    std::cout << "[GPS_FACTOR_DEBUG] factor added to node=" << to_id
              << " confidence=" << entry.gps.confidence
              << " weight=" << gps_factor.weight << std::endl;

    m_gps_buffer.clear();
}

double FactorGraphManager::getLastNodeTimestamp() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (time_to_id.empty()) return -1.0;
    return time_to_id.rbegin()->first;
}
