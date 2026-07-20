#include "OctomapAdapter.hpp"
#include "Thread/GpsPoseStore.hpp"
#include <cmath>
#include <iostream>

void OctomapAdapter::updateFromFrame(
    const std::shared_ptr<FramePointers>& frame)
{
    if (!frame || !frame->lidar) return;

    // EKF state: yaw (for rotation) and z (for height)
    Eigen::VectorXd ekf_state =
        StatePriorityQueue::getInstance().getStateVector();
    const float ekf_yaw = static_cast<float>(ekf_state(static_cast<int>(
                              Config::StateMembersRobot::StateYaw)));
    const float ekf_z   = static_cast<float>(ekf_state(static_cast<int>(
                              Config::StateMembersRobot::StateZ)));

    // World position: GPS x/y (same source as shiftOrigin + D*), EKF fallback.
    // This ensures the octomap sensor_origin matches the grid's coordinate frame.
    float robot_x = static_cast<float>(ekf_state(static_cast<int>(
                        Config::StateMembersRobot::StateX)));
    float robot_y = static_cast<float>(ekf_state(static_cast<int>(
                        Config::StateMembersRobot::StateY)));
    if (GpsPoseStore::getInstance().hasValid()) {
        auto gp = GpsPoseStore::getInstance().get();
        robot_x = static_cast<float>(gp.x);
        robot_y = static_cast<float>(gp.y);
    }

    // sensor_origin in world frame
    const octomap::point3d sensor_origin(robot_x, robot_y, ekf_z);

    // global_cloud is in robot-body frame after LidarProcess::applyTransform.
    // Convert each point to world/map frame before inserting into octomap,
    // so lidarLayer cells land at the correct world positions relative to the robot.
    //   world_x = robot_x + cos(yaw)*body_x - sin(yaw)*body_y
    //   world_y = robot_y + sin(yaw)*body_x + cos(yaw)*body_y
    //   world_z = ekf_z  + body_z
    const float cos_yaw = std::cos(ekf_yaw);
    const float sin_yaw = std::sin(ekf_yaw);

    octomap::Pointcloud octo_cloud;
    float first_bx = 0, first_by = 0, first_bz = 0;
    float first_wx = 0, first_wy = 0, first_wz = 0;
    bool  has_first = false;
    {
        std::lock_guard<std::mutex> lock(frame->frame_mutex);
        auto& src = frame->lidar->global_cloud;
        if (!src || src->empty()) return;
        for (const auto& pt : src->points) {
            const float wx = robot_x + cos_yaw * pt.x - sin_yaw * pt.y;
            const float wy = robot_y + sin_yaw * pt.x + cos_yaw * pt.y;
            const float wz = ekf_z   + pt.z;
            octo_cloud.push_back(wx, wy, wz);
            if (!has_first) {
                first_bx = pt.x; first_by = pt.y; first_bz = pt.z;
                first_wx = wx;   first_wy = wy;   first_wz = wz;
                has_first = true;
            }
        }
    }

    static int frame_dbg_count = 0;
    if (++frame_dbg_count % 50 == 0 && has_first) {
        std::cout << "[LIDAR_COSTMAP_FRAME_DEBUG]"
                  << " cloud_frame_before_octomap=world_converted"
                  << " robot_x=" << robot_x << " robot_y=" << robot_y
                  << " robot_yaw=" << ekf_yaw
                  << " first_body_point=(" << first_bx << "," << first_by << "," << first_bz << ")"
                  << " first_world_point=(" << first_wx << "," << first_wy << "," << first_wz << ")"
                  << " sensor_origin_world=(" << robot_x << "," << robot_y << "," << ekf_z << ")"
                  << " cloud_size=" << octo_cloud.size()
                  << std::endl;
    }

    // Insert world-frame cloud. sensor_origin is also world-frame → ray-casting is correct.
    // Clear before each scan so moved objects are removed immediately instead of
    // waiting for log-odds decay (which takes 4-8 scans with default probabilities).
    std::mutex& active_mutex = m_mapData ? m_mapData->octreeMutex : m_mutex;
    std::lock_guard<std::mutex> map_lock(active_mutex);
    activeTree()->clear();
    activeTree()->insertPointCloud(
        octo_cloud, sensor_origin, 15.0, true);
    activeTree()->updateInnerOccupancy();
}

octomap::OcTree* OctomapAdapter::getTree() {
    return activeTree();
}

bool OctomapAdapter::isOccupied(
    float x, float y, float z) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    auto* node = activeTree()->search(x, y, z);
    if (!node) return false;
    return activeTree()->isNodeOccupied(node);
}

bool OctomapAdapter::isOccupiedInBox(
    float min_x, float max_x,
    float min_y, float max_y,
    float min_z, float max_z) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto it = activeTree()->begin_leafs_bbx(
            octomap::point3d(min_x, min_y, min_z),
            octomap::point3d(max_x, max_y, max_z));
         it != activeTree()->end_leafs_bbx(); ++it)
    {
        if (activeTree()->isNodeOccupied(*it)) return true;
    }
    return false;
}