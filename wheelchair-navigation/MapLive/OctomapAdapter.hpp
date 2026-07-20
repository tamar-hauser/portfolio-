#pragma once
// ============================================================
//  OctomapAdapter.hpp
// ============================================================
#include "MapData.hpp"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include "SensorFrame.hpp"
#include "Thread/StatePriorityQueue.hpp"
#include "ConfigNavigation.hpp"
class OctomapAdapter {
public:
    explicit OctomapAdapter(std::shared_ptr<MapData> mapData = nullptr)
        : m_mapData(std::move(mapData))
    {
        if (!m_mapData) {
            m_ownTree = std::make_unique<octomap::OcTree>(RESOLUTION);
            configureTree(m_ownTree.get());
        } else {
            configureTree(m_mapData->octree.get());
        }
    }

    void updateFromFrame(const std::shared_ptr<FramePointers>& frame);
    bool isOccupied(float x, float y, float z) const;
    bool isOccupiedInBox(float min_x, float max_x,
                         float min_y, float max_y,
                         float min_z, float max_z) const;
    octomap::OcTree* getTree();

private:
    std::shared_ptr<MapData>         m_mapData;
    std::unique_ptr<octomap::OcTree> m_ownTree;
    mutable std::mutex               m_mutex;
    int                              m_updateCounter = 0;

    octomap::OcTree* activeTree() {
        return m_mapData ? m_mapData->octree.get() : m_ownTree.get();
    }
    const octomap::OcTree* activeTree() const {
        return m_mapData ? m_mapData->octree.get() : m_ownTree.get();
    }

    static void configureTree(octomap::OcTree* tree) {
        if (!tree) return;
        tree->setOccupancyThres(0.5);
        tree->setProbHit(0.7);
        tree->setProbMiss(0.4);
        tree->setClampingThresMin(0.12);
        tree->setClampingThresMax(0.97);
    }
};