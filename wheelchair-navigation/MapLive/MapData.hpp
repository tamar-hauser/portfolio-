#pragma once
#include <vector>          // ← היה חסר
#include <array>
#include <cstdint>
#include <cmath>
#include <shared_mutex>
#include <mutex>
#include <memory>
#include <octomap/OcTree.h>
#include "ConfigNavigation.hpp"
struct CellUpdate {
    int col;
    int row;
    uint8_t new_cost;
};
struct MapData {
    std::unique_ptr<octomap::OcTree> octree;

    // שכבת ליידר
    std::array<uint8_t, GRID_W * GRID_H> lidarLayer      {};
    std::array<float,   GRID_W * GRID_H> lidarConfidence {};

    // שכבת אובייקטים
    std::array<uint8_t, GRID_W * GRID_H> objectLayer      {};
    std::array<float,   GRID_W * GRID_H> objectConfidence {};
    std::array<uint8_t, GRID_W * GRID_H> ownerCount       {};

    std::array<uint8_t, GRID_W * GRID_H> inflationRadiusMap {};
    std::array<uint8_t, GRID_W * GRID_H> costMapFused {};
    std::array<uint8_t, GRID_W * GRID_H> routeLimitLayer {};

    std::vector<CellUpdate> cells;
    float originX = 0.f;
    float originY = 0.f;
    double origin_lat = 0.0;
    double origin_lon = 0.0;

    mutable std::shared_mutex rwMutex;
    mutable std::mutex octreeMutex;

    MapData() : octree(std::make_unique<octomap::OcTree>(RESOLUTION)) {
        octree->setOccupancyThres(0.5);
        octree->setProbHit(0.7);
        octree->setProbMiss(0.4);
        octree->setClampingThresMin(0.12);
        octree->setClampingThresMax(0.97);

        const uint8_t base_r =
            static_cast<uint8_t>(std::ceil(INFLATION_R / RESOLUTION));
        lidarLayer.fill(COST_UNKNOWN);
        lidarConfidence.fill(0.0f);
        objectLayer.fill(COST_UNKNOWN);
        objectConfidence.fill(0.0f);
        ownerCount.fill(0);
        inflationRadiusMap.fill(base_r);
        costMapFused.fill(COST_UNKNOWN);
        routeLimitLayer.fill(COST_LETHAL);   // ← ברירת מחדל: הכל חסום

    }

    MapData(const MapData&)            = delete;
    MapData& operator=(const MapData&) = delete;
   inline bool inBounds(int row, int col) const {
        return row >= 0 && row < GRID_H && col >= 0 && col < GRID_W;
    }

    inline int flatIdx(int row, int col) const {
        return row * GRID_W + col;
    }

    inline bool worldToGrid(float wx, float wy, int& col, int& row) const {
        col = static_cast<int>((wx - originX) / RESOLUTION);
        row = static_cast<int>((wy - originY) / RESOLUTION);
        return inBounds(row, col);
    }

    inline void gridToWorld(int col, int row, float& wx, float& wy) const {
        wx = originX + (col + 0.5f) * RESOLUTION;
        wy = originY + (row + 0.5f) * RESOLUTION;
    }

    // ── שאילתות ניווט ───────────────────────────────────────────────
    inline bool isCellNavigable(int col, int row) const {
        if (!inBounds(row, col)) return false;
        int idx = flatIdx(row, col);
        if (routeLimitLayer[idx] == COST_LETHAL) return false;
        return costMapFused[idx] < COST_INFLATED;
    }

    inline const std::array<uint8_t, GRID_W * GRID_H>& getFusedMap() const {
        return costMapFused;
    }

    inline std::vector<CellUpdate> getCells() {
        return cells;
    }
};