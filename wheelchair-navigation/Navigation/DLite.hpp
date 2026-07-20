// DLite.hpp
#pragma once
#include <unordered_map>

#include <array>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <functional>
#include <utility>
#include "MapLocal.hpp"


static constexpr float INF = std::numeric_limits<float>::infinity();

class DLite {
public:
    struct Key {
        float k1, k2;
        bool operator>(const Key& o) const;
        bool operator<(const Key& o) const;
    };

    struct Cell {

        float g   = INF;
        float rhs = INF;
    };

    using PQItem = std::pair<Key, int>;
    DLite() : cells(GRID_W * GRID_H), cost_map_(GRID_W * GRID_H, 0) {}
    DLite(int w, int h, float res);
    float getStartG() const { return cells[start_].g; }
    void initialize(int start_col, int start_row,
                    int goal_col,  int goal_row,
                    const std::array<uint8_t, GRID_W * GRID_H>& fused_map);

    void updateAndReplan(
        const std::vector<CellUpdate>& updates,
        int robot_col, int robot_row);

    std::vector<std::pair<int,int>> getPath() const;
    void setGoal(int new_col, int new_row);
private:
    int   W_   = GRID_W;
    int   H_   = GRID_H;
    float RES_ = 1.0f;

    int   start_ = 0;
    int   goal_  = 0;
    float km_    = 0.0f;

    std::vector<uint8_t> cost_map_;
    std::vector<Cell> cells;
    std::unordered_map<int, Key> in_queue_;   // ← תיקון: שחזור השדה שהוסר בטעות
    std::priority_queue<
        PQItem,
        std::vector<PQItem>,
        std::greater<PQItem>
    > pq_;

    // std::unordered_map<int, Key> in_queue_;

    int                flat(int col, int row) const;
    std::pair<int,int> toColRow(int idx) const;
    float              h(int a, int b) const;
    float              moveCost(int from, int to) const;
    Key                calcKey(int s) const;
    void               computeShortestPath();
    void               updateCell(int u);
    std::vector<int>   neighbors(int idx) const;
};