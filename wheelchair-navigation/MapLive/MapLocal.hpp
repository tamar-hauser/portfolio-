#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <Eigen/Dense>
#include "MapData.hpp"
#include "ConfigNavigation.hpp"
#include "GraphMap.hpp"

class OctomapAdapter;
struct TrackedObject;
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
struct TrackerFrame {
    std::vector<int> updated;
    std::vector<int> removed;
};

struct ObjectEntry {
    int  track_id    = -1;
    bool is_dynamic  = false;
    int  last_row    = 0;
    int  last_col    = 0;
    int  body_r      = 0;
    int  inflation_r = 0;
    std::vector<int> painted_indices;
};

class MapLocal {
public:
    MapLocal() = default;
    explicit MapLocal(std::shared_ptr<MapData> data);

    void setMapData(std::shared_ptr<MapData> data) {
        m_data = std::move(data);
    }
    bool worldToGrid(float wx, float wy, int& col, int& row) const {
        return m_data->worldToGrid(wx, wy, col, row);
    }
    bool inBounds(int row, int col) const {
        return m_data->inBounds(row, col);
    }
    bool isCellNavigable(int col, int row) const {
        return m_data->isCellNavigable(col, row);
    }
    void shiftOrigin(float new_robot_x, float new_robot_y);

    void buildFromOctomap();

    void updateObjects(
    const TrackerFrame& batch,
    std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObject
     );
    // void markCurbSegments(const Edge edge);

    void inflationPass();

    void buildRouteLimitLayer(
    const std::vector<uint64_t>&path_node_ids,
    const std::unordered_map<uint64_t, Node>& graph);

    void clearRouteLimitLayer();
    void fuseMaps();
    std::vector<CellUpdate> getCells(){return this->cells;}
    // Consume and clear the accumulated cell updates (thread-safe).
    std::vector<CellUpdate> consumeCellUpdates();
    // Backwards-compatible name used in NavigationManager: flush the pending cell updates.
    std::vector<CellUpdate> flushCells();
    uint8_t getCostForPlanner(float wx, float wy) const;
    uint8_t getCostForPlanner(int row, int col)   const;
    uint8_t getCost      (float wx, float wy) const;
    float   getConfidence(float wx, float wy) const;
    bool    isNavigable  (float wx, float wy) const { return getCost(wx, wy) < COST_INFLATED; }

    float getOriginX() const { return m_data->originX; }
    float getOriginY() const { return m_data->originY; }
   
    const std::array<uint8_t, GRID_W * GRID_H>& getFusedMap()      const { return m_data->costMapFused;   }
    const std::array<uint8_t, GRID_W * GRID_H>& rawLidarLayer()    const { return m_data->lidarLayer;     }
    const std::array<uint8_t, GRID_W * GRID_H>& rawObjectLayer()   const { return m_data->objectLayer;    }
    const std::array<uint8_t, GRID_W * GRID_H>& rawRouteLayer()  const { return m_data->routeLimitLayer; }
    void registerCellChange(int row,int col,uint8_t new_cost);
private:
    std::shared_ptr<MapData> m_data;
    std::unordered_map<int, ObjectEntry> m_objectRegistry;
    std::vector<CellUpdate> cells;

    inline int flatIdx(int row, int col) const { return row * GRID_W + col; }

    bool clipSegmentToGrid(
        Eigen::Vector2f& p1,
        Eigen::Vector2f& p2) const;

    void markEdgeCorridor(
        const Eigen::Vector2f& from_pos,
        const Eigen::Vector2f& to_pos,
        float corridor_half_width);
        
    void applyNodeCurbRestrictions(
    const std::vector<uint64_t>& path_node_ids,
    const std::unordered_map<uint64_t, Node>& graph);

    static float distancePointToSegment(
        const Eigen::Vector2f& p,
        const Eigen::Vector2f& a,
        const Eigen::Vector2f& b);

    static const Edge* findEdgeBetween(
        const std::unordered_map<uint64_t, Node>& graph,
        uint64_t from_id,
        uint64_t to_id);

    void paintCircleOnRouteLayer(
        const Eigen::Vector2f& center_pos,
        float                  radius_m,
        uint8_t                cost);

    void eraseObjectFootprint(ObjectEntry& entry);

    void paintObject(const TrackedObject& obj, ObjectEntry& entry);

    void markBodyOnObjectLayer(int rowC, int colC,
                               int body_r, int inflation_r,
                               bool isStaticConfirmed, float obj_confidence,
                               std::unordered_set<int>& painted_set);

    void applyDynamicForwardField(int rowC, int colC,
                                  const Eigen::Vector3f& vel,
                                  float speed, float obj_confidence,
                                  std::unordered_set<int>& painted_set);

    void inflateObjectCell(int rowC, int colC, int r,
                           std::unordered_set<int>& painted_set);

    static int securitySpace(const std::string& type_label);
};
