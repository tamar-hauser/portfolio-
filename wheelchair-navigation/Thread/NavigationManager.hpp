#pragma once

#include <memory>
#include <mutex>
#include <atomic>
#include <vector>
#include <unordered_map>
#include <optional>
#include <chrono>

#include "MapData.hpp"
#include "MapLocal.hpp"
#include "OctomapAdapter.hpp"
#include "SensorFrame.hpp"
#include "TrackedObject/TrackedObject.hpp"
#include "DLite.hpp"
#include "AStar.hpp"
#include "GraphMap.hpp"
// #include "EdgeSubdivider.hpp" // להחזיר רק כשבאמת משתמשים בו
#include "Location.hpp"
struct GraphResult;
static constexpr float NODE_REACH_THRESHOLD = 3.0f;

class NavigationManager {
public:
    static NavigationManager& getInstance() {
        static NavigationManager instance;
        return instance;
    }

    std::shared_ptr<MapData> getMapData() const { 
        return m_mapData; 
    }

    // ---------- Global navigation ----------
    void createMapGlobal(Location myplace, Location mygoal);
    void tryAdvanceGlobalNode(const Eigen::VectorXd& state);
    std::size_t getGlobalPathIndex() const { return m_globalPathIndex; }
    // מחזיר את שתי נקודות הקצה (בקואורדינטות עולם, מ') של הקשת הנוכחית
    // (m_globalPath[index] -> m_globalPath[index+1]) וחצי-רוחב המסדרון שלה,
    // לפי אותה לוגיקה כמו MapLocal::buildRouteLimitLayer. false אם אין קשת נוכחית.
    bool getCurrentArcWorldCorridor(float& x0, float& y0, float& x1, float& y1, float& halfWidth) const;
    const Node& getNodeById(uint64_t id) const;
    std::vector<uint64_t> getPathGlobal() const;
    std::vector<uint64_t> getPateGlobal() const {
        return getPathGlobal();
    }

    // מחזיר מיקום כל צומת בנתיב הגלובלי בקואורדינטות מקומיות [מ'] (East, North)
    std::vector<std::pair<float,float>> getPathPositionsLocal() const;

    // ---------- Local map ----------
    void updateFromLidar(
        const std::shared_ptr<FramePointers>& frame,
        float robot_x,
        float robot_y
    );
    void updateFromObjects(
        const FramePointers f,
        std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObjects
    );


    void finalize();

    // ---------- D* Lite ----------
    void runDLite(
        const std::shared_ptr<FramePointers>& frame,
        const std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObjects,
        const Eigen::VectorXd& state
    );

    // ---------- Stop / Safety flags ----------
    void setDynamicBlocked(bool blocked);
    void setTrafficBlocked(bool blocked);
    void setCrossingBlocked(bool blocked);
    bool shouldStop() const;

    bool stoppedByDynamic() const {
        return m_dynamicBlocked.load();
    }

    bool stoppedByTraffic() const {
        return m_trafficBlocked.load();
    }

    bool stoppedByCrossing() const {
        return m_crossingBlocked.load();
    }

    // ---------- Local path ----------
    void updatePath(std::vector<std::pair<int,int>> newPath);

    std::optional<std::pair<int,int>> getNextStep();

    // מחזיר את התא הראשון בנתיב שנמצא במרחק >= lookahead_dist מהרובוט.
    // אם כל הנתיב קצר מהמרחק — מחזיר את התא האחרון.
    std::optional<std::pair<int,int>> getLookaheadStep(
        float lookahead_dist,
        const Eigen::VectorXd& state,
        const MapData& mapData);

    void popNextStep();

    std::vector<std::pair<int,int>> getPathLiveTime() const;

    // ---------- Current global node / edge ----------
    Node* getCurrentNode() const;
    Edge* getCurrentEdge() const;

    Location getLocation() const {
        return myGoal;
    }

private:
    NavigationManager()
        : m_mapData(std::make_shared<MapData>())
        , m_octomap(m_mapData)      // ← חדש
        , m_mapLocal(m_mapData)
    {}

    

    NavigationManager(const NavigationManager&) = delete;
    NavigationManager& operator=(const NavigationManager&) = delete;

    void updateCurrentEdge();
    bool chooseLocalGoal(
        float robot_x,
        float robot_y,
        int robot_col,
        int robot_row,
        int& goal_col,
        int& goal_row
    );

    bool chooseLookaheadGoal(
        float robot_x,
        float robot_y,
        int robot_col,
        int robot_row,
        int& goal_col,
        int& goal_row
    );

    bool goalChanged(int goal_col, int goal_row) const;

    bool findAlternativeLocalGoal(
        int goal_col,
        int goal_row,
        int& alt_col,
        int& alt_row
    );
    std::shared_ptr<MapData> m_mapData;
    OctomapAdapter           m_octomap;
    MapLocal                 m_mapLocal;
    DLite                    m_dlite;
    AStar                    m_astar;

    bool m_dliteInitialized = false;
    int  m_lastGoalCol = -1;
    int  m_lastGoalRow = -1;

    std::atomic_bool m_dynamicBlocked  {false};
    std::atomic_bool m_trafficBlocked  {false};
    std::atomic_bool m_crossingBlocked {false};

    std::chrono::steady_clock::time_point m_lastDynamicDangerTime =
        std::chrono::steady_clock::time_point::min();

    std::vector<std::pair<int,int>> m_currentPath;
    mutable std::mutex m_pathMutex;

    std::unordered_map<uint64_t, Node> myMap;
    std::vector<uint64_t> m_globalPath;
    std::size_t m_globalPathIndex = 0;

    Node* m_currentNode = nullptr;
    Edge* m_currentEdge = nullptr;

    Location myGoal;


};