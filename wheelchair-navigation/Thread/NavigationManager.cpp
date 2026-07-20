#include "NavigationManager.hpp"
#include "MapLive/OctomapAdapter.hpp"
#include "WheelchairBinding.hpp"
#include <iostream>
#include <string>
#include <limits>
#include <cmath>
#include <algorithm>

void NavigationManager::updateFromLidar(const std::shared_ptr<FramePointers>& frame,
                                        float robot_x, float robot_y)
{
    m_octomap.updateFromFrame(frame);
    m_mapLocal.shiftOrigin(robot_x, robot_y);
    m_mapLocal.buildFromOctomap();
    m_mapLocal.inflationPass();
}

void NavigationManager::updateFromObjects(
    FramePointers frameP,
    std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObjects)
{
    TrackerFrame frame;
    frame.updated = frameP.idObjectUpdate;
    frame.removed = frameP.idObjectRemove;
    m_mapLocal.updateObjects(frame, trackedObjects);
}
// void NavigationManager::updateCurbs(const std::vector<CurbSegment>& curbs)
// {
//     if (!curbs.empty())
//         m_mapLocal.markCurbSegments(curbs);
// }

void NavigationManager::finalize()
{
    m_mapLocal.fuseMaps();
}

// void NavigationManager::runAStar()
// {
//   m_globalPath.m_astar.run(myMap);
// }


void NavigationManager::createMapGlobal(Location myplace, Location mygoal)
{
    std::cout << "[NAV] createMapGlobal start: building global graph and running A*" << std::endl;
    myGoal = mygoal;

    GraphResult res = buildGraphFromPython(
        myplace.lat, myplace.lon,
        mygoal.lat,  mygoal.lon,
        "wheelchair_config.json"
    );

    myMap = std::move(res.graph);

    m_currentNode = nullptr;
    m_currentEdge = nullptr;

    auto it = myMap.find(res.source_node);
    if (it != myMap.end()) {
        m_currentNode = &it->second;
    } else {
        std::cerr << "[ERROR] createMapGlobal: source node not found in graph\n";
        return;
    }

    std::cout << "[NAV] createMapGlobal graph loaded: " << myMap.size() << " nodes"
              << " source=" << res.source_node
              << " target=" << res.target_node << std::endl;

    // ← תיקון: m_astar.run מקבל 3 פרמטרים — myMap, source, target (לא 1 כפי שהיה)
    m_globalPath = m_astar.run(myMap, res.source_node, res.target_node);

    if (m_globalPath.empty()) {
        std::cerr << "[ERROR] createMapGlobal: A* found no path\n";
        return;
    }

    m_globalPathIndex = 0;

    if (m_globalPath.size() > 1) {
        uint64_t next_id = m_globalPath[1];
        for (auto& e : m_currentNode->edges) {
            if (e.target == next_id) {
                m_currentEdge = &e;
                break;
            }
        }
    }

    std::cout << "[NAV] createMapGlobal done: global path size=" << m_globalPath.size() << " nodes" << std::endl;

    // הדפסת כל נקודות הנתיב הגלובלי — כדי לאמת שהן על המדרכה, ושהמסלול
    // מתקדם בהדרגה לכיוון היעד (dist_to_goal צריך לרדת, לא להישאר תקוע/לעלות).
    const double lat_rad = m_mapData->origin_lat * (M_PI / 180.0);
    const double goal_x = (myGoal.lon - m_mapData->origin_lon) * 111320.0 * std::cos(lat_rad);
    const double goal_y = (myGoal.lat - m_mapData->origin_lat) * 111320.0;

    std::cout << "[NAV][PATH] Global path GPS waypoints (goal=(" << goal_x << "," << goal_y << ")):" << std::endl;
    for (std::size_t i = 0; i < m_globalPath.size(); ++i) {
        auto it = myMap.find(m_globalPath[i]);
        if (it == myMap.end()) continue;
        const Node& n = it->second;
        float local_x = static_cast<float>((n.lon - m_mapData->origin_lon) * 111320.0 * std::cos(lat_rad));
        float local_y = static_cast<float>((n.lat - m_mapData->origin_lat) * 111320.0);
        double dist_to_goal = std::hypot(goal_x - local_x, goal_y - local_y);
        std::cout << "[NAV][PATH]  node[" << i << "] id=" << m_globalPath[i]
                  << " lat=" << n.lat << " lon=" << n.lon
                  << " local=(" << local_x << "," << local_y << ")m"
                  << " dist_to_goal=" << dist_to_goal << "m"
                  << std::endl;
    }
}


///לא הבנתי את המימוש אי אפשר לממש את זה בצורה אחרת
//הוא בודק אם הצומת הבאה קרובה למיקום הכיסא אם כן שולפים אותו מרשימת הצמתים ומעדכנים את המשתנים במחלקה
// void NavigationManager::tryAdvanceGlobalNode(const Eigen::VectorXd& state)
// {
//     if (m_currentNode == nullptr) return;
//     if (m_globalPath.empty())     return;

//     // חישוב מרחק לצומת הנוכחי
//     constexpr double METERS_PER_DEG_LAT = 111320.0;
//     const double lat_rad = m_currentNode->lat * (M_PI / 180.0);
//     const float node_x   = static_cast<float>(m_currentNode->lon * METERS_PER_DEG_LAT * std::cos(lat_rad));
//     const float node_y   = static_cast<float>(m_currentNode->lat * METERS_PER_DEG_LAT);

//     const float dx = static_cast<float>(
//         state(static_cast<int>(Config::StateMembersRobot::StateX))) - node_x;
//     const float dy = static_cast<float>(
//         state(static_cast<int>(Config::StateMembersRobot::StateY))) - node_y;
//     const float dist = std::sqrt(dx*dx + dy*dy);

//     // לא הגענו עדיין — אין מה לעשות
//     if (dist > NODE_REACH_THRESHOLD) return;

//     // מצא את הצומת הנוכחי במסלול הגלובלי והתקדם לבא
//     for (std::size_t i = 0; i + 1 < m_globalPath.size(); ++i)
//     {
//         auto it = myMap.find(m_globalPath[i]);
//         if (it == myMap.end()) continue;

//         // בדיקה לפי lat/lon (אין id בתוך Node — משווים לפי מפתח המפה)
//         if (&it->second == m_currentNode)
//         {
//             uint64_t next_id = m_globalPath[i + 1];
//             auto next_it = myMap.find(next_id);
//             if (next_it == myMap.end()) return;

//             // עדכון הצומת הנוכחי
//             m_currentNode = &next_it->second;

//             // עדכון הקשת הנוכחית (מהצומת הקודם לבא)
//             m_currentEdge = nullptr;
//             for (auto& e : it->second.edges) {
//                 if (e.target == next_id) {
//                     m_currentEdge = &e;
//                     break;
//                 }
//             }

//             std::cout << "[Nav] התקדמות לצומת " << next_id
//                       << " (" << i + 1 << "/" << m_globalPath.size() - 1 << ")\n";
//             return;
//         }
//     }
// }


const Node& NavigationManager::getNodeById(uint64_t id) const
{
    auto it = myMap.find(id);
    if (it == myMap.end())
        throw std::out_of_range("[NavigationManager::getNodeById] id לא נמצא במפה");
    return it->second;
}

// void NavigationManager::popNextStep()
// {
//     std::lock_guard<std::mutex> lock(m_path_mutex);
//     if (!m_currentPath.empty())
//         m_currentPath.erase(m_currentPath.begin());
// }




// void NavigationManager::runDLite(FramePointers f,const std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObject)///מקבלת מיקום בגריד של הרובוט בגריד
// {
//     m_mapLocal.shiftOrigin(m_mapData->originX,m_mapData->originY);
//     m_octomap.updateFromFrame(f,m_mapData->octree);
//     m_mapLocal.buildFromOctomap(m_octomap);
//     m_mapLocal.updateObjects({f.idObjectoUpdate,f.idObjectoremove},trackedObject);
//     m_mapLocal.inflationPass();
//     m_mapLocal.fuseMaps();
//     ///אם אני קרובה לצומת וזה בתוך הגריד אז הצומת היעד
//     //הצומת הבאה שמומרת לגריד
//     //אם לא תיתן לי צומת בתוך הגריד שזה אומר 5 מטר
//     float goal_col;
//     float goal_row;
//     if (!m_dliteInitialized) {
//         m_dlite.initialize(m_mapData->originX,m_mapData->originY,
//                            goal_col,  goal_row,
//                            m_mapData->costMapFused);
//         m_dliteInitialized = true;
//         m_lastGoalCol = goal_col;
//         m_lastGoalRow = goal_row;
//     } else {
//         m_dlite.setGoal()
//         m_dlite.updateAndReplan(m_mapLocal.getCells(),
//                                     m_mapData., robot_row);
//         }
//     }

//     m_currentPath = m_dlite.getPath();
//     bool noPath = m_currentPath.empty() ||
//                   m_dlite.getStartG() == DLite::INF; // צריך getter קטן ב-DLite
// ////לא הבנתי מה הסיפור של זה
//     if (noPath) {
//         int goal_flat = goal_row * GRID_W + goal_col;
//         m_mapData->costMapFused[goal_flat] = COST_LETHAL;

//         int   best_col = -1, best_row = -1;
//         float best_dist = std::numeric_limits<float>::max();

//         const int SEARCH_R = 5; // חיפוש בטווח 5 תאים
//         for (int dr = -SEARCH_R; dr <= SEARCH_R; ++dr) {
//             for (int dc = -SEARCH_R; dc <= SEARCH_R; ++dc) {
//                 int nc = goal_col + dc;
//                 int nr = goal_row + dr;
//                 if (nc < 0 || nc >= GRID_W || nr < 0 || nr >= GRID_H) continue;

//                 int idx = nr * GRID_W + nc;
//                 if (m_mapData->costMapFused[idx] >= COST_LETHAL) continue;
//                 if (m_mapData->costMapFused[idx] == COST_UNKNOWN) continue;

//                 float dist = std::sqrt(float(dr*dr + dc*dc));
//                 if (dist < best_dist) {
//                     best_dist = dist;
//                     best_col  = nc;
//                     best_row  = nr;
//                 }
//             }
//         }

//         // --- שלב ב: הפעל A* ---
//         if (best_col >= 0) {
//             // יש יעד חלופי — A* אליו
//             float alt_goal_x, alt_goal_y;
//             gridToWorld(best_col, best_row, alt_goal_x, alt_goal_y);

//             m_currentPath = m_astar.findPath(
//                 robot_col,  robot_row,
//                 best_col,   best_row,
//                 m_mapData->costMapFused
//             );
//         } else {
//             // אין שום תא נגיש בקרבת היעד
//             m_currentPath.clear();
//         }

//         // D* Lite יאותחל מחדש בקריאה הבאה (המפה השתנתה)
//         m_dliteInitialized = false;

//         // לוג / callback לשכבה עליונה
//         onNoPathFound(goal_x, goal_y); // ממשק שתממשי לפי הצורך
//         return;
//     }



////לוגיקה של הרמזור
void NavigationManager::setDynamicBlocked(bool blocked)
{
    m_dynamicBlocked.store(blocked);

    if (blocked) {
        m_lastDynamicDangerTime = std::chrono::steady_clock::now();
    }
}

void NavigationManager::setTrafficBlocked(bool blocked)
{
    m_trafficBlocked.store(blocked);
}

void NavigationManager::setCrossingBlocked(bool blocked)
{
    m_crossingBlocked.store(blocked);
}

bool NavigationManager::shouldStop() const
{
    constexpr auto DYNAMIC_HOLD_TIME = std::chrono::milliseconds(800);

    const auto now = std::chrono::steady_clock::now();

    bool dynamicHold = false;
    if (m_lastDynamicDangerTime != std::chrono::steady_clock::time_point::min()) {
        dynamicHold = (now - m_lastDynamicDangerTime) < DYNAMIC_HOLD_TIME;
    }

    return m_dynamicBlocked.load()
        || dynamicHold
        || m_trafficBlocked.load()
        || m_crossingBlocked.load();
}

std::vector<uint64_t> NavigationManager::getPathGlobal() const
{
    return m_globalPath;
}

std::vector<std::pair<float,float>> NavigationManager::getPathPositionsLocal() const
{
    std::vector<std::pair<float,float>> result;
    if (!m_mapData) return result;
    const double lat_rad = m_mapData->origin_lat * (M_PI / 180.0);
    for (uint64_t id : m_globalPath) {
        auto it = myMap.find(id);
        if (it == myMap.end()) continue;
        float lx = static_cast<float>((it->second.lon - m_mapData->origin_lon) * 111320.0 * std::cos(lat_rad));
        float ly = static_cast<float>((it->second.lat - m_mapData->origin_lat) * 111320.0);
        result.emplace_back(lx, ly);
    }
    return result;
}


// קריאה בלבד — לא נוגעת ב-m_globalPath/m_globalPathIndex/myMap, רק קוראת
// מהם. משמשת לוויזואליזציה (main.cpp), לא ע"י שום לוגיקת ניווט.
bool NavigationManager::getCurrentArcWorldCorridor(
    float& x0, float& y0, float& x1, float& y1, float& halfWidth) const
{
    if (!m_mapData) return false;
    if (m_globalPath.empty() || m_globalPathIndex + 1 >= m_globalPath.size()) return false;

    uint64_t curId  = m_globalPath[m_globalPathIndex];
    uint64_t nextId = m_globalPath[m_globalPathIndex + 1];

    auto itCur  = myMap.find(curId);
    auto itNext = myMap.find(nextId);
    if (itCur == myMap.end() || itNext == myMap.end()) return false;

    const double lat_rad = m_mapData->origin_lat * (M_PI / 180.0);
    x0 = static_cast<float>((itCur->second.lon  - m_mapData->origin_lon) * 111320.0 * std::cos(lat_rad));
    y0 = static_cast<float>((itCur->second.lat  - m_mapData->origin_lat) * 111320.0);
    x1 = static_cast<float>((itNext->second.lon - m_mapData->origin_lon) * 111320.0 * std::cos(lat_rad));
    y1 = static_cast<float>((itNext->second.lat - m_mapData->origin_lat) * 111320.0);

    float raw_width = 6.0f;
    for (const auto& e : itCur->second.edges) {
        if (e.target == nextId && e.width > 0.1) {
            raw_width = static_cast<float>(e.width);
            break;
        }
    }
    float clamped = raw_width;
    if (clamped < 5.0f) clamped = 5.0f;
    if (clamped > 10.0f) clamped = 10.0f;
    halfWidth = clamped * 0.5f;

    return true;
}

void NavigationManager::updatePath(std::vector<std::pair<int,int>> newPath)
{
    std::lock_guard<std::mutex> lock(m_pathMutex);
    m_currentPath = std::move(newPath);
}

std::optional<std::pair<int,int>> NavigationManager::getNextStep()
{
    std::lock_guard<std::mutex> lock(m_pathMutex);

    if (m_currentPath.empty()) {
        return std::nullopt;
    }

    return m_currentPath.front();
}

std::optional<std::pair<int,int>> NavigationManager::getLookaheadStep(
    float lookahead_dist,
    const Eigen::VectorXd& state,
    const MapData& mapData)
{
    std::lock_guard<std::mutex> lock(m_pathMutex);
    if (m_currentPath.empty()) return std::nullopt;

    const float rx = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateX)));
    const float ry = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateY)));

    std::pair<int,int> selected = m_currentPath.back();
    int selected_idx = static_cast<int>(m_currentPath.size()) - 1;

    for (int i = 0; i < static_cast<int>(m_currentPath.size()); ++i) {
        const auto& cell = m_currentPath[i];
        float cx = mapData.originX + (cell.first  + 0.5f) * RESOLUTION;
        float cy = mapData.originY + (cell.second + 0.5f) * RESOLUTION;
        float dist = std::sqrt((cx - rx)*(cx - rx) + (cy - ry)*(cy - ry));
        if (dist >= lookahead_dist) {
            selected = cell;
            selected_idx = i;
            break;
        }
    }

    float cx = mapData.originX + (selected.first  + 0.5f) * RESOLUTION;
    float cy = mapData.originY + (selected.second + 0.5f) * RESOLUTION;
    float final_dist = std::sqrt((cx - rx)*(cx - rx) + (cy - ry)*(cy - ry));

    std::cout << "[LOOKAHEAD_TARGET_DEBUG] path_size=" << m_currentPath.size()
              << " selected_index=" << selected_idx
              << " selected_target_x=" << cx
              << " selected_target_y=" << cy
              << " distance_from_robot=" << final_dist
              << std::endl;

    // אבחון בלבד: דאמפ מלא של ה-local path (קואורדינטות עולם) + אינדקס נבחר +
    // מרחק מהנקודה הנבחרת ל-final_goal, כדי לוודא שה-path מתקדם בכיוון היעד.
    {
        const double lat_rad = m_mapData->origin_lat * (M_PI / 180.0);
        const double final_goal_x = (myGoal.lon - m_mapData->origin_lon) * 111320.0 * std::cos(lat_rad);
        const double final_goal_y = (myGoal.lat - m_mapData->origin_lat) * 111320.0;
        const double dist_selected_to_goal = std::hypot(final_goal_x - cx, final_goal_y - cy);

        std::cout << "[LOCAL_PATH_DEBUG] selected_index=" << selected_idx
                  << " final_goal=(" << final_goal_x << "," << final_goal_y << ")"
                  << " dist_selected_to_goal=" << dist_selected_to_goal
                  << " path=[";
        for (int i = 0; i < static_cast<int>(m_currentPath.size()); ++i) {
            const auto& cell = m_currentPath[i];
            float px = mapData.originX + (cell.first  + 0.5f) * RESOLUTION;
            float py = mapData.originY + (cell.second + 0.5f) * RESOLUTION;
            std::cout << "(" << px << "," << py << ")";
            if (i + 1 < static_cast<int>(m_currentPath.size())) std::cout << ",";
        }
        std::cout << "]" << std::endl;
    }

    // אבחון בלבד (בעיה 3): תמונה מלאה — robot/goal/global path/local path/
    // lookahead/זוויות/dot — כדי לדעת אם ה-target באמת מוביל ליעד.
    {
        const double lat_rad = m_mapData->origin_lat * (M_PI / 180.0);
        const double final_goal_x = (myGoal.lon - m_mapData->origin_lon) * 111320.0 * std::cos(lat_rad);
        const double final_goal_y = (myGoal.lat - m_mapData->origin_lat) * 111320.0;

        auto nodeToLocal = [&](uint64_t id, double& nx, double& ny) -> bool {
            auto it = myMap.find(id);
            if (it == myMap.end()) return false;
            nx = (it->second.lon - m_mapData->origin_lon) * 111320.0 * std::cos(lat_rad);
            ny = (it->second.lat - m_mapData->origin_lat) * 111320.0;
            return true;
        };

        double first_x = 0, first_y = 0;
        bool has_first = !m_globalPath.empty() && nodeToLocal(m_globalPath[0], first_x, first_y);

        double current_x = 0, current_y = 0;
        bool has_current = (m_globalPathIndex < m_globalPath.size())
                            && nodeToLocal(m_globalPath[m_globalPathIndex], current_x, current_y);

        double next_x = 0, next_y = 0;
        bool has_next = (m_globalPathIndex + 1 < m_globalPath.size())
                         && nodeToLocal(m_globalPath[m_globalPathIndex + 1], next_x, next_y);

        const double angle_to_goal   = std::atan2(final_goal_y - ry, final_goal_x - rx);
        const double angle_to_target = std::atan2(cy - ry, cx - rx);

        const double gdx = final_goal_x - rx, gdy = final_goal_y - ry;
        const double tdx = cx - rx,           tdy = cy - ry;
        const double glen = std::hypot(gdx, gdy), tlen = std::hypot(tdx, tdy);
        const double dot_target_goal = (glen > 1e-6 && tlen > 1e-6)
                                        ? (tdx * gdx + tdy * gdy) / (tlen * glen) : 0.0;

        std::cout << "[NAV_PATH_DEBUG]"
                  << " robot=(" << rx << "," << ry << ")"
                  << " final_goal=(" << final_goal_x << "," << final_goal_y << ")"
                  << " current_global_index=" << m_globalPathIndex
                  << "/" << m_globalPath.size();

        std::cout << " global_first=";
        if (has_first) std::cout << "(" << first_x << "," << first_y << ")"; else std::cout << "(none)";

        std::cout << " global_current=";
        if (has_current) std::cout << "(" << current_x << "," << current_y << ")"; else std::cout << "(none)";

        std::cout << " global_next=";
        if (has_next) std::cout << "(" << next_x << "," << next_y << ")"; else std::cout << "(none)";

        std::cout << " selected_local_goal=(" << cx << "," << cy << ")"
                  << " lookahead_target=(" << cx << "," << cy << ")"
                  << " dstar_first5=[";
        for (int i = 0; i < std::min(5, static_cast<int>(m_currentPath.size())); ++i) {
            float px = mapData.originX + (m_currentPath[i].first  + 0.5f) * RESOLUTION;
            float py = mapData.originY + (m_currentPath[i].second + 0.5f) * RESOLUTION;
            std::cout << "(" << px << "," << py << ")";
            if (i < std::min(5, static_cast<int>(m_currentPath.size())) - 1) std::cout << ",";
        }
        std::cout << "]"
                  << " angle_to_goal=" << angle_to_goal
                  << " angle_to_target=" << angle_to_target
                  << " dot_target_goal=" << dot_target_goal
                  << std::endl;
    }

    return selected;
}

void NavigationManager::popNextStep()
{
    std::lock_guard<std::mutex> lock(m_pathMutex);

    if (!m_currentPath.empty()) {
        m_currentPath.erase(m_currentPath.begin());
    }
}

std::vector<std::pair<int,int>> NavigationManager::getPathLiveTime() const
{
    std::lock_guard<std::mutex> lock(m_pathMutex);
    return m_currentPath;
}

Node* NavigationManager::getCurrentNode() const
{
    return m_currentNode;
}

Edge* NavigationManager::getCurrentEdge() const
{
    return m_currentEdge;
}


void NavigationManager::tryAdvanceGlobalNode(const Eigen::VectorXd& state)
{
    if (m_globalPath.empty()) return;
    if (m_globalPathIndex + 1 >= m_globalPath.size()) return;

    // סף נפרד וקטן בהרבה מ-NODE_REACH_THRESHOLD (3 מ') — מטרתו לזהות שהרובוט
    // *ממש* הגיע לצומת הבאה (לא רק "מתקרב"), כדי לא להישאר נעול על מטרה
    // שמאחורי הרובוט בפועל אם יש מכשול ממש בצומת. 0.5 מ' נבחר במקום 0.3 מ'
    // כדי לא להיות רגיש לרעש GPS/רזולוציית grid (0.2 מ').
    constexpr float GLOBAL_NODE_ADVANCE_THRESHOLD_M = 0.5f;

    const std::size_t indexBefore = m_globalPathIndex;
    uint64_t nextNodeId = m_globalPath[m_globalPathIndex + 1];

    auto nextIt = myMap.find(nextNodeId);
    if (nextIt == myMap.end()) {
        std::cerr << "[WARN] tryAdvanceGlobalNode: nextNodeId=" << nextNodeId << " not found in map" << std::endl;
        return;
    }

    const Node& nextNode = nextIt->second;

    constexpr double METERS_PER_DEG_LAT = 111320.0;

const double lat_rad = m_mapData->origin_lat * (M_PI / 180.0);
const float node_x = static_cast<float>(
    (nextNode.lon - m_mapData->origin_lon) * METERS_PER_DEG_LAT * std::cos(lat_rad));
const float node_y = static_cast<float>(
    (nextNode.lat - m_mapData->origin_lat) * METERS_PER_DEG_LAT);

    const float robot_x = static_cast<float>(
        state(static_cast<int>(Config::StateMembersRobot::StateX)));

    const float robot_y = static_cast<float>(
        state(static_cast<int>(Config::StateMembersRobot::StateY)));

    const float dx = robot_x - node_x;
    const float dy = robot_y - node_y;
    const float dist = std::sqrt(dx * dx + dy * dy);

    const bool reached = dist <= GLOBAL_NODE_ADVANCE_THRESHOLD_M;

    if (reached) {
        // הגענו לצומת הבאה — מקדמים. m_globalPath[m_globalPathIndex+1] יצביע
        // אוטומטית מעכשיו על הצומת שאחרי זו שהגענו אליה (chooseLookaheadGoal
        // ממילא תמיד מכוון ל-m_globalPath[index+1], אז ה"lookahead אחרי
        // הצומת" קורה בלי שינוי נוסף).
        ++m_globalPathIndex;

        auto currentIt = myMap.find(m_globalPath[m_globalPathIndex]);
        if (currentIt != myMap.end()) {
            m_currentNode = &currentIt->second;
        }

        updateCurrentEdge();
    }

    std::cout << "[GLOBAL_INDEX_ADVANCE_DEBUG]"
              << " current_index_before=" << indexBefore
              << " current_index_after=" << m_globalPathIndex
              << " robot_x=" << robot_x << " robot_y=" << robot_y
              << " next_node_x=" << node_x << " next_node_y=" << node_y
              << " dist_to_next_node=" << dist
              << " threshold=" << GLOBAL_NODE_ADVANCE_THRESHOLD_M
              << " advanced=" << (reached ? 1 : 0)
              << " reason=" << (reached ? "next_node_reached" : "not_close_enough")
              << std::endl;

    if (reached) {
        std::cout << "[NAV] tryAdvanceGlobalNode: advanced to node "
                  << m_globalPath[m_globalPathIndex]
                  << " (" << m_globalPathIndex << "/"
                  << (m_globalPath.size() - 1) << ")"
                  << " robot=(" << robot_x << "," << robot_y << ")" << std::endl;
    }
}

void NavigationManager::updateCurrentEdge()
{
    m_currentEdge = nullptr;

    if (m_globalPath.empty()) return;
    if (m_globalPathIndex + 1 >= m_globalPath.size()) return;

    auto currentIt = myMap.find(m_globalPath[m_globalPathIndex]);
    if (currentIt == myMap.end()) return;

    uint64_t nextId = m_globalPath[m_globalPathIndex + 1];

    for (auto& edge : currentIt->second.edges) {
        if (edge.target == nextId) {
            m_currentEdge = &edge;
            return;
        }
    }
}

bool NavigationManager::goalChanged(int goal_col, int goal_row) const
{
    int dc = goal_col - m_lastGoalCol;
    int dr = goal_row - m_lastGoalRow;
    return (dc * dc + dr * dr) > (5 * 5);
}
bool NavigationManager::chooseLookaheadGoal(
    float robot_x, float robot_y,
    int   robot_col, int robot_row,
    int&  goal_col,  int& goal_row)
{
    // היעד הדינמי הוא הצומת הגלובלי הבא — לא final_goal. אם הוא בתוך הגריד
    // המקומי, משתמשים בו ישירות. אם הוא מחוץ לגריד, חותכים נקודה על אותו קו
    // ישר (רובוט -> צומת הבא) בתוך גבולות הגריד — כך שהכיוון (heading) נשאר
    // מדויק לכיוון הצומת הבא, רק המרחק קצר יותר.
    if (m_globalPath.empty() ||
        m_globalPathIndex + 1 >= m_globalPath.size()) {
        std::cerr << "[WARN] chooseLookaheadGoal: no next node"
                  << std::endl;
        return false;
    }

    constexpr double M_PER_DEG = 111320.0;
    const double lat_rad = m_mapData->origin_lat * (M_PI / 180.0);

    auto it = myMap.find(m_globalPath[m_globalPathIndex + 1]);
    if (it == myMap.end()) {
        std::cerr << "[WARN] chooseLookaheadGoal: next node id not found in map" << std::endl;
        return false;
    }

    const Node& nextNode = it->second;
    const double next_x = (nextNode.lon - m_mapData->origin_lon) * M_PER_DEG * std::cos(lat_rad);
    const double next_y = (nextNode.lat - m_mapData->origin_lat) * M_PER_DEG;

    const double dx   = next_x - robot_x;
    const double dy   = next_y - robot_y;
    const double dist = std::hypot(dx, dy);

    int col = 0, row = 0;

    // ניסיון ראשון: הצומת הבא עצמו, אם הוא בתוך הגריד וניתן למעבר
    if (dist > 1e-6 &&
        m_mapData->worldToGrid(static_cast<float>(next_x), static_cast<float>(next_y), col, row) &&
        m_mapLocal.getCostForPlanner(row, col) < COST_LETHAL) {
        goal_col = col;
        goal_row = row;
        return true;
    }

    if (dist <= 1e-6) {
        // הרובוט כבר נמצא בפועל על הצומת הבא
        goal_col = robot_col;
        goal_row = robot_row;
        return true;
    }

    // הצומת הבא מחוץ לגריד (או חסום) — חותכים נקודה על אותו קו ישר, בתוך
    // הגריד, ומחפשים מהקצה ופנימה תא שניתן למעבר.
    const double ux = dx / dist;
    const double uy = dy / dist;
    constexpr double MAX_REACH_M = 18.0; // בתוך חצי-רוחב הגריד המקומי (~20מ')
    const double reach = std::min(dist, MAX_REACH_M);

    for (double t = reach; t > 0.5; t -= RESOLUTION) {
        const float px = static_cast<float>(robot_x + ux * t);
        const float py = static_cast<float>(robot_y + uy * t);
        if (!m_mapData->worldToGrid(px, py, col, row)) continue;
        if (m_mapLocal.getCostForPlanner(row, col) >= COST_LETHAL) continue;
        goal_col = col;
        goal_row = row;
        return true;
    }

    std::cerr << "[WARN] chooseLookaheadGoal: "
              << "no navigable point toward next node" << std::endl;
    return false;
}

bool NavigationManager::findAlternativeLocalGoal(
    int goal_col,
    int goal_row,
    int& alt_col,
    int& alt_row
)
{
    constexpr int SEARCH_RADIUS = 8;

    float bestDist = std::numeric_limits<float>::max();
    bool found = false;

    for (int dr = -SEARCH_RADIUS; dr <= SEARCH_RADIUS; ++dr)
    {
        for (int dc = -SEARCH_RADIUS; dc <= SEARCH_RADIUS; ++dc)
        {
            int col = goal_col + dc;
            int row = goal_row + dr;

            if (!m_mapLocal.inBounds(row, col)) {
                continue;
            }

            if (m_mapLocal.getCostForPlanner(row, col) >= COST_LETHAL) {
                continue;
            }

            float dist = std::sqrt(static_cast<float>(dc * dc + dr * dr));

            if (dist < bestDist)
            {
                bestDist = dist;
                alt_col = col;
                alt_row = row;
                found = true;
            }
        }
    }

    if (found)
        std::cout << "[NAV] findAlternativeLocalGoal: found alt goal=(" << alt_col << "," << alt_row
                  << ") dist=" << bestDist << " from original=(" << goal_col << "," << goal_row << ")" << std::endl;
    else
        std::cerr << "[WARN] findAlternativeLocalGoal: no navigable cell found near goal=("
                  << goal_col << "," << goal_row << ")" << std::endl;

    return found;
}

bool NavigationManager::chooseLocalGoal(
    float robot_x, float robot_y,
    int   robot_col, int robot_row,
    int&  goal_col, int& goal_row)
{
    // היעד הדינמי = הצומת הגלובלי הבא (לא final_goal). final_goal משמש רק
    // לבדיקת סיום מסלול, לא ל-steering. ראה chooseLookaheadGoal לפרטים.
    const bool ok = chooseLookaheadGoal(robot_x, robot_y, robot_col, robot_row, goal_col, goal_row);

    // אבחון: לוודא שה-heading של chosen_local_goal תואם ל-next_global_node
    // ולא ל-final_goal.
    {
        constexpr double M_PER_DEG = 111320.0;
        const double lat_rad = m_mapData->origin_lat * (M_PI / 180.0);

        const double final_goal_x = (myGoal.lon - m_mapData->origin_lon) * M_PER_DEG * std::cos(lat_rad);
        const double final_goal_y = (myGoal.lat - m_mapData->origin_lat) * M_PER_DEG;

        bool has_next = false;
        double next_x = 0.0, next_y = 0.0;
        if (m_globalPathIndex + 1 < m_globalPath.size()) {
            auto it = myMap.find(m_globalPath[m_globalPathIndex + 1]);
            if (it != myMap.end()) {
                next_x = (it->second.lon - m_mapData->origin_lon) * M_PER_DEG * std::cos(lat_rad);
                next_y = (it->second.lat - m_mapData->origin_lat) * M_PER_DEG;
                has_next = true;
            }
        }

        const double heading_to_next_node  = has_next ? std::atan2(next_y - robot_y, next_x - robot_x) : 0.0;
        const double heading_to_final_goal = std::atan2(final_goal_y - robot_y, final_goal_x - robot_x);

        double heading_to_chosen_goal = 0.0;
        double chosen_x = 0.0, chosen_y = 0.0;
        if (ok) {
            chosen_x = m_mapData->originX + (goal_col + 0.5) * RESOLUTION;
            chosen_y = m_mapData->originY + (goal_row + 0.5) * RESOLUTION;
            heading_to_chosen_goal = std::atan2(chosen_y - robot_y, chosen_x - robot_x);
        }

        std::cout << "[LOCAL_GOAL_DEBUG]"
                  << " current_global_index=" << m_globalPathIndex << "/" << m_globalPath.size()
                  << " next_global_node=" << (has_next ? "(" + std::to_string(next_x) + "," + std::to_string(next_y) + ")" : "(none)")
                  << " final_goal=(" << final_goal_x << "," << final_goal_y << ")"
                  << " robot_position=(" << robot_x << "," << robot_y << ")"
                  << " chosen_local_goal=" << (ok ? "(" + std::to_string(chosen_x) + "," + std::to_string(chosen_y) + ")" : "(none)")
                  << " heading_to_next_node=" << heading_to_next_node
                  << " heading_to_chosen_goal=" << heading_to_chosen_goal
                  << " heading_to_final_goal=" << heading_to_final_goal
                  << std::endl;
    }

    if (ok) return true;

    std::cerr << "[WARN] chooseLocalGoal: no local goal found for robot=("
              << robot_x << "," << robot_y << ")" << std::endl;
    return false;
}

void NavigationManager::runDLite(
    const std::shared_ptr<FramePointers>& frame,
    const std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObjects,
    const Eigen::VectorXd& state
)
{
    static int dlite_count = 0;
    ++dlite_count;
    if (dlite_count % 50 == 0)
        std::cout << "[DSTAR] runDLite entered (iter=" << dlite_count << ")" << std::endl;

    if (!frame) {
        std::cerr << "[WARN] runDLite: frame is null, skipping navigation" << std::endl;
        updatePath({});
        return;
    }

    // אם יש חסימת בטיחות — לא מתכננים מסלול חדש
    if (shouldStop()) {
        if (dlite_count % 50 == 0)
            std::cerr << "[WARN] runDLite: navigation blocked (dynamic/traffic/crossing), skipping"
                      << " dynamic=" << m_dynamicBlocked.load()
                      << " traffic=" << m_trafficBlocked.load()
                      << " crossing=" << m_crossingBlocked.load() << std::endl;
        updatePath({});
        return;
    }

    // ----------------------------------------------------
    // 1. שליפת מיקום הרובוט מתוך state
    // ----------------------------------------------------
    const float robot_x = static_cast<float>(
        state(static_cast<int>(Config::StateMembersRobot::StateX))
    );

    const float robot_y = static_cast<float>(
        state(static_cast<int>(Config::StateMembersRobot::StateY))
    );

    if (dlite_count % 50 == 0)
        std::cout << "[DSTAR] runDLite robot=(" << robot_x << "," << robot_y << ")" << std::endl;

    // ----------------------------------------------------
    // 2. עדכון מפה מקומית סביב הרובוט
    // ----------------------------------------------------
    m_mapLocal.shiftOrigin(robot_x, robot_y);

    m_octomap.updateFromFrame(frame);

    m_mapLocal.buildFromOctomap();

    // ← תיקון: ObjectEntry → TrackerFrame; batch.updated/removed → frame fields
    TrackerFrame batch;
    batch.updated = frame->idObjectUpdate;
    batch.removed = frame->idObjectRemove;

    // MapLocal::updateObjects דורשת map לא const,
    // לכן יוצרים עותק מקומי כדי לא לשנות את המקור.
    auto trackedObjectsCopy = trackedObjects;

    m_mapLocal.updateObjects(batch, trackedObjectsCopy);

    m_mapLocal.inflationPass();

    // routeLimitLayer must be rebuilt each cycle — shiftOrigin fills new cells with COST_LETHAL
    // Without this call every cell stays LETHAL and isCellNavigable() always returns false
    if (!m_globalPath.empty()) {
        m_mapLocal.buildRouteLimitLayer(m_globalPath, myMap);
    }

    m_mapLocal.fuseMaps();

    // [LIDAR_COSTMAP_VALIDATE] — verify that lidarLayer obstacles land near the robot cell.
    // After the body→world transform fix in OctomapAdapter, LETHAL cells should appear
    // within sensor range of (GRID_W/2, GRID_H/2) whenever obstacles are present.
    {
        static int validate_count = 0;
        if (++validate_count % 25 == 0) {
            const int robot_col = GRID_W / 2;
            const int robot_row = GRID_H / 2;
            constexpr int SEARCH_R = 30;  // 6 m at RESOLUTION=0.2
            int   best_col  = -1, best_row = -1;
            float best_dist = std::numeric_limits<float>::infinity();
            for (int dr = -SEARCH_R; dr <= SEARCH_R; ++dr) {
                for (int dc = -SEARCH_R; dc <= SEARCH_R; ++dc) {
                    const int r = robot_row + dr, c = robot_col + dc;
                    if (r < 0 || r >= GRID_H || c < 0 || c >= GRID_W) continue;
                    if (m_mapData->lidarLayer[r * GRID_W + c] == COST_LETHAL) {
                        const float d = std::sqrt(float(dr*dr + dc*dc));
                        if (d < best_dist) {
                            best_dist = d;
                            best_col  = c;
                            best_row  = r;
                        }
                    }
                }
            }
            float   near_wx    = 0.0f, near_wy = 0.0f;
            uint8_t near_fused = 0;
            if (best_col >= 0) {
                near_wx    = m_mapData->originX + (best_col + 0.5f) * RESOLUTION;
                near_wy    = m_mapData->originY + (best_row + 0.5f) * RESOLUTION;
                near_fused = m_mapData->costMapFused[best_row * GRID_W + best_col];
            }
            std::cout << "[LIDAR_COSTMAP_VALIDATE]"
                      << " robot_cell=(" << robot_col << "," << robot_row << ")"
                      << " nearest_lidar_obstacle_cell=(" << best_col << "," << best_row << ")"
                      << " nearest_lidar_obstacle_world_x=" << near_wx
                      << " nearest_lidar_obstacle_world_y=" << near_wy
                      << " distance_robot_to_lidar_obstacle_cells=" << best_dist
                      << " distance_robot_to_lidar_obstacle_m=" << (best_dist * RESOLUTION)
                      << " costMapFused_cost_at_obstacle=" << static_cast<int>(near_fused)
                      << " lidarLayer_obstacle_near_robot=" << (best_col >= 0 && best_dist <= 15.0f ? 1 : 0)
                      << std::endl;
        }
    }

    // skip D*Lite path planning until at least one LiDAR frame has been processed —
    // early runDLite calls from main.cpp (every 200ms) happen before the first LiDAR
    // arrives at t≈0.5s; without LiDAR the entire costMapFused is COST_UNKNOWN=255
    // and no cell is navigable, so planning is pointless and the logs would be misleading.
    {
        static bool has_ever_had_lidar = false;
        if (!has_ever_had_lidar && frame && !frame->lidar->list.empty())
            has_ever_had_lidar = true;
        if (!has_ever_had_lidar) {
            if (dlite_count <= 3 || dlite_count % 50 == 0)
                std::cout << "[NAV] runDLite: waiting for first LiDAR frame (call=" << dlite_count << "), skipping path planning" << std::endl;
            return;
        }
    }

    // ----------------------------------------------------
    // 3. המרת מיקום הרובוט לגריד
    // ----------------------------------------------------
    int robot_col = 0;
    int robot_row = 0;

    if (!m_mapData->worldToGrid(robot_x, robot_y, robot_col, robot_row)) {
        std::cerr << "[WARN] runDLite: robot position not in grid x=" << robot_x
                  << " y=" << robot_y << ", resetting DLite" << std::endl;
        updatePath({});
        m_dliteInitialized = false;
        return;
    }
    if (dlite_count % 50 == 0)
        std::cout << "[NAV][LOCAL] robot cell=(" << robot_col << "," << robot_row
                  << ") pos=(" << robot_x << "," << robot_y << ")" << std::endl;
    {
        int _idx = robot_row * GRID_W + robot_col;
        std::cerr << "[DIAG] robot_cell=(" << robot_col << "," << robot_row << ")"
                  << " fused=" << (int)m_mapData->costMapFused[_idx]
                  << " lidar=" << (int)m_mapData->lidarLayer[_idx]
                  << " object=" << (int)m_mapData->objectLayer[_idx]
                  << " route=" << (int)m_mapData->routeLimitLayer[_idx] << std::endl;

        // item 3: nearest LETHAL cell to robot, bounded radial search (cheap, local only)
        constexpr int MAX_SEARCH_R = 100; // cells (=20m at RESOLUTION=0.2)
        float nearest_obstacle_distance_to_robot = std::numeric_limits<float>::infinity();
        for (int r = 0; r <= MAX_SEARCH_R; ++r) {
            bool found_this_ring = false;
            for (int dr = -r; dr <= r && !found_this_ring; ++dr) {
                for (int dc = -r; dc <= r; ++dc) {
                    if (std::max(std::abs(dr), std::abs(dc)) != r) continue; // ring only
                    int rr = robot_row + dr, cc = robot_col + dc;
                    if (rr < 0 || rr >= GRID_H || cc < 0 || cc >= GRID_W) continue;
                    if (m_mapData->costMapFused[rr * GRID_W + cc] == COST_LETHAL) {
                        float dist = std::hypot(static_cast<float>(dr), static_cast<float>(dc)) * RESOLUTION;
                        if (dist < nearest_obstacle_distance_to_robot)
                            nearest_obstacle_distance_to_robot = dist;
                        found_this_ring = true;
                    }
                }
            }
            if (found_this_ring) break;
        }
        std::cerr << "[OBSTACLE_MAP_DEBUG] nearest_obstacle_distance_to_robot="
                  << nearest_obstacle_distance_to_robot << std::endl;
    }

    // ניקוי footprint של הרובוט: תאים קרובים לרובוט לא יחסמו את D* Lite
    {
        constexpr int FOOTPRINT_R = 3;  // תאים (3 * 0.2m = 0.6m)
        int before_cost = m_mapData->costMapFused[robot_row * GRID_W + robot_col];
        for (int dr = -FOOTPRINT_R; dr <= FOOTPRINT_R; ++dr) {
            for (int dc = -FOOTPRINT_R; dc <= FOOTPRINT_R; ++dc) {
                if (dr*dr + dc*dc > FOOTPRINT_R*FOOTPRINT_R) continue;
                int c = robot_col + dc;
                int r = robot_row + dr;
                if (c >= 0 && c < GRID_W && r >= 0 && r < GRID_H)
                    m_mapData->costMapFused[r * GRID_W + c] = 0;
            }
        }
        int after_cost = m_mapData->costMapFused[robot_row * GRID_W + robot_col];
        std::cout << "[DSTAR_START_DEBUG] before_footprint_clear=" << before_cost
                  << " after=" << after_cost
                  << " start_cost=" << after_cost << std::endl;
    }

    // ----------------------------------------------------
    // 4. קידום במסלול הגלובלי אם הגענו לצומת הבאה
    // ----------------------------------------------------
    tryAdvanceGlobalNode(state);

    // ----------------------------------------------------
    // 5. בחירת יעד מקומי בתוך הגריד
    // ----------------------------------------------------
    int goal_col = 0;
    int goal_row = 0;

    if (!chooseLocalGoal(robot_x, robot_y, robot_col, robot_row, goal_col, goal_row)) {
        std::cerr << "[WARN] runDLite: no local goal available, clearing path" << std::endl;
        updatePath({});
        m_dliteInitialized = false;
        return;
    }

    // ----------------------------------------------------
    // 6. הרצת D* Lite
    // ----------------------------------------------------
    std::size_t changed_cells_count = 0;
    bool replan_triggered = false;

    if (!m_dliteInitialized)
    {
        // אתחול ראשון — טעינה מלאה של המפה וחישוב מאפס
        std::cout << "[DSTAR] D* Lite first init: robot=(" << robot_col << "," << robot_row
                  << ") goal=(" << goal_col << "," << goal_row << ")" << std::endl;
        m_dlite.initialize(
            robot_col,
            robot_row,
            goal_col,
            goal_row,
            m_mapLocal.getFusedMap()
        );
        m_dliteInitialized = true;
        m_lastGoalCol = goal_col;
        m_lastGoalRow = goal_row;
    }
    else if (goalChanged(goal_col, goal_row))
    {
        // היעד המקומי השתנה — עדכון incremental, שומר g-values קיימים
        if (dlite_count % 20 == 0)
            std::cout << "[DSTAR] D* Lite setGoal: (" << m_lastGoalCol << "," << m_lastGoalRow
                      << ") -> (" << goal_col << "," << goal_row << ")" << std::endl;
        auto cell_updates = m_mapLocal.flushCells();
        changed_cells_count = cell_updates.size();
        replan_triggered = true;
        m_dlite.updateAndReplan(cell_updates, robot_col, robot_row);
        m_dlite.setGoal(goal_col, goal_row);
        m_lastGoalCol = goal_col;
        m_lastGoalRow = goal_row;
    }
    else
    {
        if (dlite_count % 50 == 0)
            std::cout << "[DSTAR] D* Lite replanning: robot=(" << robot_col << "," << robot_row << ")" << std::endl;
        auto cell_updates = m_mapLocal.flushCells();
        changed_cells_count = cell_updates.size();
        replan_triggered = true;
        m_dlite.updateAndReplan(cell_updates, robot_col, robot_row);
    }

    std::vector<std::pair<int,int>> newPath = m_dlite.getPath();

    {
        int blocked_cells_on_current_path = 0;
        bool path_crosses_lethal_cell = false;
        for (const auto& [pc, pr] : newPath) {
            if (pc < 0 || pc >= GRID_W || pr < 0 || pr >= GRID_H) continue;
            if (m_mapData->costMapFused[pr * GRID_W + pc] == COST_LETHAL) {
                ++blocked_cells_on_current_path;
                path_crosses_lethal_cell = true;
            }
        }
        std::cout << "[DSTAR_OBSTACLE_DEBUG]"
                  << " changed_cells_count=" << changed_cells_count
                  << " blocked_cells_on_current_path=" << blocked_cells_on_current_path
                  << " replan_triggered=" << replan_triggered
                  << " path_crosses_lethal_cell=" << path_crosses_lethal_cell
                  << std::endl;
    }

    // ----------------------------------------------------
    // 7. אם אין מסלול — חיפוש יעד חלופי
    // ----------------------------------------------------
    const bool noPath = newPath.empty() || m_dlite.getStartG() >= INF;

    if (noPath)
    {
        std::cerr << "[WARN] runDLite: D* Lite returned no path, searching alternative goal" << std::endl;
        int alt_col = -1;
        int alt_row = -1;

        if (findAlternativeLocalGoal(goal_col, goal_row, alt_col, alt_row))
        {
            std::cout << "[DSTAR] D* Lite re-initializing with alt goal=(" << alt_col << "," << alt_row << ")" << std::endl;
            m_dlite.initialize(
                robot_col,
                robot_row,
                alt_col,
                alt_row,
                m_mapLocal.getFusedMap()
            );

            m_dliteInitialized = true;
            m_lastGoalCol = alt_col;
            m_lastGoalRow = alt_row;

            newPath = m_dlite.getPath();

            if (newPath.empty() || m_dlite.getStartG() >= INF) {  // ← תיקון: INF גלובלי
                std::cerr << "[WARN] runDLite: alt goal also has no path, clearing" << std::endl;
                updatePath({});
                return;
            }

            std::cout << "[DSTAR] runDLite: alt path found size=" << newPath.size() << std::endl;
            updatePath(std::move(newPath));
            return;
        }

        std::cerr << "[WARN] runDLite: no alternative goal found, clearing path" << std::endl;
        updatePath({});
        m_dliteInitialized = false;
        return;
    }

    // ----------------------------------------------------
    // 8. שמירת המסלול המקומי
    // ----------------------------------------------------
    if (dlite_count % 50 == 0)
        std::cout << "[DSTAR] runDLite: path found size=" << newPath.size()
                  << " robot=(" << robot_x << "," << robot_y << ")" << std::endl;
    updatePath(std::move(newPath));
}
