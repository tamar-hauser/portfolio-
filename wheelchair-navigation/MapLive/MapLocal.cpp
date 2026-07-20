#include "MapLocal.hpp"
#include "OctomapAdapter.hpp"
#include "TrackedObject/TrackedObject.hpp"
#include "GraphMap.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <memory>
#include <mutex>
#include <shared_mutex>
static constexpr float DEFAULT_CORRIDOR_WIDTH_M = 6.0f;
static constexpr float MIN_CORRIDOR_WIDTH_M     = 5.0f;
static constexpr float MAX_CORRIDOR_WIDTH_M     = 10.0f;
static constexpr float CURB_BLOCK_RADIUS_M      = 0.6f;    

// ===================================================================
//  בנאי
// ===================================================================
MapLocal::MapLocal(std::shared_ptr<MapData> data)
    : m_data(std::move(data))
{}

static Eigen::Vector2f latLonToLocal(
    double lat,
    double lon,
    double origin_lat,
    double origin_lon)
{
    static constexpr double METERS_PER_DEG_LAT = 111320.0;

    double origin_lat_rad = origin_lat * M_PI / 180.0;
    double meters_per_deg_lon =
        METERS_PER_DEG_LAT * std::cos(origin_lat_rad);

    float x = static_cast<float>((lon - origin_lon) * meters_per_deg_lon);
    float y = static_cast<float>((lat - origin_lat) * METERS_PER_DEG_LAT);

    return Eigen::Vector2f(x, y);
}

// ===================================================================
void MapLocal::shiftOrigin(float new_robot_x, float new_robot_y)
{
    std::unique_lock lock(m_data->rwMutex);

    float new_ox = new_robot_x - (GRID_W * RESOLUTION * 0.5f);
    float new_oy = new_robot_y - (GRID_H * RESOLUTION * 0.5f);

    int shift_col = static_cast<int>(std::round((new_ox - m_data->originX) / RESOLUTION));
    int shift_row = static_cast<int>(std::round((new_oy - m_data->originY) / RESOLUTION));

    if (shift_col == 0 && shift_row == 0) return;

    const uint8_t base_r =
        static_cast<uint8_t>(std::ceil(INFLATION_R / RESOLUTION));

    // ניקוי תאים שנכנסו לחלון (שתי השכבות)
    auto clearCell = [&](int idx) {
        m_data->lidarLayer[idx]         = COST_UNKNOWN;
        m_data->lidarConfidence[idx]    = 0.0f;
        m_data->objectLayer[idx]        = COST_UNKNOWN;
        m_data->objectConfidence[idx]   = 0.0f;
        m_data->ownerCount[idx]         = 0;
        m_data->inflationRadiusMap[idx] = base_r;
        m_data->routeLimitLayer[idx]    = COST_LETHAL;  // תאים חדשים — חסומים עד לבניית מסדרון
    };


    if (shift_col != 0) {
        int abs_sc  = std::min(std::abs(shift_col), GRID_W);
        int start_c = (shift_col > 0) ? (GRID_W - abs_sc) : 0;
        int end_c   = start_c + abs_sc;
        for (int r = 0; r < GRID_H; ++r)
            for (int c = start_c; c < end_c; ++c)
                clearCell(flatIdx(r, c));
    }

    if (shift_row != 0) {
        int abs_sr  = std::min(std::abs(shift_row), GRID_H);
        int start_r = (shift_row > 0) ? (GRID_H - abs_sr) : 0;
        int end_r   = start_r + abs_sr;
        for (int r = start_r; r < end_r; ++r)
            for (int c = 0; c < GRID_W; ++c)
                clearCell(flatIdx(r, c));
    }

    m_data->originX = new_ox;
    m_data->originY = new_oy;

    // תיקון #5: עדכן קואורדינטות גריד בכל entry
    for (auto& [id, entry] : m_objectRegistry) {
        entry.last_col -= shift_col;
        entry.last_row -= shift_row;
        // painted_indices: אינדקסים שטוחים = row*GRID_W+col
        // צריך להזיז אותם בדיוק כמו שהגריד זזה
        std::vector<int> shifted;
        shifted.reserve(entry.painted_indices.size());
        for (int idx : entry.painted_indices) {
            int r = idx / GRID_W - shift_row;
            int c = idx % GRID_W - shift_col;
            if (r >= 0 && r < GRID_H && c >= 0 && c < GRID_W)
                shifted.push_back(flatIdx(r, c));
            // תאים שיצאו מהחלון — נזרקים; אובייקט דינמי יצבע מחדש
        }
        entry.painted_indices = std::move(shifted);
    }
}

// ===================================================================
//  buildFromOctomap
//
//  תיקון #4: כותב אך ורק ל-lidarLayer + lidarConfidence.
//  objectLayer לא נגעת — אובייקטים לא נמחקים בין פריימי ליידר.
// ===================================================================
void MapLocal::buildFromOctomap()
{
    // Bug 1 fix: נועל octreeMutex לפני rwMutex (תמיד באותו סדר — מניעת deadlock)
    std::lock_guard<std::mutex> tree_lock(m_data->octreeMutex);
    std::unique_lock lock(m_data->rwMutex);

    octomap::OcTree* tree = m_data->octree.get();
    if (!tree) return;

    float cx = m_data->originX + (GRID_W * RESOLUTION * 0.5f);
    float cy = m_data->originY + (GRID_H * RESOLUTION * 0.5f);

    // Bug 6 fix: איפוס רק חלון 10m — שומר נתוני ליידר היסטוריים מחוץ לטווח
    {
        int col_min = std::max(0,        static_cast<int>((cx - 10.0f - m_data->originX) / RESOLUTION));
        int col_max = std::min(GRID_W-1, static_cast<int>((cx + 10.0f - m_data->originX) / RESOLUTION) + 1);
        int row_min = std::max(0,        static_cast<int>((cy - 10.0f - m_data->originY) / RESOLUTION));
        int row_max = std::min(GRID_H-1, static_cast<int>((cy + 10.0f - m_data->originY) / RESOLUTION) + 1);
        for (int row = row_min; row <= row_max; ++row)
            for (int col = col_min; col <= col_max; ++col) {
                int idx = flatIdx(row, col);
                m_data->lidarLayer[idx]      = COST_UNKNOWN;
                m_data->lidarConfidence[idx] = 0.0f;
            }
    }

    octomap::point3d min_pt(cx - 10.0f, cy - 10.0f, -0.3f);
    octomap::point3d max_pt(cx + 10.0f, cy + 10.0f,  2.0f);

    const uint8_t base_r =
        static_cast<uint8_t>(std::ceil(INFLATION_R / RESOLUTION));

    // שלב א: FREE
    for (auto it = tree->begin_leafs_bbx(min_pt, max_pt);
              it != tree->end_leafs_bbx(); ++it) {
        if (tree->isNodeOccupied(*it)) continue;
        int col, row;
        if (!worldToGrid(it.getX(), it.getY(), col, row)) continue;
        int idx = flatIdx(row, col);
        if (m_data->lidarLayer[idx] == COST_UNKNOWN)
            m_data->lidarLayer[idx] = COST_FREE;
    }

    // שלב ב: LETHAL דורס FREE
    for (auto it = tree->begin_leafs_bbx(min_pt, max_pt);
              it != tree->end_leafs_bbx(); ++it) {
        if (!tree->isNodeOccupied(*it)) continue;
        int col, row;
        if (!worldToGrid(it.getX(), it.getY(), col, row)) continue;
        int idx = flatIdx(row, col);
        m_data->lidarLayer[idx]         = COST_LETHAL;
        m_data->lidarConfidence[idx]    = static_cast<float>(it->getOccupancy());
        m_data->inflationRadiusMap[idx] = base_r;
    }
}
////מה צריך לבדוק 
///איך יוצרים כניסה צריך לעבור על כל האובייקטים ליצור להם מבנה כניסה ואז לנקד אותם במפה
// ===================================================================
//  updateObjects
//
//  תיקון #1: ניפוח מקומי רק כאן (inflationPass על lidarLayer בלבד)
//  תיקון #2: eraseObjectFootprint משתמש ב-ownerCount
//  תיקון #3: paintObject אוסף לתוך unordered_set → אין כפילויות
// ===================================================================
void MapLocal::updateObjects(const TrackerFrame& batch,std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObject)
{
    std::unique_lock lock(m_data->rwMutex);

    std::cout << "[MAP] updateObjects called: updated=" << batch.updated.size()
              << " removed=" << batch.removed.size() << std::endl;

    // ---- שלב 1: מחיקת אובייקטים שנעלמו ----
    for (int tid : batch.removed) {
        auto it = m_objectRegistry.find(tid);
        if (it == m_objectRegistry.end()) continue;
        eraseObjectFootprint(it->second);
        m_objectRegistry.erase(it);
        trackedObject[tid] = nullptr;
    }

    // ---- שלב 2+3: עדכון אובייקטים פעילים ----
    for (const int object : batch.updated) {
        auto obj = trackedObject[object];
        if (!obj) continue;
        const int tid = obj->id;

        const Eigen::Vector3f vel = obj->velocity();
        const float speed  = vel.norm();
        const bool is_dyn = (speed >= DYNAMIC_SPEED_THRESHOLD);

        auto reg_it = m_objectRegistry.find(tid);

        if (reg_it == m_objectRegistry.end()) {
            // אובייקט חדש
            ObjectEntry entry;
            entry.track_id   = tid;
            entry.is_dynamic = is_dyn;
            paintObject(*obj, entry);
            m_objectRegistry.emplace(tid, std::move(entry));
        }
        else {
            ObjectEntry& entry = reg_it->second;

            if (is_dyn) {
                // דינמי: נקה מיקום ישן ↔ צייר במיקום החדש
                eraseObjectFootprint(entry);
                entry.is_dynamic = true;
                paintObject(*obj, entry);
            }
            else {
                // סטטי: רק מגביר confidence — לא מזיז, לא מנפח שוב
                float boost = CONFIRM_BOOST * obj->confidence;
                for (int idx : entry.painted_indices) {
                    float& conf = m_data->objectConfidence[idx];
                    conf = std::min(1.0f, conf + boost);
                }
            }
        }
    }

    {
        int obj_cells = 0;
        for (int i = 0; i < GRID_W * GRID_H; ++i)
            if (m_data->objectLayer[i] != COST_UNKNOWN) ++obj_cells;
        std::cout << "[MAP] objectLayer cells=" << obj_cells << std::endl;
    }
}

// // ===================================================================
// //פונקציה צריכה שינוי
// // ===================================================================
// void MapLocal::markCurbSegments(const Edge& edge)
// {
//     std::unique_lock lock(m_data->rwMutex);

//     const uint8_t curb_r =
//         static_cast<uint8_t>(std::ceil(INFLATION_R / RESOLUTION));

//     for (const auto& curb : edge.) {
//         if (curb.height_m > 0.0f && curb.height_m <= CURB_HEIGHT_THRESHOLD)
//             continue;

//         Eigen::Vector2f seg = curb.end - curb.start;
//         float len = seg.norm();
//         if (len < 1e-4f) continue;

//         Eigen::Vector2f along = seg / len;
//         Eigen::Vector2f perp  = {-along.y(), along.x()};
//         float half_width = (curb.width_m > 0.0f) ? curb.width_m * 0.5f : RESOLUTION * 0.5f;

//         int   steps    = static_cast<int>(len / (RESOLUTION * 0.5f)) + 1;
//         float step_len = len / steps;

//         for (int s = 0; s <= steps; ++s) {
//             Eigen::Vector2f center = curb.start + along * (s * step_len);
//             int width_steps = static_cast<int>(half_width / RESOLUTION) + 1;

//             for (int w = -width_steps; w <= width_steps; ++w) {
//                 float offset = w * RESOLUTION;
//                 if (std::abs(offset) > half_width) continue;

//                 Eigen::Vector2f pt = center + perp * offset;
//                 int col, row;
//                 if (!worldToGrid(pt.x(), pt.y(), col, row)) continue;

//                 int idx = flatIdx(row, col);
//                 m_data->lidarLayer[idx]         = COST_LETHAL;
//                 m_data->lidarConfidence[idx]    = 1.0f;
//                 m_data->inflationRadiusMap[idx] = curb_r;
//             }
//         }
//     }
// }

// ===================================================================
//  inflationPass
//
//  תיקון #1: רץ רק על lidarLayer — אובייקטים מנוּפחים ב-paintObject.
//  אין חפיפה בין שני מנגנוני הניפוח.
// ===================================================================
void MapLocal::inflationPass()
{
    std::unique_lock lock(m_data->rwMutex);

    const auto snapshot_cost   = m_data->lidarLayer;
    const auto snapshot_radius = m_data->inflationRadiusMap;

    for (int row = 0; row < GRID_H; ++row) {
        for (int col = 0; col < GRID_W; ++col) {
            int src_idx = flatIdx(row, col);
            if (snapshot_cost[src_idx] != COST_LETHAL) continue;

            int   r        = snapshot_radius[src_idx];
            float radius_m = r * RESOLUTION;

            for (int dr = -r; dr <= r; ++dr) {
                for (int dc = -r; dc <= r; ++dc) {
                    int nr = row + dr, nc = col + dc;
                    if (!inBounds(nr, nc)) continue;

                    float dist_m = std::sqrt(float(dr*dr + dc*dc)) * RESOLUTION;
                    if (dist_m > radius_m) continue;

                    float   ratio    = 1.0f - (dist_m / radius_m);
                    uint8_t inf_cost = static_cast<uint8_t>(COST_INFLATED * ratio);
                    int     nidx     = flatIdx(nr, nc);

                    if (m_data->lidarLayer[nidx] == COST_LETHAL) continue;
                    if (m_data->lidarLayer[nidx] < inf_cost) {
                        m_data->lidarLayer[nidx]      = inf_cost;
                        float inf_conf = ratio * 0.8f;
                        if (m_data->lidarConfidence[nidx] < inf_conf)
                            m_data->lidarConfidence[nidx] = inf_conf;
                    }
                }
            }
        }
    }
}

// ===================================================================
//  fuseMaps — ממזג lidarLayer + objectLayer → costMapFused
//
//  כלל מיזוג: max של שתי השכבות.
//  LETHAL מכל שכבה → LETHAL בפלט.
//  UNKNOWN בשתיהן → UNKNOWN.
// ===================================================================
void MapLocal::fuseMaps()
{
    std::unique_lock lock(m_data->rwMutex);

    for (int idx = 0; idx < GRID_W * GRID_H; ++idx) {

        if (m_data->routeLimitLayer[idx] == COST_LETHAL) {
        if (m_data->costMapFused[idx] != COST_LETHAL) {
            m_data->costMapFused[idx] = COST_LETHAL;
            registerCellChange(idx / GRID_W, idx % GRID_W, COST_LETHAL);
        }
            continue;
        }
        uint8_t lc = m_data->lidarLayer[idx];
        uint8_t oc = m_data->objectLayer[idx];

        // ── מכשול (ליידר או אובייקט) ──────────────────────────────
        if (lc == COST_LETHAL) {
            if (m_data->costMapFused[idx] != COST_LETHAL) {
                m_data->costMapFused[idx] = COST_LETHAL;
                registerCellChange(idx / GRID_W, idx % GRID_W, COST_LETHAL);
            }
            continue;
        }

        // ── שתיהן UNKNOWN ──────────────────────────────────────────
        if (lc == COST_UNKNOWN && oc == COST_UNKNOWN) {
            // תא בקורידור המסלול הגלובלי שטרם מוּפה → פתוח לניווט
            uint8_t new_cost = (m_data->routeLimitLayer[idx] == COST_FREE)
                               ? COST_FREE : COST_UNKNOWN;
            if (m_data->costMapFused[idx] != new_cost) {
                m_data->costMapFused[idx] = new_cost;
                registerCellChange(idx / GRID_W, idx % GRID_W, new_cost);
            }
            continue;
        }

        // ── מיזוג רגיל עם confidence ───────────────────────────────
        uint8_t effective_l = (lc == COST_UNKNOWN) ? 0u : lc;
        uint8_t effective_o = (oc == COST_UNKNOWN) ? 0u : oc;
        uint8_t combined    = std::max(effective_l, effective_o);

        // Lidar inflation costs are NOT scaled by confidence — confidence scaling was
        // halving the effective cost (e.g. 96 × 0.4 = 38) making D*Lite route too
        // close to walls. Object costs (dynamic obstacles) retain confidence weighting.
        uint8_t cost;
        if (effective_l > 0 && oc == COST_UNKNOWN) {
            // pure lidar cell: use full lidar cost without confidence scaling
            cost = effective_l;
        } else if (oc == COST_LETHAL) {
            // Bug 4 fix: אובייקט LETHAL תמיד חוסם — ללא תלות ב-confidence
            cost = COST_LETHAL;
        } else {
            float conf   = std::max(m_data->lidarConfidence[idx],
                                    m_data->objectConfidence[idx]);
            float weight = std::max(0.01f, conf);
            cost = static_cast<uint8_t>(combined * weight);
        }

        if (m_data->costMapFused[idx] != cost) {
            m_data->costMapFused[idx] = cost;
            registerCellChange(idx / GRID_W, idx % GRID_W, cost);
        }
    }

    static int fuse_diag = 0;
    if (++fuse_diag % 100 == 0) {
        int free_c = 0, unknown_c = 0, lethal_c = 0, other_c = 0;
        int lidar_lethal_c = 0, object_lethal_c = 0;
        for (int i = 0; i < GRID_W * GRID_H; ++i) {
            uint8_t v = m_data->costMapFused[i];
            if (v == COST_FREE)         ++free_c;
            else if (v == COST_UNKNOWN) ++unknown_c;
            else if (v == COST_LETHAL)  ++lethal_c;
            else                        ++other_c;

            if (m_data->lidarLayer[i]  == COST_LETHAL) ++lidar_lethal_c;
            if (m_data->objectLayer[i] == COST_LETHAL) ++object_lethal_c;
        }
        std::cout << "[MAP] fuseMaps done: free=" << free_c
                  << " unknown=" << unknown_c
                  << " lethal=" << lethal_c
                  << " inflated=" << other_c << std::endl;
        std::cout << "[OBSTACLE_MAP_DEBUG]"
                  << " lidar_lethal_cells=" << lidar_lethal_c
                  << " object_lethal_cells=" << object_lethal_c
                  << " fused_lethal_cells=" << lethal_c
                  << " inflated_cells=" << other_c
                  << std::endl;
    }
}

// ===================================================================
//  getCost / getConfidence — קוראים מ-costMapFused
// ===================================================================

uint8_t MapLocal::getCostForPlanner(int row, int col) const
{
    // ללא נעילה — נקראת גם פנימית אחרי שהמתקשר כבר נועל
    if (!inBounds(row, col)) return COST_LETHAL;

    int idx = flatIdx(row, col);

    // בדיקת מסדרון — בינארי טהור, לא מעורבב עם confidence
    if (m_data->routeLimitLayer[idx] == COST_LETHAL)
        return COST_LETHAL;

    // תוך המסדרון — ערך המשוקלל מ-fuseMaps
    return m_data->costMapFused[idx];
}

// Bug 5 fix: מימוש הגרסה עם קואורדינטות עולם שהייתה מוצהרת בלי מימוש
uint8_t MapLocal::getCostForPlanner(float wx, float wy) const
{
    int col, row;
    if (!worldToGrid(wx, wy, col, row)) return COST_LETHAL;
    return getCostForPlanner(row, col);
}

float MapLocal::getConfidence(float wx, float wy) const
{
    std::shared_lock lock(m_data->rwMutex);
    int col, row;
    if (!worldToGrid(wx, wy, col, row)) return 0.0f;
    int idx = flatIdx(row, col);
    return std::max(m_data->lidarConfidence[idx],
                    m_data->objectConfidence[idx]);
}
uint8_t MapLocal::getCost(float wx, float wy) const
{
    std::shared_lock lock(m_data->rwMutex);
    int col, row;
    if (!worldToGrid(wx, wy, col, row)) return COST_UNKNOWN;
    return m_data->costMapFused[flatIdx(row, col)];
}

// float MapLocal::getConfidence(float wx, float wy) const
// {
//     std::shared_lock lock(m_data->rwMutex);
//     int col, row;
//     if (!worldToGrid(wx, wy, col, row)) return 0.0f;
//     int idx = flatIdx(row, col);
//     return std::max(m_data->lidarConfidence[idx],
//                     m_data->objectConfidence[idx]);
// }

// ===================================================================
//  eraseObjectFootprint
//
//  תיקון #2: ownerCount — מוחק תא רק כשאין עוד בעלים.
//  מונע מחיקה של תא שנצבע גם ע"י אובייקט אחר.
// ===================================================================
void MapLocal::registerCellChange(int row, int col, uint8_t new_cost)
{
    this->cells.push_back({ col, row, new_cost });
}

// Return the accumulated cell updates and clear the internal buffer.
// Caller receives ownership of the vector contents. Thread-safe (uses MapData mutex).
std::vector<CellUpdate> MapLocal::consumeCellUpdates()
{
    std::unique_lock lock(m_data->rwMutex);
    std::vector<CellUpdate> out = std::move(this->cells);
    this->cells.clear();
    return out;
}

std::vector<CellUpdate> MapLocal::flushCells()
{
    return consumeCellUpdates();
}





void MapLocal::eraseObjectFootprint(ObjectEntry& entry)
{
    for (int idx : entry.painted_indices) {
        if (m_data->ownerCount[idx] > 0)
            m_data->ownerCount[idx]--;

        if (m_data->ownerCount[idx] == 0) {
            m_data->objectLayer[idx] = COST_UNKNOWN;
            m_data->objectConfidence[idx] = 0.0f;
        }
        // אם עדיין יש בעלים → התא נשאר כפי שהוא
    }
    entry.painted_indices.clear();
}

// ===================================================================
//  paintObject
//
//  תיקון #1: inflateObjectCell במקום inflationPass לאובייקטים.
//  תיקון #3: כל הציור נאסף ל-unordered_set → convert ל-vector בסוף.
// ===================================================================
void MapLocal::paintObject(const TrackedObject& obj, ObjectEntry& entry)
{
    const Eigen::Vector3f pos   = obj.position();
    const Eigen::Vector3f vel   = obj.velocity();
    const float           speed = vel.norm();

    // objects too large to be treated as trackable bodies (buildings/walls/merged
    // clusters) skip the normal body+security-space object-tracking treatment, but
    // are still painted as a static obstacle into lidarLayer so they don't vanish
    // from the costmap entirely.
    constexpr float MAX_OBJ_DIM = 3.0f;  // [m] anything wider than 3m = wall/building
    if (obj.length > MAX_OBJ_DIM || obj.width > MAX_OBJ_DIM) {
        std::cout << "[MAP] paintObject: skipping oversized object "
                  << "length=" << obj.length << " width=" << obj.width << std::endl;

        int colC, rowC;
        if (!worldToGrid(pos.x(), pos.y(), colC, rowC)) return;

        // רצפת מינימום: clustering יכול לדווח מימד דק-להפליא (לדוגמה גזע עץ
        // שנתפס ב-LiDAR בזווית צרה -> width~0.03 מ') שלא משקף את הגודל האמיתי.
        // בלי רצפה, ה-LETHAL הנצבע יהיה קו דק כחוט וקל לעקוף בלי שום מרחק בטיחות.
        constexpr float MIN_OVERSIZED_HALF_M = 0.3f;  // [m]
        constexpr float OVERSIZED_INFLATION_MARGIN_M = 0.5f;  // [m] "משקל" נוסף מסביב

        const int min_half_cells = static_cast<int>(std::ceil(MIN_OVERSIZED_HALF_M / RESOLUTION));
        int half_cells_x = std::max({1, min_half_cells,
            static_cast<int>(std::ceil((obj.length * 0.5f) / RESOLUTION))});
        int half_cells_y = std::max({1, min_half_cells,
            static_cast<int>(std::ceil((obj.width  * 0.5f) / RESOLUTION))});

        int cells_painted = 0;
        for (int dr = -half_cells_y; dr <= half_cells_y; ++dr) {
            for (int dc = -half_cells_x; dc <= half_cells_x; ++dc) {
                int r = rowC + dr;
                int c = colC + dc;
                if (!inBounds(r, c)) continue;
                int idx = flatIdx(r, c);
                m_data->lidarLayer[idx]      = COST_LETHAL;
                m_data->lidarConfidence[idx] = std::max(m_data->lidarConfidence[idx], static_cast<float>(obj.confidence));
                ++cells_painted;
            }
        }

        // טבעת "משקל" נוספת מסביב לליבה ה-LETHAL — לא חוסמת לחלוטין, אבל
        // נותנת ל-D*Lite עלות גבוהה כך שהוא יעדיף לתכנן מסלול עם מרחק בטיחות,
        // לא לגעת בקצה הליבה.
        const int margin_cells = static_cast<int>(std::ceil(OVERSIZED_INFLATION_MARGIN_M / RESOLUTION));
        const int inflate_half_x = half_cells_x + margin_cells;
        const int inflate_half_y = half_cells_y + margin_cells;
        int cells_inflated = 0;
        for (int dr = -inflate_half_y; dr <= inflate_half_y; ++dr) {
            for (int dc = -inflate_half_x; dc <= inflate_half_x; ++dc) {
                if (std::abs(dr) <= half_cells_y && std::abs(dc) <= half_cells_x) continue; // כבר LETHAL
                int r = rowC + dr;
                int c = colC + dc;
                if (!inBounds(r, c)) continue;
                int idx = flatIdx(r, c);
                if (m_data->lidarLayer[idx] != COST_LETHAL) {
                    m_data->lidarLayer[idx] = std::max(m_data->lidarLayer[idx], COST_INFLATED);
                    ++cells_inflated;
                }
            }
        }

        std::cout << "[OVERSIZED_OBJECT_HANDLED_AS_STATIC_OBSTACLE]"
                  << " length=" << obj.length
                  << " width=" << obj.width
                  << " cells_painted=" << cells_painted
                  << " cells_inflated=" << cells_inflated
                  << std::endl;
        return;
    }

    int colC, rowC;
    if (!worldToGrid(pos.x(), pos.y(), colC, rowC)) return;

    float real_radius = std::max(obj.length, obj.width) * 0.5f;
    int   body_r      = std::max(OBJ_BODY_RADIUS_MIN,
                                 static_cast<int>(std::ceil(real_radius / RESOLUTION)));
    int   sec_r       = securitySpace(obj.type_label);

    entry.last_row    = rowC;
    entry.last_col    = colC;
    entry.body_r      = body_r;
    entry.inflation_r = sec_r;
    entry.is_dynamic  = (speed >= DYNAMIC_SPEED_THRESHOLD);

    // תיקון #3: set מונע כפילויות
    std::unordered_set<int> painted_set;

    bool isStatic = (speed < DYNAMIC_SPEED_THRESHOLD);

    // 1. גוף LETHAL
    markBodyOnObjectLayer(rowC, colC, body_r, sec_r,
                          isStatic, obj.confidence, painted_set);

    // 2. שדה קדמי (דינמיים בלבד)
    if (!isStatic)
    applyDynamicForwardField(rowC, colC, vel, speed,
                                 obj.confidence, painted_set);

    // 3. ניפוח מקומי — תיקון #1: כאן ורק כאן, לא ב-inflationPass
    inflateObjectCell(rowC, colC, sec_r, painted_set);

    // Convert set → vector + עדכן ownerCount
    entry.painted_indices.assign(painted_set.begin(), painted_set.end());
    for (int idx : entry.painted_indices)
        m_data->ownerCount[idx]++;
}

// ===================================================================
//  markBodyOnObjectLayer — כותב גוף LETHAL לשכבת אובייקטים
// ===================================================================
void MapLocal::markBodyOnObjectLayer(int rowC, int colC,
                                     int body_r, int inflation_r,
                                     bool isStaticConfirmed, float obj_confidence,
                                     std::unordered_set<int>& painted_set)
{
    for (int dr = -body_r; dr <= body_r; ++dr) {
        for (int dc = -body_r; dc <= body_r; ++dc) {
            int nr = rowC + dr, nc = colC + dc;
            if (!inBounds(nr, nc)) continue;
            if (std::sqrt(float(dr*dr + dc*dc)) > body_r) continue;

            int idx = flatIdx(nr, nc);
            m_data->objectLayer[idx] = COST_LETHAL;
            m_data->inflationRadiusMap[idx] = static_cast<uint8_t>(inflation_r);
            if (isStaticConfirmed) {
                float existing = m_data->objectConfidence[idx];
                m_data->objectConfidence[idx] =
                    (existing > 0.1f)
                    ? std::min(1.0f, existing + CONFIRM_BOOST * obj_confidence)
                    : obj_confidence;
            } else {
                m_data->objectConfidence[idx] =
                    std::max(m_data->objectConfidence[idx], obj_confidence);
            }

            painted_set.insert(idx);
        }
    }
}

// ===================================================================
//  applyDynamicForwardField — שדה כוח קדמי לאובייקטים דינמיים
// ===================================================================
void MapLocal::applyDynamicForwardField(int rowC, int colC,
                                        const Eigen::Vector3f& vel,
                                        float speed, float obj_confidence,
                                        std::unordered_set<int>& painted_set)
{
    const float dir_x = vel.x() / speed;
    const float dir_y = vel.y() / speed;

    for (int dr = -DYN_REACH; dr <= DYN_REACH; ++dr) {
        for (int dc = -DYN_REACH; dc <= DYN_REACH; ++dc) {
            int nr = rowC + dr, nc = colC + dc;
            if (!inBounds(nr, nc)) continue;

            float d_fwd = float(dc) * dir_x + float(dr) * dir_y;
            if (d_fwd <= 0.0f) continue;

            float dist_total = std::sqrt(float(dr*dr + dc*dc));
            if (dist_total > DYN_REACH) continue;

            float cos_theta = std::max(-1.0f, std::min(1.0f, d_fwd / dist_total));
            float theta     = std::acos(cos_theta);
            float weight    = std::exp(-d_fwd / DYN_SIGMA_D) *
                              std::exp(-(theta*theta) / (2.0f*DYN_SIGMA_TH*DYN_SIGMA_TH));

            auto  dyn_cost = static_cast<uint8_t>(COST_LETHAL * weight);
            int   idx      = flatIdx(nr, nc);

            if (m_data->objectLayer[idx] == COST_LETHAL) continue;
            if (m_data->objectLayer[idx] < dyn_cost) {
                m_data->objectLayer[idx]      = dyn_cost;
                float conf = obj_confidence * weight;
                if (m_data->objectConfidence[idx] < conf)
                    m_data->objectConfidence[idx] = conf;
                painted_set.insert(idx);
            }
        }
    }
}

// ===================================================================
//  inflateObjectCell — ניפוח מקומי לאובייקט בודד (שכבת אובייקטים)
//  תיקון #1: הניפוח כתוב כאן, לא ב-inflationPass הגלובלי
// ===================================================================
void MapLocal::inflateObjectCell(int rowC, int colC, int r,
                                 std::unordered_set<int>& painted_set)
{
    float radius_m = r * RESOLUTION;

    for (int dr = -r; dr <= r; ++dr) {
        for (int dc = -r; dc <= r; ++dc) {
            int nr = rowC + dr, nc = colC + dc;
            if (!inBounds(nr, nc)) continue;

            float dist_m = std::sqrt(float(dr*dr + dc*dc)) * RESOLUTION;
            if (dist_m > radius_m) continue;

            float   ratio    = 1.0f - (dist_m / radius_m);
            uint8_t inf_cost = static_cast<uint8_t>(COST_INFLATED * ratio);
            int     nidx     = flatIdx(nr, nc);

            if (m_data->objectLayer[nidx] == COST_LETHAL) continue;
            if (m_data->objectLayer[nidx] < inf_cost) {
                m_data->objectLayer[nidx] = inf_cost;
                float inf_conf = ratio * 0.8f;
                if (m_data->objectConfidence[nidx] < inf_conf)
                    m_data->objectConfidence[nidx] = inf_conf;
                painted_set.insert(nidx);
            }
        }
    }
}

// ===================================================================
//  securitySpace
// ===================================================================
int MapLocal::securitySpace(const std::string& type_label)
{
    if (type_label == "person" || type_label == "pedestrian")
        return static_cast<int>(std::ceil((INFLATION_R + 0.3f) / RESOLUTION));
    if (type_label == "child")
        return static_cast<int>(std::ceil((INFLATION_R + 0.5f) / RESOLUTION));
    return static_cast<int>(std::ceil(INFLATION_R / RESOLUTION));
}



void MapLocal::paintCircleOnRouteLayer(
    const Eigen::Vector2f& center_pos,
    float                  radius_m,
    uint8_t                cost)
{
    int col_c, row_c;
    if (!worldToGrid(center_pos.x(), center_pos.y(), col_c, row_c))
        return;

    int radius_cells = static_cast<int>(std::ceil(radius_m / RESOLUTION));

    for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
        for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
            int nr = row_c + dr, nc = col_c + dc;
            if (!inBounds(nr, nc)) continue;

            float dist_m = std::sqrt(float(dr*dr + dc*dc)) * RESOLUTION;
            if (dist_m > radius_m) continue;

            m_data->routeLimitLayer[flatIdx(nr, nc)] = cost;
        }
    }
}

// void MapLocal::registerCellChange(int row,int col,uint8_t new_cost)
// {
//     this->cells.push_back({
//         col,
//         row,
//         new_cost
//     });
// }

void MapLocal::markEdgeCorridor(
    const Eigen::Vector2f& from_pos,
    const Eigen::Vector2f& to_pos,
    float                  corridor_half_width)
{
    // Bounding box בקואורדינטות עולמיות
    float min_x = std::min(from_pos.x(), to_pos.x()) - corridor_half_width;
    float max_x = std::max(from_pos.x(), to_pos.x()) + corridor_half_width;
    float min_y = std::min(from_pos.y(), to_pos.y()) - corridor_half_width;
    float max_y = std::max(from_pos.y(), to_pos.y()) + corridor_half_width;

    // המרה לאינדקסי גריד + חיתוך לגבולות
    int col_min = std::max(0,        static_cast<int>((min_x - m_data->originX) / RESOLUTION));
    int col_max = std::min(GRID_W-1, static_cast<int>((max_x - m_data->originX) / RESOLUTION) + 1);
    int row_min = std::max(0,        static_cast<int>((min_y - m_data->originY) / RESOLUTION));
    int row_max = std::min(GRID_H-1, static_cast<int>((max_y - m_data->originY) / RESOLUTION) + 1);

    for (int row = row_min; row <= row_max; ++row) {
        for (int col = col_min; col <= col_max; ++col) {
            // מרכז התא בקואורדינטות עולמיות
            float wx = m_data->originX + (col + 0.5f) * RESOLUTION;
            float wy = m_data->originY + (row + 0.5f) * RESOLUTION;

            float dist = distancePointToSegment(
                Eigen::Vector2f(wx, wy), from_pos, to_pos);

            if (dist <= corridor_half_width)
                m_data->routeLimitLayer[flatIdx(row, col)] = COST_FREE;
        }
    }
}

// -------------------------------------------------------------------
//  applyNodeCurbRestrictions  (ללא נעילה — נקראת מתוך buildRouteLimitLayer)
//
//  סוגר עיגול קטן סביב צמתי מעבר חציה עם שפת מדרכה גבוהה מדי.
//  צומת עם has_traffic_light — נשאר פתוח (לוגיקה עתידית).
// -------------------------------------------------------------------

float MapLocal::distancePointToSegment(
    const Eigen::Vector2f& p,
    const Eigen::Vector2f& a,
    const Eigen::Vector2f& b)
{
    const Eigen::Vector2f ab = b - a;
    const Eigen::Vector2f ap = p - a;
    const float len_sq = ab.squaredNorm();

    if (len_sq < 1e-8f)
        return (p - a).norm();

    const float t = std::clamp(ap.dot(ab) / len_sq, 0.0f, 1.0f);
    const Eigen::Vector2f closest = a + t * ab;
    return (p - closest).norm();
}

// ← תיקון: המימוש היה חסר לחלוטין
const Edge* MapLocal::findEdgeBetween(
    const std::unordered_map<uint64_t, Node>& graph,
    uint64_t from_id,
    uint64_t to_id)
{
    auto it = graph.find(from_id);
    if (it == graph.end()) return nullptr;

    for (const Edge& e : it->second.edges) {
        if (e.target == to_id)
            return &e;
    }
    return nullptr;
}
void MapLocal::applyNodeCurbRestrictions(
    const std::vector<uint64_t>&              path_node_ids,
    const std::unordered_map<uint64_t, Node>& graph)
{
    for (uint64_t node_id : path_node_ids) {
        auto it = graph.find(node_id);
        if (it == graph.end()) continue;

        const Node& node = it->second;

        if (node.is_crossing && node.kerb_height > CURB_HEIGHT_THRESHOLD) {
            Eigen::Vector2f node_pos = latLonToLocal(
                node.lat, node.lon,
                m_data->origin_lat, m_data->origin_lon);  // ← node במקום from_node
            paintCircleOnRouteLayer(node_pos, CURB_BLOCK_RADIUS_M, COST_LETHAL);
        }
    }
}


// flushCells() is implemented as a thin wrapper around the locked
// consumeCellUpdates() above to ensure atomic, thread-safe consumption
// of the pending cell update buffer.
// -------------------------------------------------------------------
//  clearRouteLimitLayer  (ללא נעילה — נקראת מתוך buildRouteLimitLayer)
// -------------------------------------------------------------------
void MapLocal::clearRouteLimitLayer()
{
    m_data->routeLimitLayer.fill(COST_LETHAL);
}

// -------------------------------------------------------------------
//  buildRouteLimitLayer  — PUBLIC
//
//  שלבים:
//    1. בדיקת תקינות path (>= 2 צמתים).
//    2. נעילת mutex.
//    3. clearRouteLimitLayer() — הכל COST_LETHAL.
//    4. לכל זוג צמתים עוקבים:
//         א. מציאת Edge בגרף.
//         ב. המרת lat/lon למטרים.
//         ג. חישוב corridor_half עם clamp.
//         ד. markEdgeCorridor().
//    5. applyNodeCurbRestrictions() — חסימת curb גבוה.
// -------------------------------------------------------------------
void MapLocal::buildRouteLimitLayer(
    const std::vector<uint64_t>&              path_node_ids,
    const std::unordered_map<uint64_t, Node>& graph)
{
    if (path_node_ids.size() < 2) {
        std::unique_lock lock(m_data->rwMutex);
        m_data->routeLimitLayer.fill(COST_LETHAL);
        return;
    }

    std::unique_lock lock(m_data->rwMutex);
    clearRouteLimitLayer();

    static bool brl_first = true;
    if (brl_first) {
        brl_first = false;
        std::cout << "[MAP][BRL] FIRST CALL:"
                  << " origin_lat=" << m_data->origin_lat
                  << " origin_lon=" << m_data->origin_lon
                  << " originX=" << m_data->originX
                  << " originY=" << m_data->originY
                  << " mapW=" << GRID_W * RESOLUTION << "m"
                  << " mapH=" << GRID_H * RESOLUTION << "m"
                  << " path=" << path_node_ids.size() << " nodes"
                  << std::endl;
    }

    // קטע גישה: רובוט → צומת ראשון
    {
        auto first_it = graph.find(path_node_ids[0]);
        if (first_it != graph.end()) {
            Eigen::Vector2f robot_pos(
                m_data->originX + GRID_W * RESOLUTION * 0.5f,
                m_data->originY + GRID_H * RESOLUTION * 0.5f);
            Eigen::Vector2f first_pos = latLonToLocal(
                first_it->second.lat, first_it->second.lon,
                m_data->origin_lat, m_data->origin_lon);

            Eigen::Vector2f p1 = robot_pos;
            Eigen::Vector2f p2 = first_pos;
            if (clipSegmentToGrid(p1, p2))
                markEdgeCorridor(p1, p2, DEFAULT_CORRIDOR_WIDTH_M * 0.5f);
        }
    }

    // לולאת צמתים עוקבים
    for (std::size_t i = 0; i + 1 < path_node_ids.size(); ++i) {
        const uint64_t from_id = path_node_ids[i];
        const uint64_t to_id   = path_node_ids[i + 1];

        auto from_it = graph.find(from_id);
        auto to_it   = graph.find(to_id);
        if (from_it == graph.end() || to_it == graph.end()) continue;

        Eigen::Vector2f from_pos = latLonToLocal(
            from_it->second.lat, from_it->second.lon,
            m_data->origin_lat,  m_data->origin_lon);
        Eigen::Vector2f to_pos = latLonToLocal(
            to_it->second.lat, to_it->second.lon,
            m_data->origin_lat, m_data->origin_lon);

        const Edge* edge      = findEdgeBetween(graph, from_id, to_id);
        float       raw_width = DEFAULT_CORRIDOR_WIDTH_M;
        if (edge != nullptr && edge->width > 0.1f)
            raw_width = static_cast<float>(edge->width);

        float corridor_half = std::clamp(raw_width,
                                         MIN_CORRIDOR_WIDTH_M,
                                         MAX_CORRIDOR_WIDTH_M) * 0.5f;

        // ← חיתוך לגבולות הגריד
        if (!clipSegmentToGrid(from_pos, to_pos)) {
            if (i < 2)
                std::cout << "[MAP][BRL] seg" << i
                          << " entirely out of grid, skipping"
                          << std::endl;
            continue;  // ← דלג על קטע זה
        }

        // ← מגיעים לכאן רק אם הקטע עובר דרך הגריד
        if (i < 2) {
            int col_f = 0, row_f = 0, col_t = 0, row_t = 0;
            bool in_f = m_data->worldToGrid(
                from_pos.x(), from_pos.y(), col_f, row_f);
            bool in_t = m_data->worldToGrid(
                to_pos.x(), to_pos.y(), col_t, row_t);

            std::cout << "[MAP][BRL] seg" << i
                      << " from=(" << from_pos.x() << "," << from_pos.y()
                      << ") grid=(" << col_f << "," << row_f
                      << ") inBounds=" << in_f
                      << " | to=(" << to_pos.x() << "," << to_pos.y()
                      << ") grid=(" << col_t << "," << row_t
                      << ") inBounds=" << in_t
                      << " half=" << corridor_half << std::endl;

            int before = 0;
            for (int k = 0; k < GRID_W * GRID_H; ++k)
                if (m_data->routeLimitLayer[k] == COST_FREE) ++before;

            markEdgeCorridor(from_pos, to_pos, corridor_half);

            int after = 0;
            for (int k = 0; k < GRID_W * GRID_H; ++k)
                if (m_data->routeLimitLayer[k] == COST_FREE) ++after;

            std::cout << "[MAP][BRL] seg" << i
                      << " marked=" << (after - before)
                      << " cells free (total so far=" << after << ")"
                      << std::endl;
        } else {
            markEdgeCorridor(from_pos, to_pos, corridor_half);
        }
    }

    // חסימת צמתי מעבר חציה עם curb גבוה
    applyNodeCurbRestrictions(path_node_ids, graph);

    {
        int route_free = 0;
        for (int i = 0; i < GRID_W * GRID_H; ++i)
            if (m_data->routeLimitLayer[i] == COST_FREE) ++route_free;
        std::cout << "[MAP] buildRouteLimitLayer: free=" << route_free
                  << "/" << GRID_W * GRID_H
                  << " path=" << path_node_ids.size() << " nodes"
                  << std::endl;
    }
}  // ← סוגר נכון של buildRouteLimitLayer


bool MapLocal::clipSegmentToGrid(
    Eigen::Vector2f& p1,
    Eigen::Vector2f& p2) const
{
    const float x_min = m_data->originX;
    const float x_max = m_data->originX + GRID_W * RESOLUTION;
    const float y_min = m_data->originY;
    const float y_max = m_data->originY + GRID_H * RESOLUTION;

    if ((p1.x() < x_min && p2.x() < x_min) ||
        (p1.x() > x_max && p2.x() > x_max) ||
        (p1.y() < y_min && p2.y() < y_min) ||
        (p1.y() > y_max && p2.y() > y_max))
        return false;

    Eigen::Vector2f dir  = p2 - p1;
    float           len  = dir.norm();
    if (len < 1e-4f) return false;
    Eigen::Vector2f unit = dir / len;

    float t_min = 0.0f;
    float t_max = len;

    if (std::abs(unit.x()) > 1e-6f) {
        float t1 = (x_min - p1.x()) / unit.x();
        float t2 = (x_max - p1.x()) / unit.x();
        if (t1 > t2) std::swap(t1, t2);
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
    }

    if (std::abs(unit.y()) > 1e-6f) {
        float t1 = (y_min - p1.y()) / unit.y();
        float t2 = (y_max - p1.y()) / unit.y();
        if (t1 > t2) std::swap(t1, t2);
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
    }

    if (t_min >= t_max) return false;

    p2 = p1 + unit * t_max;
    p1 = p1 + unit * t_min;
    return true;
}