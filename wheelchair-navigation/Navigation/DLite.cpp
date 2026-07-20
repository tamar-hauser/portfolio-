#include "DLite.hpp"

// ==========================================
// Key operators
// ==========================================
bool DLite::Key::operator>(const Key& o) const {
    return k1 > o.k1 || (k1 == o.k1 && k2 > o.k2);
}

bool DLite::Key::operator<(const Key& o) const {
    return o > *this;
}

// ==========================================
// Constructor
// ==========================================
DLite::DLite(int w, int h, float res)
    : W_(w), H_(h), RES_(res),
      cells(w * h),
      cost_map_(w * h, 0)
{}

// ==========================================
// initialize
// ==========================================
void DLite::initialize(int start_col, int start_row,
                        int goal_col,  int goal_row,
                        const std::array<uint8_t, GRID_W * GRID_H>& fused_map)
{
    start_ = flat(start_col, start_row);
    goal_  = flat(goal_col, goal_row);

    // 1. טעינת המפה
    for (int i = 0; i < W_ * H_; ++i)
    {
        cost_map_[i] = fused_map[i];
    }

    // 2. איפוס מצבים
    for (auto& c : cells)
    {
        c.g = INF;
        c.rhs = INF;
    }

    // 3. אתחול יעד
    cells[goal_].rhs = 0.0f;

    pq_ = decltype(pq_){};   // במקום: pq_.clear();
    in_queue_.clear(); 
    pq_.push({ calcKey(goal_), goal_ });
    in_queue_[goal_] = calcKey(goal_);

    // diagnostic: count navigable cells in cost_map
    {
        int nav = 0;
        for (int i = 0; i < W_ * H_; ++i)
            if (cost_map_[i] < COST_LETHAL) ++nav;
        std::cerr << "[DSTAR] cost_map: navigable=" << nav << "/" << W_*H_
                  << " start=" << (int)cost_map_[start_]
                  << " goal=" << (int)cost_map_[goal_] << "\n";
    }

    // 4. ריצה ראשונית אחת בלבד
    computeShortestPath();
}

// ==========================================
// updateAndReplan
// ==========================================
void DLite::updateAndReplan(
    const std::vector<CellUpdate>& updates,
    int robot_col,
    int robot_row)
{
    km_ += h(start_, flat(robot_col, robot_row));
    start_ = flat(robot_col, robot_row);

    // עדכון רק תאים ששונו
    for (const auto& u : updates)
    {
        int idx = flat(u.col, u.row);

        if (cost_map_[idx] == u.new_cost)
            continue;

        cost_map_[idx] = u.new_cost;

        updateCell(idx);

        // חשוב מאוד ליציבות D*
        for (int n : neighbors(idx))
            updateCell(n);
    }

    computeShortestPath();
}

// ==========================================
// getPath
// ==========================================
std::vector<std::pair<int,int>> DLite::getPath() const {
    std::vector<std::pair<int,int>> path;
    int cur = start_;
    if (cells[cur].g >= INF) return path; // אין מסלול

    std::unordered_set<int> visited; // מניעת לולאות
    visited.insert(cur); // מסמן כנבקר, לא דוחף לנתיב

    for (int step = 0; step < W_ * H_; ++step) {
        if (cur == goal_) break;

        int   best_n    = -1;
        float best_cost = INF;
        for (int n : neighbors(cur)) {
            if (visited.count(n)) continue; // לא לחזור לתא שביקרנו
            float c = moveCost(cur, n) + cells[n].g;
            if (c < best_cost) { best_cost = c; best_n = n; }
        }

        if (best_n < 0 || best_cost >= INF) break; // אין מסלול
        cur = best_n;
        path.push_back(toColRow(cur));
        visited.insert(cur);
    }

    // אם לא הגענו ליעד — המסלול לא שלם
    if (cur != goal_) return {};
    return path;
}

// ==========================================
// פונקציות עזר פרטיות
// ==========================================
int DLite::flat(int col, int row) const {
    return row * W_ + col;
}

std::pair<int,int> DLite::toColRow(int idx) const {
    return { idx % W_, idx / W_ };
}

float DLite::h(int a, int b) const {
    auto [ac, ar] = toColRow(a);
    auto [bc, br] = toColRow(b);
    return RES_ * std::sqrt(float((ac-bc)*(ac-bc) +
                                   (ar-br)*(ar-br)));
}

float DLite::moveCost(int from, int to) const {
    uint8_t c = cost_map_[to];
    if (c >= COST_LETHAL)  return INF;
    if (c == COST_UNKNOWN) return 10.0f * RES_;

    // בדוק אם המהלך אלכסוני
    auto [fc, fr] = toColRow(from);
    auto [tc, tr] = toColRow(to);
    float step = (fc != tc && fr != tr) ? RES_ * 1.4142f : RES_;

    return step * (1.0f + static_cast<float>(c) / 255.0f);
}

DLite::Key DLite::calcKey(int s) const {
    float mn = std::min(cells[s].g, cells[s].rhs);
    return { mn + h(start_, s) + km_, mn };
}

void DLite::updateCell(int u) {
    if (u != goal_) {
        float best = INF;
        for (int n : neighbors(u)) {
            float c = moveCost(u, n) + cells[n].g;
            if (c < best) best = c;
        }
        cells[u].rhs = best;
    }

    // הסר ערך ישן אם קיים
    in_queue_.erase(u);

    if (cells[u].g != cells[u].rhs) {
        Key k = calcKey(u);
        pq_.push({ k, u });
        in_queue_[u] = k;
    }
}

void DLite::computeShortestPath()
{
    while (!pq_.empty())
    {
        auto [k_old, u] = pq_.top();
        pq_.pop();

        // stale check via in_queue_: skip if not in map or key changed
        auto it = in_queue_.find(u);
        if (it == in_queue_.end() ||
            it->second < k_old || k_old < it->second)
        {
            continue;
        }
        in_queue_.erase(u); // consume — prevents duplicate-key else-branch cascade

        if (!(k_old < calcKey(start_)) &&
            cells[start_].rhs == cells[start_].g)
        {
            break;
        }

        if (cells[u].g > cells[u].rhs)
        {
            cells[u].g = cells[u].rhs;

            for (int s : neighbors(u))
                updateCell(s);
        }
        else
        {
            cells[u].g = INF;

            updateCell(u);

            for (int s : neighbors(u))
                updateCell(s);
        }
    }
}

void DLite::setGoal(int new_col, int new_row)
{
    // 1. ביטול יעד ישן — rhs חוזר ל-INF ומתפשט לשכנים
    cells[goal_].rhs = INF;
    updateCell(goal_);

    // 2. הגדרת יעד חדש — rhs=0 מסמן "כאן הסוף"
    goal_ = flat(new_col, new_row);
    cells[goal_].rhs = 0.0f;

    // 3. הכנסה לתור — computeShortestPath יידע מאיפה להתחיל
    Key k = calcKey(goal_);
    pq_.push({ k, goal_ });
    in_queue_[goal_] = k;

    // 4. חישוב מחדש (incremental — שומר g-values מצמתים קיימים)
    computeShortestPath();
}


std::vector<int> DLite::neighbors(int idx) const
{
    std::vector<int> nb;
    auto [c, r] = toColRow(idx);

    for (int dr = -1; dr <= 1; ++dr)
    for (int dc = -1; dc <= 1; ++dc)
    {
        if (dr == 0 && dc == 0)
            continue;

        int nc = c + dc;
        int nr = r + dr;

        if (nr < 0 || nr >= H_ || nc < 0 || nc >= W_)
            continue;

        // בדיקה בסיסית לחסימה
        if (cost_map_[flat(nc, nr)] >= COST_LETHAL)
            continue;

        // אם זה אלכסון → בדיקת "משולש פנוי"
        if (dc != 0 && dr != 0)
        {
            int n1 = flat(c + dc, r);     // צד אופקי
            int n2 = flat(c, r + dr);     // צד אנכי

            if (cost_map_[n1] >= COST_LETHAL ||
                cost_map_[n2] >= COST_LETHAL)
            {
                continue; // חיתוך פינה → אסור
            }
        }

        nb.push_back(flat(nc, nr));
    }

    return nb;
}