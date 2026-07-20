#pragma once

#include <vector>
#include <cstdint>
#include <cmath>

// ─────────────────────────────────────────────
// טיפוסי נתונים בסיסיים למפת כיסא גלגלים
// ─────────────────────────────────────────────

enum SurfaceType : uint8_t {
    ASPHALT        = 0,
    CONCRETE       = 1,
    PAVING_STONES  = 2,
    COBBLESTONE    = 3,
    GRAVEL         = 4,
    GRASS          = 5,
    SAND           = 6,
    UNKNOWN        = 7
};

struct Edge {
    uint64_t    target;
    double      effort_cost;
    double      length;
    double      width;
    double      incline;
    SurfaceType surface;
    double      safe_speed;
};

struct Node {
    // נתוני מיקום
    double lat = 0.0;
    double lon = 0.0;

    // נתוני נגישות
    double kerb_height      = 0.0;
    bool   is_crossing      = false;
    bool   has_traffic_light= false;

    // רשימת סמיכויות
    std::vector<Edge> edges;

    // ── משתני עזר לאלגוריתמי ניווט (לא נשמרים ב-Binary) ──
    double  g_value = 1e9;     // עלות מצטברת מהמוצא
    double  h_value = 0.0;     // הערכת מרחק לייעד (heuristic)
    Node*   parent  = nullptr; // מצביע להורה במסלול
    bool    visited = false;   // סמן לבדיקת ביקור

    double f_value() const { return g_value + h_value; }
};
