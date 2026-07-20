#include "AStar.hpp"
#include <algorithm>
#include <limits>
#include <cstring>
#include <iostream>

vector<uint64_t> AStar::run(unordered_map<uint64_t, Node>& graph,
                             uint64_t start_id,
                             uint64_t end_id)
{
    if (graph.find(start_id) == graph.end() ||
        graph.find(end_id)   == graph.end())
    {
        cerr << "[AStar] start_id או end_id לא נמצאו בגרף\n";
        return {};
    }

    for (auto& [id, node] : graph) {
        node.g_value = numeric_limits<double>::infinity();
        node.h_value = 0.0;
        node.parent  = nullptr;
        node.visited = false;
    }

    Node* start_node = &graph.at(start_id);
    Node* end_node   = &graph.at(end_id);

    start_node->g_value = 0.0;
    start_node->h_value = CalculateHValue(start_node, end_node);

    // ── תור עדיפויות (min-heap לפי f) ────────────────────────────
    priority_queue<Node*, vector<Node*>, CompareNodes> open_list;
    open_list.push(start_node);

    // ── לולאת חיפוש ──────────────────────────────────────────────
    while (!open_list.empty()) {
        Node* current = open_list.top();
        open_list.pop();
        if (current->visited) continue;
        current->visited = true;
       
        // ── בדיקת הגעה ליעד ──────────────────────────────────────
        if (current == end_node) {
            return ConstructFinalPath(end_node, graph);
        }

        // ── פיתוח שכנים ──────────────────────────────────────────
        AddNeighbors(current, end_node, graph, open_list);
    }

    cerr << "[AStar] לא נמצא מסלול בין " << start_id << " ל-" << end_id << "\n";
    return {};
}

void AStar::AddNeighbors(Node* current,
                          Node* goal,
                          unordered_map<uint64_t, Node>& graph,
                          priority_queue<Node*,
                                         vector<Node*>,
                                         CompareNodes>& open_list)
{
    for (const Edge& edge : current->edges) {
        auto it = graph.find(edge.target);
        if (it == graph.end()) continue;

        Node* neighbor = &it->second;
        if (neighbor->visited) continue;

        double tentative_g = current->g_value + edge.effort_cost;

        if (tentative_g < neighbor->g_value) {
            neighbor->parent  = current;
            neighbor->g_value = tentative_g;
            neighbor->h_value = CalculateHValue(neighbor, goal);
            open_list.push(neighbor);
        }
    }
}

vector<uint64_t> AStar::ConstructFinalPath(
    const Node* goal,
    const unordered_map<uint64_t, Node>& graph) const
{
    vector<uint64_t> path;

    const Node* curr = goal;
    while (curr != nullptr) {
        // מציאת ה-ID של הצומת לפי כתובת בזיכרון
        for (const auto& [id, node] : graph) {
            if (&node == curr) {
                path.push_back(id);
                break;
            }
        }
        curr = curr->parent;
    }

    reverse(path.begin(), path.end());
    return path;
}

double AStar::CalculateHValue(const Node* node, const Node* goal) const
{
    const double lat1 = toRadians(node->lat);
    const double lon1 = toRadians(node->lon);
    const double lat2 = toRadians(goal->lat);
    const double lon2 = toRadians(goal->lon);

    const double dLat = lat2 - lat1;
    const double dLon = lon2 - lon1;

    const double a = sin(dLat / 2) * sin(dLat / 2)
                   + cos(lat1) * cos(lat2)
                   * sin(dLon / 2) * sin(dLon / 2);
    const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    constexpr double R = 6371000.0; // רדיוס כדור הארץ [מ']
    constexpr double MIN_EFFORT_MULTIPLER = 1.0;       // חייב להיות ≤ min(effort/length)

    return R * c * MIN_EFFORT_MULTIPLER;
}