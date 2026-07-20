#pragma once 
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <string>
#include <queue>
#include <cmath>
#include "GraphMap.hpp"
#define _USE_MATH_DEFINES
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

using namespace std;
constexpr double DEG_TO_RAD = M_PI / 180.0;

class AStar {
public:
    AStar() = default;
    vector<uint64_t> run(unordered_map<uint64_t, Node>& graph,
                         uint64_t start_id,
                         uint64_t end_id);    
    // פונקציה שמפעילה את החיפוש
    void FindPath(uint64_t start_id, uint64_t end_id);

private: 
    struct CompareNodes {
        bool operator()(Node* a, Node* b) { return a->f_value() > b->f_value(); }
    };  
    double CalculateHValue(const Node* node, const Node* goal) const;
    void   AddNeighbors(Node* current,Node* goal,unordered_map<uint64_t, Node>& graph,
            priority_queue<Node*,vector<Node*>,CompareNodes>& open_list);                                                                          
    vector<uint64_t> ConstructFinalPath(const Node* goal,
                                        const unordered_map<uint64_t, Node>& graph) const;
    static double toRadians(double deg) { return deg * DEG_TO_RAD; }

};