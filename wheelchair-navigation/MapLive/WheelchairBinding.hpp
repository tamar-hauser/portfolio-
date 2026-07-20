#pragma once

#include <vector>
#include <unordered_map>
#include <cstdint>
#include <string>

#include "MapData.hpp"
#include "GraphMap.hpp"

struct GraphResult {
    std::unordered_map<uint64_t, Node> graph;
    uint64_t source_node;
    uint64_t target_node;
};

GraphResult buildGraphFromPython(double orig_lat, double orig_lon,
                                 double dest_lat, double dest_lon,
                                 const std::string& config_path = "wheelchair_config.json");