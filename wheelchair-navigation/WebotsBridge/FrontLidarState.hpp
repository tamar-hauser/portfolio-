#pragma once
#include <atomic>
#include <limits>

namespace FrontLidar {
    inline std::atomic<float>  g_min_range_m{std::numeric_limits<float>::infinity()};
    inline std::atomic<double> g_min_range_ts{-1.0};
}
