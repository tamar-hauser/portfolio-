#pragma once
#include <mutex>
#include <unordered_map>
#include <memory>
#include <cmath>
#include "SensorFrame.hpp"
#include "Database/FrameDatabase.hpp"
#include <iostream>

class SensorFrameManager {
private:
    std::unordered_map<long long, std::shared_ptr<FramePointers>> frame_map;
    bool initialization = false;

    std::unordered_map<int, std::shared_ptr<TrackedObject>> trackedObject;

    std::mutex hub_mutex;

    // 2Hz = חלון זמן של 0.5 שניות לכל frame
    static constexpr double FRAME_PERIOD_SEC = 0.5;

    SensorFrameManager() = default;

public:
    static SensorFrameManager& getInstance() {
        static SensorFrameManager instance;
        return instance;
    }

    std::shared_ptr<FramePointers> getOrCreateFrame(double ts) {
        long long bucket = static_cast<long long>(std::floor(ts / FRAME_PERIOD_SEC));

        std::lock_guard<std::mutex> lock(hub_mutex);

        auto it = frame_map.find(bucket);
        if (it == frame_map.end()) {
            // מוצא את ה-frame הקודם (bucket הכי גבוה שקטן מ-bucket הנוכחי)
            long long prev_bucket = -1;
            std::shared_ptr<FramePointers> prev_frame_ptr;
            for (auto& [b, fp] : frame_map) {
                if (b < bucket && b > prev_bucket) {
                    prev_bucket = b;
                    prev_frame_ptr = fp;
                }
            }
            auto frame = std::make_shared<FramePointers>();
            frame->timestamp = bucket * FRAME_PERIOD_SEC;
            if (prev_frame_ptr) {
                frame->previousFrame = prev_frame_ptr;
            }
            frame_map[bucket] = frame;
            std::cout << "[FRAME] getOrCreateFrame ts=" << ts << " bucket=" << bucket;
            if (prev_frame_ptr)
                std::cout << " prev_bucket=" << prev_bucket;
            std::cout << std::endl;
            return frame;
        }

        static int reuse_count = 0;
        if (++reuse_count % 50 == 0)
            std::cout << "[FRAME] getOrCreateFrame: reusing bucket=" << bucket
                      << " ts=" << ts << std::endl;
        return it->second;
    }

    // alias — שומר על תאימות עם קוד קיים
    std::shared_ptr<FramePointers> getOrAddFrame(double ts) {
        return getOrCreateFrame(ts);
    }

    // מחזיר את ה-frame הישן ביותר שחלון הזמן שלו נסגר (קיים bucket חדש יותר)
    // מבטיח שכל החיישנים של אותו bucket הספיקו להיכנס לפני שמתחילים לעבד
    std::shared_ptr<FramePointers> getProcessFrame() {
        std::lock_guard<std::mutex> lock(hub_mutex);

        static int count = 0;
        bool should_log = (++count % 50 == 0);

        if (should_log)
            std::cout << "[FRAME] getProcessFrame frames count=" << frame_map.size() << std::endl;

        if (frame_map.empty()) {
            if (should_log)
                std::cerr << "[WARN] getProcessFrame found no processable frame" << std::endl;
            return nullptr;
        }

        // מוצא את ה-bucket הגבוה ביותר (חלון הזמן הנוכחי — עדיין פתוח)
        long long max_bucket = -1;
        for (auto& [b, frame] : frame_map)
            if (frame) max_bucket = std::max(max_bucket, b);

        // מוצא את ה-frame החדש ביותר שלא עובד ושחלונו נסגר (b < max_bucket)
        long long best_bucket = -1;
        std::shared_ptr<FramePointers> best_frame;

        for (auto& [b, frame] : frame_map) {
            if (!frame) continue;
            if (should_log)
                std::cout << "[FRAME] getProcessFrame checking bucket=" << b
                          << " ts=" << frame->timestamp
                          << " processed=" << frame->is_processed
                          << " window_closed=" << (b < max_bucket) << std::endl;
            if (!frame->is_processed && b < max_bucket) {
                if (best_bucket == -1 || b > best_bucket) {
                    best_bucket = b;
                    best_frame = frame;
                }
            }
        }

        if (!best_frame) {
            if (should_log)
                std::cerr << "[WARN] getProcessFrame found no processable frame" << std::endl;
            return nullptr;
        }

        // skip all stale frames older than the selected (newest) frame
        int skipped_count = 0;
        for (auto& [b, frame] : frame_map) {
            if (frame && !frame->is_processed && b < max_bucket && b < best_bucket) {
                frame->is_processed = true;
                ++skipped_count;
            }
        }
        if (skipped_count > 0)
            std::cout << "[FRAME] skipped " << skipped_count << " stale frames older than bucket=" << best_bucket << std::endl;

        // [FRAME][READY] — always print when a frame is selected for processing
        // read counts without frame_mutex (window is closed, no new writes expected)
        std::cout << "[FRAME][READY] bucket=" << best_bucket
                  << " ts=" << best_frame->timestamp
                  << "\n  imu_samples="     << (best_frame->imu     ? best_frame->imu->list.size()     : 0u)
                  << "\n  encoder_samples=" << (best_frame->encoder ? best_frame->encoder->list.size() : 0u)
                  << "\n  has_gps="         << (best_frame->gps     ? best_frame->gps->isValid         : false)
                  << "\n  has_lidar="       << (best_frame->lidar   ? !best_frame->lidar->list.empty() : false)
                  << "\n  lidar_objects="   << (best_frame->lidar   ? best_frame->lidar->list.size()   : 0u)
                  << "\n  has_radar="       << (best_frame->radar   ? !best_frame->radar->list.empty() : false)
                  << "\n  radar_objects="   << (best_frame->radar   ? best_frame->radar->list.size()   : 0u)
                  << "\n  has_camera="      << (best_frame->camera  ? !best_frame->camera->list.empty(): false)
                  << "\n  camera_objects="  << (best_frame->camera  ? best_frame->camera->list.size()  : 0u)
                  << "\n  processed="       << best_frame->is_processed
                  << "\n  window_closed="   << (best_bucket < max_bucket)
                  << std::endl;

        return best_frame;
    }

    void cleanupOldFrames(double current_ts) {
        std::lock_guard<std::mutex> lock(hub_mutex);
        double threshold = current_ts - 5.0;
        for (auto it = frame_map.begin(); it != frame_map.end(); ) {
            if (it->second && it->second->timestamp < threshold) {
                FrameDatabase::getInstance().saveFrame(it->first, *it->second);
                it = frame_map.erase(it);
            } else {
                ++it;
            }
        }
    }

    std::unordered_map<int, std::shared_ptr<TrackedObject>> getTrackedObject()
    {
        std::lock_guard<std::mutex> lock(hub_mutex);
        return trackedObject;
    }

    void setTrackedObjects(const std::vector<TrackedObject>& list)
    {
        std::lock_guard<std::mutex> lock(hub_mutex);
        trackedObject.clear();
        for (const auto& obj : list) {
            trackedObject[obj.id] = std::make_shared<TrackedObject>(obj);
        }
    }
};
