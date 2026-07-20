#pragma once
#include <unordered_map>
#include <memory>
#include <atomic>
#include "SensorFusionM.hpp"
#include "SensorFusionObject.hpp"
#include "TrackedObject/TrackedObject.hpp"
#include "Constants.hpp"
#include <climits>
class SensorFusionManager : public SensorFusionM<SensorFusionObject, TrackedObject> {
public:
    SensorFusionManager() = default;

    std::vector<TrackedObject> getTrackedListAsVector(
        std::unordered_map<int, std::shared_ptr<TrackedObject>> list);

    std::vector<std::shared_ptr<TrackedObject>> getBestObjectsForICP();

    std::vector<int> getObjectUpdate() { return ObjectUpdate; }
    std::vector<int> getObjectRemove() { return ObjectRemove; }
    const std::unordered_map<int, std::shared_ptr<TrackedObject>>& getTrackedMap() const { return tracked_map; }

    void initTrackedMap(const std::vector<TrackedObject>& list) {
        tracked_map.clear();
        for (const auto& obj : list) {
            tracked_map[obj.id] = std::make_shared<TrackedObject>(obj);
        }
    }

    void clearFrameLists() {
        ObjectUpdate.clear();
        ObjectRemove.clear();
    }

private:
    inline static std::atomic<int> my_static_id{0};
    std::unordered_map<int, std::shared_ptr<TrackedObject>> tracked_map;
    std::vector<int>          ObjectUpdate;
    std::vector<int>          ObjectRemove;
    std::vector<TrackedObject> vec;

    void          updateObject      (SensorFusionObject& sensor_object, TrackedObject& track) override;
    TrackedObject createNewObject   (SensorFusionObject& sensor_object) override;
    void          increaseUncertainty(TrackedObject& track) override;

    float calculateICPQualityScore(const TrackedObject& obj);
};