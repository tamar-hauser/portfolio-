#include "SensorFusionManager.hpp"
#include "SensorFusionData.hpp"
#include <opencv2/core/eigen.hpp>
#include <algorithm>
#include <cmath>
#include "SensorFusionM.hpp"

// SensorFusionManager::SensorFusionManager(std::unordered_map<int, std::shared_ptr<TrackedObject>> list) 
//     : tracked_map(std::move(list)) {}

std::vector<TrackedObject> SensorFusionManager::getTrackedListAsVector(
    std::unordered_map<int, std::shared_ptr<TrackedObject>> list)
{
    tracked_map = std::move(list);
    vec.clear();
    vec.reserve(tracked_map.size());
    for (auto const& [id, track_ptr] : tracked_map) {
        if (track_ptr)
            vec.push_back(*track_ptr);
    }
    return vec;
}


TrackedObject SensorFusionManager::createNewObject(SensorFusionObject& sf) {
TrackedObject to;
    SensorFusionData Sd;
    Sd.process(sf);
    
    to.id = ++my_static_id; 
    to.Z = sf.Z.cast<double>();
    to.H = sf.H.cast<double>();
    to.R = sf.R.cast<double>();
    to.F = sf.F.cast<double>();
    
    to.filter.x.setZero();
    to.filter.x.head<3>() = sf.position().template cast<double>();        
    
    if (sf.has_radar) {
        to.filter.x.segment<3>(3) = sf.filter.x.segment<3>(3).template cast<double>();   
    }

    to.filter.P = Eigen::Matrix<double, Config::StateSizeObject, Config::StateSizeObject>::Identity();
    to.filter.P.block<3, 3>(3, 3) *= 10.0f;
    
    to.length = sf.length;
    to.width = sf.width;
    to.height = sf.height;
    to.type_label = sf.type_label;
    to.confidence = sf.confidence;
    to.timestamp = sf.timestamp;
    to.traffic_light_color=sf.traffic_light_color;
    to.has_lidar = sf.has_lidar;
    to.has_camera = sf.has_camera;
    to.has_radar = sf.has_radar;
    if (sf.has_lidar) {
        to.cloud = sf.cloud;
    }
    to.bounding_box = sf.bounding_box;
    to.frames_active = 1;
    to.updated_this_frame = true;
    to.path.push_back(sf.position());
    ObjectUpdate.push_back(to.id);
    return to;
}

void SensorFusionManager::updateObject(SensorFusionObject& sensor_object, TrackedObject& track) {
    float alpha = calculateAlpha(sensor_object, track);

    track.length = (alpha * track.length) + ((1.0f - alpha) * sensor_object.length);
    track.width  = (alpha * track.width)  + ((1.0f - alpha) * sensor_object.width);
    track.height = (alpha * track.height) + ((1.0f - alpha) * sensor_object.height);

    SensorFusionData Sd;
    Sd.process(sensor_object);
    track.Z = sensor_object.Z.template cast<double>();
    track.H = sensor_object.H.template cast<double>();
    track.R = sensor_object.R.template cast<double>();
    track.F = sensor_object.F.template cast<double>();

    // Predict
    Eigen::Matrix<double, Config::StateSizeObject, Config::StateSizeObject> Q =
        Eigen::Matrix<double, Config::StateSizeObject, Config::StateSizeObject>::Identity() * 0.01;
    Eigen::Matrix<double, Config::StateSizeObject, 1> x_next = track.F * track.filter.x;
    track.filter.predict(x_next, track.F, Q);

    // Update
    if (track.Z.rows() > 0) {
        Eigen::VectorXd z_pred = track.H * track.filter.x;
        track.filter.updateDynamic(track.Z, track.H, track.R, z_pred);
    }

    if (sensor_object.has_lidar) {
        track.cloud = sensor_object.cloud;
        track.has_lidar = true;
    }
    if (sensor_object.has_camera) {
        track.image_points = sensor_object.img_points;
        track.has_camera = true;
        track.bounding_box = sensor_object.bounding_box;
    }
    track.traffic_light_color=sensor_object.traffic_light_color;
    track.type_label = sensor_object.type_label;
    track.confidence = (alpha * track.confidence) + ((1.0f - alpha) * sensor_object.confidence);
    track.timestamp = sensor_object.timestamp;
    track.frames_active++;
    track.missed_frames_counter = 0;
    track.updated_this_frame = true;
    track.path.push_back(track.position());
    ObjectUpdate.push_back(track.id);
}

void SensorFusionManager::increaseUncertainty(TrackedObject& track)
{
    track.updated_this_frame = false;
    track.missed_frames_counter++;
    track.confidence *= 0.95f;
    track.filter.P   *= 1.1f;

    const bool is_dynamic = (track.velocity().norm() >= 0.5f);

    if (is_dynamic && track.missed_frames_counter >= 7) {
        ObjectRemove.push_back(track.id);
        return;
    }

    if (!is_dynamic && track.missed_frames_counter >= 15) {
        ObjectRemove.push_back(track.id);
        return;
    }
}

float SensorFusionManager::calculateICPQualityScore(const TrackedObject& obj) {
    if (!obj.cloud || obj.cloud->size() < 10 || !obj.has_lidar || !obj.updated_this_frame) {
        return 0.0f; 
    }

    float velocity = obj.velocity().norm();
    float static_score = (velocity < 0.5f) ? 1.0f : (velocity < 2.0f ? 0.5f : 0.0f);
    float veteran_score = std::min(obj.frames_active / 30.0f, 1.0f);
    float confidence_score = obj.confidence;
    float volume = obj.length * obj.width * obj.height;
    float size_score = std::min(volume / 8.0f, 1.0f); 
    float stability_score = std::max(0.0f, 1.0f - (obj.missed_frames_counter * 0.2f));

    return (Config::CW_STATIC * static_score) + 
           (Config::W_VETERAN * veteran_score) + 
           (Config::W_CONFIDENCE * confidence_score) +
           (Config::W_SIZE * size_score) +
           (Config::W_STABILITY * stability_score);
}
std::vector<std::shared_ptr<TrackedObject>>
SensorFusionManager::getBestObjectsForICP()
{
    std::vector<std::shared_ptr<TrackedObject>> best_objects;
    for (auto const& [id, track_ptr] : tracked_map) {
        if (!track_ptr) continue;
        float score = calculateICPQualityScore(*track_ptr);
        if (score > 0.6f)
            best_objects.push_back(track_ptr);
    }
    return best_objects;
}