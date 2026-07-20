#pragma once
#include <memory>
#include <vector>
#include <mutex>

#include "LidarList.hpp"
#include "SensorFusion/SensorFusionList.hpp"
#include "Radar/RadarList.hpp"   
#include "Camera/CameraList.hpp"
#include "Imu/ImuList.hpp"
#include "Encoder/EncoderList.hpp"
#include "GPS/GpsObject.hpp"
#include "TrackedObject/TrackeList.hpp" // וודא שזה שם הקובץ הנכון
#include "TrackedObject/TrackedObject.hpp" // וודא שזה שם הקובץ הנכון

struct FramePointers {
    std::vector<std::shared_ptr<TrackedObject>> tracked;
    std::shared_ptr<SensorFusionList> sensorfusion;
    std::shared_ptr<LidarList> lidar;
    std::shared_ptr<RadarList> radar;
    std::shared_ptr<CameraList> camera;
    std::shared_ptr<ImuList> imu;
    std::shared_ptr<EncoderList> encoder;
    std::shared_ptr<GpsObject> gps;
    std::vector<int> idObjectUpdate;
    std::vector<int> idObjectRemove;
    std::weak_ptr<FramePointers> previousFrame; 
    std::shared_ptr<FramePointers> nextFrame;
    std::mutex frame_mutex;
    bool is_processed;
    double timestamp;

    FramePointers()
    : sensorfusion(std::make_shared<SensorFusionList>()), // התווסף אתחול שהיה חסר
      lidar(std::make_shared<LidarList>()),
      radar(std::make_shared<RadarList>()),
      camera(std::make_shared<CameraList>()),
      imu(std::make_shared<ImuList>()),
      encoder(std::make_shared<EncoderList>()),
      gps(std::make_shared<GpsObject>()),
      is_processed(false),
      timestamp(0.0)
    {
        // tracked הוא std::vector ולכן הוא מאותחל אוטומטית כווקטור ריק.
        // אין צורך לרשום אותו ברשימת האתחול למעלה.
    }
};