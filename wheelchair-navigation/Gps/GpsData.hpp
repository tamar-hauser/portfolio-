#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include "SensorData.hpp"
#include "GpsObject.hpp"
#include <cmath>     // נוסף עבור std::exp
#include <algorithm> // נוסף עבור std::clamp

class GpsData : public SensorData<GpsObject> {
public:
    GpsData() = default;    
    void process( GpsObject& myGPS) override;

private:
    // בונה את וקטור המדידה [X, Y, Z]
     void buildZ( GpsObject& myGPS)override; 
     void buildH( GpsObject& myGPS)override;
     void buildR( GpsObject& myGPS)override;
     void convertToLocalCoords(double lat, double lon, double& x, double& y);
};

